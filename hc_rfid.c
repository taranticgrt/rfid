#include <string.h> // for memcpy
#include <math.h>	// for ceil, used in Delay_us
#include "hc_rfid.h"

#warning Remove this hardware dependency
#include "stm32l4xx_hal.h"

#include "rfid_const.h"

#define ErrorReg 0x06

/// HARDWARE DEPENDENT OR OS DEPENDENT

extern SPI_HandleTypeDef hspi1;
static uint8_t RC522_SPI_transfer(uint8_t data);
static void MFRC522_write(uint8_t addr, uint8_t val);
static uint8_t MFRC522_read(uint8_t addr);

#define RST_Up() HAL_GPIO_WritePin(PORT_RFID_RST, PIN_RFID_RST, GPIO_PIN_SET)
#define RST_Down() HAL_GPIO_WritePin(PORT_RFID_RST, PIN_RFID_RST, GPIO_PIN_RESET)
#define NSS_Up() HAL_GPIO_WritePin(PORT_RFID_NSS, PIN_RFID_NSS, GPIO_PIN_SET)
#define NSS_Down() HAL_GPIO_WritePin(PORT_RFID_NSS, PIN_RFID_NSS, GPIO_PIN_RESET)

// Script functions.
// Check document jcf file COMMANDS_en.docx in the docs folder.
static void command_CLL(uint8_t REG[NUM_REGS], uint8_t, uint8_t, uint8_t);
static void command_SR(uint8_t REG[NUM_REGS], uint8_t, uint8_t, uint8_t);
static void command_SRR(uint8_t REG[NUM_REGS], uint8_t, uint8_t, uint8_t);
static void command_GR(uint8_t REG[NUM_REGS], uint8_t, uint8_t, uint8_t);
static void command_SLP(uint8_t REG[NUM_REGS], uint8_t, uint8_t, uint8_t);
static void command_RE(uint8_t REG[NUM_REGS], uint8_t, uint8_t, uint8_t);
static void command_MOV(uint8_t REG[NUM_REGS], uint8_t, uint8_t, uint8_t);
static void command_RF(uint8_t REG[NUM_REGS], uint8_t, uint8_t, uint8_t);

////////////////////////////////////////////////////////////////////////////////////
/// Script Functions
////////////////////////////////////////////////////////////////////////////////////

typedef void (*TypeCommandFunc)(uint8_t REG[NUM_REGS], uint8_t, uint8_t, uint8_t);

typedef struct
{
	uint8_t cmd;
	TypeCommandFunc f;
} command_pair;

// These must match MFRC522_command_list from rfid_const.h
command_pair command_table[] = {
	{CMD_END, NULL},
	{CMD_CLL, command_CLL},
	{CMD_SR, command_SR},
	{CMD_SRR, command_SRR},
	{CMD_GR, command_GR},
	{CMD_SLP, command_SLP},
	{CMD_RE, command_RE},
	{CMD_MOV, command_MOV},
	{CMD_RF, command_RF},
	{0, NULL} // used to identify the end of the talbe.
};

/*
This function is the core of the RFID tag reading.
It takes a sequence of commands, stored in CMDs, and run then one by one.
For the sequence of commands, check the read script RFID_read_script.c.

As the script is executed, the registers in REG[NUM_REGS] are modified and store
the relevant information.

If the script finds no error, it will return 0.

TODO: add tag data error checking.
*/
unsigned int MFRC522_read_tag_data(uint8_t REG[NUM_REGS], const uint8_t CMDs[][4], HC_RFID_info *tag_data)
{
	unsigned int cnt = 0;
	unsigned int end = 0;
	memset(tag_data, 0, sizeof(*tag_data));
	while (!end)
	{
		MFRC522_command_list cmd = (MFRC522_command_list)CMDs[cnt][0];
		int index = 0;
		while (command_table[index].cmd != 0)
		{
			if (command_table[index].cmd == cmd)
			{
				break;
			}
			index++;
		}
		if (command_table[index].cmd == CMD_INVALID)
		{
			return 1;
		}
		if (command_table[index].cmd != CMD_END)
		{
			command_table[index].f(REG, CMDs[cnt][1], CMDs[cnt][2], CMDs[cnt][3]);
			cnt++;
		}
		else
		{
			end = 1;
		}
	}

	if (REG[REG_ATQH] == 0 && REG[REG_ATQL] == 4)
	{
		tag_data->tag_present = 1;
	}
	if (tag_data->tag_present)
	{
		for (unsigned k = 0; k < RFID_UID_SIZE; k++)
		{
			tag_data->UID[k] = REG[REG_ML0 + k];
		}
		tag_data->uid_valid = tag_data->tag_present;
		tag_data->data_valid = 1;
		for (unsigned k = 0; k < RFID_BLOCK_SIZE; k++)
		{
			tag_data->data[k] = REG[REG_DATA0 + k];
		}
	}
	return 0;
}

//////////////////////// SUPPORT FUNCTIONS ///////////////////////////
static void command_CLL(uint8_t REG[NUM_REGS], uint8_t param1, uint8_t param2, uint8_t param3)
{
	UNUSED(param1);
	UNUSED(param2);
	UNUSED(param3);
	memset(REG, 0, sizeof(REG[0]) * NUM_REGS);
}
/*
Command:  SR *
Synopsis: SR <address> <data>
Description:
The SR function sets a Rc52x register, located at address <address> according to data <data>, both specified as an 8-bit HEX value.
*/
static void command_SR(uint8_t REG[NUM_REGS], uint8_t address, uint8_t data, uint8_t param3)
{
	UNUSED(REG);
	UNUSED(param3);
	MFRC522_write(address, data);
}

static void command_SRR(uint8_t REG[NUM_REGS], uint8_t address, uint8_t reg, uint8_t param3)
{
	UNUSED(param3);
	MFRC522_write(address, REG[reg]);
}

/*
Command:  GR
Synopsis: GR <address>
Description:
The GR function gets data from a Rc52x register,
located at address <address> (8-bit HEX).
The retrieved value is stored in IOR.
*/
static void command_GR(uint8_t REG[NUM_REGS], uint8_t addr, uint8_t param2, uint8_t param3)
{
	UNUSED(param2);
	UNUSED(param3);
	uint8_t c = MFRC522_read(addr);
	REG[REG_IOR] = c;
	if (addr == ErrorReg)
	{
		REG[REG_IOE] = c;
	}
}

/*
Command:  RE
Synopsis: RE <address> <data>
Description:
The RE function compares a Rc52x register,
located at address <address> to data, specified in the <data> parameter.
If equal, IOR is 0, otherwise 1. All values are in 8-bit HEX format.
*/
static void command_RE(uint8_t REG[NUM_REGS], uint8_t address, uint8_t data, uint8_t param3)
{
	UNUSED(param3);
	uint8_t c = MFRC522_read(address);
	REG[REG_IOR] = (c == data) ? 0 : 1;
}
/*
Command:  RF
Synopsis: RF <address> <data> <mask>
Description:
The RF function compares a Rc52x register, located at address <address> to data,
specified in the <data> parameter, AND'ed with the content of <mask>.
If equal, IOR is 0, otherwise 1. All values are in 8-bit HEX format.
*/
static void command_RF(uint8_t REG[NUM_REGS], uint8_t address, uint8_t data, uint8_t mask)
{
	uint8_t registerValue = MFRC522_read(address);
	uint8_t maskedValue = registerValue & mask;
	REG[REG_IOR] = (maskedValue == data) ? 0 : 1;
}

/*
 Command:  SLP *
Synopsis: SLP <timeout_ms>
Description:
The SLP function waits for the time is specified by <timeout_ms>, in [ms] to expire.
*/
static void command_SLP(uint8_t REG[NUM_REGS], uint8_t ms, uint8_t param2, uint8_t param3)
{
	UNUSED(REG);
	UNUSED(param2);
	UNUSED(param3);
	HAL_Delay(ms);
}

/*
 Command:  MOV
Synopsis: MOV <destination> <source>
Description:
The MOV function copies User Register or plain data
from <source> to <destination>. Data are in 8-bit HEX format.
*/
static void command_MOV(uint8_t REG[NUM_REGS], uint8_t destination, uint8_t source, uint8_t param3)
{
	UNUSED(param3);
	REG[destination] = REG[source];
}

////////////////////////////////////////////////////////////////////////////////////
/// HARDWARE DEPENDENT
////////////////////////////////////////////////////////////////////////////////////

void MFRC522_hard_reset(void)
{
	RST_Down();
	HAL_Delay(100);
	RST_Up();
	HAL_Delay(10);
}

void MFRC522_write(uint8_t addr, uint8_t val)
{
	/* CS LOW */
	NSS_Down();
	HAL_Delay(1); // Delay_us(100);
	// The address is located:0XXXXXX0
	RC522_SPI_transfer((addr << 1) & 0x7E);
	RC522_SPI_transfer(val);
	/* CS HIGH */
	HAL_Delay(1); // Delay_us(100);
	NSS_Up();
	HAL_Delay(1); // Delay_us(100);
}

uint8_t MFRC522_read(uint8_t addr)
{
	uint8_t val;
	/* CS LOW */
	NSS_Down();
	HAL_Delay(1); // Delay_us(100);

	// The address is located:1XXXXXX0
	RC522_SPI_transfer(((addr << 1) & 0x7E) | 0x80);
	val = RC522_SPI_transfer(0x00);

	/* CS HIGH */
	HAL_Delay(1); // Delay_us(10);
	NSS_Up();
	HAL_Delay(1); // Delay_us(10);
	return val;
}

static uint8_t RC522_SPI_transfer(uint8_t data)
{
	uint8_t rx_data;
	HAL_StatusTypeDef result = HAL_SPI_TransmitReceive(&hspi1, &data, &rx_data, 1, 100);
	return rx_data;
}
