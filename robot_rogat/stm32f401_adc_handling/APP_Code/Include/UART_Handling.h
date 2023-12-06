#ifndef __UART_HANDLING_H__
#define __UART_HANDLING_H__

#include "stm32f4xx_hal.h"

#include "ADC_Handling.h"

//-----------------------------------------------------------------------------

#define PREAMBLE_MSB    0xAA
#define PREAMBLE_LSB    0xDE
#define DATA_SIZE       21



// Define a packed struct
#pragma pack(push, 1) // Set packing alignment to 1 byte
typedef struct
{
	uint8_t PreambleMSB;
	uint8_t PreambleLSB;
	uint8_t MsgLen;

	uint16_t Data[ADC_NUM_OF_CHANNELS];

	uint8_t Checksum;

} Message_t;
#pragma pack(pop) // Restore the original packing alignment

//-----------------------------------------------------------------------------

extern void UART_InitFrame();
extern void PrepareFrameAndSend();

#endif // __UART_HANDLING_H__
