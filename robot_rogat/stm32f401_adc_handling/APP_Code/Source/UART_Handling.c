
#include "usart.h"
#include "UART_Handling.h"

//-----------------------------------------------------------------------------

static Message_t Message;

//-----------------------------------------------------------------------------

static void CalcChecsum();

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------

void UART_InitFrame()
{
	Message.PreambleMSB = PREAMBLE_MSB;
	Message.PreambleLSB = PREAMBLE_LSB;

	Message.MsgLen = DATA_SIZE;
}

//-----------------------------------------------------------------------------

void PrepareFrameAndSend()
{
	for(int i=0; i<ADC_NUM_OF_CHANNELS; i++)
	{
		Message.Data[i] = ADC_Read((ADC_CH_t) i);
	}

	// Calculate the checksum and add it to message
	CalcChecsum();

	// send message over UART
	HAL_UART_Transmit(&huart1, (const uint8_t*) &Message, sizeof(Message), 100);
}

//-----------------------------------------------------------------------------

static void CalcChecsum()
{
	int MsgLen = sizeof(Message);
	uint8_t *pMsg = (uint8_t*)&Message;
	uint8_t MsgChk = 0;

	for(int i=0; i<MsgLen; i++)
	{
		MsgChk += *pMsg;

		pMsg++;
	}

	MsgChk = ~MsgChk;

	Message.Checksum = MsgChk;
}

//-----------------------------------------------------------------------------
