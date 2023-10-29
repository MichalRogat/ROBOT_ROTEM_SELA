
#include "main.h"
#include "adc.h"
#include "string.h"

#include "ADC_Handling.h"


//-----------------------------------------------------------------------------

// this is needed as it is defined in main.c by the CubeIDE...
extern DMA_HandleTypeDef hdma_adc1;

//-----------------------------------------------------------------------------

// initialize all cell's to ZERO
static volatile uint16_t adc1RowValuesFromDma[ADC_NUM_OF_CHANNELS] __attribute__((aligned(4)));

static uint16_t adc1ReadRowValues[ADC_NUM_OF_CHANNELS] = {0};

static uint8_t ConevrsionInProgress;

//-----------------------------------------------------------------------------

static void DmaConversionComplete(ADC_HandleTypeDef *hadc);


//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------

void ADC_Init()
{
	HAL_StatusTypeDef stat;


	// set all array's cells to ZERO
	memset((void*)adc1RowValuesFromDma, 0, sizeof(adc1RowValuesFromDma));
	memset((void*)adc1ReadRowValues, 0, sizeof(adc1ReadRowValues));


	// register the DMA conversion complete callback
	stat = HAL_ADC_RegisterCallback(&hadc1, HAL_ADC_CONVERSION_COMPLETE_CB_ID, DmaConversionComplete);
	if (stat != HAL_OK)  // &hdma_adc1
	{
//		RetVal = STATUS_FAIL;
//		break;
	}

	ConevrsionInProgress = 0;
}

//-----------------------------------------------------------------------------

HAL_StatusTypeDef AdcDmaStartConvert()
{
	HAL_StatusTypeDef RetVal;

	//	HAL_GPIO_WritePin(BOARD_MCU_LED_1_PORT, BOARD_MCU_LED_1_PIN, GPIO_PIN_SET);

	ConevrsionInProgress = 1;

	RetVal = HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&adc1RowValuesFromDma[0], ADC_NUM_OF_CHANNELS);

	return RetVal;
}

//-----------------------------------------------------------------------------// The DMA transfer complete is actually registered with the ADC !!
//
// Conversion takes about ?? uSec
//
static void DmaConversionComplete(ADC_HandleTypeDef *hadc)
{
	float OldVal, NewVal;
	//
//	HAL_GPIO_WritePin(BOARD_MCU_LED_1_PORT, BOARD_MCU_LED_1_PIN, GPIO_PIN_RESET);

	// we scan 10 channels at 12.5Mhz and set to 112 cycles per conversion.
	// this mean that the conversion takes about ((12 + 122) * 10) * (1 / 12500000))
	for(int i=0; i<ADC_NUM_OF_CHANNELS; i++)
	{
		OldVal = (adc1ReadRowValues[i] * AVERAG_OLD);

		NewVal = (adc1RowValuesFromDma[i] * AVERAG_NEW);

		// as the average is in float we added 0.5 to round the total to a round decimal value
		adc1ReadRowValues[i] = (uint16_t) (OldVal + NewVal + 0.5);
	}

	ConevrsionInProgress = 0;
}

//-----------------------------------------------------------------------------

uint16_t ADC_Read(ADC_CH_t Channel)
{
	return adc1ReadRowValues[Channel];
}

//-----------------------------------------------------------------------------

uint8_t GetConevrsionInProgress()
{
	return ConevrsionInProgress;
}
