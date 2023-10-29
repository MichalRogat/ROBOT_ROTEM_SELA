#ifndef __ADC_HANDLING_H__
#define __ADC_HANDLING_H__

#define ADC_NUM_OF_CHANNELS   10

#define AVERAGE_WIN           8.0
#define AVERAG_OLD            ((float)((AVERAGE_WIN - 1) / AVERAGE_WIN))
#define AVERAG_NEW            ((float)(1 / AVERAGE_WIN))

typedef enum
{
	ADC_CH_0 = 0,
	ADC_CH_1 = 1,  // Motor_1_current
	ADC_CH_2 = 2,  // Motor_2_current
	ADC_CH_3 = 3,  // Elevation_Motor_1
	ADC_CH_4 = 4,  // Turn_Motor_1
	ADC_CH_5 = 5,  // Turn_Motor_2
	ADC_CH_6 = 6,  // Joint_Motor_1
	ADC_CH_7 = 7,  // FullTank_1
	ADC_CH_8 = 8,  // FullTank_2
	ADC_CH_9 = 9   // FullTank_3

} ADC_CH_t;

//-----------------------------------------------------------------------------

extern void ADC_Init();
extern HAL_StatusTypeDef AdcDmaStartConvert();
extern uint16_t ADC_Read(ADC_CH_t Channel);
extern uint8_t GetConevrsionInProgress();


#endif // __ADC_HANDLING_H__
