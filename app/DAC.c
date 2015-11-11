#include "stm32f4xx_hal.h"

DAC_ChannelConfTypeDef sConfig;


void DAC_Init(DAC_HandleTypeDef* hdac)
{
	  hdac->Instance = DAC;											// DAC base address, only one DAC address

	  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
	  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
	  HAL_DAC_ConfigChannel(hdac, &sConfig, DAC1_CHANNEL_1 );
	  HAL_DAC_Start(hdac, DAC1_CHANNEL_1);
}
