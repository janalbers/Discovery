#include <stdlib.h>
#include "stm32f4xx_hal.h"
#include "stm32f407xx.h"
static uint32_t mainlooptime;

uint32_t Gen_Getms5Tick(void)
{
	return HAL_GetTick()/5;
}

uint32_t Gen_Getms50Tick(void)
{
	return HAL_GetTick()/50;
}

uint32_t Gen_Getms500Tick(void)
{
	return HAL_GetTick()/500;
}


/*----------------------------------------------------------------------------
  delays number of tick Systicks (happens every 1 ms)
 *----------------------------------------------------------------------------*/
void Gen_Delay (uint32_t dlyTicks)
{
    uint32_t curTicks;

#ifndef __NO_SYSTICK
    curTicks = HAL_GetTick();
    while ((HAL_GetTick() - curTicks) < dlyTicks);
#else
    for (curTicks = 0; curTicks < (dlyTicks * 0x1000); curTicks++) __NOP();
#endif
}




void Gen_SetMainTimer(uint32_t timerticks)
{
	mainlooptime = HAL_GetTick() + timerticks;
}

uint32_t Gen_GetMainTimer()
{
	if (HAL_GetTick() < mainlooptime ) return 0; else return 1;
}

/**
* @brief This function configures the Software ExtI Hanlder.
*/
void Gen_SwExtIConfig(void)
{
#define PWR_EXTI_LINE_PVD  ((uint32_t)EXTI_IMR_MR16)  /*!< External interrupt line 16 Connected to the PVD EXTI Line */

	EXTI->IMR |= EXTI_IMR_MR22; //Interrupt 22 not masked any more
	EXTI->EMR |= EXTI_IMR_MR22; //Event 22 not masked any more


}
void Gen_SwExtI23_Handler(void)
{
}

void SwExtI22_Handler(void)
{
}
