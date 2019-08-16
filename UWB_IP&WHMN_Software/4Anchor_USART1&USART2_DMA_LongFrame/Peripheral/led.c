#include "LED.h"

void LED_init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;

	/*Configure GPIO pin Output Level */
  //HAL_GPIO_WritePin(LED_ALL_GPIO_PORT, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC0 PC1 PC10 PC11 PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(LED_ALL_GPIO_PORT, &GPIO_InitStruct);
}

void LED_on(LED_pin led)
{
	switch(led)
	{
		case LEDY_RANGING:
			LED_Y_Reset();
			break;
		case LEDG_SYNC:
			LED_G_Reset();
			break;
		case LEDB_MODE:
			LED_B_Reset();
			break;
		// add start -- by WangPengGuy
		case LEDR_USER1:
			LED_USER1_Reset();
		  break; 
		case LEDR_USER2:
			LED_USER2_Reset();
		  break;
    // add end -- by WangPengGuy		
		default:
			break;
	}
}

void LED_off(LED_pin led)
{
	switch(led)
	{
		case LEDY_RANGING:
			LED_Y_Set();
			break;
		case LEDG_SYNC:
			LED_G_Set();
			break;
		case LEDB_MODE:
			LED_B_Set();
			break;
		// add start -- by WangPengGuy
		case LEDR_USER1:
			LED_USER1_Set();
		  break; 
		case LEDR_USER2:
			LED_USER2_Set();
		  break;
		// add end -- by WangPengGuy	
		default:
			break;
	}
}

void LED_toggle(LED_pin led)
{
	switch(led)
	{
		case LEDY_RANGING:
			LED_Y_Toggle();
			break;
		case LEDG_SYNC:
			LED_G_Toggle();
			break;
		case LEDB_MODE:
			LED_B_Toggle();
			break;
		// add start -- by WangPengGuy
		case LEDR_USER1:
			LED_USER1_Toggle();
		  break; 
		case LEDR_USER2:
			LED_USER2_Toggle();
		  break;
		// add end -- by WangPengGuy	
		default:
			break;
	}
}
