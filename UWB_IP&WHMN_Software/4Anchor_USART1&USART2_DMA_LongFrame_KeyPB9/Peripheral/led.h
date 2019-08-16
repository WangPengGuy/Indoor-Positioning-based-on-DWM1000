#ifndef __LED_H
#define __LED_H

#include "stm32f4xx.h"
/*====================================================================================================*/

typedef enum{
	LEDY_RANGING = GPIO_PIN_10,
	LEDG_SYNC = GPIO_PIN_11,
	LEDB_MODE = GPIO_PIN_12,
	LEDR_USER1 = GPIO_PIN_0,
	LEDR_USER2 = GPIO_PIN_1,
}LED_pin;

#define ledRanging LEDY_RANGING
#define ledSync	LEDG_SYNC
#define ledMode	LEDB_MODE
#define ledUser1 LEDR_USER1
#define ledUser2 LEDR_USER2

/*====================================================================================================*/
//LED1,LEDY_RANGING
#define LED_Y_PIN                 GPIO_PIN_10
#define LED_Y_GPIO_PORT           GPIOC
#define LED_Y_Set()               HAL_GPIO_WritePin(LED_Y_GPIO_PORT, LED_Y_PIN, GPIO_PIN_SET);
#define LED_Y_Reset()             HAL_GPIO_WritePin(LED_Y_GPIO_PORT, LED_Y_PIN, GPIO_PIN_RESET);
#define LED_Y_Toggle()            HAL_GPIO_TogglePin(LED_Y_GPIO_PORT, LED_Y_PIN)

//LED2,LEDG_SYNC
#define LED_G_PIN                 GPIO_PIN_11
#define LED_G_GPIO_PORT           GPIOC
#define LED_G_Set()               HAL_GPIO_WritePin(LED_G_GPIO_PORT, LED_G_PIN, GPIO_PIN_SET);
#define LED_G_Reset()             HAL_GPIO_WritePin(LED_G_GPIO_PORT, LED_G_PIN, GPIO_PIN_RESET);
#define LED_G_Toggle()            HAL_GPIO_TogglePin(LED_G_GPIO_PORT, LED_G_PIN)

//LED3,LEDB_MODE
#define LED_B_PIN                 GPIO_PIN_12
#define LED_B_GPIO_PORT           GPIOC
#define LED_B_Set()               HAL_GPIO_WritePin(LED_B_GPIO_PORT, LED_B_PIN, GPIO_PIN_SET);
#define LED_B_Reset()             HAL_GPIO_WritePin(LED_B_GPIO_PORT, LED_B_PIN, GPIO_PIN_RESET);
#define LED_B_Toggle()            HAL_GPIO_TogglePin(LED_B_GPIO_PORT, LED_B_PIN)

//LED4,LEDR_USER1
#define LED_USER1_PIN                 GPIO_PIN_0
#define LED_USER1_GPIO_PORT           GPIOC
#define LED_USER1_Set()               HAL_GPIO_WritePin(LED_USER1_GPIO_PORT, LED_USER1_PIN, GPIO_PIN_SET);
#define LED_USER1_Reset()             HAL_GPIO_WritePin(LED_USER1_GPIO_PORT, LED_USER1_PIN, GPIO_PIN_RESET);
#define LED_USER1_Toggle()            HAL_GPIO_TogglePin(LED_USER1_GPIO_PORT, LED_USER1_PIN)

//LED5,LEDR_USER2
#define LED_USER2_PIN                 GPIO_PIN_1
#define LED_USER2_GPIO_PORT           GPIOC
#define LED_USER2_Set()               HAL_GPIO_WritePin(LED_USER2_GPIO_PORT, LED_USER2_PIN, GPIO_PIN_SET);
#define LED_USER2_Reset()             HAL_GPIO_WritePin(LED_USER2_GPIO_PORT, LED_USER2_PIN, GPIO_PIN_RESET);
#define LED_USER2_Toggle()            HAL_GPIO_TogglePin(LED_USER2_GPIO_PORT, LED_USER2_PIN)

#define LED_ALL_GPIO_PORT         GPIOC

//#define KEY_PIN                   GPIO_Pin_8
//#define KEY_GPIO_PORT             GPIOA
//#define KEY_Read()                (__GPIO_READ(KEY_GPIO_PORT, KEY_PIN) == KEY_PIN)

#define ledOn LED_on
#define ledOff LED_off
#define ledToggle LED_toggle
/*====================================================================================================*/
/*====================================================================================================*/
void LED_init(void);
void LED_on(LED_pin led);
void LED_off(LED_pin led);
void LED_toggle(LED_pin led);

/*====================================================================================================*/
/*====================================================================================================*/
#endif
