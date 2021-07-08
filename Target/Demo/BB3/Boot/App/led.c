/************************************************************************************//**
* \file         Demo/ARMCM7_STM32F7_Nucleo_F746ZG_CubeIDE/Boot/App/led.c
* \brief        LED driver source file.
* \ingroup      Boot_ARMCM7_STM32F7_Nucleo_F746ZG_CubeIDE
* \internal
*----------------------------------------------------------------------------------------
*                          C O P Y R I G H T
*----------------------------------------------------------------------------------------
*   Copyright (c) 2020  by Feaser    http://www.feaser.com    All rights reserved
*
*----------------------------------------------------------------------------------------
*                            L I C E N S E
*----------------------------------------------------------------------------------------
* This file is part of OpenBLT. OpenBLT is free software: you can redistribute it and/or
* modify it under the terms of the GNU General Public License as published by the Free
* Software Foundation, either version 3 of the License, or (at your option) any later
* version.
*
* OpenBLT is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
* without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
* PURPOSE. See the GNU General Public License for more details.
*
* You have received a copy of the GNU General Public License along with OpenBLT. It
* should be located in ".\Doc\license.html". If not, contact Feaser to obtain a copy.
*
* \endinternal
****************************************************************************************/

/****************************************************************************************
* Include files
****************************************************************************************/
#include "boot.h"                                /* bootloader generic header          */
#include "led.h"                                 /* module header                      */
#include <stdio.h>
#include "stm32f7xx.h"                           /* STM32 CPU and HAL header           */
#include "stm32f7xx_ll_gpio.h"                   /* STM32 LL GPIO header               */
#include "file.h"
#include "main.h"
#include <string.h>

/****************************************************************************************
* Local data declarations
****************************************************************************************/
/** \brief Holds the desired LED blink interval time. */
static blt_int16u ledBlinkIntervalMs;
uint16_t percent = 0;
static double value = 0;
volatile uint8_t erase = 0;
volatile uint8_t programming = 0;
volatile uint8_t errorJump = 0;
volatile uint32_t currentAddress;
volatile uint32_t fwSize;
volatile uint32_t startAddress;
volatile uint8_t missingSD;

/************************************************************************************//**
** \brief     Initializes the LED blink driver.
** \param     interval_ms Specifies the desired LED blink interval time in milliseconds.
** \return    none.
**
****************************************************************************************/
void LedBlinkInit(blt_int16u interval_ms)
{
  /* store the interval time between LED toggles */
  ledBlinkIntervalMs = interval_ms;
} /*** end of LedBlinkInit ***/


/************************************************************************************//**
** \brief     Task function for blinking the LED as a fixed timer interval.
** \return    none.
**
****************************************************************************************/

void LedBlinkTask(void)
{
  static blt_bool ledOn = BLT_FALSE;
  static blt_int32u nextBlinkEvent = 0;

	  /* check for blink event */
	  if (TimerGet() >= nextBlinkEvent)
	  {
	  if(erase || programming){
		static char displayData[20];
		ledBlinkIntervalMs = 350;
		LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_15);
		LL_GPIO_SetOutputPin(GPIOD, LL_GPIO_PIN_12);
		if(erase){
			BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
			BSP_LCD_DisplayStringAt(0, 120, (uint8_t*)"Erasing old firmware...", CENTER_MODE);
			BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
			BSP_LCD_FillRect(42,148,percent,17);
		}
		else{
			value = currentAddress-134250496;
			value = (value/fwSize)*200;
			percent+= (long)value;
			if (percent>399){percent=400;}
			FileLibLongToIntString(percent/2-100,displayData);
			strcat(displayData,"%");
			BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
			BSP_LCD_DisplayStringAt(0, 120, (uint8_t*)"                            ", CENTER_MODE);
			BSP_LCD_DisplayStringAt(0, 120, (uint8_t*)"Flashing: ", CENTER_MODE);
			BSP_LCD_DisplayStringAt(280, 120, (uint8_t*)displayData, LEFT_MODE);
			BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
			BSP_LCD_FillRect(42,148,percent,17);
		}
		if(percent < 200){
			if(erase){
				percent++;
			}
		}
	  }
	  if(errorJump){
			LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_15);
			LL_GPIO_SetOutputPin(GPIOD, LL_GPIO_PIN_12);
		  BSP_LCD_SetTextColor(0);
		  BSP_LCD_FillRect(0,0,480,282);
		  BSP_LCD_Clear(0);
		  BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
		  BSP_LCD_DisplayStringAt(0, 100, (uint8_t*)"Update failed!", CENTER_MODE);
		  BSP_LCD_DisplayStringAt(0, 130, (uint8_t*)"Please restart and try again,", CENTER_MODE);
		  BSP_LCD_DisplayStringAt(0, 160, (uint8_t*)"or use USB DFU bootloader.", CENTER_MODE);
		  HAL_Delay(3000);
		  while(1){HAL_Delay(3000);CpuStartUserProgram();}
	  }
	  if(missingSD){
			LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_15);
			LL_GPIO_SetOutputPin(GPIOD, LL_GPIO_PIN_12);
		  BSP_LCD_SetTextColor(0);
		  BSP_LCD_FillRect(0,0,480,282);
		  BSP_LCD_Clear(0);
		  BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
		  BSP_LCD_DisplayStringAt(0, 100, (uint8_t*)"SD error!", CENTER_MODE);
		  BSP_LCD_DisplayStringAt(0, 130, (uint8_t*)"Please, turn off and check SD", CENTER_MODE);
		  HAL_Delay(3000);
		  while(1){HAL_Delay(3000);CpuStartUserProgram();}
	  }
		/* toggle the LED state */
		if (ledOn == BLT_FALSE)
		{
		  ledOn = BLT_TRUE;
		  //LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_15);
		}
		else
		{
		  ledOn = BLT_FALSE;
		  //LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_15);
		  //LL_GPIO_ResetOutputPin(GPIOD, LL_GPIO_PIN_12);
		}
		/* schedule the next blink event */
		nextBlinkEvent = TimerGet() + ledBlinkIntervalMs;
	  }
} /*** end of LedBlinkTask ***/


/************************************************************************************//**
** \brief     Cleans up the LED blink driver. This is intended to be used upon program
**            exit.
** \return    none.
**
****************************************************************************************/
void LedBlinkExit(void)
{
  /* turn the LED off */
  LL_GPIO_ResetOutputPin(GPIOD, LL_GPIO_PIN_12);
  LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_15);
} /*** end of LedBlinkExit ***/


/*********************************** end of led.c **************************************/
