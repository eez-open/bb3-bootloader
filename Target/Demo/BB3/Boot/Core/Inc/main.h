/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f7xx_hal.h"

#include "stm32f7xx_ll_rcc.h"
#include "stm32f7xx_ll_bus.h"
#include "stm32f7xx_ll_system.h"
#include "stm32f7xx_ll_exti.h"
#include "stm32f7xx_ll_cortex.h"
#include "stm32f7xx_ll_utils.h"
#include "stm32f7xx_ll_pwr.h"
#include "stm32f7xx_ll_dma.h"
#include "stm32f7xx_ll_gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
extern volatile uint8_t erase;
extern volatile uint8_t programming;

typedef struct
{
  uint32_t TextColor;
  uint32_t BackColor;
  //sFONT    *pFont;
}LCD_DrawPropTypeDef;

typedef struct
{
  int16_t X;
  int16_t Y;
}Point, * pPoint;

/**
  * @brief  LCD status structure definition
  */
#define LCD_OK                 ((uint8_t)0x00)
#define LCD_ERROR              ((uint8_t)0x01)
#define LCD_TIMEOUT            ((uint8_t)0x02)

/**
  * @brief  LCD FB_StartAddress
  */
//#define LCD_FB_START_ADDRESS       ((uint32_t)&LCD_FB_START)

/**
  * @brief  LCD color
  */
#define LCD_COLOR_BLUE          ((uint32_t)0xFF0000FF)
#define LCD_COLOR_GREEN         ((uint32_t)0xFF00FF00)
#define LCD_COLOR_RED           ((uint32_t)0xFFFF0000)

//static LTDC_HandleTypeDef  hLtdcHandler;
//static DMA2D_HandleTypeDef hDma2dHandler;
/* Default LCD configuration with LCD Layer 1 */


#define LCD_LayerCfgTypeDef    LTDC_LayerCfgTypeDef
#define MAX_LAYER_NUMBER       ((uint32_t)2)
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void BSP_LCD_Clear(uint32_t Color);
void BSP_LCD_FillRect(uint16_t Xpos, uint16_t Ypos, uint16_t Width, uint16_t Height);
void BSP_LCD_DrawHLine(uint16_t Xpos, uint16_t Ypos, uint16_t Length);
void BSP_LCD_DrawVLine(uint16_t Xpos, uint16_t Ypos, uint16_t Length);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define MCO_Pin LL_GPIO_PIN_0
#define MCO_GPIO_Port GPIOH
#define USB_DM_Pin LL_GPIO_PIN_11
#define USB_DM_GPIO_Port GPIOA
#define USB_DP_Pin LL_GPIO_PIN_12
#define USB_DP_GPIO_Port GPIOA
void   MX_USB_OTG_FS_PCD_Init(void);
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
