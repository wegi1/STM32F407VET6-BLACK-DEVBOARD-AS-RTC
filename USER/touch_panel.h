/*********************************************************************************************************
*
* File                : touch_panel.h
* Hardware Environment: 
* Build Environment   : RealView MDK-ARM  Version: 4.20
* Version             : V1.0
* By                  : 
*
*                                  (c) Copyright 2005-2011, WaveShare
*                                       http://www.waveshare.net
*                                          All Rights Reserved
*
*********************************************************************************************************/

#ifndef _TOUCH_PANEL_H_
#define _TOUCH_PANEL_H_

/* Includes ------------------------------------------------------------------*/
#include "stm32f407xx.h"
#include "stm32f4xx.h"
#include "main.h"
/**
 * @brief Definition for TouchPanel
 */

#define TP_CS_PIN          LCDTP_CS_Pin
#define TP_CS_PORT         LCDTP_CS_GPIO_Port
//#define TP_CS_PIN          T_CS_Pin
//#define TP_CS_PORT         T_CS_GPIO_Port
#define TP_CS_CLK          __GPIOB_CLK_ENABLE();

#define TP_IRQ_PIN         LCDTP_IRQ_Pin
#define TP_IRQ_PORT        LCDTP_IRQ_GPIO_Port
#define TP_IRQ_CLK         __GPIOB_CLK_ENABLE();


#define TP_CS(x)  x ? HAL_GPIO_WritePin(TP_CS_PORT, TP_CS_PIN, GPIO_PIN_SET): HAL_GPIO_WritePin(TP_CS_PORT, TP_CS_PIN, GPIO_PIN_RESET)

#define TP_INT_IN   HAL_GPIO_ReadPin(TP_IRQ_PORT, TP_IRQ_PIN)


/* Private typedef -----------------------------------------------------------*/
typedef  struct Point
{
   uint16_t x;
   uint16_t y;
} POINT;


typedef struct Matrix 
{            
int32_t An,
            Bn,     
            Cn,   
            Dn,    
            En,    
            Fn,     
            Divider ;
} MATRIX ;

/* Private variables ---------------------------------------------------------*/
extern POINT ScreenSample[3];
extern POINT DisplaySample[3];
extern MATRIX matrix ;
extern POINT  display ;

/* Private define ------------------------------------------------------------*/

#define  CHX   0x90
#define  CHY   0xD0

/* Private function prototypes -----------------------------------------------*/        

POINT * Read_Ads7846(void);
void TouchPanel_Calibrate(void);
void DrawCross(uint16_t Xpos, uint16_t Ypos);

void paint_proc(void);
void live_show_ts(void);
FunctionalState setCalibrationMatrix( POINT * displayPtr, POINT * screenPtr, MATRIX * matrixPtr);
FunctionalState getDisplayPoint(POINT * displayPtr, POINT * screenPtr, MATRIX * matrixPtr );

#endif

/*********************************************************************************************************
      END FILE
*********************************************************************************************************/


