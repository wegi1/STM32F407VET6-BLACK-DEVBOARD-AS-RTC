/*
 * ili9341.c
 *
 *  Created on: 10.09.2019
 *      Author: bolo
 */


// FSMC_NE1 FSMC_A18
//LCD_REG       (*(volatile u16*)((uint32_t)(0x60000000)))
//LCD_RAM       (*(volatile u16*)((uint32_t)(0x60080000)))

// |------------------------------|
// |   LCD_PIN  |   CPU_PIN       |
// |------------------------------|
// | 1-GND      | GND             |
// | 2-RST      | RST  [NRST]     |
// |------------------------------|
// |        data lines            |
// |------------------------------|
// | 3-DB15     | PD10 [FSMC_D15] |
// | 4-DB14     | PD9  [FSMC_D14] |
// | 5-DB13     | PD8  [FSMC_D13] |
// | 6-DB12     | PE15 [FSMC_D12] |
// | 7-DB11     | PE14 [FSMC_D11] |
// | 8-DB10     | PE13 [FSMC_D10] |
// | 9-DB09     | PE12 [FSMC_D9]  |
// | 10-DB08    | PE11 [FSMC_D8]  |
// | 11-DB07    | PE10 [FSMC_D7]  |
// | 12-DB06    | PE9  [FSMC_D6]  |
// | 13-DB05    | PE8  [FSMC_D5]  |
// | 14-DB04    | PE7  [FSMC_D4]  |
// | 15-DB03    | PD1  [FSMC_D3]  |
// | 16-DB02    | PD0  [FSMC_D2]  |
// | 17-DB01    | PD15 [FSMC_D1]  |
// | 18-DB00    | PD14 [FSMC_D0]  |
// |------------------------------|
// |     control signals          |
// |------------------------------|
// | 19-RD      | PD4  [FSMC_NOE] |
// | 20-WR      | PD5  [FSMC_NWE] |
// | 21-RS      | PD13 [FSMC_A18] |
// | 22-CS_M    | PD7  [FSMC_NE1] |
// |------------------------------|
// |    TFT CONTROL SIGNALS       |
// |------------------------------|
// | 23-SCK     | PB13            |
// | 24-SCS     | PB12            |
// | 25-SI      | PB15            |
// | 26-SO      | PB14            |
// | 27-INT     | PC5             |
// |------------------------------|
// |       LCD BACK LIGHT         |
// |------------------------------|
// | 28-BLK     | PB0 (OR 1?)     |
// |------------------------------|
// | 29-SET     | NC              |
// |------------------------------|
// |       POWER SUPPLY           |
// |------------------------------|
// | 30-GND     | GND             |
// | 31-3V3     | 3.3V            |
// | 32-GND     | GND             |
// |------------------------------|



//**************************************************************************************

// |------------------------------|
// |   LCD_PIN  |   CPU_PIN       |
// |------------------------------|
// |     PORT_D_ signals FSMC     |
// |------------------------------|
// | 8-DB2      | PD0  [FSMC_D2]  | - alternate function FSMC PORTD GPIO_AF_FSMC (0x0C)
// | 9-DB3      | PD1  [FSMC_D3]  | - GPIOD->AFR[0]
// |                              |
// | 4-RD       | PD4  [FSMC_NOE] |
// | 3-WR       | PD5  [FSMC_NWE] |
// |                              |
// | 1-CS       | PD7  [FSMC_NE1] | - GPIOD->AFR[0] = 0xC0CC00CC;
// |******************************|
// | 19-DB13    | PD8  [FSMC_D13] | - alternate function FSMC PORTD GPIO_AF_FSMC (0x0C)
// | 20-DB14    | PD9  [FSMC_D14] | - GPIOD->AFR[1]
// | 21-DB15    | PD10 [FSMC_D15] |
// |                              |
// | 2-RS       | PD13 [FSMC_A18] |
// | 6-DB0      | PD14 [FSMC_D0]  |
// | 7-DB1      | PD15 [FSMC_D1]  | - GPIOD->AFR[1] = 0xCCC00CCC;
// |------------------------------|
// |     PORT_E_ signals FSMC     | - alternate function FSMC PORTE GPIO_AF_FSMC (0x0C)
// |------------------------------| - GPIOE->AFR[0]
// |                              |
// | 10-DB4     | PE7  [FSMC_D4]  | - GPIOE->AFR[0] = 0xC0000000;
// |******************************|
// | 11-DB5     | PE8  [FSMC_D5]  | - alternate function FSMC PORTE GPIO_AF_FSMC (0x0C)
// | 12-DB6     | PE9  [FSMC_D6]  | - GPIOE->AFR[1]
// | 13-DB7     | PE10 [FSMC_D7]  |
// | 14-DB8     | PE11 [FSMC_D8]  |
// | 15-DB9     | PE12 [FSMC_D9]  |
// | 16-DB10    | PE13 [FSMC_D10] |
// | 17-DB11    | PE14 [FSMC_D11] |
// | 18-DB12    | PE15 [FSMC_D12] | - GPIOE->AFR[1] = 0xCCCCCCCC;
// |------------------------------|
// |                              |
// |      OTHER   signals         |
// |                              |
// |------------------------------|
// | 5-RST      | PC5             |
// |------------------------------|
// |       LCD BACK LIGHT         |
// |------------------------------|
// | 23-LIG     | PB0             |
// |------------------------------|
//
//
//GPIOD->AFR[0] = 0xC0CC00CC;
//GPIOD->AFR[1] = 0xCCC00CCC;
//GPIOE->AFR[0] = 0xC0000000;
//GPIOE->AFR[1] = 0xCCCCCCCC;
//
//
// |------------------------------|
// |    TFT CONTROL SIGNALS       |
// |------------------------------|
// | 29-MISO    | PE0             |
// | 30-MOSI    | PE1             |
// | 31-TINT    | PE2             |
// |------------------------------|
// | 32-NC      | PC3             |
// |------------------------------|
// | 33-TCS     | PB2             |
// | 34-SCK     | PB1             |
// |------------------------------|

#include "ili9341.h"
#include "stdlib.h"
//#include "demos.h"
//#include "GUI.h"
#include "main.h"
/*---------------------------------------------------------------------*/

//lcd display


//lcd bottom funtions
//extern uint8_t LCD_WORK_ORIENTATION ;
//extern uint8_t LCD_NOT_WORK_ORIENTATION ;
//extern uint8_t LCD_PORTRAIT_WORK_ORIENTATION ;
//extern uint8_t LCD_PORTRAIT_NOT_WORK_ORIENTATION ;

uint8_t LCD_WORK_ORIENTATION =  LCD_ORIENTATION_LANDSCAPE;
uint8_t LCD_NOT_WORK_ORIENTATION ;
uint8_t LCD_PORTRAIT_WORK_ORIENTATION ;
uint8_t LCD_PORTRAIT_NOT_WORK_ORIENTATION ;
/*---------------------------------------------------------------------*/
static const uint8_t init_tab[] = {
        ILI_PCB, 3, 0x00, 0xC1, 0X30, /* 0xCF */ \
        ILI_POSC, 4, 0x64, 0x03, 0X12, 0X81, /* 0xED */  \
        ILI_DTCA_ic, 3, 0x85, 0x10, 0x7A, /* 0xE8 */ \
        ILI_PCA, 5, 0x39, 0x2C, 0x00, 0x34, 0x02, /* 0xCB */ \
        ILI_PRC,1, 0x20, /* 0xF7 */ \
        ILI_DTCB, 2, 0x00, 0x00, /* 0xEA */ \
        ILI_PWCTRL1, 1, 0x1B, /* 0xC0 */ \
        ILI_PWCTRL2,1, 0x01, /* 0xC1 */ \
        ILI_VMCTRL1, 2, 0x30, 0x30, /* 0xC5 */ \
        ILI_VMCTRL2, 1, 0XB7, /* 0xC7 */ \
        ILI_MADCTL, 1, 0x48, /* 0x36 */ \
        ILI_PIXSET, 1, 0x55, /* 0x3A */ \
//      ILI_FRMCTR1, 2, 0x00, 0x1A,  0xB1
        ILI_DISCTRL, 2, 0x0A, 0xA2, /* 0xB6 */  \
        ILI_E3G, 1, 0x00, /* 0xF2 */ \
        ILI_GAMSET, 1, 0x01, /* 0x26 */ \
        ILI_PGAMCTRL, 15, 0x0F, 0x2A, 0x28, 0x08, 0x0E, 0x08, 0x54, 0XA9, 0x43, 0x0A, 0x0F, 0x00, 0x00, 0x00, 0x00,  \
        ILI_NGAMCTRL, 15, 0x00, 0x15, 0x17, 0x07, 0x11, 0x06, 0x2B, 0x56, 0x3C, 0x05, 0x10, 0x0F, 0x3F, 0x3F, 0x0F,  \
        ILI_PASET, 4, 0x00, 0x00, 0x01, 0x3f, /* 0x2B */ \
        ILI_CASET, 4, 0x00, 0x00, 0x00, 0xef, /* 0x2A */ \
        ILI_SLPOUT, 120, /* 0x11 */ \
        ILI_DISPON, 0, /* 0x29 */ \
		ILI_FRMCTR1, 2, 0x01, 0x12, /* 0xB1 */ \
        0x00
};
//-------------------------------------------------------------------------------------------
volatile uint32_t OS_TimeMS = 0; //!!! delete and declare as extern ! when use STemWIN !
//-------------------------------------------------------------------------------------------

lcdPropertiesTypeDef  lcdProperties = { ILI9341_PIXEL_WIDTH, ILI9341_PIXEL_HEIGHT, LCD_ORIENTATION_PORTRAIT,true, true, 0x08 };


//******************************************************************************************************



/********************************************************************************************************
 *  Function: LCD_OpenWin
 *  Object: lcd open window for display
 *  Input: x0,y0, x1, y1
 *  Output: none
 *  brief: none
 ********************************************************************************************************/
void LCD_OpenWin(u16 x0, u16 y0, u16 x1, u16 y1)
{

    LCD_REG = (ILI_CASET);
    LCD_RAM = (x0 >> 8);
    LCD_RAM = (x0);
    LCD_RAM = (x1 >> 8);
    LCD_RAM = (x1);
    LCD_REG = (ILI_PASET);
    LCD_RAM = (y0 >> 8);
    LCD_RAM = (y0);
    LCD_RAM = (y1 >> 8);
    LCD_RAM = (y1);
    LCD_REG = (ILI_RAMWR);
}



/********************************************************************************************************
 *  Function: LCD_Put_Pixel
 *  Object: Display a point at screen
 *  Input: site and color
 *  Output: none
 *  brief: none
 ********************************************************************************************************/
void LCD_Put_Pixel(u16 x0, u16 y0, u16 Color)
{

//    LCD_REG = (ILI_CASET);
//    LCD_RAM = (y0>>8);
//    LCD_RAM = (y0);
//    LCD_RAM = (y0>>8);
//    LCD_RAM = (y0);
//    LCD_REG = (ILI_PASET);
//    LCD_RAM = (x0>>8);
//    LCD_RAM = (x0);
//    LCD_RAM = (x0>>8);
//    LCD_RAM = (x0);
//    LCD_REG = (ILI_RAMWR);

    LCD_REG = (ILI_CASET);
    LCD_RAM = (x0 >> 8);
    LCD_RAM = (x0);
    LCD_RAM = (x0 >> 8);
    LCD_RAM = (x0);
    LCD_REG = (ILI_PASET);
    LCD_RAM = (y0 >> 8);
    LCD_RAM = (y0);
    LCD_RAM = (y0 >> 8);
    LCD_RAM = (y0);
    LCD_REG = (ILI_RAMWR);
    LCD_RAM = (Color);
}

/********************************************************************************************************
 *  Function: LCD_ClrScr
 *  Object: lcd clear screen
 *  Input: backcolor
 *  Output: none
 *  brief: none
 ********************************************************************************************************/
void LCD_ClrScr(u16 BackColor)
{
    uint32_t i;
    LCD_OpenWin(0, 0, lcdProperties.width, lcdProperties.height);
    for(i = 0; i < (SCREEN_PIXELS); i++)
        LCD_RAM = BackColor ;
}
//*******************************************************************************************************
//*******************************************************************************************************
//*******************************************************************************************************

void LCD_WriteReg(u16 data)
{
    LCD_REG = data;
}
void LCD_WriteData(u16 data)
{
    LCD_RAM = data;
}
/*****************************************************************************************************
 *
 * initialize sequence ILI_9341
 *
 ***************************************************************************************************/
void LCD_ILI9341_init(void)
{
	// TODO reset sequence with register 0x01
#define setPWM(x) __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, x);
extern TIM_HandleTypeDef htim3;

    u32 i = 0;
    u32 i2;
    u8 counter;

    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4); // enable PWM back light
    setPWM(0);

    //--------------
    while(init_tab[i] != 0)
    {
        LCD_REG = (u16) init_tab[i] ;
        counter = init_tab[i+1];
        i += 2;
        if(counter > 15)
        {
            HAL_Delay(counter);
        }
        else if(counter > 0)
        {
            for(i2 = 0; i2 < counter ; i2++)
            {
                LCD_RAM = init_tab[i+i2] ;
            }
            i += counter;
        }
    }

    lcdSetOrientation(LCD_ORIENTATION_PORTRAIT);
    LCD_ClrScr(0);  // init screen as black


    setPWM(26000);

}
static const uint8_t tab_orientations[] = {0x68, 0xe8, 0xa8, 0x28, 0x08, 0x48, 0x88, 0xc8};
//----------------------
void lcdSetOrientation(lcdOrientationTypeDef value)
{
//	uint8_t data;
//	data = (uint8_t) value;
	uint16_t x,y;
	if(value > 7) { value = 0;}

	lcdProperties.orientation = value;
	lcdProperties.MADCTL_VAL = tab_orientations[value];

	y = ILI9341_PIXEL_WIDTH ;
	x = ILI9341_PIXEL_HEIGHT ;

	if(value < 4) {
		x = y;
		y = ILI9341_PIXEL_HEIGHT ;
	}

	lcdProperties.width = x;
	lcdProperties.height = y;
	LCD_WriteReg(ILI9341_MEMCONTROL);
	LCD_WriteData(lcdProperties.MADCTL_VAL);





	LCD_WORK_ORIENTATION = lcdProperties.orientation;
	if((LCD_WORK_ORIENTATION &3) != 0) {
		LCD_WORK_ORIENTATION = LCD_ORIENTATION_LANDSCAPE_ROTATE;
		LCD_NOT_WORK_ORIENTATION = LCD_ORIENTATION_LANDSCAPE ;
		LCD_PORTRAIT_WORK_ORIENTATION = LCD_ORIENTATION_PORTRAIT_ROTATE_MIRROR;
		LCD_PORTRAIT_NOT_WORK_ORIENTATION = LCD_ORIENTATION_PORTRAIT;
	} else {
		LCD_WORK_ORIENTATION = LCD_ORIENTATION_LANDSCAPE;
		LCD_NOT_WORK_ORIENTATION = LCD_ORIENTATION_LANDSCAPE_ROTATE ;
		LCD_PORTRAIT_WORK_ORIENTATION = LCD_ORIENTATION_PORTRAIT;
		LCD_PORTRAIT_NOT_WORK_ORIENTATION = LCD_ORIENTATION_PORTRAIT_ROTATE_MIRROR;
	}

}
//**************************************************************************************************************

//**************************************************************************************************************
//**************************************************************************************************************


