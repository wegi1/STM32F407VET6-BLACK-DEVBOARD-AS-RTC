/*
 * TP_operations.h
 *
 *  Created on: Aug 27, 2025
 *      Author: BOLO
 */

#ifndef TP_OPERATIONS_H_
#define TP_OPERATIONS_H_

#include "inttypes.h"
#include "types_h.h"
#include "calibrate.h"

void draw_cross(uint16_t x, uint16_t y, uint16_t color);
void draw_bigPixel(uint16_t x, uint16_t y, uint16_t color);
//============================================= Okablowanie zasilania = ==== ============================================== ==//
// Moduł LCD mikrokontroler STM32
// VCC podłączone do DC5V/3.3V //zasilanie
// GND do GND // Masa zasilania
//================================================ ================================================== ==========================================================================//
// Moduł LCD mikrokontroler STM32
// T_IRQ podłączony do PC5   // sygnał przerwania dotyku ekranu dotykowego
// T_DO  = MISO        PB14  // sygnał odczytu magistrali SPI ekranu dotykowego
// T_DIN = MOSI        PB15  // ekran dotykowy sygnał zapisu magistrali SPI
// T_CS podłączony do  PB12  // sygnał sterujący wyborem układu ekranu dotykowego
// T_CLK podłączony do PB13  // sygnał zegara magistrali SPI ekranu dotykowego
// ************************************************** ***********************************************



#define BITBAND(addr, bitnum) ((addr & 0xF0000000)+0x2000000+((addr &0xFFFFF)<<5)+(bitnum<<2))
#define MEM_ADDR(addr)  *((volatile unsigned long  *)(addr))
#define BIT_ADDR(addr, bitnum)   MEM_ADDR(BITBAND(addr, bitnum))

#define GPIOA_ODR_Addr    (GPIOA_BASE+20)
#define GPIOB_ODR_Addr    (GPIOB_BASE+20)
#define GPIOC_ODR_Addr    (GPIOC_BASE+20)
#define GPIOD_ODR_Addr    (GPIOD_BASE+20)
#define GPIOE_ODR_Addr    (GPIOE_BASE+20)
#define GPIOF_ODR_Addr    (GPIOF_BASE+20)
#define GPIOG_ODR_Addr    (GPIOG_BASE+20)

#define GPIOA_IDR_Addr    (GPIOA_BASE+16)
#define GPIOB_IDR_Addr    (GPIOB_BASE+16)
#define GPIOC_IDR_Addr    (GPIOC_BASE+16)
#define GPIOD_IDR_Addr    (GPIOD_BASE+16)
#define GPIOE_IDR_Addr    (GPIOE_BASE+16)
#define GPIOF_IDR_Addr    (GPIOF_BASE+16)
#define GPIOG_IDR_Addr    (GPIOG_BASE+16)


#define PAout(n)   BIT_ADDR(GPIOA_ODR_Addr,n)
#define PAin(n)    BIT_ADDR(GPIOA_IDR_Addr,n)

#define PBout(n)   BIT_ADDR(GPIOB_ODR_Addr,n)
#define PBin(n)    BIT_ADDR(GPIOB_IDR_Addr,n)

#define PCout(n)   BIT_ADDR(GPIOC_ODR_Addr,n)
#define PCin(n)    BIT_ADDR(GPIOC_IDR_Addr,n)

#define PDout(n)   BIT_ADDR(GPIOD_ODR_Addr,n)
#define PDin(n)    BIT_ADDR(GPIOD_IDR_Addr,n)

#define PEout(n)   BIT_ADDR(GPIOE_ODR_Addr,n)
#define PEin(n)    BIT_ADDR(GPIOE_IDR_Addr,n)

#define PFout(n)   BIT_ADDR(GPIOF_ODR_Addr,n)
#define PFin(n)    BIT_ADDR(GPIOF_IDR_Addr,n)

#define PGout(n)   BIT_ADDR(GPIOG_ODR_Addr,n)
#define PGin(n)    BIT_ADDR(GPIOG_IDR_Addr,n)




// Połącz piny z układem ekranu dotykowego
#define PEN  PCin(5)      //PC5   INT (PEN)
#define DOUT PBin(14)     //PB14  MISO
#define TDIN PBout(15)    //PB15  MOSI
#define TCLK PBout(13)    //PB13  SCLK
#define TCS  PBout(12)    //PB12  CS



void lets_calibrate_ts(uint8_t orientation);
void delay_us(u16 delay);

#endif /* TP_OPERATIONS_H_ */
