/**************************************************************************************
 * configs/stm3210e-ek/src/uucp_lcd.c
 * arch/arm/src/board/up_lcd.c
 *
 *   Copyright (C) 2010 Gregory Nutt. All rights reserved.
 *   Author: Akshay Mishra <akshay@dspworks.in>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 **************************************************************************************/

/**************************************************************************************
The Below is for UUCP. The CSTN is based on the uc1698 from ultrachip
 *
 *   LCD Module Pin Out:                         STM32 PIO:
 *  -------------------------------------------- --------------------------------------
 *   Pin Symbol Function                         Pin            Default 
 *  ---- ------ -------------------------------- -------------- --------
 *   1   K      Backlight ground                 N/A            ---     
 *   2   A      Backlight Supply                 N/A            ---     
 *   3   GND    Ground                           N/A            ---     
 *   4   VDD    Power (3.3V)                     N/A 		---	
 *   5   NC     N/A 
 *   6   NC     N/A
 *   7   /CS    Chip Select (Low)                PG12	        NE4     
 *   8   RES    RESET	                         PB7                    
 *   9   DI     Register Select (Comm/Data)      A1 		A0 (address line 0)                   
 *   10  WR     Write/Read			 PD5		NWE
 *   11  RD     Write/Read			 PD4		NOE
 *   12  DB7    Data bus                         PE10           D7      
 *   13  DB6    Data bus                         PE9            D6      
 *   14  DB5    Data bus                         PE8            D5      
 *   15  DB4    Data bus                         PE7            D4      
 *   16  DB3    Data bus                         PD1            D3      
 *   17  DB2    Data bus                         PD0            D2      
 *   18  DB1    Data bus                         PD15           D1      
 *   19  DB0    Data bus                         PD14           D0      
 *   20  GND    Ground                           N/A            ---     
 *
 * The LCD module gets its reset by the ST chip.
 *
 * The STM32 communicates with the LCD through 8-bit Data Bus 
 * Adrress to write to the LCD  - FSMC_NE4 needs to be enabled. 
 *
 *
 *
 **************************************************************************************/

/**************************************************************************************
 * Included Files
 **************************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/lcd/lcd.h>
#include <nuttx/rgbcolors.h>

#include <arch/irq.h>

#include "up_arch.h"
#include "uc1698.h"
#include "stm3210e-internal.h"
#include "stm32_internal.h"

/**************************************************************************************
 * Pre-processor Definitions
 **************************************************************************************/

/* Configuration **********************************************************************/

/* Define the following to enable register-level debug output */

#undef CONFIG_LCD_REGDEBUG
#define LCDREGBASE 0x6C000000
#define LCDDATBASE 0x6C000001

/* Verbose debug must also be enabled */

/* Graphics Capbilities ***************************************************************/

/* LCD resolution: 160 (columns) by 128 (rows).  The physical dimensions of the device
 * are really 240 (columns) by 320 (rows), but unless CONFIG_SAM3U_240x320 is defined,
 * we swap rows and columns in setcursor to make things behave nicer (there IS a
 * performance hit for this swap!).
 */

#  define UUCP_XRES         128
#  define UUCP_YRES         160

/* Color depth and format. BPP=16 R=6, G=6, B=5: RRRR RBBB BBBG GGGG */

#define UUCP_BPP          16
#define UUCP_RGBFMT       FB_FMT_RGB16_565


/**************************************************************************************
 * Private Function Protototypes
 **************************************************************************************/

/* Low-level UC1698 Register access */

static void uucp_putreg(uint16_t reg,  uint8_t data);
static uint8_t uucp_getreg(uint16_t reg);

/* Misc. LCD Helper Functions */

static void uucp_setcursor(fb_coord_t row, fb_coord_t col);
//static inline void uucp_wrsetup(void);
static inline void uucp_wrram(uint8_t color);
static inline uint8_t uucp_rdram(void);
static void uucp_lcdon(void);
static void uucp_lcdoff(void);

#ifdef CONFIG_DEBUG_GRAPHICS
static void uucp_dumpreg(uint8_t startreg, uint8_t endreg);
#else
#  define uucp_dumpreg(startreg,endreg)
#endif

/* LCD Data Transfer Methods */

static int uucp_putrun(fb_coord_t row, fb_coord_t col, FAR const uint8_t *buffer,
             size_t npixels);
static int uucp_getrun(fb_coord_t row, fb_coord_t col, FAR uint8_t *buffer,
             size_t npixels);

/* LCD Configuration */

static int uucp_getvideoinfo(FAR struct lcd_dev_s *dev,
             FAR struct fb_videoinfo_s *vinfo);
static int uucp_getplaneinfo(FAR struct lcd_dev_s *dev, unsigned int planeno,
             FAR struct lcd_planeinfo_s *pinfo);

/* LCD RGB Mapping */

#ifdef CONFIG_FB_CMAP
#  error "RGB color mapping not supported by this driver"
#endif

/* Cursor Controls */

#ifdef CONFIG_FB_HWCURSOR
#  error "Cursor control not supported by this driver"
#endif

/* LCD Specific Controls */

static int uucp_getpower(struct lcd_dev_s *dev);
static int uucp_setpower(struct lcd_dev_s *dev, int power);
static int uucp_getcontrast(struct lcd_dev_s *dev);
static int uucp_setcontrast(struct lcd_dev_s *dev, unsigned int contrast);

/**************************************************************************************
 * Private Data
 **************************************************************************************/

/* This is working memory allocated by the LCD driver for each LCD device
 * and for each color plane.  This memory will hold one raster line of data.
 * The size of the allocated run buffer must therefor be at least
 * (bpp * xres / 8).  Actual alignment of the buffer must conform to the
 * bitwidth of the underlying pixel type.
 *
 * If there are multiple planes, they may share the same working buffer
 * because different planes will not be operate on concurrently.  However,
 * if there are multiple LCD devices, they must each have unique run buffers.
 */

static uint16_t g_runbuffer[UUCP_XRES];


/**************************************************************************************
 * Private Functions
 **************************************************************************************/

/**************************************************************************************
 * Name:  uucp_putreg
 *
 * Description:
 *   Write to a UC1698U register
 *
 **************************************************************************************/

static void uucp_putreg(uint16_t reg,  uint8_t data)
{
 // regdbg("base: %08x RS: %04x data: %04x\n", LCDREGBASE, LCDDATBASE: data);
  putreg8(reg, LCDREGBASE);
  putreg8(data, LCDDATBASE);
}

/**************************************************************************************
 * Name:  uucp_getreg
 *
 * Description:
 *   Read from a HX834x register
 *
 **************************************************************************************/

static uint8_t uucp_getreg(uint16_t reg)
{
  uint8_t data;
  putreg8(reg, LCDREGBASE);
  data = getreg8(LCDREGBASE);

  return data;
}

/**************************************************************************************
 * Name:  uucp_setcursor
 *
 * Description:
 *   Set the LCD cursor position.
 *
 **************************************************************************************/

static void uucp_setcursor(fb_coord_t row, fb_coord_t col)
{
  uint8_t  x1;
  uint8_t  x2;
  uint8_t  y1;
  uint8_t  y2;

  /* Get the upper and lower x and y positions */

  x1  = (uint8_t)col;
  x2  = (uint8_t)((uint16_t)col >> 8);

  y1  = (uint8_t)row;
  y2  = (uint8_t)((uint16_t)row >> 8);

  /* Then set the cursor position */

         Uc1698_SetReg(UC1698_SetCAL+x1);
         Uc1698_SetReg(UC1698_SetCAM+x2);
         Uc1698_SetReg(UC1698_SetRAL+y1);
         Uc1698_SetReg(UC1698_SetRAM+y2);
}

#if 0

/**************************************************************************************
 * Name:  uucp_wrsetup
 *
 * Description:
 *   Set up for a GRAM write operation.
 *
 **************************************************************************************/

static inline void uucp_wrsetup(void)
{
  putreg16(HX8347_R22H, LCD_BASE);
}
#endif

/**************************************************************************************
 * Name: uucp_wrram
 *
 * Description:
 *   Write to the 16-bit GRAM register
 *
 **************************************************************************************/

static inline void uucp_wrram(uint8_t color)
{
  putreg8(color, LCDDATBASE);
}

/**************************************************************************************
 * Name: uucp_rdram
 *
 * Description:
 *   Read from the 16-bit GRAM register
 *
 **************************************************************************************/

static inline uint8_t uucp_rdram(void)
{
  return getreg8(LCDDATBASE);
}

/**************************************************************************************
 * Name:  uucp_lcdon
 *
 * Description:
 *   Turn the LCD on
 *
 **************************************************************************************/

static void uucp_lcdon(void)
{
  /* Display ON Setting */

  gvdbg("ON\n");
  Uc1698_SetReg(UC1698_SetDE+0x07);      //display on,select on/off mode.Green Enhance mode disable
}

/**************************************************************************************
 * Name:  uucp_lcdoff
 *
 * Description:
 *   Turn the LCD off
 *
 **************************************************************************************/

static void uucp_lcdoff(void)
{
  gvdbg("OFF\n");
}

/**************************************************************************************
 * Name:  uucp_dumpreg
 *
 * Description:
 *   Dump a range of LCD registers.
 *
 **************************************************************************************/

#undef CONFIG_DEBUG_GRAPHICS
#ifdef CONFIG_DEBUG_GRAPHICS

static void uucp_dumpreg(uint8_t startreg, uint8_t endreg)
{
  uint16_t value;
  uint8_t  addr;

  for (addr = startreg; addr <= endreg; addr++)
    {
      value = uucp_getreg(addr);
      gdbg(" %02x: %04x\n", addr, value);
    }
}
#endif

/**************************************************************************************
 * Name:  uucp_putrun
 *
 * Description:
 *   This method can be used to write a partial raster line to the LCD:
 *
 *   row     - Starting row to write to (range: 0 <= row < yres)
 *   col     - Starting column to write to (range: 0 <= col <= xres-npixels)
 *   buffer  - The buffer containing the run to be written to the LCD
 *   npixels - The number of pixels to write to the LCD
 *             (range: 0 < npixels <= xres-col)
 *
 **************************************************************************************/

static int uucp_putrun(fb_coord_t row, fb_coord_t col, FAR const uint8_t *buffer,
                        size_t npixels)
{
  uint16_t *run = (uint16_t*)buffer;
  unsigned int i;

  /* Buffer must be provided and aligned to a 16-bit address boundary */

  gvdbg("row: %d col: %d npixels: %d\n", row, col, npixels);
  DEBUGASSERT(buffer && ((uintptr_t)buffer & 1) == 0);

  /* Set up to write the run. */

  uucp_setcursor(row, col);
//  uucp_wrsetup();

  /* Write the run to GRAM. */

  for (i = 0; i < npixels; i++)
    {
      /* Write the pixel pixel to GRAM */

      uucp_wrram(*run++);
    }
  return OK;
}

/**************************************************************************************
 * Name:  uucp_getrun
 *
 * Description:
 *   This method can be used to read a partial raster line from the LCD:
 *
 *  row     - Starting row to read from (range: 0 <= row < yres)
 *  col     - Starting column to read read (range: 0 <= col <= xres-npixels)
 *  buffer  - The buffer in which to return the run read from the LCD
 *  npixels - The number of pixels to read from the LCD
 *            (range: 0 < npixels <= xres-col)
 *
 **************************************************************************************/

static int uucp_getrun(fb_coord_t row, fb_coord_t col, FAR uint8_t *buffer,
                        size_t npixels)
{
  uint16_t *run = (uint16_t*)buffer;
  unsigned int i;

  /* Buffer must be provided and aligned to a 16-bit address boundary */

  gvdbg("row: %d col: %d npixels: %d\n", row, col, npixels);
  DEBUGASSERT(buffer && ((uintptr_t)buffer & 1) == 0);

  /* Set up to read the run */

  uucp_setcursor(row, col);

  /* Read the run from GRAM. */

  for (i = 0; i < npixels; i++)
    {
      /* Read the next pixel */

      *run++ = uucp_rdram();
    }
  return OK;
}


/**************************************************************************************
 * Name:  uucp_setpower
 *
 * Description:
 *   Enable/disable LCD panel power (0: full off - CONFIG_LCD_MAXPOWERL: full on). On
 *   backlit LCDs, this setting may correspond to the backlight setting.
 *
 *   LCD backlight is made of 4 white chip LEDs in parallel, driven by an AAT3194 charge
 *   pump, MN4. The AAT3194 is controlled by the SAM3U4E through a single line. Simple
 *   Serial Control (S2Cwire) interface, which permits to enable, disable, and set the
 *   LED drive current (LED brightness control) from a 32-level logarithmic scale. Four
 *   resistors R93/R94/R95/R96 are implemented for optional current limitation.
 *
 **************************************************************************************/

static int uucp_setpower(struct lcd_dev_s *dev, int power)
{
#if 0
  struct uucp_dev_s *priv = (struct uucp_dev_s *)dev;
  unsigned int i;

  gvdbg("power: %d\n", power);
  DEBUGASSERT(power <= CONFIG_LCD_MAXPOWER);

  /* Switch off backlight */

  uucp_gpiowrite(GPIO_LCD_BKL, false);

  /* For for at least 500uS to drain the charge pump */

  up_udelay(500);

  /* Set new backlight level by pumping "level" times */

  for (i = 0; i < power; i++)
    {
      uucp_gpiowrite(GPIO_LCD_BKL, false);
      uucp_gpiowrite(GPIO_LCD_BKL, false);
      uucp_gpiowrite(GPIO_LCD_BKL, false);
      uucp_gpiowrite(GPIO_LCD_BKL, true);;
      uucp_gpiowrite(GPIO_LCD_BKL, true);;
      uucp_gpiowrite(GPIO_LCD_BKL, true);;
    }

  /* This delay seems to be required... perhaps because of the big current jump? */

  if (power != LCD_FULL_OFF)
    {
      up_mdelay(100);
    }

  priv->power = power;
//We are not handling this for now
#endif
  gvdbg("Power not implemented\n");
  return OK;
}

/**************************************************************************************
 * Name:  uucp_getcontrast
 *
 * Description:
 *   Get the current contrast setting (0-CONFIG_LCD_MAXCONTRAST).
 *
 **************************************************************************************/

static int uucp_getcontrast(struct lcd_dev_s *dev)
{
  gvdbg("Not implemented\n");
  return -ENOSYS;
}

/**************************************************************************************
 * Name:  uucp_getcontrast
 *
 * Description:
 *   Set LCD panel contrast (0-CONFIG_LCD_MAXCONTRAST).
 *
 **************************************************************************************/

static int uucp_setcontrast(struct lcd_dev_s *dev, unsigned int contrast)
{
  gvdbg("contrast: %d\n", contrast);
  return -ENOSYS;
}

/**************************************************************************************
 * Public Functions
 **************************************************************************************/

/**************************************************************************************
 * Name:  up_lcdinitialize
 *
 * Description:
 *   Initialize the LCD video hardware.  The initial state of the LCD is fully
 *   initialized, display memory cleared, and the LCD ready to use.
 *
 **************************************************************************************/

int up_lcdinitialize(void)
{
#ifdef CONFIG_DEBUG_GRAPHICS
  uint16_t hxregval;
#endif
  uint32_t regval;
  unsigned int i;

  gvdbg("Initializing\n");

  /* Enable LCD EXTCS2 pins */

  //uucp_configgpio(GPIO_LCD_NCS2);	//
  //uucp_configgpio(GPIO_LCD_RST);	//PB7 on STM32
 // uucp_lcd_rst();			//reset pulse to LCD
//  uucp_lcd_ledctrl();			//enable transistor for led backlight, PA4. Set it high, in gpio

  /* Configure LCD Backlight Pin */

  /* Enable SMC peripheral clock */

  //putreg32((1 << SAM3U_PID_SMC), SAM3U_PMC_PCER);
  stm32_configgpio(UUCP_LED1); //turn on the backlight
  stm32_gpiowrite(UUCP_LED1, true);
  stm32_configgpio(GPIO_PA4); //turn on the backlight
  stm32_gpiowrite(GPIO_PA4, false);
//  regdbg("PMC PCSR: %08x SMC: %08x\n", getreg32(SAM3U_PMC_PCSR), (1 << SAM3U_PID_SMC));

  /* Configure SMC CS2 */

//to configure access for STM32


  /* Check HX8347 Chip ID */

#ifdef CONFIG_DEBUG_GRAPHICS
#endif

  /* Initialize LCD controller (HX8347) -- Magic code from Atmel LCD example */

  /* Start internal OSC */

 Uc1698_SetReg(UC1698_SetBR+1);         //Bias Ratio:0:5,1:10,2:11,3:12
 Uc1698_SetReg(UC1698_SetPM);           //electronic potentionmeter
 Uc1698_SetReg(224);           //electronic potentionmeter
 Uc1698_SetReg(UC1698_SetPC+UC1698_LcdCapLarge+UC1698_VlcdInter);   //power control set as internal power
 Uc1698_SetReg(UC1698_SetTC+UC1698_TC05);   //set temperate compensation as 0%

 Uc1698_SetReg(UC1698_SetMAP+0x00);             //MX & MY disable
 Uc1698_SetReg(UC1698_SetCP+1);         //rgb-rgb
 Uc1698_SetReg(UC1698_SetCM+2);         //64k colours 5-6-5 16bit RGB

 SetWindows(0,0,128,160);

  /* Fill the display memory with the color BLACK */

  uucp_setcursor(0, 0);
//  uucp_wrsetup();
  for (i = 0; i < (UUCP_XRES * UUCP_YRES); i++)
    {
        uucp_wrram(RGB16_BLACK);
    }

  /* Turn the LCD on (but with the backlight off) */

  uucp_lcdon();
  return OK;
}


/**************************************************************************************
 * Name:  up_lcduninitialize
 *
 * Description:
 *   Unitialize the framebuffer support.
 *
 **************************************************************************************/

void up_lcduninitialize(void)
{
  /* Turn the LCD off */

  uucp_lcdoff();

  /* Set LCD backlight to FULL off */

//  uucp_setpower(&g_lcddev_s.dev, LCD_FULL_OFF);

  /* Disable SMC peripheral clock */

//  putreg32((1 << SAM3U_PID_SMC), SAM3U_PMC_PCDR);
}

void Uc1698_SetReg(char CDATA){

	putreg8(CDATA,LCDREGBASE);

}

void SetWindows(int x,int y,int w,int h)
{

         Uc1698_SetReg(UC1698_SetWC0);          //wpc0:column
         Uc1698_SetReg(x);                              //start from 130
         Uc1698_SetReg(UC1698_SetWC1);          //wpc1
         Uc1698_SetReg(x+w);                            //end:272

         Uc1698_SetReg(UC1698_SetWR0);          //wpp0:row
         Uc1698_SetReg(y);                              //start from 0
         Uc1698_SetReg(UC1698_SetWR1);          //wpp1
         Uc1698_SetReg(y+h);                            //end 160
}

