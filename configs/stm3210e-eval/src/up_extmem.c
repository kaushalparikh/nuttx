/************************************************************************************
 * configs/stm3210e-eval/src/up_extmem.c
 * arch/arm/src/board/up_extmem.c
 *
 *   Copyright (C) 2009 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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
 ************************************************************************************/

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <assert.h>
#include <debug.h>

#include <arch/board/board.h>

#include "chip.h"
#include "up_arch.h"

#include "stm32_fsmc.h"
#include "stm32_gpio.h"
#include "stm32_internal.h"
#include "stm3210e-internal.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

#ifndef CONFIG_STM32_FSMC
#  warning "FSMC is not enabled"
#endif

#if STM32_NGPIO_PORTS < 6
#  error "Required GPIO ports not enabled"
#endif

/************************************************************************************
 * Public Data
 ************************************************************************************/

/* 512Kx16 SRAM is connected to bank2 of the FSMC interface and both 8- and 16-bit
 * accesses are allowed by BLN0 and BLN1 connected to BLE and BHE of SRAM,
 * respectively.
 *
 * Pin Usage (per schematic)
 *
 *                         FLASH   SRAM    NAND    LCD
 *   D[0..15]              [0..15] [0..15] [0..7]  [0..15]
 *   A[0..23]              [0..22] [0..18] [16,17] [0]
 *   FSMC_NBL0  PE0   OUT  ~BLE    ---     ---     ---
 *   FSMC_NBL1  PE1   OUT  ~BHE    ---     ---     ---
 *   FSMC_NE2   PG9   OUT  ---     ~E      ---     ---
 *   FSMC_NE3   PG10  OUT  ~CE     ---     ---     ---
 *   FSMC_NE4   PG12  OUT  ---     ---     ---     ~CS
 *   FSMC_NWE   PD5   OUT  ~WE     ~W      ~W      ~WR/SCL
 *   FSMC_NOE   PD4   OUT  ~OE     ~G      ~R      ~RD
 *   FSMC_NWAIT PD6   IN   ---     R~B     ---     ---
 *   FSMC_INT2  PG6*  IN   ---     ---     R~B     ---
 *
 *   *JP7 will switch to PD6
 */

/* It would be much more efficient to brute force these all into the
 * the appropriate registers.  Just a little tricky.
 */

/* GPIO configurations common to SRAM and NOR Flash */

#define NCOMMON_CONFIG 37

const uint16_t g_commonconfig[NCOMMON_CONFIG] =
{
  /* A0... A18 */

  GPIO_NPS_A0,  GPIO_NPS_A1,  GPIO_NPS_A2,  GPIO_NPS_A3,
  GPIO_NPS_A4,  GPIO_NPS_A5,  GPIO_NPS_A6,  GPIO_NPS_A7,
  GPIO_NPS_A8,  GPIO_NPS_A9,  GPIO_NPS_A10, GPIO_NPS_A11,
  GPIO_NPS_A12, GPIO_NPS_A13, GPIO_NPS_A14, GPIO_NPS_A15,
  GPIO_NPS_A16, GPIO_NPS_A17, GPIO_NPS_A18,

  /* D0... D15 */

  GPIO_NPS_D0,  GPIO_NPS_D1,  GPIO_NPS_D2,  GPIO_NPS_D3,
  GPIO_NPS_D4,  GPIO_NPS_D5,  GPIO_NPS_D6,  GPIO_NPS_D7,
  GPIO_NPS_D8,  GPIO_NPS_D9,  GPIO_NPS_D10, GPIO_NPS_D11,
  GPIO_NPS_D12, GPIO_NPS_D13, GPIO_NPS_D14, GPIO_NPS_D15,

  /* NOE, NWE  */

  GPIO_NPS_NOE, GPIO_NPS_NWE
};

/************************************************************************************
 * Private Data
 ************************************************************************************/

/************************************************************************************
 * Private Functions
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: stm32_initfsmc
 *
 * Description:
 *   Initialize GPIOs for NOR or SRAM
 *
 ************************************************************************************/

void stm32_initfsmc(void)
{
  int i;

  /* Configure GPIOs */

  for (i = 0; i < NCOMMON_CONFIG; i++)
    {
      stm32_configgpio(g_commonconfig[i]);
    }
}

/************************************************************************************
 * Name: stm32_enablefsmc
 *
 * Description:
 *  enable clocking to the FSMC module
 *
 ************************************************************************************/

void stm32_enablefsmc(void)
{
  uint32_t regval;

  /* Enable AHB clocking to the FSMC */

  regval  = getreg32( STM32_RCC_AHBENR);
  regval |= RCC_AHBENR_FSMCEN;
  putreg32(regval, STM32_RCC_AHBENR);
}

/************************************************************************************
 * Name: stm32_disablefsmc
 *
 * Description:
 *  enable clocking to the FSMC module
 *
 ************************************************************************************/

void stm32_disablefsmc(void)
{
  uint32_t regval;

  /* Enable AHB clocking to the FSMC */

  regval  = getreg32( STM32_RCC_AHBENR);
  regval &= ~RCC_AHBENR_FSMCEN;
  putreg32(regval, STM32_RCC_AHBENR);
}

/************************************************************************************
 * Name: stm32_fsmccontextsave
 *
 * Description:
 *  Save current GPIOs that will used by external memory configurations
 *
 ************************************************************************************/

void stm32_fsmccontextsave(struct extmem_save_s *save)
{
  DEBUGASSERT(save != NULL);
  save->gpiod_crl = getreg32(STM32_GPIOE_CRL);
  save->gpiod_crh = getreg32(STM32_GPIOE_CRH);
  save->gpioe_crl = getreg32(STM32_GPIOD_CRL);
  save->gpioe_crh = getreg32(STM32_GPIOD_CRH);
  save->gpiof_crl = getreg32(STM32_GPIOF_CRL);
  save->gpiof_crh = getreg32(STM32_GPIOF_CRH);
  save->gpiog_crl = getreg32(STM32_GPIOG_CRL);
  save->gpiog_crh = getreg32(STM32_GPIOG_CRH);  
}

/************************************************************************************
 * Name: stm32_fsmccontextrestore
 *
 * Description:
 *  Restore GPIOs that were used by external memory configurations
 *
 ************************************************************************************/

void stm32_fsmccontextrestore(struct extmem_save_s *restore)
{
  DEBUGASSERT(restore != NULL);
  putreg32(restore->gpiod_crl, STM32_GPIOE_CRL);
  putreg32(restore->gpiod_crh, STM32_GPIOE_CRH);
  putreg32(restore->gpioe_crl, STM32_GPIOD_CRL);
  putreg32(restore->gpioe_crh, STM32_GPIOD_CRH);
  putreg32(restore->gpiof_crl, STM32_GPIOF_CRL);
  putreg32(restore->gpiof_crh, STM32_GPIOF_CRH);
  putreg32(restore->gpiog_crl, STM32_GPIOG_CRL);
  putreg32(restore->gpiog_crh, STM32_GPIOG_CRH);  
}

/************************************************************************************
 * Name: stm32_selectsram
 *
 * Description:
 *   Initialize GPIOs for NOR or SRAM and memory interface
 *
 ************************************************************************************/

void stm32_selectsram(void)
{
  /* Configure new GPIO state */

  stm32_configgpio(GPIO_NPS_NE2);
  stm32_configgpio(GPIO_NPS_NBL0);
  stm32_configgpio(GPIO_NPS_NBL1);

  /* Bank2 NOR/SRAM control register configuration */

  putreg32(FSMC_BCR_MWID16|FSMC_BCR_WREN, STM32_FSMC_BCR2);

  /* Bank2 NOR/SRAM timing register configuration */

  putreg32(FSMC_BTR_ADDSET(1)|FSMC_BTR_ADDHLD(1)|FSMC_BTR_DATAST(2)|FSMC_BTR_BUSTRUN(1)|
           FSMC_BTR_CLKDIV(1)|FSMC_BTR_DATLAT(2)|FSMC_BTR_ACCMODA, STM32_FSMC_BTR2);

  putreg32(0xffffffff, STM32_FSMC_BWTR2);

  /* Enable the bank */

  putreg32(FSMC_BCR_MBKEN|FSMC_BCR_MWID16|FSMC_BCR_WREN, STM32_FSMC_BCR2);
}

/************************************************************************************
 * Name: stm32_deselectsram
 *
 * Description:
 *   Disable SRAM
 *
 ************************************************************************************/

void stm32_deselectsram(void)
{
  /* Restore registers to their power up settings */

  putreg32(0x000030d2, STM32_FSMC_BCR2);

  /* Bank1 NOR/SRAM timing register configuration */

  putreg32(0x0fffffff, STM32_FSMC_BTR2);
}

/************************************************************************************
 * Name: stm32_selectusb
 *
 * Description:
 *   Initialize GPIOs for USB controller and memory interface
 *
 ************************************************************************************/
 
void stm32_selectusb(void)
{
  /* Configure new GPIO state */

  stm32_configgpio(GPIO_NPS_NE3);

  /* Bank1 NOR/SRAM control register configuration */

  putreg32(FSMC_BCR_MWID16|FSMC_BCR_WREN|FSMC_BCR_EXTMOD, STM32_FSMC_BCR3);

  /* Bank1 NOR/SRAM timing register configuration */

  putreg32(FSMC_BTR_ADDSET(2)|FSMC_BTR_ADDHLD(1)|FSMC_BTR_DATAST(2)|FSMC_BTR_BUSTRUN(1)|
           FSMC_BTR_CLKDIV(1)|FSMC_BTR_DATLAT(2)|FSMC_BTR_ACCMODA, STM32_FSMC_BTR3);

  putreg32(FSMC_BTR_ADDSET(2)|FSMC_BTR_ADDHLD(1)|FSMC_BTR_DATAST(2)|
           FSMC_BTR_CLKDIV(1)|FSMC_BTR_DATLAT(2)|FSMC_BTR_ACCMODA, STM32_FSMC_BWTR3);

  /* Enable the bank */

  //putreg32(FSMC_BCR_MBKEN|FSMC_BCR_NOR|FSMC_BCR_FACCEN|FSMC_BCR_MWID16|FSMC_BCR_WREN, STM32_FSMC_BCR3);
  putreg32(FSMC_BCR_MBKEN|FSMC_BCR_MWID16|FSMC_BCR_WREN|FSMC_BCR_EXTMOD, STM32_FSMC_BCR3);
}

/************************************************************************************
 * Name: stm32_deselectusb
 *
 * Description:
 *   Disable USB controller
 *
 ************************************************************************************/

void stm32_deselectusb(void)
{
  /* Restore registers to their power up settings */

  putreg32(0x000030d2, STM32_FSMC_BCR3);

  /* Bank1 NOR/SRAM timing register configuration */

  putreg32(0x0fffffff, STM32_FSMC_BTR3);
  putreg32(0x0fffffff, STM32_FSMC_BWTR3);
}

/************************************************************************************
 * Name: stm32_selectlcd
 *
 * Description:
 *   Initialize to the LCD
 *
 ************************************************************************************/

void stm32_selectlcd(void)
{
}

/************************************************************************************
 * Name: stm32_deselectlcd
 *
 * Description:
 *   Disable the LCD
 *
 ************************************************************************************/

void stm32_deselectlcd(void)
{
}

/************************************************************************************
 * Name: stm32_usbregaccess
 *
 * Description:
 *   Set FSMC timings for register read/write
 *
 ************************************************************************************/

void stm32_usbregaccess(void)
{
  uint32_t regval;

  /* Set the inter read/write delay */

  regval  = getreg32(STM32_FSMC_BTR3);
  regval &= (~FSMC_BTR_ADDSET_MASK);
  
  putreg32(regval|FSMC_BTR_ADDSET(10), STM32_FSMC_BTR3);

  regval  = getreg32(STM32_FSMC_BWTR3);
  regval &= (~FSMC_BWTR_ADDSET_MASK);
  
  putreg32(regval|FSMC_BWTR_ADDSET(10), STM32_FSMC_BWTR3);
}

/************************************************************************************
 * Name: stm32_usbmemaccess
 *
 * Description:
 *   Set FSMC timings for memory read/write
 *
 ************************************************************************************/

void stm32_usbmemaccess(void)
{
  uint32_t regval;
  
  /* Set the inter read/write delay */

  regval  = getreg32(STM32_FSMC_BTR3);
  regval &= (~FSMC_BTR_ADDSET_MASK);
  
  putreg32(regval|FSMC_BTR_ADDSET(2), STM32_FSMC_BTR3);

  regval  = getreg32(STM32_FSMC_BWTR3);
  regval &= (~FSMC_BWTR_ADDSET_MASK);
  
  putreg32(regval|FSMC_BWTR_ADDSET(2), STM32_FSMC_BWTR3);
}

/************************************************************************************
 * Name: stm32_usbgetreg32
 *
 * Description:
 *   Read 32-bit register
 *
 ************************************************************************************/
 
uint32_t stm32_usbgetreg32(uint32_t addr)
{
  return getreg32((USB_BASE+addr));
}

/************************************************************************************
 * Name: stm32_usbputreg32
 *
 * Description:
 *   Write 32-bit register
 *
 ************************************************************************************/
 
void stm32_usbputreg32(uint32_t val, uint32_t addr)
{
  putreg32(val, (USB_BASE+addr));
}

