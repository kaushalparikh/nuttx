/************************************************************************************
 * configs/stm3210e-eval/src/up_selectcstn.c
 * arch/arm/src/board/up_selectcstn.c
 *
 *   Copyright (C) 2009 Gregory Nutt. All rights reserved.
 *   Author: Akshay Mishra, akshay@dspworks.in
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

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <debug.h>

#include "chip.h"
#include "up_arch.h"

#include "stm32_fsmc.h"
#include "stm32_gpio.h"
#include "stm32_internal.h"
#include "stm3210e-internal.h"

#ifdef CONFIG_STM32_FSMC

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

#if STM32_NGPIO_PORTS < 6
#  error "Required GPIO ports not enabled"
#endif


/* CSTN connected to D0...7 and A0 is the command/Data for 0/1 
 * The FSMC4 is connected to the CSTN. NE4 is the OE. WE/OE are common
 * Pin Usage (per schematic)
 *                     CSTN
 *   D[0..8]           [0..8]
 *   A[0]              [0]
 *   PSMC_NE4   PG10  OUT  ~CE   
 *   PSMC_NWE   PD5   OUT  ~WE  
 *   PSMC_NOE   PD4   OUT  ~OE 
 *
 */

/* GPIO configurations unique to SRAM  */

static const uint16_t g_cstnconfig[] =
{
  /* NE4  */

  GPIO_NPS_NE4
};
#define NCSTN_CONFIG (sizeof(g_cstnconfig)/sizeof(uint16_t))

/************************************************************************************
 * Name: stm32_selectcstn
 *
 * Description:
 *   Initialize to access CSTN
 *
 ************************************************************************************/

void stm32_selectcstn(void)
{
  /* Configure new GPIO state */

  stm32_extmemgpios(g_commonconfig, NCOMMON_CONFIG);
  stm32_extmemgpios(g_cstnconfig, NCSTN_CONFIG);

  /* Enable AHB clocking to the FSMC */

  stm32_enablefsmc();

  /* Bank1 NOR/SRAM control register configuration */

  putreg32(FSMC_BCR_MWID8|FSMC_BCR_WREN, STM32_FSMC_BCR4);

  /* Bank1 NOR/SRAM timing register configuration */

  putreg32(FSMC_BTR_ADDSET(16)|FSMC_BTR_ADDHLD(6)|FSMC_BTR_DATAST(6)|FSMC_BTR_BUSTRUN(8)|
           FSMC_BTR_CLKDIV(1), STM32_FSMC_BTR4);

  putreg32(0xffffffff, STM32_FSMC_BCR4);

  /* Enable the bank */

  putreg32(FSMC_BCR_MBKEN|FSMC_BCR_MWID8|FSMC_BCR_WREN, STM32_FSMC_BCR4);
}

#endif /* CONFIG_STM32_FSMC */


