/****************************************************************************
 * include/nuttx/usb/isp1761_host.h
 *
 *   Copyright (C) 2010 Gregory Nutt. All rights reserved.
 *   Author: Author: Kaushal Parikh <kaushal@dspworks.in>
 *
 *   References: "ISP1761 Hi-Speed USB On-The-Go Controller"
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
 ****************************************************************************/

#ifndef __INCLUDE_NUTTX_USB_ISP1761_HOST_H
#define __INCLUDE_NUTTX_USB_ISP1761_HOST_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>
 
/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/

/* ISP1761 EHCI Operation register space offset */

#define ISP1761_OPERATIONAL_OFFSET(addr)   (addr + 0x0020) /* Operational register space offset */

/* ISP1761 specific Operational registers (section 8.2) */

#define ISP1761_ISOPTD_DONE_OFFSET         0x0130 /* Isochronous PTD done map */
#define ISP1761_ISOPTD_SKIP_OFFSET         0x0134 /* Isochronous PTD skip map */
#define ISP1761_ISOPTD_LASTPTD_OFFSET      0x0138 /* Isochronous PTD last PTD */

#define ISP1761_INTPTD_DONE_OFFSET         0x0140 /* Interrupt PTD done map */
#define ISP1761_INTPTD_SKIP_OFFSET         0x0144 /* Interrupt PTD skip map */
#define ISP1761_INTPTD_LASTPTD_OFFSET      0x0148 /* Interrupt PTD last PTD */

#define ISP1761_ATLPTD_DONE_OFFSET         0x0150 /* Asynchronous PTD done map */
#define ISP1761_ATLPTD_SKIP_OFFSET         0x0154 /* Asynchronous PTD skip map */
#define ISP1761_ATLPTD_LASTPTD_OFFSET      0x0158 /* Asynchronous PTD last PTD */

/* ISP1761 specific Configuration registers (section 8.3) */

#define ISP1761_HW_MODECTRL_OFFSET         0x0300 /* HWModeControl: Hardware mode control */
#define ISP1761_HCCHIPID_OFFSET            0x0304 /* HcChipID: ISP1761 chip identity */
#define ISP1761_HCSCRATCH_OFFSET           0x0308 /* HcScratch: Scratch register */
#define ISP1761_SW_RESET_OFFSET            0x030c /* SWReset: Software reset */
#define ISP1761_HCDMACONFIG_OFFSET         0x0330 /* HcDMAConfiguration: DMA configuration */
#define ISP1761_HCBUFSTS_OFFSET            0x0334 /* HcBufferStatus: Buffer status */
#define ISP1761_ATLDONETIMEOUT_OFFSET      0x0338 /* ATLDoneTimeout: ATL done timeout */
#define ISP1761_MEMORY_OFFSET              0x033c /* Memory: Memory bank/read */
#define ISP1761_EDGEINTRCOUNT_OFFSET       0x0340 /* EdgeInterruptCount: Edge interrupt count */
#define ISP1761_DMASTARTADDR               0x0344 /* DMAStartAddress: DMA start address */
#define ISP1761_POWERDOWNCTRL_OFFSET       0x0354 /* PowerDownControl: Power down control */

/* ISP1761 specific Interrupt registers section (8.4) */

#define ISP1761_HCINTR_OFFSET              0x0310 /* HcInterrupt: Interrupt */
#define ISP1761_HCINTREN_OFFSET            0x0314 /* HcInterruptEnable: Interrupt enable */
#define ISP1761_ISOIRQMASKOR_OFFSET        0x0318 /* Isochronous IRQ mask OR register */
#define ISP1761_INTIRQMASKOR_OFFSET        0x031c /* Interrupt IRQ mask OR register */
#define ISP1761_ATLIRQMASKOR_OFFSET        0x0320 /* Asynchronous IRQ mask OR register */
#define ISP1761_ISOIRQMASKAND_OFFSET       0x0324 /* Isochronous IRQ mask AND register */
#define ISP1761_INTIRQMASKAND_OFFSET       0x0328 /* Interrupt IRQ mask AND register */
#define ISP1761_ATLIRQMASKAND_OFFSET       0x032c /* Asynchronous IRQ mask AND register */

/* Register bit definitions *************************************************/

/* HWModeControl: Hardware mode control (section 8.3.1) */

#define ISP1761_HW_MODECTRL_GLOBALINTREN   (1 << 0)  /* Bit 0: Global interrupt enable */
#define ISP1761_HW_MODECTRL_INTRLEVEL      (1 << 1)  /* Bit 1: Interrupt level         */
#  define ISP1761_HW_MODECTRL_INTRLEVEL_L  (0 << 1)
#  define ISP1761_HW_MODECTRL_INTRLEVEL_E  (1 << 1)
#define ISP1761_HW_MODECTRL_INTRPOL        (1 << 2)  /* Bit 2: Interrupt polarity      */
#  define ISP1761_HW_MODECTRL_INTRPOL_L    (0 << 2)
#  define ISP1761_HW_MODECTRL_INTRPOL_H    (1 << 2)
                                                     /* Bits 3-4: Reserved */
#define ISP1761_HW_MODECTRL_DREQPOL        (1 << 5)  /* Bit 5: DREQ polarity */
#  define ISP1761_HW_MODECTRL_DREQPOL_L    (0 << 5)
#  define ISP1761_HW_MODECTRL_DREQPOL_H    (1 << 5)
#define ISP1761_HW_MODECTRL_DACKPOL        (1 << 6)  /* Bit 6: DACK polarity */
#  define ISP1761_HW_MODECTRL_DACKPOL_L    (0 << 6)
#  define ISP1761_HW_MODECTRL_DACKPOL_H    (1 << 6)
                                                     /* Bit 7: Reserved */
#define ISP1761_HW_MODECTRL_DATBUSWID      (1 << 8)  /* Bit 8: Data bus width */
#  define ISP1761_HW_MODECTRL_DATBUSWID_16 (0 << 8)
#  define ISP1761_HW_MODECTRL_DATBUSWID_32 (1 << 8)
#define ISP1761_HW_MODECTRL_COMNDMA        (1 << 9)  /* Bit 9: Common DMA REQ signals for host & peripheral */
#define ISP1761_HW_MODECTRL_COMNINTR       (1 << 10) /* Bit 10: Common interrupt for host & peripheral */
#define ISP1761_HW_MODECTRL_DEVDMA         (1 << 11) /* Bit 11: Common DMA ACK signals for host & peripheral */
                                                     /* Bits 12-14: Reserved */
#define ISP1761_HW_MODECTRL_ANADIGIOC      (1 << 15) /* Bit 15: Analog or digital overcurrent */
#  define ISP1761_HW_MODECTRL_DIGIOC       (0 << 15)
#  define ISP1761_HW_MODECTRL_ANAOC        (1 << 15)
                                                     /* Bits 16-30: Reserved */
#define ISP1761_HW_MODECTRL_ALLATXRESET    (1 << 31) /* Bit 31: All ATX reset */

/* HcChipID: ISP1761 chip identity (section 8.3.2) */

#define ISP1761_HCCHIPID_ID_SHIFT          (0)       /* Bits 0-15: Chip ID */
#define ISP1761_HCCHIPID_ID_MASK           (0xffff << ISP1761_HCCHIPID_ID_SHIFT)

#define ISP1761_HCCHIPID_VERSION_SHIFT     (16)      /* Bits 16-31: Hardware version */
#define ISP1761_HCCHIPID_VERSION_MASK      (0xffff << ISP1761_HCCHIPID_VERSION_SHIFT)

/* SWReset: Software reset (section 8.3.4) */

#define ISP1761_SW_RESET_ALL               (1 << 0)  /* Bit 0: Reset all registers */
#define ISP1761_SW_RESET_HC                (1 << 1)  /* Bit 1: Reset host controller registers */
                                                     /* Bits 2-31: Reserved */

/* HcDMAConfiguration: DMA configuration (section 8.3.5) */

#define ISP1761_HCDMACONFIG_RDWRSEL        (1 << 0)  /* Bit 0: DMA read/write select */
#  define ISP1761_HCDMACONFIG_WRSEL        (0 << 0)
#  define ISP1761_HCDMACONFIG_RDSEL        (1 << 0)
#define ISP1761_HCDMACONFIG_EN             (1 << 1)  /* Bit 1: Enable DMA */
#define ISP1761_HCDMACONFIG_BURSTLEN_SHIFT (2)       /* Bits 2-3: DMA burst length */
#define ISP1761_HCDMACONFIG_BURSTLEN_MASK  (0x3 << ISP1761_HCDMACONFIG_BURSTLEN_SHIFT)
#  define ISP1761_HCDMACONFIG_BURSTLEN_1   (0 << ISP1761_HCDMACONFIG_BURSTLEN_SHIFT)
#  define ISP1761_HCDMACONFIG_BURSTLEN_4   (1 << ISP1761_HCDMACONFIG_BURSTLEN_SHIFT)
#  define ISP1761_HCDMACONFIG_BURSTLEN_8   (2 << ISP1761_HCDMACONFIG_BURSTLEN_SHIFT)
#  define ISP1761_HCDMACONFIG_BURSTLEN_16  (3 << ISP1761_HCDMACONFIG_BURSTLEN_SHIFT)
                                                     /* Bits 4-7: Reserved */

#define ISP1761_HCDMACONFIG_COUNTER_SHIFT  (8)       /* Bits 8-31: DMA counter */
#define ISP1761_HCDMACONFIG_COUNTER_MASK   (0xffffff << ISP1761_HCDMACONFIG_COUNTER_SHIFT)

/* HcBufferStatus: Buffer status (section 8.3.6) */

#define ISP1761_HCBUFSTS_ATLFILLED         (1 << 0)  /* Bit 0: Asynchronous buffer filled */
#define ISP1761_HCBUFSTS_INTFILLED         (1 << 1)  /* Bit 1: Interrupt buffer filled */
#define ISP1761_HCBUFSTS_ISOFILLED         (1 << 2)  /* Bit 2: Isochronous buffer filled */
                                                     /* Bits 3-31: Reserved */

/* Memory: Memory bank/read (section 8.3.8) */

#define ISP1761_MEMORY_RDADDR_SHIFT        (0)       /* Bits 0-15: Start address for memory read */
#define ISP1761_MEMORY_RDADDR_MASK         (0xffff << ISP1761_MEMORY_RDADDR_SHIFT)

#define ISP1761_MEMORY_BANKSEL_SHIFT       (16)      /* Bits 16-17: Memory bank select */
#define ISP1761_MEMORY_BANKSEL_MASK        (0x3 << ISP1761_MEMORY_BANKSEL_SHIFT)
                                                     /* Bits 18-31: Reserved */

/* EdgeInterruptCount: Edge interrupt count (section 8.3.11) */

#define ISP1761_EDGEINTRCOUNT_CLK_SHIFT    (0)       /* Bits 0-15: Number of clocks */
#define ISP1761_EDGEINTRCOUNT_CLK_MASK     (0xffff << ISP1761_EDGEINTRCOUNT_CLK_SHIFT)
#define ISP1761_EDGEINTRCOUNT_CLK(n)       ((n << ISP1761_EDGEINTRCOUNT_CLK_SHIFT) & ISP1761_EDGEINTRCOUNT_CLK_MASK)
                                                     /* Bits 16-23: Reserved */

#define ISP1761_EDGEINTRCOUNT_MINWID_SHIFT (24)      /* Bits 24-31: Minimum duration in uSOF between IRQ */
#define ISP1761_EDGEINTRCOUNT_MINWID_MASK  (0xff << ISP1761_EDGEINTRCOUNT_MINWID_SHIFT)
#define ISP1761_EDGEINTRCOUNT_MINWID(n)    ((n << ISP1761_EDGEINTRCOUNT_MINWID_SHIFT) & ISP1761_EDGEINTRCOUNT_MINWID_MASK)

/* DMAStartAddress: DMA start address (section 8.3.12) */

#define ISP1761_DMASTARTADDR_SHIFT         (0)       /* Bits 0-15: Start address for DMA */
#define ISP1761_DMASTARTADDR_MASK          (0xffff << ISP1761_DMASTARTADDR_SHIFT)

/* PowerDownControl: Power down control (section 8.3.13) */

#define ISP1761_POWERDOWNCTRL_HCCLKEN      (1 << 0)  /* Bit 0: Host controller clock enable */
#define ISP1761_POWERDOWNCTRL_OC1PWR       (1 << 1)  /* Bit 1: Overcurrent 1 powered off during suspend */
#define ISP1761_POWERDOWNCTRL_OC2PWR       (1 << 2)  /* Bit 2: Overcurrent 2 powered off during suspend */
#define ISP1761_POWERDOWNCTRL_OC3PWR       (1 << 3)  /* Bit 3: Overcurrent 3 powered off during suspend */
#define ISP1761_POWERDOWNCTRL_VREGON       (1 << 4)  /* Bit 4: Vreg in low power mode during suspend */
#define ISP1761_POWERDOWNCTRL_BIASEN       (1 << 5)  /* Bit 5: Bias circuits powered on in suspend */
                                                     /* Bits 6-9: Reserved */
#define ISP1761_POWERDOWNCTRL_VBATDETPWR   (1 << 10) /* Bit 10: Vbat detector powered off in suspend */
#define ISP1761_POWERDOWNCTRL_PORT2PD      (1 << 11) /* Bit 11: Port 2 internal pull-down connected */
#define ISP1761_POWERDOWNCTRL_PORT3PD      (1 << 12) /* Bit 12: Port 3 internal pull-down connected */
                                                     /* Bits 13-15: Reserved */

#define ISP1761_POWERDOWNCTRL_CLKOFF_SHIFT (16)      /* Bits 16-31: Wake-up duration after event */
#define ISP1761_POWERDOWNCTRL_CLKOFF_MASK  (0xffff << ISP1761_POWERDOWNCTRL_CLKOFF_SHIFT)

/* Interrupt */

                                                     /* Bit 0: Reserved */
#define ISP1761_HCINTR_SOTITL              (1 << 1)  /* Bit 1: SOT ITL interrupt */
                                                     /* Bit 2: Reserved */
#define ISP1761_HCINTR_DMAEOT              (1 << 3)  /* Bit 3: SOT ITL interrupt */
                                                     /* Bit 4: Reserved */
#define ISP1761_HCINTR_HCSUSP              (1 << 5)  /* Bit 5: Host controller suspend */
#define ISP1761_HCINTR_CLKREADY            (1 << 6)  /* Bit 6: Clock ready */
#define ISP1761_HCINTR_INT                 (1 << 7)  /* Bit 7: INT interrupt */
#define ISP1761_HCINTR_ATL                 (1 << 8)  /* Bit 8: ATL interrupt */
#define ISP1761_HCINTR_ISO                 (1 << 9)  /* Bit 9: ISO interrupt */
#define ISP1761_HCINTR_OTG                 (1 << 10) /* Bit 10: OTG interrupt */
                                                     /* Bits 11-31: Reserved */

#define ISP1761_HCINTR_ALL                 (ISP1761_HCINTR_SOTITL|ISP1761_HCINTR_DMAEOT|ISP1761_HCINTR_HCSUSP|  \
                                            ISP1761_HCINTR_CLKREADY|ISP1761_HCINTR_INT|ISP1761_HCINTR_ATL|      \
                                            ISP1761_HCINTR_ISO|ISP1761_HCINTR_OTG)

/* Interrupt enable */

                                                     /* Bit 0: Reserved */
#define ISP1761_HCINTREN_SOTITL            (1 << 1)  /* Bit 1: SOT ITL interrupt */
                                                     /* Bit 2: Reserved */
#define ISP1761_HCINTREN_DMAEOT            (1 << 3)  /* Bit 3: DMA EOT interrupt */
                                                     /* Bit 4: Reserved */
#define ISP1761_HCINTREN_HCSUSP            (1 << 5)  /* Bit 5: Host controller suspend */
#define ISP1761_HCINTREN_CLKREADY          (1 << 6)  /* Bit 6: Clock ready */
#define ISP1761_HCINTREN_INT               (1 << 7)  /* Bit 7: INT interrupt */
#define ISP1761_HCINTREN_ATL               (1 << 8)  /* Bit 8: ATL interrupt */
#define ISP1761_HCINTREN_ISO               (1 << 9)  /* Bit 9: ISO interrupt */
#define ISP1761_HCINTREN_OTG               (1 << 10) /* Bit 10: OTG interrupt */
                                                     /* Bits 11-31: Reserved */

/* Proprietary transfer descriptors bit definitions (section 8.5) ***********/
/* Common bit definitions for all PTD types */

/* DW0 */

#define ISP1761_PTD_DW0_V                  (1 << 0)  /* Bit 0: PTD valid */
                                                     /* Bit 1-2: Reserved */

#define ISP1761_PTD_DW0_TRANSFERSIZE_SHIFT (3)       /* Bits 3-17: Number of bytes to transfer */
#define ISP1761_PTD_DW0_TRANSFERSIZE_MASK  (0x7fff << ISP1761_PTD_DW0_TRANSFERSIZE_SHIFT)

#define ISP1761_PTD_DW0_MAXPACKET_SHIFT    (18)      /* Bits 18-28: Maximum packet size */
#define ISP1761_PTD_DW0_MAXPACKET_MASK     (0x7ff << ISP1761_PTD_DW0_MAXPACKET_SHIFT)

#define ISP1761_PTD_DW0_TTMAXPACKET_SHIFT  (18)      /* Bits 18-28: TT maximum packet size */
#define ISP1761_PTD_DW0_TTMAXPACKET_MASK   (0x7ff << ISP1761_PTD_DW0_TTMAXPACKET_SHIFT)

#define ISP1761_PTD_DW0_MULT_SHIFT         (29)      /* Bits 29-30: Number of successive packets */
#define ISP1761_PTD_DW0_MULT_MASK          (0x3 << ISP1761_PTD_DW0_MULT_SHIFT)

#define ISP1761_PTD_DW0_EPLSB              (1 << 31) /* Bit 31: LSB of endpoint */

/* DW1 */

#define ISP1761_PTD_DW1_EPMSB_SHIFT        (0)       /* Bits 0-2: MSB[3:1] of endpoint */
#define ISP1761_PTD_DW1_EPMSB_MASK         (0x7 << ISP1761_PTD_DW1_EPMSB_SHIFT)

#define ISP1761_PTD_DW1_DEVADDR_SHIFT      (3)       /* Bits 3-9: Device address */
#define ISP1761_PTD_DW1_DEVADDR_MASK       (0x7f << ISP1761_PTD_DW1_DEVADDR_SHIFT)

#define ISP1761_PTD_DW1_TOKEN_SHIFT        (10)      /* Bit 10-11: Token PID */
#define ISP1761_PTD_DW1_TOKEN_MASK         (0x3 << ISP1761_PTD_DW1_TOKEN_SHIFT)
#  define ISP1761_PTD_DW1_TOKEN_OUT        (0 << ISP1761_PTD_DW1_TOKEN_SHIFT)
#  define ISP1761_PTD_DW1_TOKEN_IN         (1 << ISP1761_PTD_DW1_TOKEN_SHIFT)
#  define ISP1761_PTD_DW1_TOKEN_SETUP      (2 << ISP1761_PTD_DW1_TOKEN_SHIFT)
#  define ISP1761_PTD_DW1_TOKEN_PING       (3 << ISP1761_PTD_DW1_TOKEN_SHIFT)

#define ISP1761_PTD_DW1_EPTYPE_SHIFT       (12)      /* Bits 12-13: Endpoint type */
#define ISP1761_PTD_DW1_EPTYPE_MASK        (0x3 <<  ISP1761_PTD_DW1_EPTYPE_SHIFT)
#  define ISP1761_PTD_DW1_EPTYPE_CTRL      (0 << ISP1761_PTD_DW1_EPTYPE_SHIFT)
#  define ISP1761_PTD_DW1_EPTYPE_ISO       (1 << ISP1761_PTD_DW1_EPTYPE_SHIFT)
#  define ISP1761_PTD_DW1_EPTYPE_BULK      (2 << ISP1761_PTD_DW1_EPTYPE_SHIFT)
#  define ISP1761_PTD_DW1_EPTYPE_INT       (3 << ISP1761_PTD_DW1_EPTYPE_SHIFT)

#define ISP1761_PTD_DW1_S                  (1 << 14) /* Bit 14: Split/HS transaction */
#  define ISP1761_PTD_DW1_HS               (0 << 14)
#  define ISP1761_PTD_DW1_SPLIT            (1 << 14)
                                                     /* Bits 15-31: Reserved */

/* DW2 */

#define ISP1761_PTD_DW2_uFRAME_SHIFT       (0)       /* Bits 0-2: Don't care
                                                             3-7: PTD execute frame number */
#define ISP1761_PTD_DW2_uFRAME_MASK        (0xff << ISP1761_PTD_DW2_uFRAME_SHIFT)
                                                     /* Bits 0-7: PTD type dependent */
#define ISP1761_PTD_DW2_DATADDR_SHIFT      (8)       /* Bits 8-23: Data start address */
#define ISP1761_PTD_DW2_DATADDR_MASK       (0xffff << ISP1761_PTD_DW2_DATADDR_SHIFT)
                                                     /* Bit 24: Reserved */

#define ISP1761_PTD_DW2_RL_SHIFT           (25)      /* Bits 25-28: Transaction retry count */
#define ISP1761_PTD_DW2_RL_MASK            (0xf << ISP1761_PTD_DW2_RL_SHIFT)
                                                     /* Bits 29-31: Reserved */

/* DW3 */

#define ISP1761_PTD_DW3_TRANSFERRED_SHIFT  (0)       /* Bits 0-14: Number of bytes transferred */
#define ISP1761_PTD_DW3_TRANSFERRED_MASK   (0x7fff << ISP1761_PTD_DW3_TRANSFERRED_SHIFT)
                                                     /* Bits 15-18: Reserved */

#define ISP1761_PTD_DW3_NAKCOUNT_SHIFT     (19)      /* Bits 19-22: NACK count */
#define ISP1761_PTD_DW3_NAKCOUNT_MASK      (0xf << ISP1761_PTD_DW3_NAKCOUNT_SHIFT)

#define ISP1761_PTD_DW3_CERR_SHIFT         (23)      /* Bits 23-24: Error counter */
#define ISP1761_PTD_DW3_CERR_MASK          (0x3 << ISP1761_PTD_DW3_CERR_SHIFT)

#define ISP1761_PTD_DW3_DT                 (1 << 25) /* Bit 25: Data toggle */
#define ISP1761_PTD_DW3_P                  (1 << 26) /* Bit 26: Ping set */
#define ISP1761_PTD_DW3_SC                 (1 << 27) /* Bit 27: Start/complete (for split transactions) */
#  define ISP1761_PTD_DW3_S                (0 << 27)
#  define ISP1761_PTD_DW3_C                (1 << 27)
#define ISP1761_PTD_DW3_X                  (1 << 28) /* Bit 28: Transaction error */
#define ISP1761_PTD_DW3_B                  (1 << 29) /* Bit 29: Transaction babble detected */
#define ISP1761_PTD_DW3_H                  (1 << 30) /* Bit 30: Transaction halted */
#define ISP1761_PTD_DW3_A                  (1 << 31) /* Bit 31: Transaction active */

/* DW4 */

#define ISP1761_PTD_DW4_uSA_SHIFT          (0)       /* Bits 0-7: uSOF start split active */
#define ISP1761_PTD_DW4_uSA_MASK           (0xff << ISP1761_PTD_DW4_uSA_SHIFT)

#define ISP1761_PTD_DW4_uSOFSTS_SHIFT      (8)       /* Bits 8-31: Status of uSOF 0 to 7 */
#define ISP1761_PTD_DW4_uSOFSTS_MASK(n)    (0x7 << (ISP1761_PTD_DW4_uSOFSTS_SHIFT+(3*n)))
#define ISP1761_PTD_DW4_uSOFSTS_ERR(n)     (1 << (ISP1761_PTD_DW4_uSOFSTS_SHIFT+(3*n)))
#define ISP1761_PTD_DW4_uSOFSTS_BABBLE(n)  (1 << (ISP1761_PTD_DW4_uSOFSTS_SHIFT+(3*n)+1))
#define ISP1761_PTD_DW4_uSOFSTS_DATERR(n)  (1 << (ISP1761_PTD_DW4_uSOFSTS_SHIFT+(3*n)+2))

/* DW5 for HS Isochronous/Interrupt PTD */

#define ISP1761_HSPTD_DW5_uSOF0IN_SHIFT    (0)       /* Bits 0-11: Bytes received during uSOF0 */
#define ISP1761_HSPTD_DW5_uSOF0IN_MASK     (0xfff << ISP1761_PTD_DW5_uSOF0IN_SHIFT)

#define ISP1761_HSPTD_DW5_uSOF1IN_SHIFT    (12)      /* Bits 12-23: Bytes received during uSOF1 */
#define ISP1761_HSPTD_DW5_uSOF1IN_MASK     (0xfff << ISP1761_PTD_DW5_uSOF1IN_SHIFT)

#define ISP1761_HSPTD_DW5_uSOF2INLSB_SHIFT (24)      /* Bits 24-31: LSB[7:0] of bytes received during uSOF2 */
#define ISP1761_HSPTD_DW5_uSOF2INLSB_MASK  (0xff << ISP1761_PTD_DW5_uSOF2INLSB_SHIFT)

/* DW6 for HS Isochronous/Interrupt PTD */

#define ISP1761_HSPTD_DW6_uSOF2INMSB_SHIFT (0)       /* Bits 0-3: MSB[11:8] of bytes received during uSOF2 */
#define ISP1761_HSPTD_DW6_uSOF2INMSB_MASK  (0xf << ISP1761_PTD_DW6_uSOF2INMSB_SHIFT)

#define ISP1761_HSPTD_DW6_uSOF3IN_SHIFT    (4)       /* Bits 4-15: Bytes received during uSOF3 */
#define ISP1761_HSPTD_DW6_uSOF3IN_MASK     (0xfff << ISP1761_PTD_DW6_uSOF3IN_SHIFT)

#define ISP1761_HSPTD_DW6_uSOF4IN_SHIFT    (16)      /* Bits 16-27: Bytes received during uSOF4 */
#define ISP1761_HSPTD_DW6_uSOF4IN_MASK     (0xfff << ISP1761_PTD_DW6_uSOF4IN_SHIFT)

#define ISP1761_HSPTD_DW6_uSOF5INLSB_SHIFT (28)      /* Bits 28-31: LSB[3:0] of bytes received during uSOF5 */
#define ISP1761_HSPTD_DW6_uSOF5INLSB_MASK  (0xf << ISP1761_PTD_DW6_uSOF5INLSB_SHIFT)

/* DW7 for HS Isochronous/Interrupt PTD */

#define ISP1761_HSPTD_DW7_uSOF5INMSB_SHIFT (0)       /* Bits 0-7: MSB[11:4] of bytes received during uSOF5 */
#define ISP1761_HSPTD_DW7_uSOF5INMSB_MASK  (0xff << ISP1761_PTD_DW7_uSOF5INMSB_SHIFT)

#define ISP1761_HSPTD_DW7_uSOF6IN_SHIFT    (8)       /* Bits 8-19: Bytes received during uSOF6 */
#define ISP1761_HSPTD_DW7_uSOF6IN_MASK     (0xfff << ISP1761_PTD_DW7_uSOF6IN_SHIFT)

#define ISP1761_HSPTD_DW7_uSOF7IN_SHIFT    (20)      /* Bits 20-31: Bytes received during uSOF7 */
#define ISP1761_HSPTD_DW7_uSOF7IN_MASK     (0xfff << ISP1761_PTD_DW7_uSOF7IN_SHIFT)

/* DW5 for Split Isochronous/Interrupt PTD */

#define ISP1761_sPTD_DW5_uSCS_SHIFT        (0)       /* Bits 0-7: uSOF complete split active */
#define ISP1761_sPTD_DW5_uSCS_MASK         (0xff << ISP1761_sPTD_DW5_uSCS_SHIFT)

#define ISP1761_sPTD_DW5_uSOF0IN_SHIFT     (0)       /* Bits 8-15: Bytes received during uSOF0 */
#define ISP1761_sPTD_DW5_uSOF0IN_MASK      (0xff << ISP1761_sPTD_DW5_uSOF0IN_SHIFT)

#define ISP1761_sPTD_DW5_uSOF1IN_SHIFT     (0)       /* Bits 16-23: Bytes received during uSOF1 */
#define ISP1761_sPTD_DW5_uSOF1IN_MASK      (0xff << ISP1761_sPTD_DW5_uSOF1IN_SHIFT)

#define ISP1761_sPTD_DW5_uSOF2IN_SHIFT     (0)       /* Bits 24-31: Bytes received during uSOF2 */
#define ISP1761_sPTD_DW5_uSOF2IN_MASK      (0xff << ISP1761_sPTD_DW5_uSOF2IN_SHIFT)

/* DW6 for Split Isochronous/Interrupt PTD */

#define ISP1761_sPTD_DW6_uSOF3IN_SHIFT     (0)       /* Bits 0-7: Bytes received during uSOF3 */
#define ISP1761_sPTD_DW6_uSOF3IN_MASK      (0xff << ISP1761_sPTD_DW6_uSOF3IN_SHIFT)

#define ISP1761_sPTD_DW6_uSOF4IN_SHIFT     (0)       /* Bits 8-15: Bytes received during uSOF4 */
#define ISP1761_sPTD_DW6_uSOF4IN_MASK      (0xff << ISP1761_sPTD_DW6_uSOF4IN_SHIFT)

#define ISP1761_sPTD_DW6_uSOF5IN_SHIFT     (0)       /* Bits 16-23: Bytes received during uSOF5 */
#define ISP1761_sPTD_DW6_uSOF5IN_MASK      (0xff << ISP1761_sPTD_DW6_uSOF5IN_SHIFT)

#define ISP1761_sPTD_DW6_uSOF6IN_SHIFT     (0)       /* Bits 24-31: Bytes received during uSOF6 */
#define ISP1761_sPTD_DW6_uSOF6IN_MASK      (0xff << ISP1761_sPTD_DW6_uSOF6IN_SHIFT)

/* DW7 for Split Isochronous/Interrupt PTD */

#define ISP1761_sPTD_DW7_uSOF7IN_SHIFT     (0)       /* Bits 0-7: Bytes received during uSOF7 */
#define ISP1761_sPTD_DW7_uSOF7IN_MASK      (0xff << ISP1761_sPTD_DW7_uSOF7IN_SHIFT)
                                                     /* Bits 8-31: Reserved */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Generic Proprietary Transfer Descriptor (section 8.5) */

struct isp1761_ptd_s
{
  uint32_t dw[8];
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/


#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_USB_ISP1761_HOST_H */
