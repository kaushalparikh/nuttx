/****************************************************************************
 * include/nuttx/usb/ehci.h
 *
 *   Copyright (C) 2010 Gregory Nutt. All rights reserved.
 *   Author: Author: Kaushal Parikh <kaushal@dspworks.in>
 *
 *   References: "Enhanced Host Controller Interface Specification
 *   for USB," Version 1.0
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

#ifndef __INCLUDE_NUTTX_USB_EHCI_H
#define __INCLUDE_NUTTX_USB_EHCI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>
 
/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/
/* Capability registers (section 2.2) */

#define EHCI_CAPLENGTH_OFFSET              0x0000 /* HcCapabilityLength: Capability registers length */
#define EHCI_HCIVERSION_OFFSET             0x0002 /* HcVersion: Interface version number */
#define EHCI_HCSPARAMS_OFFSET              0x0004 /* HcStructuralParams: Structural parameters */
#define EHCI_HCCPARAMS_OFFSET              0x0008 /* HcCapabilityParams: Capability parameters */
#define EHCI_HCSP_PORTROUTE_OFFSET         0x000c /* HcPortRoute: Companion port route description */

/* Operational registers (section 2.3) */
/* Offsets are relative to end of Capability register space */

#define EHCI_USBCMD_OFFSET                 0x0000 /* HcUSBCommand: USB command */
#define EHCI_USBSTS_OFFSET                 0x0004 /* HcUSBStatus: USB status */
#define EHCI_USBINTR_OFFSET                0x0008 /* HcUSBInterrupt: USB interrupt enable */
#define EHCI_FRINDEX_OFFSET                0x000c /* HcFrameIndex: Frame index */
#define EHCI_CTRLDSSEGMENT_OFFSET          0x0010 /* HcControlDataSegment: Control data structure segment */
#define EHCI_PERIODICLISTBASE_OFFSET       0x0014 /* HcPeriodListBase: Frame list base address */
#define EHCI_ASYNCLISTADDR_OFFSET          0x0018 /* HcAsyncListAddress: Next asynchronous list address */
#define EHCI_CONFIGFLAG_OFFSET             0x0040 /* HcConfigFlag: Configured flag */

#define EHCI_MAX_RHPORT                    15     /* Maximum number of EHCI root hub ports */

#define EHCI_PORTSC_OFFSET(n)              (0x0044+((n-1) << 2))
#define EHCI_PORTSC1_OFFSET                EHCI_PORTSC_OFFSET(1)  /* HcRhPort1Status:  Root hub port status 1 */
#define EHCI_PORTSC2_OFFSET                EHCI_PORTSC_OFFSET(2)  /* HcRhPort2Status:  Root hub port status 2 */
#define EHCI_PORTSC3_OFFSET                EHCI_PORTSC_OFFSET(3)  /* HcRhPort3Status:  Root hub port status 3 */
#define EHCI_PORTSC4_OFFSET                EHCI_PORTSC_OFFSET(4)  /* HcRhPort4Status:  Root hub port status 4 */
#define EHCI_PORTSC5_OFFSET                EHCI_PORTSC_OFFSET(5)  /* HcRhPort5Status:  Root hub port status 5 */
#define EHCI_PORTSC6_OFFSET                EHCI_PORTSC_OFFSET(6)  /* HcRhPort6Status:  Root hub port status 6 */
#define EHCI_PORTSC7_OFFSET                EHCI_PORTSC_OFFSET(7)  /* HcRhPort7Status:  Root hub port status 7 */
#define EHCI_PORTSC8_OFFSET                EHCI_PORTSC_OFFSET(8)  /* HcRhPort8Status:  Root hub port status 8 */
#define EHCI_PORTSC9_OFFSET                EHCI_PORTSC_OFFSET(9)  /* HcRhPort9Status:  Root hub port status 9 */
#define EHCI_PORTSC10_OFFSET               EHCI_PORTSC_OFFSET(10) /* HcRhPort10Status: Root hub port status 10 */
#define EHCI_PORTSC11_OFFSET               EHCI_PORTSC_OFFSET(11) /* HcRhPort11Status: Root hub port status 11 */
#define EHCI_PORTSC12_OFFSET               EHCI_PORTSC_OFFSET(12) /* HcRhPort12Status: Root hub port status 12 */
#define EHCI_PORTSC13_OFFSET               EHCI_PORTSC_OFFSET(13) /* HcRhPort13Status: Root hub port status 13 */
#define EHCI_PORTSC14_OFFSET               EHCI_PORTSC_OFFSET(14) /* HcRhPort14Status: Root hub port status 14 */
#define EHCI_PORTSC15_OFFSET               EHCI_PORTSC_OFFSET(15) /* HcRhPort15Status: Root hub port status 15 */

/* Register bit definitions *************************************************/

/* HcCapabilityLength: Capability registers length (section 2.2.1)
 * This register is used as an offset to add to register base to find the
 * beginning of the Operational register space.
 */

#define EHCI_CAPLENGTH_SHIFT               (0)       /* Bits 0-7: Capability registers length */
#define EHCI_CAPLENGTH_MASK                (0xff << EHCI_CAPLENGTH_SHIFT)

/* HcVersion: Interface version number (section 2.2.2)
 * This is a two-byte register containing a BCD encoding of the EHCI revision
 * number supported by this host controller. The most significant byte of
 * this register represents a major revision and the least significant byte
 * is the minor revision.
 */ 

#define EHCI_HCIVER_MINOR_SHIFT            (0)       /* Bits 0-7: HCI spec minor version (BCD) */
#define EHCI_HCIVER_MINOR_MASK             (0xff << EHCI_HCIVER_MINOR_SHIFT)

#define EHCI_HCIVER_MAJOR_SHIFT            (8)       /* Bits 8-15: HCI spec major version (BCD) */
#define EHCI_HCIVER_MAJOR_MASK             (0xff << EHCI_HCIVER_MINOR_SHIFT)

/* HcStructuralParams: Structural parameters (section 2.2.3)
 * This is a set of fields that are structural parameters: Number of downstream ports,
 * etc.
 */

#define EHCI_HCSPARAMS_NPORTS_SHIFT        (0)       /* Bits 3-0: Number downstream ports */
#define EHCI_HCSPARAMS_NPORTS_MASK         (0xf << EHCI_HCSPARAMS_NPORTS_SHIFT)

#define EHCI_HCSPARAMS_PPC                 (1 << 4)  /* Bit 4: Port power control implemented */
                                                     /* Bits 5-6: Reserved */
#define EHCI_HCSPARAMS_PRR                 (1 << 7)  /* Bit 7: Port routing rules */

#define EHCI_HCSPARAMS_NPCC_SHIFT          (8)       /* Bits 8-11: Number of ports per companion controller */
#define EHCI_HCSPARAMS_NPCC_MASK           (0xf << EHCI_HCSPARAMS_NPCC_SHIFT)

#define EHCI_HCSPARAMS_NCC_SHIFT           (12)      /* Bits 12-15: Number of companion controller */
#define EHCI_HCSPARAMS_NCC_MASK            (0xf << EHCI_HCSPARAMS_NCC_SHIFT)

#define EHCI_HCSPARAMS_PINDICATOR          (1 << 16) /* Bit 16: Port indicator control supported */
                                                     /* Bits 17-19: Reserved */

#define EHCI_HCSPARAMS_DEBUGPORT_SHIFT     (20)      /* Bits 20:23: Debug port number */
#define EHCI_HCSPARAMS_DEBUGPORT_MASK      (0xf << EHCI_HCSPARAMS_DEBUGPORT_SHIFT)
                                                     /* Bits 24-31: Reserved */

/* HcCapabilityParams: Capability parameters (section 2.2.4)
 * Multiple Mode control (time-base bit functionality), addressing capability
 */

#define EHCI_HCCPARAMS_64BITADDR           (1 << 0)  /* Bit 0: 64-bit data structure addresses */
#define EHCI_HCCPARAMS_PROGFRAMELIST       (1 << 1)  /* Bit 1: Programmable frame list supported using USBCMD */
#define EHCI_HCCPARAMS_ASYNCPARK           (1 << 2)  /* Bit 2: Async park supported using USBCMD */
                                                     /* Bit 3: Reserved */

#define EHCI_HCCPARAMS_ISOSCHEDTHRES_SHIFT (3)       /* Bits 4-6: Isochronous scheduling cache */
#define EHCI_HCCPARAMS_ISOSCHEDTHRES_MASK  (0x7 << EHCI_HCCPARAMS_ISOSCHEDTHRES_SHIFT)
#define EHCI_HCCPARAMS_ISOSCHEDCACHEEN     (1 << 7)  /* Bit 7: Isochronous scheduling cache enable */

#define EHCI_HCCPARAMS_EECP_SHIFT          (8)       /* Bits 8-15: EHCI extended capabilities pointer */
#define EHCI_HCCPARAMS_EECP_MASK           (0xff << EHCI_HCCPARAMS_EECP_SHIFT)
                                                     /* Bits 16-31: Reserved */

/* HcPortRoute: Port route (section 2.2.5)
 */

/* HcUSBCommand: USB command (section 2.3.1)
 * The Command Register indicates the command to be executed by the serial
 * bus host controller. Writing to the register causes a command to be
 * executed.
 */

#define EHCI_USBCMD_RS                     (1 << 0)  /* Bit 0: Executes scheduled transfers */
#define EHCI_USBCMD_HCRESET                (1 << 1)  /* Bit 1: HC reset */

#define EHCI_USBCMD_FRAMELISTSIZE_SHIFT    (2)       /* Bits 2-3: Frame list size */
#define EHCI_USBCMD_FRAMELISTSIZE_MASK     (0x3 << EHCI_USBCMD_FRAMELISTSIZE_SHIFT)
#  define EHCI_USBCMD_FRAMELISTSIZE_1024   (0 << EHCI_USBCMD_FRAMELISTSIZE_SHIFT)
#  define EHCI_USBCMD_FRAMELISTSIZE_512    (1 << EHCI_USBCMD_FRAMELISTSIZE_SHIFT)
#  define EHCI_USBCMD_FRAMELISTSIZE_256    (2 << EHCI_USBCMD_FRAMELISTSIZE_SHIFT)

#define EHCI_USBCMD_PERIODICSCHEDEN        (1 << 4)  /* Bit 4: Enable period schedule */
#define EHCI_USBCMD_ASYNCSCHEDEN           (1 << 5)  /* Bit 5: Enable asynchronous schedule */
#define EHCI_USBCMD_ASYNCADVANCEINTR       (1 << 6)  /* Bit 6: Interrupt on async advance doorbell */
#define EHCI_USBCMD_LIGHTHCRESET           (1 << 7)  /* Bit 7: Light HC reset */

#define EHCI_USBCMD_PARKMODECOUNT_SHIFT    (8)       /* Bits 8-9: Asynchronous park mode count */
#define EHCI_USBCMD_PARKMODECOUNT_MASK     (0x3 << EHCI_USBCMD_PARKMODECOUNT_SHIFT)
                                                     /* Bit 10: Reserved */

#define EHCI_USBCMD_PARKMODEEN             (1 << 10) /* Bit 11: Asynchronous park mode enable */
                                                     /* Bits 12-15: Reserved */

#define EHCI_USBCMD_INTRTHRES_SHIFT        (16)      /* Bits 16-32: Interrupt threshold control */
#define EHCI_USBCMD_INTRTHRES_MASK         (0xff << EHCI_USBCMD_INTRTHRES_SHIFT)
                                                     /* Bits 24-31: Reserved */

/* HcUSBStatus: USB status, sHcControlDataSegmentection (2.3.2)
 * This register indicates pending interrupts and various states of the Host
 * Controller. The status resulting from a transaction on the serial bus is
 * not indicated in this register. Software sets a bit to 0 in this register
 * by writing a 1 to it. See Section 4.15 for additional information
 * concerning USB interrupt conditions.
 */

#define EHCI_USBSTS_USBINT                 (1 << 0)  /* Bit 0: USB interrupt */
#define EHCI_USBSTS_USBERRINT              (1 << 1)  /* Bit 1: USB error interrupt */
#define EHCI_USBSTS_PORTCHANGEDETECT       (1 << 2)  /* Bit 2: Port change status */
#define EHCI_USBSTS_FRAMELISTROLLOVER      (1 << 3)  /* Bit 3: Frame list rollover from max value */
#define EHCI_USBSTS_HOSTSYSTEMERR          (1 << 4)  /* Bit 4: Host system error */
#define EHCI_USBSTS_ASYNCADVANCEINTR       (1 << 5)  /* Bit 5: Asynchronous schedule advanced */
                                                     /* Bits 6-11: Reserved */
#define EHCI_USBSTS_HCHALTED               (1 << 12) /* Bit 12: HC halted */
#define EHCI_USBSTS_RECLAMATION            (1 << 13) /* Bit 13: Empty asynchronous schedule */
#define EHCI_USBSTS_PERIODSCHEDSTS         (1 << 14) /* Bit 14: Periodic schedule status */
#define EHCI_USBSTS_ASYNCSCHEDSTS          (1 << 15) /* Bit 15: Asynchronous schedule status */
                                                     /* Bits 16-31: Reserved */

/* HcUSBInterrupt: USB interrupt (section 2.3.3)
 * This register enables and disables reporting of the corresponding
 * interrupt to the software. When a bit is set and the corresponding
 * interrupt is active, an interrupt is generated to the host.
 * Interrupt sources that are disabled in this register still appear
 * in the USBSTS to allow the software to poll for events.
 */

#define EHCI_USBINTR_USBINTEN              (1 << 0)  /* Bit 0: USB interrupt enable */
#define EHCI_USBINTR_USBERRINTEN           (1 << 1)  /* Bit 1: USB error interrupt enable */
#define EHCI_USBINTR_PORTCHANGEINTEN       (1 << 2)  /* Bit 2: Port change interrupt enable */
#define EHCI_USBINTR_FRAMELISTROLLOVEREN   (1 << 3)  /* Bit 3: Frame list rollover interrupt enable */
#define EHCI_USBINTR_HOSTSYSTEMERREN       (1 << 4)  /* Bit 4: Host system error interrupt enable */
#define EHCI_USBINTR_ASYNCADVANCEEN        (1 << 5)  /* Bit 5: Asynchronous advance interrupt enable */
                                                     /* Bits 6-31: Reserved */

/* HcFrameIndex: Frame index (section 2.3.4)
 * This register is used by the host controller to index into the
 * periodic frame list.
 */

#define EHCI_FRINDEX_FRAMEINDEX_SHIFT      (0)       /* Bits 0-13: Frame index */
#define EHCI_FRINDEX_FRAMEINDEX_MASK       (0x3fff << EHCI_FRINDEX_FRAMEINDEX_SHIFT)

/* HcControlDataSegment: Control data structure segment (section 2.3.5)
 * This 32-bit register corresponds to the most significant address
 * bits [63:32] for all EHCI data structures. If the 64-bit
 * Addressing Capability field in HCCPARAMS is a zero, then this
 * register is not used. Software cannot write to it and a read
 * from this register will return zeros.
 */

/* HcPeriodListBase: Period frame list base address (section 2.3.6)
 * This 32-bit register contains the beginning address of the
 * Periodic Frame List in the system memory.
 */

                                                     /* Bits 0-11: Reserved */
#define EHCI_PERIODICLISTBASE_ADDR_SHIFT   (12)      /* Bits 12-31: Memory address signals [31:12] */
#define EHCI_PERIODICLISTBASE_ADDR_MASK    (0xfffff << EHCI_PERIODICLISTBASE_ADDR_SHIFT)

/* HcAsyncListAddress: Current Asynchronous list address (section 2.3.7)
 * This 32-bit register contains the address of the next
 * asynchronous queue head to be executed.
 */
                                                     /* Bits 0-4: Reserved */
#define EHCI_ASYNCLISTADDR_LPL_SHIFT       (5)       /* Bits 5-31: Memory address signals [31:5] */
#define EHCI_ASYNCLISTADDR_LPL_MASK        (0x3ffffff << EHCI_ASYNCLISTADDR_LPL_SHIFT)

/* HcConfigFlag: Configure flag (section 2.3.8)
 * This register is in the auxiliary power well. It is only reset
 * by hardware when the auxiliary power is initially applied or
 * in response to a host controller reset.
 */

#define EHCI_CONFIGFLAG_CF                 (1 << 0)  /* Bit 0: Configure host */
                                                     /* Bits 1-31: Reserved */

/* HcRhPortnStatus: Port 'n' status and control (section 2.3.9)
 * A host controller must implement one or more port registers.
 * The number of port registers implemented by a particular
 * instantiation of a host controller is documented in the
 * HCSPARAMs register (section 2.2.3). Software uses this
 * information as an input parameter to determine how many
 * ports need to be serviced.
 */

#define EHCI_PORTSC_CURRENTCONNECTSTS      (1 << 0)  /* Bit 0: Current connect status */
#define EHCI_PORTSC_CONNECTSTSCHANGE       (1 << 1)  /* Bit 1: Change in current connect status */
#define EHCI_PORTSC_PORTEN                 (1 << 2)  /* Bit 2: Enable port */
#define EHCI_PORTSC_PORTENCHANGE           (1 << 3)  /* Bit 3: Change in port enable */
#define EHCI_PORTSC_OVERCURRENTACTIVE      (1 << 4)  /* Bit 4: Overcurrent protection active */
#define EHCI_PORTSC_OVERCURRENTCHANGE      (1 << 5)  /* Bit 5: Change in overcurrent protection */
#define EHCI_PORTSC_FORCEPORTRESUME        (1 << 6)  /* Bit 6: Force port resume */
#define EHCI_PORTSC_SUSPEND                (1 << 7)  /* Bit 7: Suspend port */
#define EHCI_PORTSC_PORTRESET              (1 << 8)  /* Bit 8: Reset port */
                                                     /* Bit 9: Reserved */

#define EHCI_PORTSC_LINESTATUS_SHIFT       (10)      /* Bits 10-11: Line status */
#define EHCI_PORTSC_LINESTATUS_MASK        (0x3 << EHCI_PORTSC_LINESTATUS_SHIFT)
#  define EHCI_PORTSC_LINESTATUS_SE0       (0 << EHCI_PORTSC_LINESTATUS_SHIFT)
#  define EHCI_PORTSC_LINESTATUS_JSTATE    (1 << EHCI_PORTSC_LINESTATUS_SHIFT)
#  define EHCI_PORTSC_LINESTATUS_KSTATE    (2 << EHCI_PORTSC_LINESTATUS_SHIFT)
#  define EHCI_PORTSC_LINESTATUS_UNDEF     (3 << EHCI_PORTSC_LINESTATUS_SHIFT)

#define EHCI_PORTSC_PORTPOWER              (1 << 12) /* Bit 12: Port power switch */
#define EHCI_PORTSC_PORTOWNER              (1 << 13) /* Bit 13: Port ownership    */

#define EHCI_PORTSC_PORTINDICATOR_SHIFT    (14)      /* Bits 14-15: Port indicator control */
#define EHCI_PORTSC_PORTINDICATOR_MASK     (0x3 << EHCI_PORTSC_PORTINDICATOR_SHIFT)
#  define EHCI_PORTSC_PORTINDICATOR_OFF    (0 << EHCI_PORTSC_PORTINDICATOR_SHIFT)
#  define EHCI_PORTSC_PORTINDICATOR_AMBER  (1 << EHCI_PORTSC_PORTINDICATOR_SHIFT)
#  define EHCI_PORTSC_PORTINDICATOR_GREEN  (2 << EHCI_PORTSC_PORTINDICATOR_SHIFT)
#  define EHCI_PORTSC_PORTINDICATOR_UNDEF  (3 << EHCI_PORTSC_PORTINDICATOR_SHIFT)

#define EHCI_PORTSC_PORTTEST_SHIFT         (16)      /* Bits 16-19: Port test control */
#define EHCI_PORTSC_PORTTEST_MASK          (0xf << EHCI_PORTSC_PORTTEST_SHIFT)
#  define EHCI_PORTSC_PORTTEST_DIS         (0 << EHCI_PORTSC_PORTTEST_SHIFT)
#  define EHCI_PORTSC_PORTTEST_JSTATE      (1 << EHCI_PORTSC_PORTTEST_SHIFT)
#  define EHCI_PORTSC_PORTTEST_KSTATE      (2 << EHCI_PORTSC_PORTTEST_SHIFT)
#  define EHCI_PORTSC_PORTTEST_SE0NAK      (3 << EHCI_PORTSC_PORTTEST_SHIFT)
#  define EHCI_PORTSC_PORTTEST_PACKET      (4 << EHCI_PORTSC_PORTTEST_SHIFT)
#  define EHCI_PORTSC_PORTTEST_FORCEEN     (5 << EHCI_PORTSC_PORTTEST_SHIFT)

#define EHCI_PORTSC_WKCNNT_E               (1 << 20) /* Bit 20: Wake on connect enable */
#define EHCI_PORTSC_WKDSCNNT_E             (1 << 21) /* Bit 21: Wake on disconnect enable */
#define EHCI_PORTSC_WKOC_E                 (1 << 22) /* Bit 22: Wake on over-current enable */
                                                     /* Bits 23-31: Reserved */

#define EHCI_PORTSC_RWC_BITS               (EHCI_PORTSC_CONNECTSTSCHANGE   |  \
                                            EHCI_PORTSC_PORTENCHANGE       |  \
                                            EHCI_PORTSC_OVERCURRENTCHANGE)

/* Transfer Descriptors bit definitions *************************************/

/* Next link pointer; common for iTD, siTD, QH (section 3.3, 3.4, 3.6) */

#define EHCI_NEXTLINK_T                    (1 << 0)  /* Bit 0: Terminate */

#define EHCI_NEXTLINK_TYPE_SHIFT           (1)       /* Bits 1-2: Next pointer type */
#define EHCI_NEXTLINK_TYPE_MASK            (0x3 << EHCI_NEXTLINK_TYPE_SHIFT)
#  define EHCI_NEXTLINK_TYPE_ITD           (0 << EHCI_NEXTLINK_TYPE_SHIFT)
#  define EHCI_NEXTLINK_TYPE_QH            (1 << EHCI_NEXTLINK_TYPE_SHIFT)
#  define EHCI_NEXTLINK_TYPE_sITD          (2 << EHCI_NEXTLINK_TYPE_SHIFT)
#  define EHCI_NEXTLINK_TYPE_FSTN          (3 << EHCI_NEXTLINK_TYPE_SHIFT)
                                                     /* Bits 3-4: Reserved */

#define EHCI_NEXTLINK_ADDR_SHIFT           (5)       /* Bits 5-31: Memory address signals [31:5] */
#define EHCI_NEXTLINK_ADDR_MASK            (0x7ffffff << EHCI_NEXTLINK_ADDR_SHIFT)

/* Next element pointer; common for siTD back pointer & qTD next pointer */

#define EHCI_NEXTELEMENT_T                 (1 << 0)  /* Bit 0: Terminate */
                                                     /* Bits 1-4: Reserved */

#define EHCI_NEXTELEMENT_ADDR_SHIFT        (5)       /* Bits 5-31: Memory address signals [31:5] */
#define EHCI_NEXTELEMENT_ADDR_MASK         (0x7ffffff << EHCI_NEXTELEMENT_ADDR_SHIFT)

/* Queue Element Transfer Descriptor Offset (section 3.5) */

#define EHCI_QTD_TOKEN_STS_PING            (1 << 0)  /* Bit 0: Ping PID to endpoint */
#define EHCI_QTD_TOKEN_STS_SPLITSTATE      (1 << 1)  /* Bit 1: Split transaction state */
#define EHCI_QTD_TOKEN_STS_UFRAMEMISSED    (1 << 2)  /* Bit 2: Missed uFrame */
#define EHCI_QTD_TOKEN_STS_ERR             (1 << 3)  /* Bit 3: Transaction error */
#define EHCI_QTD_TOKEN_STS_BABBLE          (1 << 4)  /* Bit 4: Transaction babble detected */
#define EHCI_QTD_STS_DATBUFERR             (1 << 5)  /* Bit 5: Transaction data buffer error (overrun/underrun) */
#define EHCI_QTD_STS_HALTED                (1 << 6)  /* Bit 6: Transaction halted */
#define EHCI_QTD_STS_ACTIVE                (1 << 7)  /* Bit 7: Transaction active */

#define EHCI_QTD_TOKEN_PIDCODE_SHIFT       (8)       /* Bits 8-9: Token type */
#define EHCI_QTD_TOKEN_PIDCODE_MASK        (0x3 << EHCI_QTD_TOKEN_PIDCODE_SHIFT)
#  define  EHCI_QTD_TOKEN_PIDCODE_OUT      (0 << EHCI_QTD_TOKEN_PIDCODE_SHIFT)
#  define  EHCI_QTD_TOKEN_PIDCODE_IN       (1 << EHCI_QTD_TOKEN_PIDCODE_SHIFT)
#  define  EHCI_QTD_TOKEN_PIDCODE_SETUP    (2 << EHCI_QTD_TOKEN_PIDCODE_SHIFT)

#define EHCI_QTD_TOKEN_ERRCOUNTER_SHIFT    (10)      /* Bits 10-11: Error counter */
#define EHCI_QTD_TOKEN_ERRCOUNTER_MASK     (0x3 << EHCI_QTD_TOKEN_ERRCOUNTER_SHIFT)

#define EHCI_QTD_TOKEN_CPAGE_SHIFT         (12)      /* Bits 12-14: Current page */
#define EHCI_QTD_TOKEN_CPAGE_MASK          (0x7 << EHCI_QTD_TOKEN_CPAGE_SHIFT)

#define EHCI_QTD_TOKEN_IOC                 (1 << 15) /* Bit 15: Interrupt on complete */

#define EHCI_QTD_TOKEN_TRANSFERSIZE_SHIFT  (16)      /* Bits 16-30: Total bytes to transfer */
#define EHCI_QTD_TOKEN_TRANSFERSIZE_MASK   (0x7fff << EHCI_QTD_TOKEN_TRANSFERSIZE_SHIFT)

#define EHCI_QTD_TOKEN_DATATOGGLE          (1 << 31) /* Bit 31: Data toggle */

#define EHCI_QTD_BUFP_SHIFT                (12)      /* Bits 12-31: Memory address signals [31-12] */
#define EHCI_QTD_BUFP_MASK                 (0xfffff << EHCI_QTD_BUFP_SHIFT)

#define EHCI_QTD_BUFP0_CUROFFSET_SHIFT     (0)       /* Bits 0-11: Current byte offset */
#define EHCI_QTD_BUFP0_CUROFFSET_MASK      (0xfff << EHCI_QTD_BUFP0_CUROFFSET_SHIFT)

/* Isochronous (High-Speed) Transfer Descriptor Offsets (section 3.3) */

#define EHCI_ITD_TRANSACTION_OFFSET_SHIFT  (0)       /* Bits 0-11: Transaction buffer page offset */
#define EHCI_ITD_TRANSACTION_OFFSET_MASK   (0xfff << EHCI_ITD_TRANSACTION_OFFSET_SHIFT)

#define EHCI_ITD_TRANSACTION_PG_SHIFT      (12)      /* Bits 12-14: Buffer page select */
#define EHCI_ITD_TRANSACTION_PG_MASK       (0x7 << EHCI_ITD_TRANSACTION_OFFSET_SHIFT)

#define EHCI_ITD_TRANSACTION_IOC           (1 << 15) /* Bit 15: Interrupt on complete */

#define EHCI_ITD_TRANSACTION_LENGTH_SHIFT  (16)      /* Bits 16-27: Transaction length */
#define EHCI_ITD_TRANSACTION_LENGTH_MASK   (0xfff << EHCI_ITD_TRANSACTION_LENGTH_SHIFT)

#define EHCI_ITD_TRANSACTION_STS_ERR       (1 << 28) /* Bit 28: Transaction error */
#define EHCI_ITD_TRANSACTION_STS_BABBLE    (1 << 29) /* Bit 29: Transaction babble detected */
#define EHCI_ITD_TRANSACTION_STS_DATBUFERR (1 << 30) /* Bit 30: Transaction data buffer error (overrun/underrun) */
#define EHCI_ITD_TRANSACTION_STS_ACTIVE    (1 << 31) /* Bit 31: Transaction active */

#define EHCI_ITD_BUFP_SHIFT                (12)      /* Bits 12-31: Memory address signals [31-12] */
#define EHCI_ITD_BUFP_MASK                 (0xfffff << EHCI_ITD_BUFP_SHIFT)

#define EHCI_ITD_BUFP0_DEVADDR_SHIFT       (0)       /* Bits 0-6: Device address */
#define EHCI_ITD_BUFP0_DEVADDR_MASK        (0x7f << EHCI_ITD_BUFP0_DEVADDR_SHIFT)
                                                     /* Bit 7: Reserved */

#define EHCI_ITD_BUFP0_EP_SHIFT            (8)       /* Bits 8-11: Endpoint number */
#define EHCI_ITD_BUFP0_EP_MASK             (0xf << EHCI_ITD_BUFP0_EP_SHIFT)

#define EHCI_ITD_BUFP1_MAXPACKET_SHIFT     (0)       /* Bits 0-10: Maximum packet size */
#define EHCI_ITD_BUFP1_MAXPACKET_MASK      (0x7ff << EHCI_ITD_BUFP1_MAXPACKET_SHIFT)

#define EHCI_ITD_BUFP1_DIRECTION           (1 << 11) /* Bit 11: Direction, IN/OUT */
#  define EHCI_ITD_BUFP1_DIRECTION_OUT     (0 << 11)
#  define EHCI_ITD_BUFP1_DIRECTION_IN      (1 << 11)

#define EHCI_ITD_BUFP2_MULTI_SHIFT         (0)       /* Bits 0-1: Transactions per uFrame */
#define EHCI_ITD_BUFP2_MULTI_MASK          (0x3 << EHCI_ITD_BUFP2_MULTI_SHIFT)
                                                     /* Bits 2-11: Reserved */

/* Split Transaction Isochronous Transfer Descriptor Offsets (section 3.4) */

#define EHCI_SITD_EPCAP_DEVADDR_SHIFT      (0)       /* Bits 0-6: Device address */
#define EHCI_SITD_EPCAP_DEVADDR_MASK       (0x7f << EHCI_SITD_EPCAP_DEVADDR_SHIFT)
                                                     /* Bit 7: Reserved */

#define EHCI_SITD_EPCAP_EP_SHIFT           (8)       /* Bits 8-11: Endpoint number */
#define EHCI_SITD_EPCAP_EP_MASK            (0xf << EHCI_SITD_EPCAP_EP_SHIFT)
                                                     /* Bits 12-15: Reserved */

#define EHCI_SITD_EPCAP_HUBADDR_SHIFT      (0)       /* Bits 16-22: Hub address */
#define EHCI_SITD_EPCAP_HUBADDR_MASK       (0x7f << EHCI_SITD_EPCAP_HUBADDR_SHIFT)
                                                     /* Bit 23: Reserved */

#define EHCI_SITD_EPCAP_TTPORTNO_SHIFT     (24)      /* Bits 24-30: TT port number */
#define EHCI_SITD_EPCAP_TTPORTNO_MASK      (0x7f << EHCI_SITD_EPCAP_TTPORTNO_SHIFT)

#define EHCI_SITD_EPCAP_DIRECTION          (1 << 31) /* Bit 31: Direction, IN/OUT */
#  define EHCI_SITD_EPCAP_DIRECTION_OUT    (0 << 11)
#  define EHCI_SITD_EPCAP_DIRECTION_IN     (1 << 11)

#define EHCI_SITD_EPSCHED_SMASK_SHIFT      (0)       /* Bits 0-7: Split start uFrame mask */
#define EHCI_SITD_EPSCHED_SMASK_MASK       (0xff << EHCI_SITD_EPSCHED_SMASK_SHIFT)

#define EHCI_SITD_EPSCHED_CMASK_SHIFT      (8)       /* Bits 8-15: Split complete uFrame mask */
#define EHCI_SITD_EPSCHED_CMASK_MASK       (0xff << EHCI_SITD_EPSCHED_CMASK_SHIFT)
                                                     /* Bits 16-31: Reserved */

                                                     /* Bit 0: Reserved */
#define EHCI_SITD_STATE_STS_SPLITXSTATE    (1 << 1)  /* Bit 1: Split transaction state */
#define EHCI_SITD_STATE_STS_UFRAMEMISSED   (1 << 2)  /* Bit 2: Missed uFrame */
#define EHCI_SITD_STATE_STS_ERR            (1 << 3)  /* Bit 3: Transaction error */
#define EHCI_SITD_STATE_STS_BABBLE         (1 << 4)  /* Bit 4: Transaction babble detected */
#define EHCI_SITD_STATE_STS_DATBUFERR      (1 << 5)  /* Bit 5: Transaction data buffer error (overrun/underrun) */
#define EHCI_SITD_STATE_STS_TTERR          (1 << 6)  /* Bit 6: TT error */
#define EHCI_SITD_STATE_STS_ACTIVE         (1 << 7)  /* Bit 7: Transaction active */

#define EHCI_SITD_STATE_CPROGMASK_SHIFT    (8)       /* Bits 8-15: Split complete uFrame progress mask */
#define EHCI_SITD_STATE_CPROGMASK_MASK     (0xff << EHCI_SITD_STATE_CPROGMASK_SHIFT)

#define EHCI_SITD_STATE_TRANSFERSIZE_SHIFT (16)      /* Bits 16-25: Total bytes to transfer */
#define EHCI_SITD_STATE_TRANSFERSIZE_MASK  (0x3ff << EHCI_SITD_STATE_TRANSFERSIZE_SHIFT)
                                                     /* Bits 26-29: Reserved */

#define EHCI_SITD_STATE_PAGESELECT         (1 << 30) /* Bit 30: Page select */
#  define EHCI_SITD_STATE_PAGESELECT_0     (0 << 30)
#  define EHCI_SITD_STATE_PAGESELECT_1     (1 << 30)

#define EHCI_SITD_STATE_IOC                (1 << 31) /* Bit 31: Interrupt on complete */

#define EHCI_SITD_BUFP_SHIFT               (12)      /* Bits 12-31: Memory address signals [31-12] */
#define EHCI_SITD_BUFP_MASK                (0xfffff << EHCI_SITD_BUFP_SHIFT)

#define EHCI_SITD_BUFP0_CUROFFSET_SHIFT    (0)       /* Bits 0-11: Current byte offset */
#define EHCI_SITD_BUFP0_CUROFFSET_MASK     (0xfff << EHCI_SITD_BUFP0_CUROFFSET_SHIFT)

#define EHCI_SITD_BUFP1_TCOUNT_SHIFT       (0)       /* Bits 0-2: Transaction count */
#define EHCI_SITD_BUFP1_TCOUNT_MASK        (0x7 << EHCI_SITD_BUFP1_TCOUNT_SHIFT)

#define EHCI_SITD_BUFP1_TP_SHIFT           (3)       /* Bits 3-4: Transaction position */
#define EHCI_SITD_BUFP1_TP_MASK            (0x3 << EHCI_SITD_BUFP1_TP_SHIFT)
#  define EHCI_SITD_BUFP1_TP_ALL           (0 << EHCI_SITD_BUFP1_TP_SHIFT)
#  define EHCI_SITD_BUFP1_TP_BEGIN         (1 << EHCI_SITD_BUFP1_TP_SHIFT)
#  define EHCI_SITD_BUFP1_TP_MID           (2 << EHCI_SITD_BUFP1_TP_SHIFT)
#  define EHCI_SITD_BUFP1_TP_END           (3 << EHCI_SITD_BUFP1_TP_SHIFT)
                                                     /* Bits 5-11: Reserved */

/* Queue Head Offsets (section 3.6) */

#define EHCI_QH_EPCHAR_DEVADDR_SHIFT       (0)       /* Bits 0-6: Device address */
#define EHCI_QH_EPCHAR_DEVADDR_MASK        (0x7f << EHCI_QH_EPCHAR_DEVADDR_SHIFT)
#  define EHCI_QH_EPCHAR_DEVADDR(addr)     (((addr) << EHCI_QH_EPCHAR_DEVADDR_SHIFT) & EHCI_QH_EPCHAR_DEVADDR_MASK)

#define EHCI_QH_EPCHAR_I                   (1 << 7)  /* Bit 7: Inactivate in next transaction */

#define EHCI_QH_EPCHAR_EP_SHIFT            (8)       /* Bits 8-11: Endpoint number */
#define EHCI_QH_EPCHAR_EP_MASK             (0xf << EHCI_QH_EPCHAR_EP_SHIFT)
#  define EHCI_QH_EPCHAR_EP(epnum)         (((epnum) << EHCI_QH_EPCHAR_EP_SHIFT) & EHCI_QH_EPCHAR_EP_MASK)

#define EHCI_QH_EPCHAR_EPSPEED_SHIFT       (12)      /* Bits 12-13: Endpoint speed */
#define EHCI_QH_EPCHAR_EPSPEED_MASK        (0x3 << EHCI_QH_EPCHAR_EPSPEED_SHIFT)
#  define EHCI_QH_EPCHAR_EPSPEED_FS        (0 << EHCI_QH_EPCHAR_EPSPEED_SHIFT)
#  define EHCI_QH_EPCHAR_EPSPEED_LS        (1 << EHCI_QH_EPCHAR_EPSPEED_SHIFT)
#  define EHCI_QH_EPCHAR_EPSPEED_HS        (2 << EHCI_QH_EPCHAR_EPSPEED_SHIFT)
#  define EHCI_QH_EPCHAR_EPSPEED(speed)    (((speed) << EHCI_QH_EPCHAR_EPSPEED_SHIFT) & EHCI_QH_EPCHAR_EPSPEED_MASK)

#define EHCI_QH_EPCHAR_DTC                 (1 << 14) /* Bit 14: Data toggle control */
#define EHCI_QH_EPCHAR_H                   (1 << 15) /* Bit 15: Head of reclamation list */

#define EHCI_QH_EPCHAR_MAXPACKET_SHIFT     (16)      /* Bits 16-26: Maximum packet length */
#define EHCI_QH_EPCHAR_MAXPACKET_MASK      (0x7ff << EHCI_QH_EPCHAR_MAXPACKET_SHIFT)
#  define EHCI_QH_EPCHAR_MAXPACKET(size)   (((size) << EHCI_QH_EPCHAR_MAXPACKET_SHIFT) & EHCI_QH_EPCHAR_MAXPACKET_MASK)

#define EHCI_QH_EPCHAR_C                   (1 << 27) /* Bit 27: Control endpoint */

#define EHCI_QH_EPCHAR_NAKRL_SHIFT         (28)      /* Bits 28-31: NAK count reload */
#define EHCI_QH_EPCHAR_NAKRL_MASK          (0xf << EHCI_QH_EPCHAR_NAKRL_SHIFT)

#define EHCI_QH_EPCAP_SMASK_SHIFT          (0)       /* Bits 0-7: Split start uFrame mask */
#define EHCI_QH_EPCAP_SMASK_MASK           (0xff << EHCI_QH_EPCAP_SMASK_SHIFT)

#define EHCI_QH_EPCAP_CMASK_SHIFT          (8)       /* Bits 8-15: Split complete uFrame mask */
#define EHCI_QH_EPCAP_CMASK_MASK           (0xff << EHCI_QH_EPCAP_CMASK_SHIFT)

#define EHCI_QH_EPCAP_HUDADDR_SHIFT        (16)      /* Bits 16-22: Hub address */
#define EHCI_QH_EPCAP_HUDADDR_MASK         (0x7f << EHCI_QH_EPCAP_HUDADDR_SHIFT)

#define EHCI_QH_EPCAP_PORT_SHIFT           (23)      /* Bits 23-29: Port number */
#define EHCI_QH_EPCAP_PORT_MASK            (0x7f << EHCI_QH_EPCAP_PORT_SHIFT)

#define EHCI_QH_EPCAP_MULT_SHIFT           (30)      /* Bits 30-31: High-bandwidth pipe multiplier */
#define EHCI_QH_EPCAP_MULT_MASK            (0x3 << EHCI_QH_EPCAP_MULT_SHIFT)

#define EHCI_QH_ALTNEXTP_NAKCOUNT_SHIFT    (1)       /* Bits 1-4: NAK counter */
#define EHCI_QH_ALTNEXTP_NAKCOUNT_MASK     (0xf << EHCI_QH_ALTNEXTP_NAKCOUNT_SHIFT)

#define EHCI_QH_BUFP1_CPROGMASK_SHIFT      (0)       /* Bits 0-7: Split complete uFrame progress mask */
#define EHCI_QH_BUFP1_CPROGMASK_MASK       (0xff << EHCI_QH_BUFP1_CPROGMASK_SHIFT)

#define EHCI_QH_BUFP2_FRAMETAG_SHIFT       (0)       /* Bits 0-4: Split transaction frame tag */
#define EHCI_QH_BUFP2_FRAMETAG_MASK        (0x1f << EHCI_QH_BUFP2_FRAMETAG_SHIFT)

#define EHCI_QH_BUFP2_SBYTES_SHIFT         (5)       /* Bits 5-11: Transferred bytes */
#define EHCI_QH_BUFP2_SBYTES_MASK          (0x7f << EHCI_QH_BUFP2_SBYTES_SHIFT)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Queue Element Transfer Descriptor (section 3.5) */

struct ehci_qtd_s
{
  volatile uint32_t nextp;
  volatile uint32_t altnextp;
  volatile uint32_t token;
  volatile uint32_t bufp[5];
};

/* Isochronous (High-Speed) Transfer Descriptor (section 3.3) */

struct ehci_itd_s
{
  volatile uint32_t nextp;
  volatile uint32_t transaction[8];
  volatile uint32_t bufp[7];
};

/* Split Transaction Isochronous Transfer Descriptor (section 3.4) */

struct ehci_sitd_s
{
  volatile uint32_t nextp;
  volatile uint32_t epcap;
  volatile uint32_t epsched;
  volatile uint32_t state;
  volatile uint32_t bufp[2];
  volatile uint32_t backp;
};

/* Queue Head (section 3.6) */

struct ehci_qh_s
{
  volatile uint32_t nextp;
  volatile uint32_t epchar;
  volatile uint32_t epcap;
  volatile uint32_t curqtdp;
  struct ehci_qtd_s overlay;
};

/* Periodic Frame Span Traversal Node (FSTN) (section 3.7) */

struct ehci_fstn_s
{
  volatile uint32_t nextp;
  volatile uint32_t backp;
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

#endif /* __INCLUDE_NUTTX_USB_EHCI_H */
