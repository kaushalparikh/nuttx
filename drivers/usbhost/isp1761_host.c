/*******************************************************************************
 * drivers/usbhost/isp1761.c
 *
 *   Copyright (C) 2010-2011 Gregory Nutt. All rights reserved.
 *   Author: Kaushal Parikh <kaushal@dspworks.in>
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
 *******************************************************************************/

/*******************************************************************************
 * Included Files
 *******************************************************************************/

#include <nuttx/version.h>
#include <nuttx/config.h>
#include <nuttx/kmalloc.h>

#include <string.h>
#include <sys/types.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/usb/usb.h>
#include <nuttx/usb/usbhost.h>
#include <nuttx/usb/ehci.h>
#include <nuttx/usb/hub.h>
#include <nuttx/usb/isp1761_host.h>

/*******************************************************************************
 * Definitions
 *******************************************************************************/

#define CONFIG_USBHOST_TDBUFSIZE  (128)

/*******************************************************************************
 * Private Types
 *******************************************************************************/

struct ehci_qtdlist_s
{
  struct ehci_qtd_s     *qtd;
  struct ehci_qtdlist_s *flink;
};

struct ehci_ep_s
{
  /* Pointer to HW iTD, siTD or qH */

  union
    {
      uint32_t           *nextp;
      struct ehci_itd_s  *itd;
      struct ehci_sitd_s *sitd;
      struct ehci_qh_s   *qh;
    } type;

  /* Software list of qH/iTD/siTD */

  struct ehci_ep_s      *flink;

  /* qTD list in-case of qH */

  struct ehci_qtd_s     *qtd;

  /* Software list of qTD */

  struct ehci_qtdlist_s qtdlist;

  /* Transfer direction */

  bool                  in;

  /* Endpoint polling interval and start frame */

  uint8_t               interval;
  int16_t               startframe;
};

struct ehci_driver_s
{
  /* Driver APIs */

  struct usbhost_driver_s drvr;
};

/*******************************************************************************
 * Private Function Prototypes
 *******************************************************************************/

static inline uint16_t ehci_getle16(const uint8_t *val);

static void ehci_putle16(uint8_t *dest, uint16_t val);

static int ehci_reset(void);

static int ehci_run(void);

static int ehci_stop(void);

static int ehci_epalloc(FAR struct usbhost_driver_s *drvr,
                        const FAR struct usbhost_epdesc_s *epdesc, 
                        usbhost_ep_t *ep);

static int ehci_epfree(FAR struct usbhost_driver_s *drvr, usbhost_ep_t ep);

static int ehci_ep0config(FAR struct usbhost_driver_s *drvr,
                          usbhost_ep_t ep0, uint8_t funcaddr,
                          uint16_t maxpacketsize);

static  int ehci_alloc(FAR struct usbhost_driver_s *drvr,
                       FAR uint8_t **buffer, FAR size_t *maxlen);

static  int ehci_free(FAR struct usbhost_driver_s *drvr, FAR uint8_t *buffer);

static  int ehci_ioalloc(FAR struct usbhost_driver_s *drvr,
                         FAR uint8_t **buffer, size_t size);

static  int ehci_iofree(FAR struct usbhost_driver_s *drvr, FAR uint8_t *buffer);

static int ehci_rhctrl(FAR struct usbhost_driver_s *drvr,
                       FAR struct usbhost_transfer_s *xfer,
                       FAR const struct usb_ctrlreq_s *cmd);

static int ehci_rhstatus(FAR struct usbhost_driver_s *drvr,
                         FAR struct usbhost_transfer_s *xfer);

static int isp1761_hw_modectrl(void);

static int isp1761_handshake(uint32_t addr, uint32_t mask,
                             uint32_t result, int32_t timeout);

static int isp1761_hw_reset(void);

static void isp1761_irq_init(void);

static void isp1761_ptd_init(void);

static void isp1761_irq_enable(void);

static void isp1761_irq_disable(void);

/*******************************************************************************
 * Private Data
 *******************************************************************************/

static const uint8_t g_rh_devdesc[] =
{
  0x12,       /*  uint8_t  bLength */
  0x01,       /*  uint8_t  bDescriptorType (Device) */
  0x00, 0x02, /*  uint16_t bcdUSB (v2.0) */
  
  0x09,	      /*  uint8_t  bDeviceClass (Hub) */
  0x00,	      /*  uint8_t  bDeviceSubClass */
  0x00,       /*  uint8_t  bDeviceProtocol (USB 2.0, no TT) */
  0x40,       /*  uint8_t  bMaxPacketSize0 (64 Bytes) */
  
  0x6b, 0x1d, /*  uint16_t idVendor; (Linux Foundation) */
  0x02, 0x00, /*  uint16_t idProduct (Device 0x0002) */
  CONFIG_VERSION_MAJOR, CONFIG_VERSION_MINOR,
              /*  uint16_t bcdDevice */
  
  0x03,       /*  uint8_t  iManufacturer */
  0x02,       /*  uint8_t  iProduct */
  0x01,       /*  uint8_t  iSerialNumber */
  0x01        /*  uint8_t  bNumConfigurations */
};

static const uint8_t g_rh_configdesc[] =
{
  /* Configuration */
  
  0x09,       /*  uint8_t  bLength */
  0x02,       /*  uint8_t  bDescriptorType (Configuration) */
  0x19, 0x00, /*  uint16_t wTotalLength */
  0x01,       /*  uint8_t  bNumInterfaces (1) */
  0x01,       /*  uint8_t  bConfigurationValue */
  0x00,       /*  uint8_t  iConfiguration */
  0xc0,       /*  uint8_t  bmAttributes  */
  0x00,       /*  uint8_t  MaxPower */
      
  /* Interface
   *
   * USB 1.1:
   * USB 2.0, single TT organization (mandatory):
   *  one interface, protocol 0
   *
   * USB 2.0, multiple TT organization (optional):
   *  two interfaces, protocols 1 (like single TT)
   *  and 2 (multiple TT mode) ... config is
   *  sometimes settable
   *  NOT IMPLEMENTED
   */
  
  0x09,       /*  uint8_t bLength; */
  0x04,       /*  uint8_t bDescriptorType (Interface) */
  0x00,       /*  uint8_t bInterfaceNumber */
  0x00,       /*  uint8_t bAlternateSetting */
  0x01,       /*  uint8_t bNumEndpoints */
  0x09,       /*  uint8_t bInterfaceClass (HUB_CLASSCODE) */
  0x00,       /*  uint8_t bInterfaceSubClass */
  0x00,       /*  uint8_t bInterfaceProtocol (usb1.1 or single tt) */
  0x00,       /*  uint8_t iInterface */
     
  /* Endpoint 
   *  Status change
   */
  
  0x07,       /*  uint8_t bLength */
  0x05,       /*  uint8_t bDescriptorType (Endpoint) */
  0x81,       /*  uint8_t bEndpointAddress (IN Endpoint 1) */
  0x03,       /*  uint8_t bmAttributes (Interrupt) */
  (USBHUB_MAX_PORTS + 8) / 8, 0x00,
              /*  uint8_t wMaxPacketSize */
  0x0c        /*  uint8_t bInterval (256ms -- usb 2.0 spec) */
};

static struct ehci_driver_s g_ehci =
{
  .drvr = 
    {
      .roothub      = NULL,
      .speed        = USB_SPEED_HIGH,
      .wait         = NULL, 
      .enumerate    = NULL, 
      .ep0configure = ehci_ep0config, 
      .epalloc      = ehci_epalloc, 
      .epfree       = ehci_epfree, 
      .alloc        = ehci_alloc,
      .free         = ehci_free, 
      .ioalloc      = ehci_ioalloc, 
      .iofree       = ehci_iofree, 
      .ctrlin       = NULL, 
      .ctrlout      = NULL, 
      .transfer     = NULL, 
      .disconnect   = NULL,  
      .rhctrl       = ehci_rhctrl,
      .rhstatus     = ehci_rhstatus,
    },
};

static struct usbhost_hal_s g_hal;

static uint32_t g_irq_count;
static uint32_t g_irq_cause;

/*******************************************************************************
 * Public Data
 *******************************************************************************/

/*******************************************************************************
 * Private Functions
 *******************************************************************************/


/****************************************************************************
 * Name: ehci_getle16
 *
 * Description:
 *   Get a (possibly unaligned) 16-bit little endian value.
 *
 * Input Parameters:
 *   val - A pointer to the first byte of the little endian value.
 *
 * Returned Values:
 *   A uint16_t representing the whole 16-bit integer value
 *
 ****************************************************************************/

static inline uint16_t ehci_getle16(const uint8_t *val)
{
  return (uint16_t)val[1] << 8 | (uint16_t)val[0];
}

/****************************************************************************
 * Name: ehci_putle16
 *
 * Description:
 *   Put a (possibly unaligned) 16-bit little endian value.
 *
 * Input Parameters:
 *   dest - A pointer to the first byte to save the little endian value.
 *   val - The 16-bit value to be saved.
 *
 * Returned Values:
 *   None
 *
 ****************************************************************************/

static void ehci_putle16(uint8_t *dest, uint16_t val)
{
  dest[0] = val & 0xff; /* Little endian means LS byte first in byte stream */
  dest[1] = val >> 8;
}

/*******************************************************************************
 * Name: ehci_reset
 *
 * Description:
 *   Reset EHCI host controller
 *
 *******************************************************************************/
 
static int ehci_reset(void)
{
  int ret;
  uint32_t regval;
  
  regval = g_hal.getreg32(ISP1761_OPERATIONAL_OFFSET(EHCI_USBCMD_OFFSET));
  regval |= EHCI_USBCMD_HCRESET;
  g_hal.putreg32(regval, ISP1761_OPERATIONAL_OFFSET(EHCI_USBCMD_OFFSET));

  ret = isp1761_handshake(ISP1761_OPERATIONAL_OFFSET(EHCI_USBCMD_OFFSET),
                          EHCI_USBCMD_HCRESET, 0, 250);
  if (ret != OK)
    {
      udbg("failed to reset controller\n");
    }

  return ret;
}

/*******************************************************************************
 * Name: ehci_run
 *
 * Description:
 *   Run EHCI host controller
 *
 *******************************************************************************/
static int ehci_run(void)
{
  int ret;
  uint32_t regval;
  
  /* Start Host controller */

  regval = g_hal.getreg32(ISP1761_OPERATIONAL_OFFSET(EHCI_USBCMD_OFFSET));
  regval |= EHCI_USBCMD_RS;
  g_hal.putreg32(regval, ISP1761_OPERATIONAL_OFFSET(EHCI_USBCMD_OFFSET));

  ret = isp1761_handshake(ISP1761_OPERATIONAL_OFFSET(EHCI_USBCMD_OFFSET),
                          EHCI_USBCMD_RS, EHCI_USBCMD_RS, 3);
  if (ret != OK)
    {
      udbg("failed to run controller\n");
      return ret;
    }
  
  /* Route all ports to EHCI controller */

  g_hal.putreg32(EHCI_CONFIGFLAG_CF, ISP1761_OPERATIONAL_OFFSET(EHCI_CONFIGFLAG_OFFSET));
  ret = isp1761_handshake(ISP1761_OPERATIONAL_OFFSET(EHCI_CONFIGFLAG_OFFSET),
                          EHCI_CONFIGFLAG_CF, EHCI_CONFIGFLAG_CF, 1);
  if (ret != OK)
    {
      udbg("failed to set config flag\n");
      return ret;
    }

  return OK;
}

/*******************************************************************************
 * Name: ehci_stop
 *
 * Description:
 *   Run EHCI host controller
 *
 *******************************************************************************/
static int ehci_stop(void)
{
  int ret;
  uint32_t regval;
  
  /* Stop Host controller */

  regval = g_hal.getreg32(ISP1761_OPERATIONAL_OFFSET(EHCI_USBCMD_OFFSET));
  regval &= (~EHCI_USBCMD_RS);
  g_hal.putreg32(regval, ISP1761_OPERATIONAL_OFFSET(EHCI_USBCMD_OFFSET));

  ret = isp1761_handshake(ISP1761_OPERATIONAL_OFFSET(EHCI_USBCMD_OFFSET),
                          EHCI_USBCMD_RS, 0, 2);
  if (ret != OK)
    {
      udbg("failed to stop controller\n");
    }
  
  /* Route all ports to ?? */

  g_hal.putreg32(0, ISP1761_OPERATIONAL_OFFSET(EHCI_CONFIGFLAG_OFFSET));
  
  return OK;
}

/*******************************************************************************
 * Name: ehci_epalloc
 *
 * Description:
 *   Allocate end-point
 *
 *******************************************************************************/
 
static int ehci_epalloc(FAR struct usbhost_driver_s *drvr,
                        const FAR struct usbhost_epdesc_s *epdesc,
                        usbhost_ep_t *ep)
{
  struct ehci_ep_s *ehci_ep;
  int ret = -ENOMEM;

  ehci_ep = kmalloc(sizeof(struct ehci_ep_s));
  if (ehci_ep != NULL)
    {
      ehci_ep->flink = NULL;
      ehci_ep->qtd   = NULL;
      
      ehci_ep->qtdlist.qtd   = NULL;
      ehci_ep->qtdlist.flink = NULL;

      if (epdesc->xfrtype == USB_EP_ATTR_XFER_ISOC)
        {
        }
      else
        {
          ehci_ep->type.qh = kmalloc(sizeof(struct ehci_qh_s));

          if (ehci_ep->type.qh != NULL)
            {
              ehci_ep->type.qh->nextp  = EHCI_NEXTLINK_TYPE_QH;
              ehci_ep->type.qh->epchar = EHCI_QH_EPCHAR_DEVADDR(epdesc->devclass->addr) |
                                         EHCI_QH_EPCHAR_EP(epdesc->addr) |
                                         EHCI_QH_EPCHAR_MAXPACKET(epdesc->mxpacketsize);

              ehci_ep->in         = epdesc->in;
              ehci_ep->interval   = epdesc->interval;
              ehci_ep->startframe = -1;

              *ep = ehci_ep;
              ret = OK;

              udbg("Total endpoint memory %d+%d\n", sizeof(struct ehci_ep_s),
                                                    sizeof(struct ehci_qh_s));
            }
          else
            {
              kfree(ehci_ep);
            }
        }
    }
  
  return ret;
}

/*******************************************************************************
 * Name: ehci_epfree
 *
 * Description:
 *   Free end-point
 *
 *******************************************************************************/
 
static int ehci_epfree(FAR struct usbhost_driver_s *drvr, usbhost_ep_t ep)
{
  struct ehci_ep_s *ehci_ep = (struct ehci_ep_s *)ep;

  if (ehci_ep->type.nextp != NULL)
    {
      kfree(ehci_ep->type.nextp);
      ehci_ep->type.nextp = NULL;
    }

  kfree(ehci_ep);
  
  return OK;
}

/*******************************************************************************
 * Name: ehci_ep0config
 *
 * Description:
 *   Configure control end-point packet size and address
 *
 *******************************************************************************/

static int ehci_ep0config(FAR struct usbhost_driver_s *drvr,
                          usbhost_ep_t ep0, uint8_t funcaddr,
                          uint16_t maxpacketsize)
{
  struct ehci_qh_s *qh = ((struct ehci_ep_s *)ep0)->type.qh;

  qh->epchar &= (~(EHCI_QH_EPCHAR_DEVADDR_MASK | EHCI_QH_EPCHAR_MAXPACKET_MASK));

  qh->epchar |= (EHCI_QH_EPCHAR_DEVADDR(funcaddr) |
                 EHCI_QH_EPCHAR_MAXPACKET(maxpacketsize));

  return OK;
}

/*******************************************************************************
 * Name: ehci_alloc
 *
 * Description:
 *   Configure control end-point packet size and address
 *
 *******************************************************************************/

static  int ehci_alloc(FAR struct usbhost_driver_s *drvr,
                       FAR uint8_t **buffer, FAR size_t *maxlen)
{
  int ret = -ENOMEM;

  *buffer = kmalloc(CONFIG_USBHOST_TDBUFSIZE);

  if (*buffer != NULL)
    {
      *maxlen = CONFIG_USBHOST_TDBUFSIZE;
      ret = OK;
    }

  return ret;
}

/*******************************************************************************
 * Name: ehci_free
 *
 * Description:
 *   Configure control end-point packet size and address
 *
 *******************************************************************************/

static int ehci_free(FAR struct usbhost_driver_s *drvr, FAR uint8_t *buffer)
{
  kfree(buffer);
  return OK;
}

/*******************************************************************************
 * Name: ehci_ioalloc
 *
 * Description:
 *   Configure control end-point packet size and address
 *
 *******************************************************************************/

static  int ehci_ioalloc(FAR struct usbhost_driver_s *drvr,
                         FAR uint8_t **buffer, size_t size)
{
  int ret = -ENOMEM;

  *buffer = kmalloc(size);

  if (*buffer != NULL)
    {
      ret = OK;
    }

  return ret;
}

/*******************************************************************************
 * Name: ehci_iofree
 *
 * Description:
 *   Configure control end-point packet size and address
 *
 *******************************************************************************/

static int ehci_iofree(FAR struct usbhost_driver_s *drvr, FAR uint8_t *buffer)
{
  kfree(buffer);
  return OK;
}

/*******************************************************************************
 * Name: ehci_rhctrl
 *
 * Description:
 *   Root hub control
 *
 *******************************************************************************/
 
static int ehci_rhctrl(FAR struct usbhost_driver_s *drvr,
                       FAR struct usbhost_transfer_s *xfer,
                       FAR const struct usb_ctrlreq_s *cmd)
{
  uint32_t regval;
  uint16_t typereq, value, index, length;
  int ret = OK;

  typereq = (cmd->type << 8) | cmd->req;
  value   = ehci_getle16(cmd->value);
  index   = ehci_getle16(cmd->index);
  length  = ehci_getle16(cmd->len);
  
  switch (typereq)
    {
      /* Device IN request, get descriptor */
      
      case ((USB_REQ_DIR_IN | USB_REQ_TYPE_STANDARD | USB_REQ_RECIPIENT_DEVICE) << 8) |
           USB_REQ_GETDESCRIPTOR:
        {
          switch ((value >> 8) & 0xff)
            {
              case USB_DESC_TYPE_DEVICE:
                {
                  memcpy(xfer->buffer, g_rh_devdesc, length);
                  break;
                }

              case USB_DESC_TYPE_CONFIG:
                {
                  memcpy(xfer->buffer, g_rh_configdesc, length);
                  break;
                }
              
              default:
                udbg("unknown root hub get descriptor request\n");
                break;
            }
          
          break;
        }

      /* Device set address */

      case ((USB_REQ_DIR_OUT | USB_REQ_TYPE_STANDARD | USB_REQ_RECIPIENT_DEVICE) << 8) |
           USB_REQ_SETADDRESS:
        {
          udbg("root hub address %d\n", value);
          break;
        }

      /* Device set configuration */

      case ((USB_REQ_DIR_OUT | USB_REQ_TYPE_STANDARD | USB_REQ_RECIPIENT_DEVICE) << 8) |
           USB_REQ_SETCONFIGURATION:
        {
          udbg("root hub configuration %d\n", value);
          break;
        }

      /* Class request, get hub descriptor */
      
      case ((USB_REQ_DIR_IN | USB_REQ_TYPE_CLASS | USB_REQ_RECIPIENT_DEVICE) << 8) |
           USB_REQ_GETDESCRIPTOR:
        {
          struct usb_hubdesc_s *hubdesc = (struct usb_hubdesc_s *)xfer->buffer;
          uint16_t characteristics;

          hubdesc->len = USB_SIZEOF_HUBDESC;
          hubdesc->type = USB_DESC_TYPE_HUB;
          
          regval = g_hal.getreg32(EHCI_HCSPARAMS_OFFSET);
          hubdesc->nports = (regval & EHCI_HCSPARAMS_NPORTS_MASK) >> EHCI_HCSPARAMS_NPORTS_SHIFT;

          udbg("root hub number of ports %d\n", hubdesc->nports);

          if (regval & EHCI_HCSPARAMS_PPC)
            {
              characteristics = 0x0009;
            }
          else
            {
              characteristics = 0x000a;
            }
          ehci_putle16(hubdesc->characteristics, characteristics);

          hubdesc->pwrondelay  = 10;
          hubdesc->ctrlcurrent = 0;

          hubdesc->devattached = 0;
          hubdesc->pwrctrlmask = 0xff;

          break;
        }

      /* Hub request, set port feature */

      case ((USB_REQ_DIR_OUT | USB_REQ_TYPE_CLASS | USB_REQ_RECIPIENT_OTHER) << 8) |
           USB_REQ_SETFEATURE:
        {
          regval = g_hal.getreg32(ISP1761_OPERATIONAL_OFFSET(EHCI_PORTSC_OFFSET(index)));
          regval &= (~EHCI_PORTSC_RWC_BITS);
                  
          switch (value)
            {
              case USBHUB_PORT_FEAT_POWER:
                {
                  regval |= EHCI_PORTSC_PORTPOWER;
                  g_hal.putreg32(regval, ISP1761_OPERATIONAL_OFFSET(EHCI_PORTSC_OFFSET(index)));
                  up_mdelay(2);
                  
                  udbg("root hub port %d power on\n", index);

                  break;
                }

              case USBHUB_PORT_FEAT_RESET:
                {
                  regval |= EHCI_PORTSC_PORTRESET;
                  regval &= (~EHCI_PORTSC_PORTEN);
                  g_hal.putreg32(regval, ISP1761_OPERATIONAL_OFFSET(EHCI_PORTSC_OFFSET(index)));
                  
                  udbg("root hub port %d reset\n", index);

                  break;
                }

              default:
                udbg("root hub unknown set feature request\n");
                break;              
            }

          break;
        }

      /* Hub request, clear port feature */

      case ((USB_REQ_DIR_OUT | USB_REQ_TYPE_CLASS | USB_REQ_RECIPIENT_OTHER) << 8) |
           USB_REQ_CLEARFEATURE:
        {
          regval = g_hal.getreg32(ISP1761_OPERATIONAL_OFFSET(EHCI_PORTSC_OFFSET(index)));
          regval &= (~EHCI_PORTSC_RWC_BITS);

          switch (value)
            {
              case USBHUB_PORT_FEAT_POWER:
                {
                  regval &= (~EHCI_PORTSC_PORTPOWER);
                  g_hal.putreg32(regval, ISP1761_OPERATIONAL_OFFSET(EHCI_PORTSC_OFFSET(index)));
                  up_mdelay(2);
                  
                  udbg("root hub port %d power off\n", index);
                  
                  break;
                }

              case USBHUB_PORT_FEAT_CCONNECTION:
                {
                  regval |= EHCI_PORTSC_CONNECTSTSCHANGE;
                  g_hal.putreg32(regval, ISP1761_OPERATIONAL_OFFSET(EHCI_PORTSC_OFFSET(index)));
                  
                  break;
                }              

              case USBHUB_PORT_FEAT_CENABLE:
                {
                  regval |= EHCI_PORTSC_PORTENCHANGE;
                  g_hal.putreg32(regval, ISP1761_OPERATIONAL_OFFSET(EHCI_PORTSC_OFFSET(index)));
                  
                  break;
                }

              case USBHUB_PORT_FEAT_COVER_CURRENT:
                {
                  regval |= EHCI_PORTSC_OVERCURRENTCHANGE;
                  g_hal.putreg32(regval, ISP1761_OPERATIONAL_OFFSET(EHCI_PORTSC_OFFSET(index)));
                  
                  break;
                }

              case USBHUB_PORT_FEAT_CRESET:
                {
                  break;
                }

              default:
                udbg("root hub unknown clear feature request\n");
                break;
            }
          
          break;
        }

      /* Hub request, get port status */

      case ((USB_REQ_DIR_IN | USB_REQ_TYPE_CLASS | USB_REQ_RECIPIENT_OTHER) << 8) |
           USB_REQ_GETSTATUS:
        {
          struct usb_portstatus_s *portstatus = (struct usb_portstatus_s *)xfer->buffer;
          uint16_t status = 0;
          uint16_t change = 0;
          
          regval = g_hal.getreg32(ISP1761_OPERATIONAL_OFFSET(EHCI_PORTSC_OFFSET(index)));
          
          if (regval & EHCI_PORTSC_CONNECTSTSCHANGE)
            {
              change |= USBHUB_PORT_STAT_CCONNECTION;
            }
          if (regval & EHCI_PORTSC_PORTENCHANGE)
            {
              change |= USBHUB_PORT_STAT_CENABLE;
            }
          if (regval & EHCI_PORTSC_OVERCURRENTCHANGE)
            {
              change |= USBHUB_PORT_STAT_COVERCURRENT;
            }

          if (regval & EHCI_PORTSC_PORTRESET)
            {
              change |= USBHUB_PORT_STAT_CRESET;
              regval &= (~(EHCI_PORTSC_PORTRESET | EHCI_PORTSC_RWC_BITS));
              g_hal.putreg32(regval, ISP1761_OPERATIONAL_OFFSET(EHCI_PORTSC_OFFSET(index)));

              ret = isp1761_handshake(ISP1761_OPERATIONAL_OFFSET(EHCI_PORTSC_OFFSET(index)),
                                      EHCI_PORTSC_PORTRESET, 0, 1);
              
              regval = g_hal.getreg32(ISP1761_OPERATIONAL_OFFSET(EHCI_PORTSC_OFFSET(index)));
            }
          
          if (regval & EHCI_PORTSC_CURRENTCONNECTSTS)
            {
              status |= (USBHUB_PORT_STAT_CONNECTION | USBHUB_PORT_STAT_HIGH_SPEED);
            }
          if (regval & EHCI_PORTSC_SUSPEND)
            {
              status |= USBHUB_PORT_STAT_SUSPEND;
            }
          if (regval & EHCI_PORTSC_OVERCURRENTACTIVE)
            {
              status |= USBHUB_PORT_STAT_OVERCURRENT;
            }
          if (regval & EHCI_PORTSC_PORTPOWER)
            {
              status |= USBHUB_PORT_STAT_POWER;
            }
          if (regval & EHCI_PORTSC_PORTEN)
            {
              status |= USBHUB_PORT_STAT_ENABLE;
            }

          ehci_putle16(portstatus->status, status);
          ehci_putle16(portstatus->change, change);

          break;
        }

      default:
        udbg("root hub unknown request\n");
        break;
    }

  xfer->status = ret;
  xfer->callback(xfer);

  return ret;
}

/*******************************************************************************
 * Name: ehci_rhstatus
 *
 * Description:
 *   Root hub control
 *
 *******************************************************************************/
 
static int ehci_rhstatus(FAR struct usbhost_driver_s *drvr,
                         FAR struct usbhost_transfer_s *xfer)
{
  return OK;
}

/*******************************************************************************
 * Name: isp1761_hw_modectrl
 *
 * Description:
 *   Change data bus width to 16-bit, analog overcurrent protection,
 *   level triggered active low IRQ. Write it twice to ensure
 *   correct upper bits if switching to 16-bit mode
 *
 *******************************************************************************/
 
static int isp1761_hw_modectrl(void)
{
  uint32_t regval;

  regval = ISP1761_HW_MODECTRL_DATBUSWID_16|ISP1761_HW_MODECTRL_ANAOC|
           ISP1761_HW_MODECTRL_INTRLEVEL_L|ISP1761_HW_MODECTRL_INTRPOL_L;
  
  g_hal.putreg32(regval, ISP1761_HW_MODECTRL_OFFSET);
  g_hal.putreg32(regval, ISP1761_HW_MODECTRL_OFFSET);

  /* Test write/read access to scratch register */
  
  g_hal.putreg32(0xdeadbabe, ISP1761_HCSCRATCH_OFFSET);
  regval = g_hal.getreg32(ISP1761_HCCHIPID_OFFSET);
  regval = g_hal.getreg32(ISP1761_HCSCRATCH_OFFSET);
  if (regval != 0xdeadbabe)
    {
      udbg("unable to set 16-bit data bus width\n");
      return -EIO;
    }

  return OK;
}

/*******************************************************************************
 * Name: isp1761_handshake
 *
 * Description:
 *   Poll register for desired result with configurable timeout
 *   in milliseconds
 *
 *******************************************************************************/

static int isp1761_handshake(uint32_t addr, uint32_t mask,
                             uint32_t result, int32_t timeout)
{
  uint32_t regval;
  
  do
    {
      regval = g_hal.getreg32(addr);
      if (regval == 0xffffffff)
        {
          return -ENODEV;
        }

      regval &= mask;
      if (regval == result)
        {
          return OK;
        }

      up_mdelay(1);
      timeout--;
        
    } while (timeout > 0);

  return -ETIMEDOUT;
}

/*******************************************************************************
 * Name: isp1761_hw_reset
 *
 * Description:
 *   Reset hardware and ATX
 *
 *******************************************************************************/
 
static int isp1761_hw_reset(void)
{
  int ret;
  uint32_t regval;

  /* HW mode control */

  ret = isp1761_hw_modectrl();
  if (ret != OK)
    {
      return ret;
    }

  g_hal.putreg32(0, ISP1761_HCBUFSTS_OFFSET);
  g_hal.putreg32(0xffffffff, ISP1761_ATLPTD_SKIP_OFFSET);
  g_hal.putreg32(0xffffffff, ISP1761_INTPTD_SKIP_OFFSET);
  g_hal.putreg32(0xffffffff, ISP1761_ISOPTD_SKIP_OFFSET);
  g_hal.putreg32(0x0, ISP1761_ATLPTD_DONE_OFFSET);
  g_hal.putreg32(0x0, ISP1761_INTPTD_DONE_OFFSET);
  g_hal.putreg32(0x0, ISP1761_ISOPTD_DONE_OFFSET);
  
  g_hal.putreg32(ISP1761_SW_RESET_ALL, ISP1761_SW_RESET_OFFSET);
  g_hal.putreg32(ISP1761_SW_RESET_HC, ISP1761_SW_RESET_OFFSET);
	up_mdelay(100);

  /* HW mode control once more after reset */
  ret = isp1761_hw_modectrl();
  if (ret != OK)
    {
      return ret;
    }

  /* ATX reset */
  
  regval = g_hal.getreg32(ISP1761_HW_MODECTRL_OFFSET);
  regval |= ISP1761_HW_MODECTRL_ALLATXRESET;
  g_hal.putreg32(regval, ISP1761_HW_MODECTRL_OFFSET);
  up_mdelay(10);
  regval &= (~ISP1761_HW_MODECTRL_ALLATXRESET);
  g_hal.putreg32(regval, ISP1761_HW_MODECTRL_OFFSET);

  return OK;
}

/*******************************************************************************
 * Name: isp1761_irq_init
 *
 * Description:
 *   Initialize IRQ
 *
 *******************************************************************************/
 
static void isp1761_irq_init(void)
{
  g_hal.putreg32(ISP1761_HCINTREN_ATL|ISP1761_HCINTREN_ISO|ISP1761_HCINTREN_INT|ISP1761_HCINTREN_SOTITL,
                 ISP1761_HCINTREN_OFFSET);                  /* Enable ATL, ISO, INT IRQ */
  g_hal.putreg32(ISP1761_EDGEINTRCOUNT_CLK(15)|ISP1761_EDGEINTRCOUNT_MINWID(2),
                 ISP1761_EDGEINTRCOUNT_OFFSET);             /* Edge interrupt width/frequency */
  
  g_hal.putreg32(0, ISP1761_ATLIRQMASKOR_OFFSET);   /* Clear OR mask for all ATL PTD */
  g_hal.putreg32(0, ISP1761_ATLIRQMASKAND_OFFSET);  /* Clear AND mask for all ATL PTD */
  g_hal.putreg32(0, ISP1761_INTIRQMASKOR_OFFSET);   /* Clear OR mask for all INT PTD */
  g_hal.putreg32(0, ISP1761_INTIRQMASKAND_OFFSET);  /* Clear AND mask for all INT PTD */
  g_hal.putreg32(0, ISP1761_ISOIRQMASKOR_OFFSET);   /* Clear OR mask for all ISO PTD */
  g_hal.putreg32(0, ISP1761_ISOIRQMASKAND_OFFSET);  /* Clear AND mask for all ISO PTD */
}

/*******************************************************************************
 * Name: isp1761_ptd_init
 *
 * Description:
 *   Initialize last PTD
 *
 *******************************************************************************/
static void isp1761_ptd_init(void)
{
  g_hal.putreg32(0x00000001, ISP1761_ATLPTD_LASTPTD_OFFSET);
  g_hal.putreg32(0x00000001, ISP1761_INTPTD_LASTPTD_OFFSET);
  g_hal.putreg32(0x00000001, ISP1761_ISOPTD_LASTPTD_OFFSET);
}

/*******************************************************************************
 * Name: isp1761_irq_enable
 *
 * Description:
 *   Enable IRQ
 *
 *******************************************************************************/
 
static void isp1761_irq_enable(void)
{
  uint32_t regval;

  g_hal.putreg32(ISP1761_HCINTR_ALL, ISP1761_HCINTR_OFFSET);  /* Clear all IRQ */

  regval = g_hal.getreg32(ISP1761_HW_MODECTRL_OFFSET);
  regval |= ISP1761_HW_MODECTRL_GLOBALINTREN;
  g_hal.putreg32(regval, ISP1761_HW_MODECTRL_OFFSET);  /* Enable interrupts */
}

/*******************************************************************************
 * Name: isp1761_irq_disable
 *
 * Description:
 *   Disable IRQ
 *
 *******************************************************************************/
 
static void isp1761_irq_disable(void)
{
  uint32_t regval;

  regval = g_hal.getreg32(ISP1761_HW_MODECTRL_OFFSET);
  regval &= (~ISP1761_HW_MODECTRL_GLOBALINTREN);
  g_hal.putreg32(regval, ISP1761_HW_MODECTRL_OFFSET);  /* Enable interrupts */
}

/*******************************************************************************
 * Public Functions
 *******************************************************************************/

/*******************************************************************************
 * Name: usbhost_initialize
 *
 * Description:
 *   Initialize USB host device controller hardware.
 *
 * Input Parameters:
 *   controller -- If the device supports more than USB host controller, then
 *     this identifies which controller is being intialized.  Normally, this
 *     is just zero.
 *
 * Returned Value:
 *   And instance of the USB host interface.  The controlling task should
 *   use this interface to (1) call the wait() method to wait for a device
 *   to be connected, and (2) call the enumerate() method to bind the device
 *   to a class driver.
 *
 * Assumptions:
 * - This function should called in the initialization sequence in order
 *   to initialize the USB device functionality.
 * - Class drivers should be initialized prior to calling this function.
 *   Otherwise, there is a race condition if the device is already connected.
 *
 *******************************************************************************/

FAR struct usbhost_driver_s *usbhost_initialize(int controller)
{
  int ret;

  g_hal.set_regaccess();

  /* ISP1761 HW reset */
  
  ret = isp1761_hw_reset();
  if (ret != OK)
    {
      return NULL;
    }

	/* EHCI Reset */
  
  ret = ehci_reset();
  if (ret != OK)
    {
      return NULL;
    }

  /* ISP1761 Interrupt init */

  isp1761_irq_init();

  return &g_ehci.drvr;
}

/*******************************************************************************
 * Name: usbhost_run
 *
 * Description:
 *   Starts USB host controller
 *
 * Input Parameters:
 *   controller -- If the device supports more than USB host controller, then
 *     this identifies which controller is being intialized.  Normally, this
 *     is just zero.
 *
 * Returned Value:
 *   OK or error
 *
 * Assumptions:
 *   None
 *
 *******************************************************************************/
 
FAR int usbhost_run(int controller)
{
  int ret;
  uint32_t regval;

  /* EHCI Controller run */

  ret = ehci_run();
  if (ret != OK)
    {
      return ret;
    }

  /* ISP1761 last PTD init and enable IRQ */

  g_irq_count = 0;
  g_irq_cause = 0;
  isp1761_ptd_init();
  isp1761_irq_enable();

  /* Connect Root hub */

  ret = usbhost_rh_connect(&g_ehci.drvr);
    if (ret != OK)
    {
      udbg("failed to connect root hub\n");

      /* Disable IRQ */
    
      isp1761_irq_disable();
      
      /* EHCI Controller stop */
    
      (void)ehci_stop();
    }
  
  return ret;
}

/*******************************************************************************
 * Name: usbhost_stop
 *
 * Description:
 *   Stops USB host controller
 *
 * Input Parameters:
 *   controller -- If the device supports more than USB host controller, then
 *     this identifies which controller is being intialized.  Normally, this
 *     is just zero.
 *
 * Returned Value:
 *   OK or error
 *
 * Assumptions:
 *   None
 *
 *******************************************************************************/
 
FAR int usbhost_stop(int controller)
{
  /* Disable IRQ */

  isp1761_irq_disable();
  
  /* EHCI Controller stop */

  (void)ehci_stop();

  /* Disconnect Root hub */

  (void)usbhost_rh_disconnect(&g_ehci.drvr);

  return OK;
}

/*******************************************************************************
 * Name: usbhost_halinitialize
 *
 * Description:
 *   Initialize USB host device controller hardware abstraction.
 *   (Used only for external host controller IC)
 *
 * Input Parameters:
 *   hal -- Hardware abstraction API
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 *******************************************************************************/

FAR void usbhost_halinitialize(FAR struct usbhost_hal_s *hal)
{
  g_hal = *hal;
}

/*******************************************************************************
 * Name: usbhost_interrupt
 *
 * Description:
 *   USB Host interrupt handler
 *
 *******************************************************************************/

FAR int usbhost_interrupt(int irq, FAR void *context)
{ 
  uint32_t regval;
      
  regval = g_hal.getreg32(ISP1761_HCINTR_OFFSET);
  g_hal.putreg32(regval, ISP1761_HCINTR_OFFSET);

  g_irq_count++;
  g_irq_cause = regval;
}

/*******************************************************************************
 * Name: usbhost_regdump
 *
 * Description:
 *   USB Host register dump
 *
 *******************************************************************************/
FAR void usbhost_regdump(int controller)
{
  udbg("USB Host register dump\n");
  udbg("  USBCMD[0x20]       %08x\n", g_hal.getreg32(ISP1761_OPERATIONAL_OFFSET(EHCI_USBCMD_OFFSET)));
  udbg("  USBSTS[0x24]       %08x\n", g_hal.getreg32(ISP1761_OPERATIONAL_OFFSET(EHCI_USBSTS_OFFSET)));
  udbg("  FRINDEX[0x2C]      %08x\n", g_hal.getreg32(ISP1761_OPERATIONAL_OFFSET(EHCI_FRINDEX_OFFSET)));
  udbg("  CONFIGFLAG[0x60]   %08x\n", g_hal.getreg32(ISP1761_OPERATIONAL_OFFSET(EHCI_CONFIGFLAG_OFFSET)));
  udbg("  PORTSC1[0x64]      %08x\n", g_hal.getreg32(ISP1761_OPERATIONAL_OFFSET(EHCI_PORTSC1_OFFSET)));
  udbg("  HcInterrupt[0x310] %08x\n", g_hal.getreg32(ISP1761_HCINTR_OFFSET));
  udbg("  Last IRQ Cause     %08x\n", g_irq_cause);
  udbg("  IRQ Count          %u\n",   g_irq_count);
}

