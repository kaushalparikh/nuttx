/*******************************************************************************
 * drivers/usbhost/usbhost_enumerate.c
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <spudmonkey@racsa.co.cr>
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

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/usb/usb.h>
#include <nuttx/usb/usbhost.h>
#include <nuttx/usb/hub.h>

/*******************************************************************************
 * Definitions
 *******************************************************************************/

/* Check for root hub */

#define ROOTHUB(class)  ((class)->parent == NULL)

/*******************************************************************************
 * Private Types
 *******************************************************************************/

/*******************************************************************************
 * Private Function Prototypes
 *******************************************************************************/

static inline uint16_t usbhost_getle16(const uint8_t *val);
static void usbhost_putle16(uint8_t *dest, uint16_t val);

static void usbhost_callback(FAR struct usbhost_xfer_s *xfer);

static inline int usbhost_devdesc(const struct usb_devdesc_s *devdesc,
                                  struct usbhost_id_s *id);
static inline int usbhost_configdesc(const uint8_t *configdesc, int desclen,
                                     struct usbhost_id_s *id);
static inline int usbhost_classbind(FAR struct usbhost_class_s *devclass,
                                    const uint8_t *configdesc, int desclen,
                                    struct usbhost_id_s *id);

/*******************************************************************************
 * Private Data
 *******************************************************************************/

/*******************************************************************************
 * Public Data
 *******************************************************************************/

/*******************************************************************************
 * Private Functions
 *******************************************************************************/

/****************************************************************************
 * Name: usbhost_getle16
 *
 * Description:
 *   Get a (possibly unaligned) 16-bit little endian value.
 *
 *******************************************************************************/

static inline uint16_t usbhost_getle16(const uint8_t *val)
{
  return (uint16_t)val[1] << 8 | (uint16_t)val[0];
}

/****************************************************************************
 * Name: usbhost_putle16
 *
 * Description:
 *   Put a (possibly unaligned) 16-bit little endian value.
 *
 *******************************************************************************/

static void usbhost_putle16(uint8_t *dest, uint16_t val)
{
  dest[0] = val & 0xff; /* Little endian means LS byte first in byte stream */
  dest[1] = val >> 8;
}

/****************************************************************************
 * Name: usbhost_callback
 *
 * Description:
 *   
 *
 *******************************************************************************/

static void usbhost_callback(FAR struct usbhost_xfer_s *xfer)
{
  sem_post(&xfer->done);
}

/*******************************************************************************
 * Name: usbhost_devdesc
 *
 * Description:
 *   A configuration descriptor has been obtained from the device.  Find the
 *   ID information for the class that supports this device.
 *
 *******************************************************************************/

static inline int usbhost_devdesc(const struct usb_devdesc_s *devdesc,
                                  struct usbhost_id_s *id)
{
  /* Clear the ID info */

  memset(id, 0, sizeof(struct usbhost_id_s));

  /* Pick off the class ID info */

  id->base     = devdesc->class;
  id->subclass = devdesc->subclass;
  id->proto    = devdesc->protocol;

  /* Pick off the VID and PID as well (for vendor specfic devices) */

  id->vid = usbhost_getle16(devdesc->vendor);
  id->pid = usbhost_getle16(devdesc->product);

  uvdbg("class:%d subclass:%04x protocol:%04x vid:%d pid:%d\n",
        id->base, id->subclass, id->proto, id->vid, id->pid);
  return OK;
}
                                
/*******************************************************************************
 * Name: usbhost_configdesc
 *
 * Description:
 *   A configuration descriptor has been obtained from the device.  Find the
 *   ID information for the class that supports this device.
 *
 *******************************************************************************/

static inline int usbhost_configdesc(const uint8_t *configdesc, int cfglen,
                                     struct usbhost_id_s *id)
{
  struct usb_cfgdesc_s *cfgdesc;
  struct usb_ifdesc_s *ifdesc;
  int remaining;

  DEBUGASSERT(configdesc != NULL && cfglen >= USB_SIZEOF_CFGDESC);

  /* Verify that we were passed a configuration descriptor */

  cfgdesc = (struct usb_cfgdesc_s *)configdesc;
  uvdbg("cfg len:%d total len:%d\n", cfgdesc->len, cfglen);

  if (cfgdesc->type != USB_DESC_TYPE_CONFIG)
    {
      return -EINVAL;
    }

  /* Skip to the next entry descriptor */

  configdesc += cfgdesc->len;
  remaining   = cfglen - cfgdesc->len;

  /* Loop where there are more dscriptors to examine */

  memset(id, 0, sizeof(FAR struct usb_desc_s));
  while (remaining >= sizeof(struct usb_desc_s))
    {
      /* What is the next descriptor? Is it an interface descriptor? */

      ifdesc = (struct usb_ifdesc_s *)configdesc;
      if (ifdesc->type == USB_DESC_TYPE_INTERFACE)
        {
          /* Yes, extract the class information from the interface descriptor.
           * Typically these values are zero meaning that the "real" ID
           * information resides in the device descriptor.
           */
 
          DEBUGASSERT(remaining >= sizeof(struct usb_ifdesc_s));
          id->base     = ifdesc->class;
          id->subclass = ifdesc->subclass;
          id->proto    = ifdesc->protocol;
          uvdbg("class:%d subclass:%d protocol:%d\n",
                id->base, id->subclass, id->proto);
          return OK;
        }

     /* Increment the address of the next descriptor */
 
      configdesc += ifdesc->len;
      remaining  -= ifdesc->len;
    }

  return -ENOENT;
}

/*******************************************************************************
 * Name: usbhost_classbind
 *
 * Description:
 *   A configuration descriptor has been obtained from the device.  Try to
 *   bind this configuration descriptor with a supported class.
 *
 *******************************************************************************/

static inline int usbhost_classbind(FAR struct usbhost_class_s *devclass,
                                    const uint8_t *configdesc, int desclen,
                                    struct usbhost_id_s *id)
{
  const struct usbhost_registry_s *reg;
  int ret = -EINVAL;

  /* Is there is a class implementation registered to support this device. */

  reg = usbhost_findclass(id);
  uvdbg("usbhost_findclass: %p\n", reg);
  if (reg != NULL)
    {
      /* Yes.. there is a class for this device.  Get an instance of
       * its interface.
       */

      ret = CLASS_CREATE(reg, devclass, id);
      uvdbg("CLASS_CREATE: %p\n", devclass->priv);
      if (devclass->priv != NULL)
        {
          /* Then bind the newly instantiated class instance */

          ret = CLASS_CONNECT(devclass, configdesc, desclen);
          if (ret != OK)
            {
              /* On failures, call the class disconnect method which
               * should then free the allocated devclass instance.
               */

              udbg("CLASS_CONNECT failed: %d\n", ret);
              CLASS_DISCONNECTED(devclass);
            }
        }
    }

  uvdbg("Returning: %d\n", ret);
  return ret;
}

/*******************************************************************************
 * Public Functions
 *******************************************************************************/

/****************************************************************************
 * Name: usbhost_ctrlxfer
 *
 * Description:
 *   Free transfer buffer memory.
 *
 * Input Parameters:
 *   priv - A reference to the class instance.
 *
 * Returned Values:
 *   On sucess, zero (OK) is returned.  On failure, an negated errno value
 *   is returned to indicate the nature of the failure.
 *
 ****************************************************************************/

int usbhost_ctrlxfer(FAR struct usbhost_class_s *devclass,
                     uint8_t type, uint8_t req, uint16_t value,
                     uint16_t index, uint16_t len,
                     FAR uint8_t *buffer)
{
  struct usbhost_xfer_s xfer;
  struct usb_ctrlreq_s cmd;
  struct timespec timeout;
  int ret;
  
  cmd.type = type;
  cmd.req  = req;
  usbhost_putle16(cmd.value, value);
  usbhost_putle16(cmd.index, index);
  usbhost_putle16(cmd.len, len);

  xfer.buffer   = buffer;
  xfer.buflen   = len;
  xfer.len      = len;
  xfer.status   = -EIO;
  xfer.devclass = devclass;
  xfer.ep       = devclass->ep0;
  xfer.callback = usbhost_callback;

  sem_init(&xfer.done, 0, 0);

  if (ROOTHUB(devclass))
    {
      ret = DRVR_RHCTRL(devclass->drvr, &xfer, &cmd);
    }
  else
    {
      if (type & USB_REQ_DIR_IN)
        {
          ret = DRVR_CTRLIN(devclass->drvr, &xfer, &cmd);
        }
      else
        {
          ret = DRVR_CTRLOUT(devclass->drvr, &xfer, &cmd);
        }
    }

  if (ret != OK)
    goto out;

  timeout.tv_sec  = 5;
  timeout.tv_nsec = 1000*1000;

  ret = sem_timedwait(&xfer.done, &timeout);
  if (ret == OK)
    {
      ret = xfer.status;
    }

out:
  sem_destroy(&xfer.done);

  return ret;
}

/****************************************************************************
 * Name: usbhost_intxfer
 *
 * Description:
 *   Free transfer buffer memory.
 *
 * Input Parameters:
 *   priv - A reference to the class instance.
 *
 * Returned Values:
 *   On sucess, zero (OK) is returned.  On failure, an negated errno value
 *   is returned to indicate the nature of the failure.
 *
 ****************************************************************************/

int usbhost_intxfer(FAR struct usbhost_class_s *devclass,
                    FAR struct usbhost_xfer_s *xfer,
                    void (*callback)(FAR struct usbhost_xfer_s *xfer))
{
  return OK;
}

/*******************************************************************************
 * Name: usbhost_enumerate
 *
 * Description:
 *   Enumerate the connected device.  As part of this enumeration process,
 *   the driver will (1) get the device's configuration descriptor, (2)
 *   extract the class ID info from the configuration descriptor, (3) call
 *   usbhost_findclass() to find the class that supports this device, (4)
 *   call the create() method on the struct usbhost_registry_s interface
 *   to get a class instance, and finally (5) call the configdesc() method
 *   of the struct usbhost_class_s interface.  After that, the class is in
 *   charge of the sequence of operations.
 *
 * Input Parameters:
 *   class - USB class information common across all classes. Whoever calls
 *      enumerate should fill address, speed, driver and parent class
 *      pointer. Enumeration will fill the control endpoint ep0,
 *      transaction translator (if applicable) and private data
 *
 * Returned Values:
 *   On success, zero (OK) is returned. On a failure, a negated errno value is
 *   returned indicating the nature of the failure
 *
 * Assumptions:
 *   - Called from a single thread so no mutual exclusion is required.
 *   - Never called from an interrupt handler.
 *
 *******************************************************************************/

int usbhost_enumerate(FAR struct usbhost_class_s *devclass)
{
  struct usbhost_id_s id;
  size_t maxlen;
  unsigned int cfglen;
  uint8_t maxpacketsize;
  uint8_t *buffer;
  int  ret;

  DEBUGASSERT(devclass != NULL &&
              devclass->drvr != NULL);

  /* Allocate IN buffers for use in this function.
   */

  ret = DRVR_ALLOC(devclass->drvr, &buffer, &maxlen);
  if (ret != OK)
    {
      udbg("failed to allocate ep0 buffer\n");
      goto errout;
    }

  /* Read first 8 bytes of the device descriptor */

  ret = usbhost_ctrlxfer(devclass, (USB_REQ_DIR_IN | USB_REQ_RECIPIENT_DEVICE),
                         USB_REQ_GETDESCRIPTOR, (USB_DESC_TYPE_DEVICE << 8),
                         0, 8, buffer);
  if (ret != OK)
    {
      udbg("failed to read ep0 max packet size\n");
      goto errout;
    }

  /* Extract the correct max packetsize from the device descriptor */

  maxpacketsize = ((struct usb_devdesc_s *)buffer)->mxpacketsize;
  uvdbg("maxpacksetsize: %d\n", maxpacketsize);

  /* Set the USB device address to the value in the 'funcaddr' input */

  ret = usbhost_ctrlxfer(devclass, (USB_REQ_DIR_OUT | USB_REQ_RECIPIENT_DEVICE),
                         USB_REQ_SETADDRESS, devclass->addr,
                         0, 0, NULL);
  if (ret != OK)
    {
      udbg("failed to set address\n");
      goto errout;
    }
  up_mdelay(2);

  /* And reconfigure EP0 */

  DRVR_EP0CONFIGURE(devclass->drvr, devclass->ep0, devclass->addr, maxpacketsize);

  /* Now read the full device descriptor */

  ret = usbhost_ctrlxfer(devclass, (USB_REQ_DIR_IN | USB_REQ_RECIPIENT_DEVICE),
                         USB_REQ_GETDESCRIPTOR, (USB_DESC_TYPE_DEVICE << 8),
                         0, USB_SIZEOF_DEVDESC, buffer);
  if (ret != OK)
    {
      udbg("failed to read device descriptor\n");
      goto errout;
    }

  /* Get class identification information from the device descriptor.  Most
   * devices set this to USB_CLASS_PER_INTERFACE (zero) and provide the
   * identification informatino in the interface descriptor(s).  That allows
   * a device to support multiple, different classes.
   */

  (void)usbhost_devdesc((struct usb_devdesc_s *)buffer, &id);

 /* Get the configuration descriptor (only), index == 0.  Should not be
  * hard-coded! More logic is needed in order to handle devices with
  * multiple configurations.
  */

  ret = usbhost_ctrlxfer(devclass, (USB_REQ_DIR_IN | USB_REQ_RECIPIENT_DEVICE),
                         USB_REQ_GETDESCRIPTOR, (USB_DESC_TYPE_CONFIG << 8),
                         0, USB_SIZEOF_CFGDESC, buffer);
  if (ret != OK)
   {
      udbg("failed to read configuration descriptor\n");
      goto errout;
    }

  /* Extract the full size of the configuration data */

  cfglen = (unsigned int)usbhost_getle16(((struct usb_cfgdesc_s *)buffer)->totallen);
  uvdbg("sizeof config data: %d\n", cfglen);

  /* Get all of the configuration descriptor data, index == 0 (Should not be
   * hard-coded!)
   */

  ret = usbhost_ctrlxfer(devclass, (USB_REQ_DIR_IN | USB_REQ_RECIPIENT_DEVICE),
                         USB_REQ_GETDESCRIPTOR, (USB_DESC_TYPE_CONFIG << 8),
                         0, cfglen, buffer);
  if (ret != OK)
    {
      udbg("failed to read full configuration description\n");
      goto errout;
    }

  /* Select device configuration 1 (Should not be hard-coded!) */

  ret = usbhost_ctrlxfer(devclass, (USB_REQ_DIR_OUT | USB_REQ_RECIPIENT_DEVICE),
                         USB_REQ_SETCONFIGURATION, 1,
                         0, 0, NULL);
  if (ret != OK)
    {
      udbg("failed to set configuration\n");
      goto errout;
    }

  /* Was the class identification information provided in the device descriptor?
   * Or do we need to find it in the interface descriptor(s)?
   */

  if (id.base == USB_CLASS_PER_INTERFACE)
    {
      /* Get the class identification information for this device from the
       * interface descriptor(s).  Hmmm.. More logic is need to handle the
       * case of multiple interface descriptors.
       */

      ret = usbhost_configdesc(buffer, cfglen, &id);
      if (ret != OK)
        {
          udbg("failed to read class identification\n");
          goto errout;
        }
    }

  /* Some devices may require this delay before initialization */

  up_mdelay(100);

  /* Parse the configuration descriptor and bind to the class instance for the
   * device.  This needs to be the last thing done because the class driver
   * will begin configuring the device.
   */

  ret = usbhost_classbind(devclass, buffer, cfglen, &id);
  if (ret != OK)
    {
      udbg("failed to bind class\n");
    }

errout:
  if (buffer != NULL)
    {
      DRVR_FREE(devclass->drvr, buffer);
    }

  return ret;
}
