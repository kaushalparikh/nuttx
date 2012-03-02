/************************************************************************************
 * include/nuttx/usb/usbhost.h
 *
 *   Copyright (C) 2010-2011 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
 *
 * References:
 *   "Universal Serial Bus Mass Storage Class, Specification Overview,"
 *   Revision 1.2,  USB Implementer's Forum, June 23, 2003.
 *
 *   "Universal Serial Bus Mass Storage Class, Bulk-Only Transport,"
 *   Revision 1.0, USB Implementer's Forum, September 31, 1999.
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

#ifndef __NUTTX_USB_USBHOST_H
#define __NUTTX_USB_USBHOST_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <semaphore.h>
#include <nuttx/wqueue.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/************************************************************************************
 * Name: CLASS_CREATE
 *
 * Description:
 *   This macro will call the create() method of struct usbhost_registry_s.  The create()
 *   method is a callback into the class implementation.  It is used to (1) create
 *   a new instance of the USB host class state and to (2) bind a USB host driver
 *   "session" to the class instance.  Use of this create() method will support
 *   environments where there may be multiple USB ports and multiple USB devices
 *   simultaneously connected.
 *
 * Input Parameters:
 *   reg - The USB host class registry entry previously obtained from a call to
 *     usbhost_findclass().
 *   drvr - An instance of struct usbhost_driver_s that the class implementation will
 *     "bind" to its state structure and will subsequently use to communicate with
 *     the USB host driver.
 *   id - In the case where the device supports multiple base classes, subclasses, or
 *     protocols, this specifies which to configure for.
 *
 * Returned Values:
 *   On success, this function will return a non-NULL instance of struct
 *   usbhost_class_s that can be used by the USB host driver to communicate with the
 *   USB host class.  NULL is returned on failure; this function will fail only if
 *   the drvr input parameter is NULL or if there are insufficient resources to
 *   create another USB host class instance.
 *
 * Assumptions:
 *   If this function is called from an interrupt handler, it will be unable to
 *   allocate memory and CONFIG_USBHOST_NPREALLOC should be defined to be a value
 *   greater than zero specify a number of pre-allocated class structures.
 *
 ************************************************************************************/

#define CLASS_CREATE(reg, class, id) ((reg)->create(class, id))

/************************************************************************************
 * Name: CLASS_CONNECT
 *
 * Description:
 *   This macro will call the connect() method of struct usbhost_class_s.  This
 *   method is a callback into the class implementation.  It is used to provide the
 *   device's configuration descriptor to the class so that the class may initialize
 *   properly
 *
 * Input Parameters:
 *   class - The USB host class entry previously obtained from a call to create().
 *   configdesc - A pointer to a uint8_t buffer container the configuration descripor.
 *   desclen - The length in bytes of the configuration descriptor.
 *   funcaddr - The USB address of the function containing the endpoint that EP0
 *     controls
 *
 * Returned Values:
 *   On success, zero (OK) is returned. On a failure, a negated errno value is
 *   returned indicating the nature of the failure
 *
 *   NOTE that the class instance remains valid upon return with a failure.  It is
 *   the responsibility of the higher level enumeration logic to call
 *   CLASS_DISCONNECTED to free up the class driver resources.
 *
 * Assumptions:
 *   - This function is probably called on the same thread that called the driver
 *     enumerate() method. This function will *not* be called from an interrupt
 *     handler.
 *   - If this function returns an error, the USB host controller driver
 *     must call to DISCONNECTED method to recover from the error
 *
 ************************************************************************************/

#define CLASS_CONNECT(class,configdesc,desclen) \
  ((class)->connect(class,configdesc,desclen))

/************************************************************************************
 * Name: CLASS_DISCONNECTED
 *
 * Description:
 *   This macro will call the disconnected() method of struct usbhost_class_s.  This
 *   method is a callback into the class implementation.  It is used to inform the
 *   class that the USB device has been disconnected.
 *
 * Input Parameters:
 *   class - The USB host class entry previously obtained from a call to create().
 *
 * Returned Values:
 *   On success, zero (OK) is returned. On a failure, a negated errno value is
 *   returned indicating the nature of the failure
 *
 * Assumptions:
 *   This function will *not* be called from an interrupt handler.
 *
 ************************************************************************************/

#define CLASS_DISCONNECTED(class) ((class)->disconnected(class))

/*******************************************************************************
 * Name: DRVR_WAIT
 *
 * Description:
 *   Wait for a device to be connected or disconneced.
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the call to
 *      the class create() method.
 *   connected - TRUE: Wait for device to be connected; FALSE: wait for device to
 *      be disconnected
 *
 * Returned Values:
 *   Zero (OK) is returned when a device in connected. This function will not
 *   return until either (1) a device is connected or (2) some failure occurs.
 *   On a failure, a negated errno value is returned indicating the nature of
 *   the failure
 *
 * Assumptions:
 *   This function will *not* be called from an interrupt handler.
 *
 *******************************************************************************/

#define DRVR_WAIT(drvr, connected) ((drvr)->wait(drvr,connected))

/************************************************************************************
 * Name: DRVR_ENUMERATE
 *
 * Description:
 *   Enumerate the connected device.  As part of this enumeration process,
 *   the driver will (1) get the device's configuration descriptor, (2)
 *   extract the class ID info from the configuration descriptor, (3) call
 *   usbhost_findclass() to find the class that supports this device, (4)
 *   call the create() method on the struct usbhost_registry_s interface
 *   to get a class instance, and finally (5) call the connect() method
 *   of the struct usbhost_class_s interface.  After that, the class is in
 *   charge of the sequence of operations.
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the call to
 *      the class create() method.
 *
 * Returned Values:
 *   On success, zero (OK) is returned. On a failure, a negated errno value is
 *   returned indicating the nature of the failure
 *
 * Assumptions:
 *   This function will *not* be called from an interrupt handler.
 *
 ************************************************************************************/

#define DRVR_ENUMERATE(drvr) ((drvr)->enumerate(drvr))

/************************************************************************************
 * Name: DRVR_EP0CONFIGURE
 *
 * Description:
 *   Configure endpoint 0.  This method is normally used internally by the
 *   enumerate() method but is made available at the interface to support
 *   an external implementation of the enumeration logic.
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the call to
 *      the class create() method.
 *   funcaddr - The USB address of the function containing the endpoint that EP0
 *     controls
 *   mps (maxpacketsize) - The maximum number of bytes that can be sent to or
 *    received from the endpoint in a single data packet
 *
 * Returned Values:
 *   On success, zero (OK) is returned. On a failure, a negated errno value is
 *   returned indicating the nature of the failure
 *
 * Assumptions:
 *   This function will *not* be called from an interrupt handler.
 *
 ************************************************************************************/

#define DRVR_EP0CONFIGURE(drvr,ep0,funcaddr,mps) ((drvr)->ep0configure(drvr,ep0,funcaddr,mps))

/************************************************************************************
 * Name: DRVR_EPALLOC
 *
 * Description:
 *   Allocate and configure one endpoint.
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the call to
 *      the class create() method.
 *   epdesc - Describes the endpoint to be allocated.
 *   ep - A memory location provided by the caller in which to receive the
 *      allocated endpoint desciptor.
 *
 * Returned Values:
 *   On success, zero (OK) is returned. On a failure, a negated errno value is
 *   returned indicating the nature of the failure
 *
 * Assumptions:
 *   This function will *not* be called from an interrupt handler.
 *
 ************************************************************************************/

#define DRVR_EPALLOC(drvr,epdesc,ep) ((drvr)->epalloc(drvr,epdesc,ep))

/************************************************************************************
 * Name: DRVR_EPFREE
 *
 * Description:
 *   Free and endpoint previously allocated by DRVR_EPALLOC.
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the call to
 *      the class create() method.
 *   ep - The endpint to be freed.
 *
 * Returned Values:
 *   On success, zero (OK) is returned. On a failure, a negated errno value is
 *   returned indicating the nature of the failure
 *
 * Assumptions:
 *   This function will *not* be called from an interrupt handler.
 *
 ************************************************************************************/

#define DRVR_EPFREE(drvr,ep) ((drvr)->epfree(drvr,ep))

/************************************************************************************
 * Name: DRVR_ALLOC
 *
 * Description:
 *   Some hardware supports special memory in which request and descriptor data can
 *   be accessed more efficiently.  This method provides a mechanism to allocate
 *   the request/descriptor memory.  If the underlying hardware does not support
 *   such "special" memory, this functions may simply map to malloc.
 *
 *   This interface was optimized under a particular assumption.  It was assumed
 *   that the driver maintains a pool of small, pre-allocated buffers for descriptor
 *   traffic.  NOTE that size is not an input, but an output:  The size of the
 *   pre-allocated buffer is returned.
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the call to
 *      the class create() method.
 *   buffer - The address of a memory location provided by the caller in which to
 *     return the allocated buffer memory address.
 *   maxlen - The address of a memory location provided by the caller in which to
 *     return the maximum size of the allocated buffer memory.
 *
 * Returned Values:
 *   On success, zero (OK) is returned. On a failure, a negated errno value is
 *   returned indicating the nature of the failure
 *
 * Assumptions:
 *   This function will *not* be called from an interrupt handler.
 *
 ************************************************************************************/

#define DRVR_ALLOC(drvr,buffer,maxlen) ((drvr)->alloc(drvr,buffer,maxlen))

/************************************************************************************
 * Name: DRVR_FREE
 *
 * Description:
 *   Some hardware supports special memory in which request and descriptor data can
 *   be accessed more efficiently.  This method provides a mechanism to free that
 *   request/descriptor memory.  If the underlying hardware does not support
 *   such "special" memory, this functions may simply map to free().
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the call to
 *      the class create() method.
 *   buffer - The address of the allocated buffer memory to be freed.
 *
 * Returned Values:
 *   On success, zero (OK) is returned. On a failure, a negated errno value is
 *   returned indicating the nature of the failure
 *
 * Assumptions:
 *   This function will *not* be called from an interrupt handler.
 *
 ************************************************************************************/

#define DRVR_FREE(drvr,buffer) ((drvr)->free(drvr,buffer))

/************************************************************************************
 * Name: DRVR_IOALLOC
 *
 * Description:
 *   Some hardware supports special memory in which larger IO buffers can
 *   be accessed more efficiently.  This method provides a mechanism to allocate
 *   the request/descriptor memory.  If the underlying hardware does not support
 *   such "special" memory, this functions may simply map to malloc.
 *
 *   This interface differs from DRVR_ALLOC in that the buffers are variable-sized.
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the call to
 *      the class create() method.
 *   buffer - The address of a memory location provided by the caller in which to
 *     return the allocated buffer memory address.
 *   buflen - The size of the buffer required.
 *
 * Returned Values:
 *   On success, zero (OK) is returned. On a failure, a negated errno value is
 *   returned indicating the nature of the failure
 *
 * Assumptions:
 *   This function will *not* be called from an interrupt handler.
 *
 ************************************************************************************/

#define DRVR_IOALLOC(drvr,buffer,buflen) ((drvr)->ioalloc(drvr,buffer,buflen))

/************************************************************************************
 * Name: DRVR_IOFREE
 *
 * Description:
 *   Some hardware supports special memory in which IO data can  be accessed more
 *   efficiently.  This method provides a mechanism to free that IO buffer
 *   memory.  If the underlying hardware does not support such "special" memory,
 *   this functions may simply map to free().
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the call to
 *      the class create() method.
 *   buffer - The address of the allocated buffer memory to be freed.
 *
 * Returned Values:
 *   On success, zero (OK) is returned. On a failure, a negated errno value is
 *   returned indicating the nature of the failure
 *
 * Assumptions:
 *   This function will *not* be called from an interrupt handler.
 *
 ************************************************************************************/

#define DRVR_IOFREE(drvr,buffer) ((drvr)->iofree(drvr,buffer))

/************************************************************************************
 * Name: DRVR_CTRLIN and DRVR_CTRLOUT
 *
 * Description:
 *   Process a IN or OUT request on the control endpoint.  These methods
 *   will enqueue the request and wait for it to complete.  Only one transfer may be
 *   queued; Neither these methods nor the transfer() method can be called again
 *   until the control transfer functions returns.
 *
 *   These are blocking methods; these functions will not return until the
 *   control transfer has completed.
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the call to
 *      the class create() method.
 *   req - Describes the request to be sent.  This request must lie in memory
 *      created by DRVR_ALLOC.
 *   buffer - A buffer used for sending the request and for returning any
 *     responses.  This buffer must be large enough to hold the length value
 *     in the request description. buffer must have been allocated using DRVR_ALLOC.
 *
 *   NOTE: On an IN transaction, req and buffer may refer to the same allocated
 *   memory.
 *
 * Returned Values:
 *   On success, zero (OK) is returned. On a failure, a negated errno value is
 *   returned indicating the nature of the failure
 *
 * Assumptions:
 *   This function will *not* be called from an interrupt handler.
 *
 ************************************************************************************/

#define DRVR_CTRLIN(drvr,xfer,cmd)  ((drvr)->ctrlin(drvr,xfer,cmd))
#define DRVR_CTRLOUT(drvr,xfer,cmd) ((drvr)->ctrlout(drvr,xfer,cmd))

/************************************************************************************
 * Name: DRVR_TRANSFER
 *
 * Description:
 *   Process a request to handle a transfer descriptor.  This method will
 *   enqueue the transfer request and rwait for it to complete.  Only one transfer may
 *   be queued; Neither this method nor the ctrlin or ctrlout methods can be called
 *   again until this function returns.
 *
 *   This is a blocking method; this functions will not return until the
 *   transfer has completed.
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the call to
 *      the class create() method.
 *   ed - The IN or OUT endpoint descriptor for the device endpoint on which to
 *      perform the transfer.
 *   buffer - A buffer containing the data to be sent (OUT endpoint) or received
 *     (IN endpoint).  buffer must have been allocated using DRVR_ALLOC
 *   buflen - The length of the data to be sent or received.
 *
 * Returned Values:
 *   On success, zero (OK) is returned. On a failure, a negated errno value is
 *   returned indicating the nature of the failure
 *
 * Assumptions:
 *   This function will *not* be called from an interrupt handler.
 *
 ************************************************************************************/

#define DRVR_TRANSFER(drvr,xfer) ((drvr)->transfer(drvr,xfer)

/************************************************************************************
 * Name: DRVR_DISCONNECT
 *
 * Description:
 *   Called by the class when an error occurs and driver has been disconnected.
 *   The USB host driver should discard the handle to the class instance (it is
 *   stale) and not attempt any further interaction with the class driver instance
 *   (until a new instance is received from the create() method).  The driver
 *   should not called the class' disconnected() method.
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the call to
 *      the class create() method.
 *
 * Returned Values:
 *   None
 *
 * Assumptions:
 *   This function will *not* be called from an interrupt handler.
 *
 ************************************************************************************/

#define DRVR_DISCONNECT(drvr) ((drvr)->disconnect(drvr))

/************************************************************************************
 * Name: DRVR_ROOTHUB
 *
 * Description:
 *   Called by hub class to control root hub.
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the call to
 *      the class create() method.
 *   req - Describes the request to be sent.  This request must lie in memory
 *      created by DRVR_ALLOC.
 *   buffer - A buffer used for sending the request and for returning any
 *     responses.  This buffer must be large enough to hold the length value
 *     in the request description. buffer must have been allocated using DRVR_ALLOC.
 *
 * Returned Values:
 *   On success, zero (OK) is returned. On a failure, a negated errno value is
 *   returned indicating the nature of the failure
 *
 * Assumptions:
 *
 ************************************************************************************/

#define DRVR_RHCTRL(drvr,xfer,cmd) ((drvr)->rhctrl(drvr,xfer,cmd))
#define DRVR_RHSTATUS(drvr,xfer,cmd) ((drvr)->rhstatus(drvr,xfer))

/************************************************************************************
 * Public Types
 ************************************************************************************/

/* This type represents one endpoint configured by the allocep() method.
 * The actual form is know only internally to the USB host controller
 * (for example, for an OHCI driver, this would probably be a pointer
 * to an endpoint descriptor).
 */

typedef FAR void *usbhost_ep_t;

/* Generic transfer structure */

struct usbhost_transfer_s
{
  uint8_t                *buffer;
  size_t                 buflen;
  size_t                 len;
  int                    status;

  /* Semaphore of synchronized transfers */

  sem_t                  done;

  /* Device class */

  struct usbhost_class_s *devclass;
  
  /* Opaque endpoint pointer; can be xHCI specific */

  usbhost_ep_t           ep;

  /* Workqueue for callback */

  struct work_s          work;
  
  /* Transfer complete callback */
  
  void (*callback)(FAR struct usbhost_transfer_s *);
};

/* This struct contains all of the information that is needed to associate a device
 * this is connected via a USB port to a class.
 */

struct usbhost_id_s
{
  uint8_t  base;     /* Base device class code (see USB_CLASS_* defines in usb.h) */
  uint8_t  subclass; /* Sub-class, depends on base class. Eg., See USBMSC_SUBCLASS_* */
  uint8_t  proto;    /* Protocol, depends on base class. Eg., See USBMSC_PROTO_* */
  uint16_t vid;      /* Vendor ID (for vendor/product specific devices) */
  uint16_t pid;      /* Product ID (for vendor/product specific devices) */
};

/* The struct usbhost_registry_s type describes information that is kept in the the
 * USB host registry.  USB host class implementations register this information so
 * that USB host drivers can later find the class that matches the device that is
 * connected to the USB port.
 */

struct usbhost_driver_s; /* Forward reference to the driver state structure */
struct usbhost_class_s;  /* Forward reference to the class state structure */
struct usbhost_registry_s
{
  /* This field is used to implement a singly-link registry structure.  Because of
   * the presence of this link, provides of structy usbhost_registry_s instances must
   * provide those instances in write-able memory (RAM).
   */

  struct usbhost_registry_s     *flink;

  /* This is a callback into the class implementation.  It is used to (1) create
   * a new instance of the USB host class state and to (2) bind a USB host driver
   * "session" to the class instance.  Use of this create() method will support
   * environments where there may be multiple USB ports and multiple USB devices
   * simultaneously connected (see the CLASS_CREATE() macro above).
   */
 
  int                            (*create)(FAR struct usbhost_class_s *class,
                                           FAR const struct usbhost_id_s *id);

  /* This information uniquely identifies the USB host class implementation that
   * goes with a specific USB device.
   */

  uint8_t                       nids;  /* Number of IDs in the id[] array */
  FAR const struct usbhost_id_s *id;   /* An array of ID info. Actual dimension is nids */
};

/* struct usbhost_class_s provides access from the USB host driver to the USB host
 * class implementation.
 */

struct usbhost_class_s
{
  /* Host driver */

  struct usbhost_driver_s *drvr;

  /* USB device address & speed */

  uint8_t                 addr;
  uint8_t                 speed;

  /* Parent class */

  struct usbhost_class_s  *parent;

  /* Control endpoint, ep0 */

  usbhost_ep_t            ep0;

  /* Transaction translator hub */

  struct usb_hubtt_s      *tt;

  /* Transaction translator port */

  uint8_t                 ttport;

  /* Class specific private data */

  void                    *priv;

  /* Provides the configuration descriptor to the class.  The configuration
   * descriptor contains critical information needed by the class in order to
   * initialize properly (such as endpoint selections).
   */

  int (*connect)(FAR struct usbhost_class_s *class, FAR const uint8_t *configdesc,
                 int desclen);

  /* This method informs the class that the USB device has been disconnected. */

  int (*disconnected)(FAR struct usbhost_class_s *class);
};

/* This structure describes one endpoint.  It is used as an input to the
 * allocep() method.  Most of this information comes from the endpoint
 * descriptor.
 */

struct usbhost_epdesc_s
{
  struct usbhost_class_s *devclass;    /* Class */
  uint8_t                addr;         /* Endpoint address */
  bool                   in;           /* Direction: true->IN */
  uint8_t                xfrtype;      /* Transfer type.  See SB_EP_ATTR_XFER_* in usb.h */
  uint8_t                interval;     /* Polling interval */
  uint16_t               mxpacketsize; /* Max packetsize */
};

/* struct usbhost_hal_s provides hardware abstraction layer for external
 * host controller IC
 */

struct usbhost_hal_s
{
  /* Set hardware interface for register access */

  void (*set_regaccess)(void);

  /* Set hardware interface for memory access */

  void (*set_memaccess)(void);

  /* Read 32-bit memory/register */

  uint32_t (*getreg32)(uint32_t addr);

  /* Write 32-bit memory/register */
  void (*putreg32)(uint32_t val, uint32_t addr);
};

/* struct usbhost_driver_s provides access to the USB host driver from the
 * USB host class implementation.
 */

struct usbhost_driver_s
{
  /* Root hub */

  struct usbhost_class_s *roothub;

  /* Controller speed, i.e. High/Full/Low */

  uint8_t                speed;

  /* Wait for a device to connect or disconnect. */

  int (*wait)(FAR struct usbhost_driver_s *drvr, bool connected);

  /* Enumerate the connected device.  As part of this enumeration process,
   * the driver will (1) get the device's configuration descriptor, (2)
   * extract the class ID info from the configuration descriptor, (3) call
   * usbhost_findclass() to find the class that supports this device, (4)
   * call the create() method on the struct usbhost_registry_s interface
   * to get a class instance, and finally (5) call the connect() method
   * of the struct usbhost_class_s interface.  After that, the class is in
   * charge of the sequence of operations.
   */

  int (*enumerate)(FAR struct usbhost_driver_s *drvr);

  /* Configure endpoint 0.  This method is normally used internally by the
   * enumerate() method but is made available at the interface to support
   * an external implementation of the enumeration logic.
   */

  int (*ep0configure)(FAR struct usbhost_driver_s *drvr, usbhost_ep_t ep0,
                      uint8_t funcaddr, uint16_t maxpacketsize);

  /* Allocate and configure an endpoint. */

  int (*epalloc)(FAR struct usbhost_driver_s *drvr,
                const FAR struct usbhost_epdesc_s *epdesc, usbhost_ep_t *ep);
  int (*epfree)(FAR struct usbhost_driver_s *drvr, usbhost_ep_t ep);

  /* Some hardware supports special memory in which transfer descriptors can
   * be accessed more efficiently.  The following methods provide a mechanism
   * to allocate and free the transfer descriptor memory.  If the underlying
   * hardware does not support such "special" memory, these functions may
   * simply map to malloc and free.
   *
   * This interface was optimized under a particular assumption.  It was assumed
   * that the driver maintains a pool of small, pre-allocated buffers for descriptor
   * traffic.  NOTE that size is not an input, but an output:  The size of the
   * pre-allocated buffer is returned.
   */

  int (*alloc)(FAR struct usbhost_driver_s *drvr,
               FAR uint8_t **buffer, FAR size_t *maxlen);
  int (*free)(FAR struct usbhost_driver_s *drvr, FAR uint8_t *buffer);

  /*   Some hardware supports special memory in which larger IO buffers can
   *   be accessed more efficiently.  This method provides a mechanism to allocate
   *   the request/descriptor memory.  If the underlying hardware does not support
   *   such "special" memory, this functions may simply map to malloc.
   *
   *   This interface differs from DRVR_ALLOC in that the buffers are variable-sized.
   */

  int (*ioalloc)(FAR struct usbhost_driver_s *drvr,
               FAR uint8_t **buffer, size_t buflen);
  int (*iofree)(FAR struct usbhost_driver_s *drvr, FAR uint8_t *buffer);

  /* Process a IN or OUT request on the control endpoint.  These methods
   * will enqueue the request and wait for it to complete.  Only one transfer may
   * be queued; Neither these methods nor the transfer() method can be called again
   * until the control transfer functions returns.
   *
   * These are blocking methods; these functions will not return until the
   * control transfer has completed.
   */

  int (*ctrlin)(FAR struct usbhost_driver_s *drvr,
                FAR struct usbhost_transfer_s *xfer,
                FAR const struct usb_ctrlreq_s *cmd);
  int (*ctrlout)(FAR struct usbhost_driver_s *drvr,
                 FAR struct usbhost_transfer_s *xfer,
                 FAR const struct usb_ctrlreq_s *cmd);

  /* Process a request to handle a transfer descriptor.  This method will
   * enqueue the transfer request and wait for it to complete.  Only one transfer may
   * be queued; Neither this method nor the ctrlin or ctrlout methods can be called
   * again until this function returns.
   *
   * This is a blocking method; this functions will not return until the
   * transfer has completed.
   */

  int (*transfer)(FAR struct usbhost_driver_s *drvr, FAR struct usbhost_transfer_s *xfer);

  /* Called by the class when an error occurs and driver has been disconnected.
   * The USB host driver should discard the handle to the class instance (it is
   * stale) and not attempt any further interaction with the class driver instance
   * (until a new instance is received from the create() method).
   */

  void (*disconnect)(FAR struct usbhost_driver_s *drvr);

  /* Called by hub class to control root hub.
   */

  int (*rhctrl)(FAR struct usbhost_driver_s *drvr,
                 FAR struct usbhost_transfer_s *xfer,
                 FAR const struct usb_ctrlreq_s *cmd);
  int (*rhstatus)(FAR struct usbhost_driver_s *drvr,
                  FAR struct usbhost_transfer_s *xfer);
};

/************************************************************************************
 * Public Data
 ************************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: usbhost_registerclass
 *
 * Description:
 *   Register a USB host class implementation.  The caller provides an instance of
 *   struct usbhost_registry_s that contains all of the information that will be
 *   needed later to (1) associate the USB host class implementation with a connected
 *   USB device, and (2) to obtain and bind a struct usbhost_class_s instance for
 *   the device.
 *
 * Input Parameters:
 *   class - An write-able instance of struct usbhost_registry_s that will be
 *     maintained in a registry.
 *
 * Returned Values:
 *   On success, this function will return zero (OK).  Otherwise, a negated errno
 *   value is returned.
 *
 ************************************************************************************/

EXTERN int usbhost_registerclass(struct usbhost_registry_s *class);

/************************************************************************************
 * Name: usbhost_findclass
 *
 * Description:
 *   Find a USB host class implementation previously registered by
 *   usbhost_registerclass().  On success, an instance of struct usbhost_registry_s
 *   will be returned.  That instance will contain all of the information that will
 *   be needed to obtain and bind a struct usbhost_class_s instance for the device.
 *
 * Input Parameters:
 *   id - Identifies the USB device class that has connect to the USB host.
 *
 * Returned Values:
 *   On success this function will return a non-NULL instance of struct
 *   usbhost_registry_s.  NULL will be returned on failure.  This function can only
 *   fail if (1) id is NULL, or (2) no USB host class is registered that matches the
 *   device class ID.
 *
 ************************************************************************************/

EXTERN const struct usbhost_registry_s *usbhost_findclass(const struct usbhost_id_s *id);

/****************************************************************************
 * Name: usbhost_hubinit
 *
 * Description:
 *   Initialize the USB hub class.  This function should be called
 *   be platform-specific code in order to initialize and register support
 *   for the USB host storage class.
 *
 * Input Parameters:
 *   None
 *
 * Returned Values:
 *   On success this function will return zero (OK);  A negated errno value
 *   will be returned on failure.
 *
 ****************************************************************************/

EXTERN int usbhost_hubinit(void);

/****************************************************************************
 * Name: usbhost_storageinit
 *
 * Description:
 *   Initialize the USB host storage class.  This function should be called
 *   be platform-specific code in order to initialize and register support
 *   for the USB host storage class.
 *
 * Input Parameters:
 *   None
 *
 * Returned Values:
 *   On success this function will return zero (OK);  A negated errno value
 *   will be returned on failure.
 *
 ****************************************************************************/

EXTERN int usbhost_storageinit(void);

/****************************************************************************
 * Name: usbhost_kbdinit
 *
 * Description:
 *   Initialize the USB storage HID keyboard class driver.  This function
 *   should be called be platform-specific code in order to initialize and
 *   register support for the USB host HID keyboard class device.
 *
 * Input Parameters:
 *   None
 *
 * Returned Values:
 *   On success this function will return zero (OK);  A negated errno value
 *   will be returned on failure.
 *
 ****************************************************************************/

EXTERN int usbhost_kbdinit(void);

/****************************************************************************
 * Name: usbhost_wlaninit
 *
 * Description:
 *   Initialize the USB WLAN class driver.  This function should be called
 *   be platform-specific code in order to initialize and register support
 *   for the USB host class device.
 *
 * Input Parameters:
 *   None
 *
 * Returned Values:
 *   On success this function will return zero (OK);  A negated errno value
 *   will be returned on failure.
 *
 ****************************************************************************/

EXTERN int usbhost_wlaninit(void);

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

EXTERN FAR struct usbhost_driver_s *usbhost_initialize(int controller);

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

EXTERN int usbhost_ctrlxfer(FAR struct usbhost_class_s *devclass,
                            uint8_t type, uint8_t req, uint16_t value,
                            uint16_t index, uint16_t len,
                            FAR uint8_t *buffer);

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

EXTERN int usbhost_intxfer(FAR struct usbhost_class_s *devclass,
                           FAR struct usbhost_transfer_s *xfer,
                           void (*callback)(FAR struct usbhost_transfer_s *));

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

EXTERN int usbhost_enumerate(FAR struct usbhost_class_s *devclass);

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

#ifdef CONFIG_USBHOST_EXTERNAL

EXTERN FAR void usbhost_halinitialize(FAR struct usbhost_hal_s *hal);

#endif

/*******************************************************************************
 * Name: usbhost_rh_connect
 *
 * Description:
 *   Registers USB host root hub
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the call to
 *      the class create() method.
 *
 * Returned Value:
 *
 * Assumptions:
 *
 *******************************************************************************/

EXTERN int usbhost_rh_connect(FAR struct usbhost_driver_s *drvr);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __NUTTX_USB_USBHOST_H */
