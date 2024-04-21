/*****************************************************************************************
 *
 * FILE NAME          : mxl_moca_osal_linux.c
 *
 *
 *
 * DATE CREATED       : Sep/14/2016
 *
 * LAST MODIFIED      :
 *
 * DESCRIPTION        : This file include linux OSAL implementation
 *
 *****************************************************************************************
 * This file is licensed under GNU General Public license.
 *
 * This file is free software: you can redistribute and/or modify it under the
 * terms of the GNU General Public License, Version 2, as published by the Free
 * Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but AS-IS and
 * WITHOUT ANY WARRANTY; without even the implied warranties of MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE, TITLE, or NONINFRINGEMENT. Redistribution,
 * except as permitted by the GNU General Public License is prohibited.
 *
 * You should have received a copy of the GNU General Public License, Version 2
 * along with this file; if not, see <http://www.gnu.org/licenses/>.
 *
 *****************************************************************************************
 * Copyright (c) 2016 MaxLinear, Inc. All rights reserved.
 ****************************************************************************************/

#include <stdarg.h>
#include <linux/hardirq.h>
#include <linux/string.h>
#include <linux/sched.h>
#include <linux/time.h>
#include <linux/pci.h>
#include <linux/version.h>
#include <linux/firmware.h>
#include <linux/types.h>
#include <linux/device.h>

#include "mxl_data_types.h"
#include "mxl_moca_config.h"
#include "mxl_moca_soc_cmd.h"
#include "mxl_moca_dev_mgr.h"
#include "mxl_moca_drv_daemon.h"
#include "mxl_moca_osal_linux.h"
#include "mxl_moca_osal.h"
#include "mxl_moca_hal_mdio_ctrl.h"
#include "mxl_moca_drv_mdio.h"
#ifdef INCLUDE_PCI
#include "mxl_moca_osal_pcie.h"
#endif

static uint32_t logMod = 0;
static uint32_t logLev = 0;

bool gGetFwViaIoctl = false;

static struct class *pClass;

// MOCA_UEVENT
#if ENABLE_ADM_STATUS_UEVENT || ENABLE_LINK_UEVENT
/*----------------------------------------------------------------------------------------
 * FUNCTION NAME : Mxl_MoCA_OsalGenerateUevent
 *
 * DATE CREATED  : Apr/20/2018
 *
 * DESCRIPTION   : Generates the Uevent that is handled by the Udev
 *
 *
 * IN PARAMS     : EPHY Addr of the MxL MoCA SOC in the specified MDIO bus
 *
 * OUT PARAMS    : none
 *
 * RETURN VALUE  : none 
 *--------------------------------------------------------------------------------------*/

void Mxl_MoCA_OsalGenerateUevent(void * pDev)
{

  struct device * dev = (struct device *) pDev;
  MxL_MoCA_OsalPrintLog(TRACE_OSWP, L_INFO, "uevent Start \n");

  if (pDev == NULL )
  {
    MxL_MoCA_OsalPrintLog(TRACE_OSWP, L_INFO, "pDev is NULL \n");
    return;
  }

  dev_set_uevent_suppress(dev, false);
  kobject_uevent(&dev->kobj, KOBJ_CHANGE); // This function sends the uevent
  MxL_MoCA_OsalPrintLog(TRACE_OSWP, L_INFO, "uevent End \n");
}
#endif
// MOCA_UEVENT
/*----------------------------------------------------------------------------------------
 * FUNCTION NAME : MxL_MoCA_OsalCreateMocaDev
 *
 *
 * DATE CREATED  : Sep/27/2016
 *
 * DESCRIPTION   : Create MoCA control device, for example: /dev/en15
 *
 *
 * IN PARAMS     : EPHY Addr of the MxL MoCA SOC in the specified MDIO bus
 *
 * OUT PARAMS    : none
 *
 * RETURN VALUE  : MXL_MOCA_OK for success, MXL_MOCA_ERR for failure
 *--------------------------------------------------------------------------------------*/

MXL_MOCA_STATUS_E MxL_MoCA_OsalCreateMocaDev(uint8_t ePhyAddr, void** ppDev)
{
  void *pDevice;
  MXL_MOCA_STATUS_E ret = MXL_MOCA_OK;

  MXL_ENTER_FUNCTION(TRACE_OSWP, "ephy addr = %d",ePhyAddr);

  // Only create once for class
  if (NULL == pClass)
  {
    pClass = class_create(THIS_MODULE, DRV_NAME);

    if (pClass == NULL)
    {
      MxL_MoCA_OsalPrintLog(TRACE_OSWP, L_ERR, "Class create failed for MoCA driver\n");
      return MXL_MOCA_ERR;
    }
  }

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 27)
  pDevice = device_create(pClass, 0, MKDEV(major, ePhyAddr), 0, "en%d", ePhyAddr);
#else
  pDevice = device_create(pClass, 0, MKDEV(major, ePhyAddr), "en%d", ePhyAddr);
#endif

  if (MxL_MoCA_OsalTestPtrErrCondition(pDevice) == MXL_MOCA_ERR)
  {
    MxL_MoCA_OsalPrintLog(TRACE_OSWP, L_ERR, "Create MoCA control device failed!\n");
    ret = MXL_MOCA_ERR;
  }
  else
  {
    MxL_MoCA_OsalPrintLog(TRACE_OSWP, L_DBG, "pDevice = %p", *ppDev);
    MxL_MoCA_OsalPrintLog(TRACE_OSWP, L_INFO, 
                          "Created MoCA control device for the driver(/dev/en%d)\n",
                           ePhyAddr);
    // Set for return
    *ppDev = pDevice;

    // Create Sysfs files to display lof and link status
    if (MXL_MOCA_OK != device_create_file((struct device *)pDevice, &dev_attr_lof))
    {
      MxL_MoCA_OsalPrintLog(TRACE_OSWP, L_INFO, "Warning: create sysfs for lof fails.\n");
    }

    if (MXL_MOCA_OK != device_create_file((struct device *)pDevice, &dev_attr_link_status))
    {
      MxL_MoCA_OsalPrintLog(TRACE_OSWP, L_INFO, "Warning: create sysfs for link_status fails.\n");
    }

    if (MXL_MOCA_OK != device_create_file((struct device *)pDevice, &dev_attr_adm_status))
    {
      MxL_MoCA_OsalPrintLog(TRACE_OSWP, L_INFO, "Warning: create sysfs for adm_status fails.\n");
    }  

    if (MXL_MOCA_OK != device_create_file((struct device *)pDevice, &dev_attr_bridge_ctrl_req_status))
    {
      MxL_MoCA_OsalPrintLog(TRACE_OSWP, L_INFO, "Warning: create sysfs for dev_attr_bridge_ctrl_req_status fails.\n");
    } 
	
	if (MXL_MOCA_OK != device_create_file((struct device *)pDevice, &dev_attr_net_proxy_req_status))
    {
      MxL_MoCA_OsalPrintLog(TRACE_OSWP, L_INFO, "Warning: create sysfs for net_proxy_req_status fails.\n");
    }      
    
	if (MXL_MOCA_OK != device_create_file((struct device *)pDevice, &dev_attr_beacon_power_dist_status))
    {
      MxL_MoCA_OsalPrintLog(TRACE_OSWP, L_INFO, "Warning: create sysfs for beacon_power_dist_status fails.\n");
    }      
    
    // Create Sysfs files to display privacy update status
    if (MXL_MOCA_OK != device_create_file((struct device *)pDevice, &dev_attr_privacy_update_status))
    {
      MxL_MoCA_OsalPrintLog(TRACE_OSWP, L_INFO, "Warning: create sysfs for privacy_update_status fails.\n");
    }
  }

  return ret;
}

/*----------------------------------------------------------------------------------------
 * FUNCTION NAME : MxL_MoCA_OsalDestroyMocaDev
 *
 *
 * DATE CREATED  : Oct/14/2016
 *
 * DESCRIPTION   : Destroy MoCA control device, will be called by MxL_MoCA_DrvExit()
 *
 *
 * IN PARAMS     : EPHY Addr of the MxL MoCA SOC in the specified MDIO bus and Device handle 
 *
 * OUT PARAMS    : none
 *
 * RETURN VALUE  : none
 *--------------------------------------------------------------------------------------*/

void MxL_MoCA_OsalDestroyMocaDev(uint8_t ePhyAddr, void* pDev)
{
  MXL_ENTER_FUNCTION(TRACE_OSWP, "ephy addr = %d",ePhyAddr);

  // Remove Sysfs files to display lof and link status
  device_remove_file((struct device *)pDev, &dev_attr_lof);
  device_remove_file((struct device *)pDev, &dev_attr_link_status);
  device_remove_file((struct device *)pDev, &dev_attr_adm_status);
  device_remove_file((struct device *)pDev, &dev_attr_bridge_ctrl_req_status);
  device_remove_file((struct device *)pDev, &dev_attr_net_proxy_req_status);
  device_remove_file((struct device *)pDev, &dev_attr_beacon_power_dist_status);
  device_remove_file((struct device *)pDev, &dev_attr_privacy_update_status);

  device_destroy(pClass, MKDEV(major, ePhyAddr));
  
  MXL_EXIT_FUNCTION(TRACE_OSWP, "status = %d", ret);
}

/*----------------------------------------------------------------------------------------
 * FUNCTION NAME : MxL_MoCA_OsalDestroyMocaDevClass
 *
 *
 * DATE CREATED  : Oct/14/2016
 *
 * DESCRIPTION   : Destroy MoCA control device class, will be called by MxL_MoCA_DrvExit()
 *
 *
 * IN PARAMS     : none
 *
 * OUT PARAMS    : none
 *
 * RETURN VALUE  : none
 *--------------------------------------------------------------------------------------*/

void MxL_MoCA_OsalDestroyMocaDevClass(void)
{
  class_destroy(pClass);
}


/*----------------------------------------------------------------------------------------
 * FUNCTION NAME : MxL_MoCA_OsalAllocGetTimeInit
 *
 *
 * DATE CREATED  : Sep/20/2016
 *
 * DESCRIPTION   : Alloc time value and get init time
 *
 * IN PARAMS     : tv  Pointer to time value pointer
 *
 * OUT PARAMS    : None
 * RETURN VALUE  : None
 *--------------------------------------------------------------------------------------*/

void MxL_MoCA_OsalAllocGetTimeInit(void **tv)
{
  *tv = MxL_MoCA_OsalAlloc(sizeof(apposal_timeval_t));

  if (*tv)
    memset(*tv, 0, sizeof(apposal_timeval_t));
}

/*----------------------------------------------------------------------------------------
 * FUNCTION NAME : MxL_MoCA_OsalGetTime
 *
 *
 * DATE CREATED  : Sep/20/2016
 *
 * DESCRIPTION   : Get current time
 *
 * IN PARAMS     : tv  Pointer to time value
 *
 * OUT PARAMS    : None
 * RETURN VALUE  : None
 *--------------------------------------------------------------------------------------*/

void MxL_MoCA_OsalGetTime(void *tv)
{
  apposal_timeval_t *ptv;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)) && (LINUX_VERSION_CODE < KERNEL_VERSION(3, 17, 0))
  struct timespec ts;
#endif

  if (!tv)
    return;

  ptv = tv;

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)) && (LINUX_VERSION_CODE < KERNEL_VERSION(3, 17, 0))
  getrawmonotonic(&ts);
  monotonic_to_bootbased(&ts);
  ptv->timeVal.tv_sec = ts.tv_sec;
  ptv->timeVal.tv_usec = ts.tv_nsec/1000;
#else
  do_gettimeofday(&ptv->timeVal);
#endif
}

/*----------------------------------------------------------------------------------------
 * FUNCTION NAME : MxL_MoCA_OsalAllocGetTime
 *
 *
 * DATE CREATED  : Sep/20/2016
 *
 * DESCRIPTION   : tv  Alloc time value and get current time
 *
 * IN PARAMS     : Pointer to time value pointer
 *
 * OUT PARAMS    : None
 * RETURN VALUE  : None
 *--------------------------------------------------------------------------------------*/

void MxL_MoCA_OsalAllocGetTime(void **tv)
{
  apposal_timeval_t *ptv ;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)) && (LINUX_VERSION_CODE < KERNEL_VERSION(3, 17, 0))
  struct timespec ts;
#endif

  if (!tv)
    return;

  if (tv && (*tv))
  {
    ptv = *tv;
  }
  else
  {
    ptv = MxL_MoCA_OsalAlloc(sizeof(apposal_timeval_t));
  }

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)) && (LINUX_VERSION_CODE < KERNEL_VERSION(3, 17, 0))
  getrawmonotonic(&ts);
  monotonic_to_bootbased(&ts);
  ptv->timeVal.tv_sec = ts.tv_sec;
  ptv->timeVal.tv_usec = ts.tv_nsec/1000;
#else
  do_gettimeofday(&ptv->timeVal);
#endif

  if (*tv == 0)
    *tv = ptv;
}

/*----------------------------------------------------------------------------------------
 * FUNCTION NAME : MxL_MoCA_OsalDiffTimeInUs
 *
 *
 * DATE CREATED  : Sep/20/2016
 *
 * DESCRIPTION   : Get time difference by us unit
 *
 * IN PARAMS     : tv1  Pointer to time value1
 * IN PARAMS     : tv2  Pointer to time value2
 *
 * OUT PARAMS    : None
 * RETURN VALUE  : Time difference by us unit
 *--------------------------------------------------------------------------------------*/

long MxL_MoCA_OsalDiffTimeInUs(void *tv1, void *tv2)
{
  apposal_timeval_t *ptv1 = tv1;
  apposal_timeval_t *ptv2 = tv2;

  if (tv1 == 0 || tv2 == 0)
  {
    //MxL_MoCA_OsalPrintLog(TRACE_CPCR, L_INFO, "NULL Pointer\n");
    return 0;
  }

  return ((ptv1->timeVal.tv_sec - ptv2->timeVal.tv_sec) * 1000000) +
         (ptv1->timeVal.tv_usec - ptv2->timeVal.tv_usec);
}

/*----------------------------------------------------------------------------------------
 * FUNCTION NAME : MxL_MoCA_OsalDiffTime
 *
 *
 * DATE CREATED  : Sep/20/2016
 *
 * DESCRIPTION   : Get time difference by sec
 *
 * IN PARAMS     : tv1  Pointer to time value1
 * IN PARAMS     : tv2  Pointer to time value2
 *
 * OUT PARAMS    : None
 * RETURN VALUE  : Time difference by sec
 *--------------------------------------------------------------------------------------*/

long MxL_MoCA_OsalDiffTime(void *tv1, void *tv2)
{
  apposal_timeval_t *ptv1 = tv1;
  apposal_timeval_t *ptv2 = tv2;

  if (tv1 == 0 || tv2 == 0)
  {
    //MxL_MoCA_OsalPrintLog(TRACE_CPCR, L_INFO, "NULL Pointer\n");
    return 0;
  }


  return ptv1->timeVal.tv_sec - ptv2->timeVal.tv_sec ;
}

/*----------------------------------------------------------------------------------------
 * FUNCTION NAME : MxL_MoCA_OsalAssignTime
 *
 *
 * DATE CREATED  : Sep/20/2016
 *
 * DESCRIPTION   : Assign time
 *
 * IN PARAMS     : tv1  Pointer to destion time value
 * IN PARAMS     : tv2  Pointer to source time value
 *
 * OUT PARAMS    : None
 * RETURN VALUE  : None
 *--------------------------------------------------------------------------------------*/

void MxL_MoCA_OsalAssignTime(void *tv1, void *tv2)
{
  apposal_timeval_t *ptv1 = tv1;
  apposal_timeval_t *ptv2 = tv2;
  if (tv1 == 0 || tv2 == 0)
  {
    //MxL_MoCA_OsalPrintLog(TRACE_CPCR, L_INFO, "NULL Pointer\n");
    return;
  }
  memcpy(ptv1, ptv2, sizeof(apposal_timeval_t));
}

/*----------------------------------------------------------------------------------------
 * FUNCTION NAME : MxL_MoCA_OsalFreeGetTime
 *
 *
 * DATE CREATED  : Sep/20/2016
 *
 * DESCRIPTION   : Free time
 *
 * IN PARAMS     : tv  Pointer to destion time value pointer
 *
 * OUT PARAMS    : None
 * RETURN VALUE  : None
 *--------------------------------------------------------------------------------------*/

void MxL_MoCA_OsalFreeGetTime(void *tv)
{
  MxL_MoCA_OsalFree(tv,  sizeof(apposal_timeval_t));
}

/*----------------------------------------------------------------------------------------
 * FUNCTION NAME : MxL_MoCA_OsalAlloc
 *
 *
 * DATE CREATED  : Sep/20/2016
 *
 * DESCRIPTION   : Allocates memory
 *
 * IN PARAMS     : size   Size of allocate
 *
 * OUT PARAMS    : None
 * RETURN VALUE  : Pointer to allocated block if successful, Null pointer otherwise
 *--------------------------------------------------------------------------------------*/

void *MxL_MoCA_OsalAlloc(int32_t size)
{
  void *pMem;

  if (size < KMALLOC_MAX)
    pMem = kmalloc(size, GFP_KERNEL /* | __GFP_DMA */);
  else
    pMem = (void *)__get_free_pages(GFP_KERNEL, get_order(size));

  return (pMem);
}

/*----------------------------------------------------------------------------------------
 * FUNCTION NAME : MxL_MoCA_OsalFree
 *
 *
 * DATE CREATED  : Sep/20/2016
 *
 * DESCRIPTION   : Free memory
 *
 * IN PARAMS     : pMem  Allocation pointer from get free pages
 * IN PARAMS     : size  Requested block size
 *
 * OUT PARAMS    : None
 * RETURN VALUE  : None
 *--------------------------------------------------------------------------------------*/

void MxL_MoCA_OsalFree(void *pMem, int32_t size)
{
  if (size < KMALLOC_MAX)
    kfree(pMem);
  else
    free_pages((unsigned long)pMem, get_order(size));

}

/*----------------------------------------------------------------------------------------
 * FUNCTION NAME : MxL_MoCA_OsalMemcpy
 *
 *
 * DATE CREATED  : Sep/20/2016
 *
 * DESCRIPTION   : Copies size bytes of memory from pFrom to pTo
 *
 * IN PARAMS     : pTo    Pointer to destination memory area
 * IN PARAMS     : pFrom  Pointer to source memory area
 * IN PARAMS     : size   Number of bytes to copy
 *
 * OUT PARAMS    : None
 * RETURN VALUE  : None
 *--------------------------------------------------------------------------------------*/

void MxL_MoCA_OsalMemcpy(void *pTo, void *pFrom, int32_t size)
{
  memcpy(pTo, pFrom, size);
}

/*----------------------------------------------------------------------------------------
 * FUNCTION NAME : MxL_MoCA_OsalMemset
 *
 *
 * DATE CREATED  : Sep/20/2016
 *
 * DESCRIPTION   : Sets size bytes of memory to a given byte value
 *
 * IN PARAMS     : pMem  Pointer to block of memory to set
 * IN PARAMS     : val   Value to set each byte to
 * IN PARAMS     : size  Number of bytes to set
 *
 * OUT PARAMS    : None
 * RETURN VALUE  : None
 *--------------------------------------------------------------------------------------*/

void MxL_MoCA_OsalMemset(void *pMem, int32_t val, int32_t size)
{
  memset(pMem, val, size);
}

/*----------------------------------------------------------------------------------------
 * FUNCTION NAME : MxL_MoCA_OsalPrintLog
 *
 *
 * DATE CREATED  : Sep/20/2016
 *
 * DESCRIPTION   : Variable argument OS print
 *
 *   KERN_EMERG      "<0>"   / * system is unusable                   * /
 *   KERN_ALERT      "<1>"   / * action must be taken immediately     * /
 *   KERN_CRIT       "<2>"   / * critical conditions                  * /
 *   KERN_ERR        "<3>"   / * error conditions                     * /
 *   KERN_WARNING    "<4>"   / * warning conditions                   * /
 *   KERN_NOTICE     "<5>"   / * normal but significant condition     * /
 *   KERN_INFO       "<6>"   / * informational                        * /
 *   KERN_DEBUG      "<7>"   / * debug-level messages                 * /
 *
 * IN PARAMS     : mod  The modules to be traced
 * IN PARAMS     : lev  Severity level
 * IN PARAMS     : fmt  Printf format string
 * IN PARAMS     : ...  Printf format arguments
 *
 * OUT PARAMS    : None
 * RETURN VALUE  : None
 *--------------------------------------------------------------------------------------*/

void MxL_MoCA_OsalPrintLog(int32_t mod, int32_t lev, const int8_t *fmt, ...)
{
  va_list args;
  static int8_t printk_buf[LOG_PRINT_BUFSIZE];

  /* When you need to see lots message pumped out, make HOST_OS_PRINTLOG_THRESHOLD HIGHER,
   * for example, you call the API with lev L_VERBOSE, and make HOST_OS_PRINTLOG_THRESHOLD = L_DBG*/
  if (lev <= HOST_OS_PRINTLOG_THRESHOLD)
  {
    /* Emit the output into the temporary buffer */
    va_start(args, fmt);
    vsnprintf(printk_buf, sizeof(printk_buf), fmt, args);
    va_end(args);

    switch (lev)
    {
      case L_ERR:
        printk(KERN_ERR     HPFMT, DRV_NAME, printk_buf); //changed to KERN_EMERG
        break ;
      case L_WARN:
        printk(KERN_WARNING HPFMT, DRV_NAME, printk_buf); // Change to KERN_ALERT
        break ;
      case L_INFO:
        printk(KERN_INFO    HPFMT, DRV_NAME, printk_buf);
        break ;
      case L_VERBOSE:
        printk(KERN_ERR     HPFMT, DRV_NAME, printk_buf);
        break ;
      case L_DBG   :
        printk(KERN_DEBUG   HPFMT, DRV_NAME, printk_buf);
        break ;
    }
  }
  else if (lev <= logLev)
  {
    /* Emit the output into the temporary buffer */
    va_start(args, fmt);
    vsnprintf(printk_buf, sizeof(printk_buf), fmt, args);
    va_end(args);

    printk(KERN_ERR HPFMT, DRV_NAME, printk_buf);
  }
}

/*----------------------------------------------------------------------------------------
 * FUNCTION NAME : MxL_MoCA_OsalPrintString
 *
 *
 * DATE CREATED  : Sep/20/2016
 *
 * DESCRIPTION   : print string (used for progress bar)
 *                 (Note Cannot use MxL_MoCA_OsalPrintLog - it is adding newline)
 *
 * IN PARAMS     : pStr - character to print
 *
 * OUT PARAMS    : None
 * RETURN VALUE  : None
 *--------------------------------------------------------------------------------------*/

void MxL_MoCA_OsalPrintString(int8_t* pStr)
{
  printk ("%s", pStr);
}


/*----------------------------------------------------------------------------------------
 * FUNCTION NAME : MxL_MoCA_OsalSetLogLev
 *
 *
 * DATE CREATED  : Sep/20/2016
 *
 * DESCRIPTION   : Set log level
 *
 * IN PARAMS     : mod  The modules to be traced
 * IN PARAMS     : lev  Severity level
 *
 * OUT PARAMS    : None
 * RETURN VALUE  : None
 *--------------------------------------------------------------------------------------*/

void MxL_MoCA_OsalSetLogLev(int32_t mod, int32_t lev)
{
  logMod = mod;
  logLev = lev;
}

/*----------------------------------------------------------------------------------------
 * FUNCTION NAME : MxL_MoCA_OsalDelay
 *
 *
 * DATE CREATED  : Sep/20/2016
 *
 * DESCRIPTION   : Delays for about timeInUsec microseconds
 *
 * IN PARAMS     : timeInUsec  Delay in microseconds
 *
 * OUT PARAMS    : None
 * RETURN VALUE  : None
 *--------------------------------------------------------------------------------------*/

void MxL_MoCA_OsalDelay(int32_t timeInUsec)
{
  if (timeInUsec < USEC_PER_MS)
  {
    udelay(timeInUsec);
  }
  else
  {
    mdelay(timeInUsec/USEC_PER_MS);
  }
}


/*----------------------------------------------------------------------------------------
 * FUNCTION NAME : MxL_MoCA_OsalDelayUsec
 *
 *
 * DATE CREATED  : Oct/02/2016
 *
 * DESCRIPTION   : Delays for about timeInUsec microseconds
 *
 * IN PARAMS     : timeInUsec  Delay in microseconds
 *
 * OUT PARAMS    : None
 * RETURN VALUE  : None
 *--------------------------------------------------------------------------------------*/

void MxL_MoCA_OsalDelayUsec(uint32_t timeInUsec)
{
  if (timeInUsec < USEC_PER_MS)
  {
    udelay(timeInUsec);
  }
  else
  {
    mdelay(timeInUsec/USEC_PER_MS);
  }
}

/*----------------------------------------------------------------------------------------
 * FUNCTION NAME : MxL_MoCA_OsalMSleep
 *
 *
 * DATE CREATED  : Sep/20/2016
 *
 * DESCRIPTION   : Sleep for about timeInMilliSec microseconds
 *
 * IN PARAMS     : timeInMilliSec  Sleep in MilliSec
 *
 * OUT PARAMS    : None
 * RETURN VALUE  : None
 *--------------------------------------------------------------------------------------*/

void MxL_MoCA_OsalMSleep(int32_t timeInMilliSec)
{
  msleep(timeInMilliSec);
}

/*----------------------------------------------------------------------------------------
 * FUNCTION NAME : MxL_MoCA_OsalSscanf
 *
 *
 * DATE CREATED  : Sep/20/2016
 *
 * DESCRIPTION   : Scans input buffer and according to format specified
 *
 * IN PARAMS     : buf  Input buffer
 * IN PARAMS     : fmt  Input format
 *
 * OUT PARAMS    : None
 * RETURN VALUE  : None
 *--------------------------------------------------------------------------------------*/

void MxL_MoCA_OsalSscanf(const int8_t *buf, const int8_t *fmt, ...)
{
  va_list args;

  va_start(args, fmt);
  vsscanf(buf, fmt, args);
  va_end(args);
}

/*----------------------------------------------------------------------------------------
 * FUNCTION NAME : MxL_MoCA_OsalTermLock
 *
 *
 * DATE CREATED  : Sep/20/2016
 *
 * DESCRIPTION   : Terminates a lock
 *
 * IN PARAMS     : pLock  Pointer to an initialized lock structure
 *
 * OUT PARAMS    : None
 * RETURN VALUE  : None
 *--------------------------------------------------------------------------------------*/

void MxL_MoCA_OsalTermLock(void *pLock)
{
  //Null code in current implemenation
}

/*----------------------------------------------------------------------------------------
 * FUNCTION NAME : MxL_MoCA_OsalTimerMod
 *
 *
 * DATE CREATED  : Sep/20/2016
 *
 * DESCRIPTION   : Modifies a timer
 *
 * IN PARAMS     : pTmr    Pointer to the timer to deactivate
 * IN PARAMS     : timeout Timeout value from MxL_MoCA_Osaltimer_expire_seconds
 *
 * OUT PARAMS    : None
 * RETURN VALUE  : 1 for the timer was active (not expired), 0 for the timer was not active
 *--------------------------------------------------------------------------------------*/

int32_t MxL_MoCA_OsalTimerMod(void *pTmr, unsigned long timeout)
{
  osal_timer_t *tmr = (osal_timer_t *)pTmr ;

  return (mod_timer(&tmr->ostimer, timeout)) ;
}

/*----------------------------------------------------------------------------------------
 * FUNCTION NAME : MxL_MoCA_OsalCopyFromUser
 *
 *
 * DATE CREATED  : Sep/20/2016
 *
 * DESCRIPTION   : Copies a block from user space to kernel space
 *
 * IN PARAMS     : to      Pointer to a kernel space buffer to receive the data
 * IN PARAMS     : form    Pointer to a user space buffer as the source data
 * IN PARAMS     : nbytes  Length for copy
 *
 * OUT PARAMS    : None
 * RETURN VALUE  : 0 for success, others for number of bytes NOT copied
 *--------------------------------------------------------------------------------------*/

unsigned long MxL_MoCA_OsalCopyFromUser(void *to, const void *from, unsigned long nbytes)
{
  unsigned long n ;

  n = copy_from_user(to, from, nbytes);

  return(n) ;
}

/*----------------------------------------------------------------------------------------
 * FUNCTION NAME : MxL_MoCA_OsalCopyToUser
 *
 *
 * DATE CREATED  : Sep/20/2016
 *
 * DESCRIPTION   : Copies a block from kernel space to user space
 *
 * IN PARAMS     : to     Pointer to a user space buffer to receive the data
 * IN PARAMS     : from   Pointer to a kernel space buffer as the source data
 * IN PARAMS     : nbytes Length for copy
 *
 * OUT PARAMS    : None
 * RETURN VALUE  : 0 for success, others for number of bytes NOT copied
 *--------------------------------------------------------------------------------------*/

unsigned long MxL_MoCA_OsalCopyToUser(void *to, const void *from, unsigned long nbytes)
{
  unsigned long n ;

  n = copy_to_user(to, from, nbytes);

  return(n) ;
}

/*----------------------------------------------------------------------------------------
 * FUNCTION NAME : MxL_MoCA_OsalMutexInit
 *
 *
 * DATE CREATED  : Sep/20/2016
 *
 * DESCRIPTION   : Initialize a mutex
 *
 * IN PARAMS     : vmt  Pointer to the mutex structure
 *
 * OUT PARAMS    : None
 * RETURN VALUE  : None
 *--------------------------------------------------------------------------------------*/

void MxL_MoCA_OsalMutexInit(void *vmt)
{
  init_MUTEX((struct semaphore *)vmt) ;    /// abstraction
}

/*----------------------------------------------------------------------------------------
 * FUNCTION NAME : MxL_MoCA_OsalMutexRelease
 *
 *
 * DATE CREATED  : Sep/20/2016
 *
 * DESCRIPTION   : Release a mutex
 *                 Do NOT call this from an ISR!
 *                 Do NOT call this if you are not the current holder of the mutex.
 *
 * IN PARAMS     : vmt  Pointer to the mutex structure
 *
 * OUT PARAMS    : None
 * RETURN VALUE  : None
 *--------------------------------------------------------------------------------------*/

void MxL_MoCA_OsalMutexRelease(void *vmt)
{
  up((struct semaphore *)vmt) ;   /// abstraction
}

/*----------------------------------------------------------------------------------------
 * FUNCTION NAME : MxL_MoCA_OsalMutexAcquire
 *
 *
 * DATE CREATED  : Sep/20/2016
 *
 * DESCRIPTION   : Acquire a mutex
 *                 Might sleep, when this returns you have the mutex.
 *                 This is NOT the preferred method. See MxL_MoCA_Osalmutex_acquire_intr().
 *                 Do NOT call this from an ISR!
 *
 * IN PARAMS     : vmt  Pointer to the mutex structure
 *
 * OUT PARAMS    : None
 * RETURN VALUE  : None
 *--------------------------------------------------------------------------------------*/

void MxL_MoCA_OsalMutexAcquire(void *vmt)
{
  down((struct semaphore *)vmt) ;   /// abstraction
}

/*----------------------------------------------------------------------------------------
 * FUNCTION NAME : MxL_MoCA_OsalMutexAcquireIntr
 *
 *
 * DATE CREATED  : Sep/20/2016
 *
 * DESCRIPTION   : Acquire (down) a mutex. Interruptible
 *
 *                 Might sleep.
 *                 When this returns 0 you have the mutex.
 *                 When this returns -EINTR you have been interrupted
 *                 and must bail out. You do NOT have the mutex. The
 *                 purpose here is to allow the caller, probably a
 *                 user space app, to be interrupted and die quickly
 *                 and quietly; rather than becoming a zombie because
 *                 the driver won't let go.
 *                 This is the preferred method.
 *                 Do NOT call this from an ISR!
 *
 * IN PARAMS     : vmt  Pointer to the mutex structure
 *
 * OUT PARAMS    : None
 * RETURN VALUE  : 0       Got the mutex
 *                 EINTR   Have been interrupted, no mutex
 *--------------------------------------------------------------------------------------*/

int32_t MxL_MoCA_OsalMutexAcquireIntr(void *vmt)
{
  int32_t rc;

  MXL_ENTER_FUNCTION(TRACE_OSWP, "vmt = %p", vmt);

  rc = down_interruptible((struct semaphore *)vmt) ;   /// abstraction

  MXL_EXIT_FUNCTION(TRACE_OSWP, "rc = %d", rc);

  return(rc) ;
}

/*----------------------------------------------------------------------------------------
 * FUNCTION NAME : MxL_MoCA_OsalTimerExpireSeconds
 *
 *
 * DATE CREATED  : Sep/20/2016
 *
 * DESCRIPTION   : Calculate a future expiration point in seconds
 *
 * IN PARAMS     : future  Number of seconds into the future
 *
 * OUT PARAMS    : None
 * RETURN VALUE  : A number for use in the timer expiration
 *--------------------------------------------------------------------------------------*/

unsigned long MxL_MoCA_OsalTimerExpireSeconds(uint32_t future)
{

  return (jiffies + (future * HZ));
}

/*----------------------------------------------------------------------------------------
 * FUNCTION NAME : MxL_MoCA_OsalTimerExpireTicks
 *
 *
 * DATE CREATED  : Sep/20/2016
 *
 * DESCRIPTION   : Calculate a future expiration point in jiffies
 *
 * IN PARAMS     : future  Number of ticks into the future
 *
 * OUT PARAMS    : None
 * RETURN VALUE  : A number for use in the timer expiration
 *--------------------------------------------------------------------------------------*/

unsigned long MxL_MoCA_OsalTimerExpireTicks(uint32_t future)
{

  return (jiffies + future);
}

// MxL: do not use timer in current code
#if defined(INCLUDE_OSAL_WQT_TIMER)

/*----------------------------------------------------------------------------------------
 * FUNCTION NAME : MxL_MoCA_OsalWqtAlloc
 *
 *
 * DATE CREATED  : Sep/20/2016
 *
 * DESCRIPTION   : Allocates a wqt entry from the heap
 *
 * IN PARAMS     : None
 *
 * OUT PARAMS    : None
 * RETURN VALUE  : 0       Allocation failure
 *                 Othres  Void pointer to opaque data
 *--------------------------------------------------------------------------------------*/

void *MxL_MoCA_OsalWqtAlloc(void)
{
  osal_wqt_t       *wqt ;

  wqt = kmalloc(sizeof(osal_wqt_t), GFP_KERNEL) ;

  return ((void *)wqt);
}

/*----------------------------------------------------------------------------------------
 * FUNCTION NAME : MxL_MoCA_OsalWqtFree
 *
 *
 * DATE CREATED  : Sep/20/2016
 *
 * DESCRIPTION   : Free a wqt entry from the heap
 *
 * IN PARAMS     : vwqt Pointer to the osal_wqt_t to be freed
 *
 * OUT PARAMS    : None
 * RETURN VALUE  : None
 *--------------------------------------------------------------------------------------*/

void MxL_MoCA_OsalWqtFree(void *vwqt)
{

  kfree(vwqt) ;
}

/*----------------------------------------------------------------------------------------
 * FUNCTION NAME : MxL_MoCA_OsalWqtTimerInit
 *
 *
 * DATE CREATED  : Sep/20/2016
 *
 * DESCRIPTION   : Initialize a timer and register it
 *
 * IN PARAMS     : vwqt Pointer to the osal_wqt_t with timer to initialize
 *
 * OUT PARAMS    : None
 * RETURN VALUE  : None
 *--------------------------------------------------------------------------------------*/

void MxL_MoCA_OsalWqtTimerInit(void *vwqt)
{
  osal_wqt_t       *wqt = (osal_wqt_t *)vwqt ;

  wqt->wqt_allocated = 1 ;  /// mark the timer inited
  init_timer(&wqt->wqt_timer) ;
}

/*----------------------------------------------------------------------------------------
 * FUNCTION NAME : MxL_MoCA_OsalWqtTimerDel
 *
 *
 * DATE CREATED  : Sep/20/2016
 *
 * DESCRIPTION   : Delete a wqt timer
 *
 * IN PARAMS     : vwqt Pointer to the osal_wqt_t with timer to deactivate
 *
 * OUT PARAMS    : None
 * RETURN VALUE  : None
 *--------------------------------------------------------------------------------------*/

void MxL_MoCA_OsalWqtTimerDel(void *vwqt)
{
  osal_wqt_t       *wqt = (osal_wqt_t *)vwqt ;

  if (wqt->wqt_allocated)
  {
    del_timer(&wqt->wqt_timer) ;
  }
}

/*----------------------------------------------------------------------------------------
 * FUNCTION NAME : MxL_MoCA_OsalWqtTimerDelSync
 *
 *
 * DATE CREATED  : Sep/20/2016
 *
 * DESCRIPTION   : Deactivate a registered timer
 *
 * IN PARAMS     : vwqt Pointer to the osal_wqt_t with timer to deactivate
 *
 * OUT PARAMS    : None
 * RETURN VALUE  : 1 the timer was active (not expired), 0 the timer was not active
 *--------------------------------------------------------------------------------------*/

int32_t MxL_MoCA_OsalWqtTimerDelSync(void *vwqt)
{
  osal_wqt_t       *wqt = (osal_wqt_t *)vwqt ;

  return (del_timer_sync(&wqt->wqt_timer)) ;
}

/*----------------------------------------------------------------------------------------
 * FUNCTION NAME : MxL_MoCA_OsalWqtTimerMod
 *
 *
 * DATE CREATED  : Sep/20/2016
 *
 * DESCRIPTION   : Modifies a timer
 *
 * IN PARAMS     : vwqt    Pointer to the osal_wqt_t with timer to modify
 *                 timeout Timeout value from MxL_MoCA_Osaltimer_expire_seconds
                           or MxL_MoCA_Osaltimer_expire_ticks.
 *
 * OUT PARAMS    : None
 * RETURN VALUE  : 1 the timer was active (not expired), 0 the timer was not active
 *--------------------------------------------------------------------------------------*/

int32_t MxL_MoCA_OsalWqtTimerMod(void *vwqt, unsigned long timeout)
{
  osal_wqt_t       *wqt = (osal_wqt_t *)vwqt ;

  return (mod_timer(&wqt->wqt_timer, timeout)) ;
}

/*----------------------------------------------------------------------------------------
 * FUNCTION NAME : MxL_MoCA_OsalWqtTimerAdd
 *
 *
 * DATE CREATED  : Sep/20/2016
 *
 * DESCRIPTION   : Add a timer, call this after HostOS_timer_setup
 *
 * IN PARAMS     : vwqt    Pointer to the osal_wqt_t with timer to add
 *
 * OUT PARAMS    : None
 * RETURN VALUE  : 1 the timer was active (not expired), 0 the timer was not active
 *--------------------------------------------------------------------------------------*/

void MxL_MoCA_OsalWqtTimerAdd(void *vwqt)
{
  osal_wqt_t       *wqt = (osal_wqt_t *)vwqt ;

  add_timer(&wqt->wqt_timer) ;
}

/*----------------------------------------------------------------------------------------
 * FUNCTION NAME : MxL_MoCA_OsalWqtTimerSetup
 *
 *
 * DATE CREATED  : Sep/20/2016
 *
 * DESCRIPTION   : Sets up a timer with function and user data
 *
 * IN PARAMS     : vwqt    Pointer to the osal_wqt_t with timer to  init
 *                 func    Callback function to call at timer expiration
 *                 data    Data to pass to callback
 *
 * OUT PARAMS    : None
 * RETURN VALUE  : None
 *--------------------------------------------------------------------------------------*/

void MxL_MoCA_OsalWqtTimerSetup(void *vwqt, timer_function_t func, unsigned long data)
{
  osal_wqt_t       *wqt = (osal_wqt_t *)vwqt ;

  wqt->wqt_timer.function = func ;
  wqt->wqt_timer.data     = data ;
}

/*----------------------------------------------------------------------------------------
 * FUNCTION NAME : MxL_MoCA_OsalWqtTimerSetTimeout
 *
 *
 * DATE CREATED  : Sep/20/2016
 *
 * DESCRIPTION   : Sets a timer's time out
 *
 * IN PARAMS     : vwqt    Pointer to the osal_wqt_t with timer to set
 *                 timeout Timeout value from MxL_MoCA_Osaltimer_expire_seconds
 *                         or MxL_MoCA_Osaltimer_expire_ticks
 *
 * OUT PARAMS    : None
 * RETURN VALUE  : None
 *--------------------------------------------------------------------------------------*/

void MxL_MoCA_OsalWqtTimerSetTimeout(void *vwqt, unsigned long timeout)
{
  osal_wqt_t       *wqt = (osal_wqt_t *)vwqt ;

  wqt->wqt_timer.expires  = timeout ;
}

/*----------------------------------------------------------------------------------------
 * FUNCTION NAME : MxL_MoCA_OsalWqtWaitqInit
 *
 *
 * DATE CREATED  : Sep/20/2016
 *
 * DESCRIPTION   : Inits a wait queue head in a osal_wqt_t
 *
 * IN PARAMS     : vwqt    Pointer to the osal_wqt_t with wait queue to init
 *
 * OUT PARAMS    : None
 * RETURN VALUE  : None
 *--------------------------------------------------------------------------------------*/

void MxL_MoCA_OsalWqtWaitqInit(void *vwqt)
{
  osal_wqt_t       *wqt = (osal_wqt_t *)vwqt ;

  init_waitqueue_head(&wqt->wqt_wq);
}

/*----------------------------------------------------------------------------------------
 * FUNCTION NAME : MxL_MoCA_OsalWqtWaitqWakeupIntr
 *
 *
 * DATE CREATED  : Sep/20/2016
 *
 * DESCRIPTION   : Wakes up a wait queue in a osal_wqt_t
 *
 * IN PARAMS     : vwqt    Pointer to the osal_wqt_t with wait queue to wake
 *
 * OUT PARAMS    : None
 * RETURN VALUE  : None
 *--------------------------------------------------------------------------------------*/

void MxL_MoCA_OsalWqtWaitqWakeupIntr(void *vwqt)
{
  osal_wqt_t       *wqt = (osal_wqt_t *)vwqt ;

  if (wqt)
  {
    wake_up_interruptible(&wqt->wqt_wq);
  }
}

/*----------------------------------------------------------------------------------------
 * FUNCTION NAME : MxL_MoCA_OsalWqtWaitqWaitEventIntr
 *
 *
 * DATE CREATED  : Sep/20/2016
 *
 * DESCRIPTION   : Waits for a wait queue event in a osal_wqt_t
 *
 * IN PARAMS     : vwqt    Pointer to the osal_wqt_t with wait queue to wait on
 *                 func    Fountion execute in the interrupt
 *                 vp      Pointer to void data
 *
 * OUT PARAMS    : None
 * RETURN VALUE  : None
 *--------------------------------------------------------------------------------------*/

void MxL_MoCA_OsalWqtWaitqWaitEventIntr(void *vwqt, osal_wqt_condition func, void *vp)
{
  osal_wqt_t       *wqt = (osal_wqt_t *)vwqt ;

  wait_event_interruptible(wqt->wqt_wq,  /** pass by value */
                           func(vp));
}

#endif  // INCLUDE_OSAL_TIMER

DECLARE_MUTEX(reset_mutex);

/*----------------------------------------------------------------------------------------
 * FUNCTION NAME : MxL_MoCA_OsalSignalPending
 *
 *
 * DATE CREATED  : Sep/20/2016
 *
 * DESCRIPTION   : Check pending signal of task
 *
 * IN PARAMS     : vtask Pointer to the task
 *
 * OUT PARAMS    : None
 * RETURN VALUE  : 0 - no pending signal, 1 - has pendign signal
 *--------------------------------------------------------------------------------------*/

int32_t MxL_MoCA_OsalSignalPending(void *vtask)
{
  struct task_struct *p;

  if (vtask== NULL)
    p = current;
  else
    p = (struct task_struct *)vtask;

  return signal_pending(p);
}

#if defined(CANDD_DRVR_SUPPORT)

/** declaring mutexes for kernel thread start/stop */
DECLARE_MUTEX(osal_thread_mutex);

osal_kthread_t hostosThread;

/*----------------------------------------------------------------------------------------
 * FUNCTION NAME : MxL_MoCA_OsalJiffiesToMsecs
 *
 *
 * DATE CREATED  : Sep/20/2016
 *
 * DESCRIPTION   : Convert jiffies to milliseconds
 *
 * IN PARAMS     : j  jiffies
 *
 * OUT PARAMS    : None
 * RETURN VALUE  : Number of milliseconds
 *--------------------------------------------------------------------------------------*/

static inline uint32_t MxL_MoCA_OsalJiffiesToMsecs(const unsigned long j)
{
  uint32_t ret;
#if HZ <= USEC_PER_MS && !(USEC_PER_MS % HZ)
  ret = (USEC_PER_MS / HZ) * j;
#elif HZ > USEC_PER_MS && !(HZ % USEC_PER_MS)
  ret = (j + (HZ / USEC_PER_MS) - 1)/(HZ / USEC_PER_MS);
#else
  ret = (j * USEC_PER_MS) / HZ;
#endif
  return ret;
}

/*----------------------------------------------------------------------------------------
 * FUNCTION NAME : MxL_MoCA_OsalMsecsToJiffies
 *
 *
 * DATE CREATED  : Sep/20/2016
 *
 * DESCRIPTION   : Convert milliseconds to jiffies
 *
 * IN PARAMS     : m  milliseconds
 *
 * OUT PARAMS    : None
 * RETURN VALUE  : Number of jiffies
 *--------------------------------------------------------------------------------------*/

static inline unsigned long MxL_MoCA_OsalMsecsToJiffies(const uint32_t m)
{
  unsigned long ret;

#if HZ <= USEC_PER_MS && !(USEC_PER_MS % HZ)
  ret = (m + (USEC_PER_MS / HZ) - 1) / (USEC_PER_MS / HZ);
#elif HZ > USEC_PER_MS && !(HZ % USEC_PER_MS)
  ret = m *(HZ / USEC_PER_MS);
#else
  ret = (m * HZ + USEC_PER_MS - 1) / USEC_PER_MS;
#endif

  if (m > MxL_MoCA_OsalJiffiesToMsecs(MAX_JIFFY_OFFSET))
    ret = MAX_JIFFY_OFFSET;

  return ret;
}

/*----------------------------------------------------------------------------------------
 * FUNCTION NAME : MxL_MoCA_OsalMsleepInterruptible
 *
 *
 * DATE CREATED  : Sep/20/2016
 *
 * DESCRIPTION   : Sleep waiting for waitqueue interruptions
 *
 * IN PARAMS     : msecs  Time in milliseconds to sleep for
 *
 * OUT PARAMS    : None
 * RETURN VALUE  : None
 *--------------------------------------------------------------------------------------*/

void MxL_MoCA_OsalMsleepInterruptible(uint32_t msecs)
{
  unsigned long timeout = MxL_MoCA_OsalMsecsToJiffies(msecs) + 1;

  while (timeout)
  {
    __set_current_state(TASK_INTERRUPTIBLE);
    timeout = schedule_timeout(timeout);
  }
}

/*----------------------------------------------------------------------------------------
 * FUNCTION NAME : MxL_MoCA_OsalThreadInterFunc
 *
 *
 * DATE CREATED  : Sep/20/2016
 *
 * DESCRIPTION   : Kernel thread internal function
 *
 * IN PARAMS     : pdata Pointer to thread struct
 *
 * OUT PARAMS    : None
 * RETURN VALUE  : None
 *--------------------------------------------------------------------------------------*/

static void MxL_MoCA_OsalThreadInterFunc(void *pdata)
{
  osal_kthread_t *pThread = (osal_kthread_t *)pdata;

  if (pdata != NULL)
  {

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,8,0)
    allow_signal(SIGKILL);
    allow_signal(SIGTERM);
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,0)
    daemonize(hostosThread.name, 0);
    allow_signal(SIGKILL);
    allow_signal(SIGTERM);
#else
    daemonize();
#endif

    /** kernel thread main function */
    pThread->func(pThread->arg);
  }

  return ;
}


/*----------------------------------------------------------------------------------------
 * FUNCTION NAME : MxL_MoCA_OsalTestPtrErrCondition
 *
 *
 * DATE CREATED  : Sep/20/2016
 *
 * DESCRIPTION   : Checks for ERR_PTR(-ENOMEM).
 *
 * IN PARAMS     : A previously kernel returned pointer from calls such as kthread...
 *
 * OUT PARAMS    : None
 * RETURN VALUE  : MXL_MOCA_OK or MXL_MOCA_ERR
 *--------------------------------------------------------------------------------------*/

int32_t MxL_MoCA_OsalTestPtrErrCondition(void *pTask)
{
  MXL_MOCA_STATUS_E ret = MXL_MOCA_OK;
  
  if (IS_ERR(pTask))
  {
    ret = MXL_MOCA_ERR;
  }
  return ret;
}

/*----------------------------------------------------------------------------------------
 * FUNCTION NAME : MxL_MoCA_OsalThreadCreateAndStart
 *
 *
 * DATE CREATED  : Sep/20/2016
 *
 * DESCRIPTION   : create and wake a thread. A wrapper functiion for kthread_run.
 *
 * IN PARAMS     : threadfn  - the function to run until signal_pending(current).
 *                 data      - data ptr for threadfn
 *                 namefmt   - printf-style name for the thread
 *                 arg       - arguments
 *
 * OUT PARAMS    : ppTask    - double pointer to the kThread task
 * RETURN VALUE  : Returns the kthread or 
 *--------------------------------------------------------------------------------------*/

MXL_MOCA_STATUS_E MxL_MoCA_OsalThreadCreateAndStart(int32_t (*threadfn)(void *data),
                                                    void *data,
                                                    const int8_t *namefmt,
                                                    void *arg,
                                                    void **ppTask)
{
  *ppTask = kthread_run(threadfn, data, namefmt, arg);

  return MxL_MoCA_OsalTestPtrErrCondition(*ppTask);
}


/*----------------------------------------------------------------------------------------
 *
 * FUNCTION NAME : MxL_MoCA_OsalGetDaemonThreadStopStatus
 *
 *
 * DATE CREATED  : Sep/20/2016
 *
 * DESCRIPTION   : should this kthread return now (kthread_stop was called to stop thread)?
 *
 * IN PARAMS     : None
 *
 * OUT PARAMS    : None
 *
 * RETURN VALUE  : MXL_MOCA_KTHREAD_SHOULD_STOP if someone had called kthread_stop. 
 *                 else, MXL_MOCA_OK
 *
 *--------------------------------------------------------------------------------------*/
MXL_MOCA_STATUS_E MxL_MoCA_OsalGetDaemonThreadStopStatus()
{
  MXL_MOCA_STATUS_E status = MXL_MOCA_OK;
  
  if (kthread_should_stop() == true)
  {
    status = MXL_MOCA_KTHREAD_SHOULD_STOP;
  }
  return status;
}

/*----------------------------------------------------------------------------------------
 * FUNCTION NAME : MxL_MoCA_OsalThreadStart
 *
 *
 * DATE CREATED  : Sep/20/2016
 *
 * DESCRIPTION   : Start a keneral thread
 *
 * IN PARAMS     : pThreadID  Pointer to new kernel threadID
 *                 pName      Pointer to thread name string
 *                 func       Pointer to thread internal function
 *                 arg        Argument to pass to thread function
 *
 * OUT PARAMS    : None
 * RETURN VALUE  : 0 - success, others - failed
 *--------------------------------------------------------------------------------------*/

int32_t MxL_MoCA_OsalThreadStart(uint32_t *pThreadID, int8_t *pName, void (*func)(void *), void *arg)
{
  int32_t ret = MXL_MOCA_OK;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 8, 0)
  struct task_struct *tsk;
#endif

  down(&osal_thread_mutex);

  strncpy(hostosThread.name, pName,16);
  hostosThread.func = func;
  hostosThread.arg = arg;

  do
  {
    /** Create kernel thread */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 8, 0)
    tsk = kthread_run((int32_t (*)(void *))MxL_MoCA_OsalThreadInterFunc, &hostosThread, hostosThread.name);
    if (IS_ERR(tsk))
    {
      ret = MXL_MOCA_ERR;
      break;
    }

    hostosThread.threadID = (uintptr_t)get_pid(task_pid(tsk));
#else
    hostosThread.threadID = kernel_thread((int32_t (*)(void *))MxL_MoCA_OsalThreadInterFunc, &hostosThread, 0);
    if (hostosThread.threadID < 0)
    {
      ret = MXL_MOCA_ERR;
      break;
    }
#endif

    *pThreadID = hostosThread.threadID;
  } while (0);

  up(&osal_thread_mutex);

  return ret;
}

/*----------------------------------------------------------------------------------------
 * FUNCTION NAME : MxL_MoCA_OsalThreadStop
 *
 *
 * DATE CREATED  : Sep/20/2016
 *
 * DESCRIPTION   : Stop a keneral thread
 *
 * IN PARAMS     : threadID Kernel threadID that to be stopped
 *
 * OUT PARAMS    : None
 * RETURN VALUE  : 0 - success, others - failed
 *--------------------------------------------------------------------------------------*/

int32_t MxL_MoCA_OsalThreadStop(unsigned long threadID)
{
  int32_t ret = MXL_MOCA_OK;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 27)
  struct pid *pPid;
#endif

  down(&osal_thread_mutex);

  do
  {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 27)
    pPid = find_get_pid(threadID);
    if(pPid == NULL)
    {
      ret = MXL_MOCA_OK;
      break;
    }

    if ((ret = kill_pid(pPid, SIGKILL, 1)))
#else
    if ((ret = kill_proc(threadID, SIGKILL, 1)))
#endif
    {
      ret = MXL_MOCA_ERR;
      break;
    }
  } while (0);

  up(&osal_thread_mutex);

  return ret;
}

#endif

/*----------------------------------------------------------------------------------------
 * FUNCTION NAME : MxL_MoCA_OsalReadWord
 *
 *
 * DATE CREATED  : Sep/20/2016
 *
 * DESCRIPTION   : register/memory read access from kernel space
 *
 * IN PARAMS     : addr Address to read from
 *
 * OUT PARAMS    : None
 * RETURN VALUE  : 32 bits value from given address
 *--------------------------------------------------------------------------------------*/

uint32_t MxL_MoCA_OsalReadWord(void *addr)
{
  uint32_t rv ;

#if defined(CONFIG_ARCH_MAXLINEAR_ECB) && (defined(CONFIG_ARCH_IXP425) || defined(CONFIG_ARCH_IXP4XX))
  rv = *(volatile uint32_t *)addr ;
#else
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 0)
  rv = readl((const volatile void *)addr);
#else
  rv = readl(addr);
#endif
#endif

#if defined(CONFIG_ARCH_LAYERSCAPE)
  rv =  (rv & 0xff) << 24
       | (rv & 0xff00) << 8
       | (rv & 0xff0000) >> 8
       | (rv & 0xff000000) >> 24;
#endif
  return (rv);
}

/*----------------------------------------------------------------------------------------
 * FUNCTION NAME : MxL_MoCA_OsalWriteWord
 *
 *
 * DATE CREATED  : Sep/20/2016
 *
 * DESCRIPTION   : Register/Memory write access from kernel space
 *
 * IN PARAMS     : val   32 bit value to write
 *                 addr  Address to write to
 *
 * OUT PARAMS    : None
 * RETURN VALUE  : None
 *--------------------------------------------------------------------------------------*/

void MxL_MoCA_OsalWriteWord(uint32_t val, void *addr)
{
 
#if defined(CONFIG_ARCH_LAYERSCAPE)
  val =  (val & 0xff) << 24
       | (val & 0xff00) << 8
       | (val & 0xff0000) >> 8
       | (val & 0xff000000) >> 24;
#endif
#if defined(CONFIG_ARCH_MAXLINEAR_ECB) && (defined(CONFIG_ARCH_IXP425) || defined(CONFIG_ARCH_IXP4XX))
  *(volatile uint32_t *)addr = val ;
#else
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 0)
  writel(val, (volatile void *)addr);
#else
  writel(val, addr);
#endif
#endif
}

/*----------------------------------------------------------------------------------------
 * FUNCTION NAME : MxL_MoCA_OsalLogInfo
 *
 *
 * DATE CREATED  : Sep/20/2016
 *
 * DESCRIPTION   : Print the debug string
 *
 * IN PARAMS     : format  String format
 *
 * OUT PARAMS    : None
 * RETURN VALUE  : None
 *--------------------------------------------------------------------------------------*/

void MxL_MoCA_OsalLogInfo(int8_t* format, ...)
{
  int8_t linebuf[400];
  va_list ap;

  va_start(ap, format);
  vsnprintf(linebuf, sizeof(linebuf), format, ap);
  va_end(ap);

  printk(KERN_INFO DRV_NAME "%s\n", linebuf);
}

/*----------------------------------------------------------------------------------------
 * FUNCTION NAME : MxL_MoCA_OsalKthreadStop
 *
 *
 * DATE CREATED  : Sep/20/2016
 *
 * DESCRIPTION   : Stop a thread created by kthread_create
 *
 * IN PARAMS     : tsk Pointer to task structure
 *
 * OUT PARAMS    : None
 * RETURN VALUE  : The result of threadfn(), or -EINTR if wake_up_process()
 *--------------------------------------------------------------------------------------*/

int32_t MxL_MoCA_OsalKthreadStop(void *tsk)
{
  return kthread_stop(tsk);
}

/*----------------------------------------------------------------------------------------
 * FUNCTION NAME : MxL_MoCA_OsalGetPointerToFirmwareData
 *
 *
 * DATE CREATED  : Sep/20/2016
 *
 * DESCRIPTION   : Get the firmware data via the firmware index
 *
 * IN PARAMS     : pFw Pointer to MxL MoCA firmware
 *                 idx Index of firmware data to get
 *
 * OUT PARAMS    : None
 * RETURN VALUE  : Pointer to firmware data, or NULL if failure
 *--------------------------------------------------------------------------------------*/

void *MxL_MoCA_OsalGetPointerToFirmwareData(MXL_MOCA_FIRMWARE_T *pFw, int32_t idx)
{
  void *p = NULL;

  if (NULL != pFw)
  {
    do
    {
      if (idx < MXL_MOCA_CCPU_IDX || idx >= MXL_MOCA_MAX_FW_IDX)
        break;

      if (pFw->fw_entry[idx])
        p = (void *)((struct firmware*)pFw->fw_entry[idx])->data;

    } while (0);
  }
  else
  {
     MxL_MoCA_OsalPrintLog(TRACE_OSWP, L_ERR, "Get Pointer To Firmware Data: NULL\n");
  }

  return p;
}

/*----------------------------------------------------------------------------------------
 * FUNCTION NAME : MxL_MoCA_OsalGetFirmwareLen
 *
 *
 * DATE CREATED  : Sep/20/2016
 *
 * DESCRIPTION   : Get the firmware length via the firmware index
 *
 * IN PARAMS     : pFw Pointer to clink firmware
 *                 idx Index of firmware data to get length
 *
 * OUT PARAMS    : None
 * RETURN VALUE  : Length of firmware with index \a idx, or 0 if not found
 *--------------------------------------------------------------------------------------*/

int32_t  MxL_MoCA_OsalGetFirmwareLen(MXL_MOCA_FIRMWARE_T *pFw, int32_t idx)
{
  int32_t len = 0;

  if (NULL != pFw)
  {
    do
    {
      if (idx < MXL_MOCA_CCPU_IDX || idx >= MXL_MOCA_MAX_FW_IDX)
        break;

      if (pFw->fw_entry[idx])
        len = ((struct firmware *)pFw->fw_entry[idx])->size;

    } while (0);
  }

  return len;
}

/*----------------------------------------------------------------------------------------
 * FUNCTION NAME : MxL_MoCA_OsalFreeFirmwares
 *
 *
 * DATE CREATED  : Sep/20/2016
 *
 * DESCRIPTION   : Free firmwares
 *
 * IN PARAMS     : pFw  Pointer to firmware data
 *
 * OUT PARAMS    : None
 * RETURN VALUE  : None
 *--------------------------------------------------------------------------------------*/

void MxL_MoCA_OsalFreeFirmwares(MXL_MOCA_FIRMWARE_T *pFw)
{
  int32_t i;
#ifdef SET_CD_FW_SUPPORT
  MXL_MOCA_FIRMWARE_ENTRY_T *pfwentry;
#endif

  MXL_ENTER_FUNCTION(TRACE_OSWP, "pFw = %p", pFw);

  if (pFw != NULL)
  {
    for (i = 0; i < MXL_MOCA_MAX_FW_IDX; i++)
    {
      if (pFw->ret & (1 << i))
      {
#ifdef SET_CD_FW_SUPPORT
        if (gGetFwViaIoctl)
        {
          if (pFw->fw_entry[i])
          {
            pfwentry = (MXL_MOCA_FIRMWARE_ENTRY_T *)pFw->fw_entry[i];

            if (pfwentry->data && MXL_MOCA_CCPU_IDX != i) // ccpu elf put in global buf
            {
              MxL_MoCA_OsalFree(pfwentry->data, pfwentry->size);
            }

            MxL_MoCA_OsalFree(pFw->fw_entry[i], sizeof(MXL_MOCA_FIRMWARE_ENTRY_T));
            pFw->fw_entry[i] = NULL;
          }
        }
        else
#endif
        {
          if ((struct firmware*)pFw->fw_entry[i])
          {
            release_firmware((struct firmware *)pFw->fw_entry[i]);
          }
        }
      }
    }

    pFw->ret = 0;
  }

  MXL_EXIT_FUNCTION(TRACE_OSWP, "status = %d", (pFw != NULL) ? pFw->ret : MXL_MOCA_ERR);
}

/*----------------------------------------------------------------------------------------
 * FUNCTION NAME : MxL_MoCA_OsalGetFirmwareAndConfigData
 *
 *
 * DATE CREATED  : Sep/30/2016
 *
 * DESCRIPTION   : Setup firmware, this will request the ccpu elf and all configs from
 *                 user space for the device
 *
 * IN PARAMS     : pDev     MoCA device
 *               : pDevName Device name
 *               : pFw      Pointer to the mxl moca firmware struct
 *
 * OUT PARAMS    : None
 *
 * RETURN VALUE  : the bit mask for the firmware requested successfully
 *--------------------------------------------------------------------------------------*/

MXL_MOCA_STATUS_E MxL_MoCA_OsalGetFirmwareAndConfigData(uint32_t            chipTypeIndex,
                                                        void                *pDev,
                                                        const int8_t        *pDevName,
                                                        MXL_MOCA_FIRMWARE_T *pFw,
                                                        int32_t             *pFwBitMaskRet)
{
  MXL_MOCA_STATUS_E status = MXL_MOCA_OK;
  int32_t i;
  int8_t path[128];
  static const int8_t *pChipSuffix[MXL_MOCA_SOC_TYPE_MAX] = {".jaws",
                                                              ".malaga",
                                                              ".cardiff",
                                                              ".leucadia",
                                                              ""};
  
  static const int8_t *pFwFiles[MXL_MOCA_MAX_FW_IDX] = {"ccpu.elf",
                                                        "clink.bin",
                                                        "mcast.bin",
                                                        "rlapm.bin",
                                                        "endet.bin",
                                                        "sapm.bin",
                                                        "rssi.bin",
                                                        "platform.bin"};
  
  MXL_ENTER_FUNCTION(TRACE_OSWP, "pDev = %p, pDevName = %s, pFw = %p", pDev, pDevName, pFw);

  if ((pDev == NULL) || (pDevName == NULL) || (pFw == NULL) || (pFwBitMaskRet == NULL))
  {
    return MXL_MOCA_ERR;
  }

  *pFwBitMaskRet = 0;
  
#ifdef SET_CD_FW_SUPPORT
  if (!gGetFwViaIoctl)
#endif
  {
    do
    {
      MxL_MoCA_OsalMemset(pFw, 0, sizeof(*pFw));

      for (i = 0; i < MXL_MOCA_MAX_FW_IDX; i++)
      {
        if (i == MXL_MOCA_CCPU_IDX)
          MxL_MoCA_OsalSnprintf(path, sizeof(path), "%s%s", pFwFiles[i], pChipSuffix[chipTypeIndex]); 
        else
          MxL_MoCA_OsalSnprintf(path, sizeof(path), "%s/%s", pDevName, pFwFiles[i]);

        if (request_firmware((const struct firmware **)&(pFw->fw_entry[i]), path, (struct device *)pDev))
        {
          MxL_MoCA_OsalPrintLog(TRACE_OSWP, L_ERR, "Can't load firmware %s\n", path);
          status = MXL_MOCA_ERR;
          break;
        }
        else
        {
          pFw->ret |= (1 << i);
        }
      }
      *pFwBitMaskRet = pFw->ret;
    }while (0);
  }

  MXL_EXIT_FUNCTION(TRACE_OSWP, "status (bitmask) = %#x", *pFwBitMask);

  return status;
}

/*----------------------------------------------------------------------------------------
 * FUNCTION NAME : MxL_MoCA_OsalGetResetMutex
 *
 *
 * DATE CREATED  : Sep/20/2016
 *
 * DESCRIPTION   : Get reset mutex
 *
 * IN PARAMS     : None
 *
 * OUT PARAMS    : None
 * RETURN VALUE  : Pointer to reset mutex
 *--------------------------------------------------------------------------------------*/

void *MxL_MoCA_OsalGetResetMutex(void)
{
  return &reset_mutex;
}

/*----------------------------------------------------------------------------------------
 * FUNCTION NAME : MxL_MoCA_OsalStrcpy
 *
 *
 * DATE CREATED  : Sep/20/2016
 *
 * DESCRIPTION   : Copy string from source to dstine
 *
 * IN PARAMS     : dst  Pointer destine string
 *                 src  Pointer source string
 *
 * OUT PARAMS    : None
 * RETURN VALUE  : Pointer to destine string
 *--------------------------------------------------------------------------------------*/

int8_t *MxL_MoCA_OsalStrcpy(int8_t *dst, const int8_t *src)
{
  return strcpy(dst, src);
}

/*----------------------------------------------------------------------------------------
 * FUNCTION NAME : MxL_MoCA_OsalSnprintf
 *
 *
 * DATE CREATED  : Sep/20/2016
 *
 * DESCRIPTION   : Get parameters from argument buffer
 *
 * IN PARAMS     : buf  Pointer to created string
 *                 size Length for bytes
 *                 fmt  Type for created string
 *
 * OUT PARAMS    : None
 * RETURN VALUE  : Length of created string
 *--------------------------------------------------------------------------------------*/

int32_t MxL_MoCA_OsalSnprintf(int8_t *buf, uint32_t size, const int8_t *fmt, ...)
{
  va_list args;
  int32_t i;

  va_start(args, fmt);
  i=vsnprintf(buf,size,fmt,args);
  va_end(args);

  return i;
}

/*----------------------------------------------------------------------------------------
 * FUNCTION NAME : MxL_MoCA_OsalHtons
 *
 *
 * DATE CREATED  : Sep/20/2016
 *
 * DESCRIPTION   : Host to network short
 *
 * IN PARAMS     : n  Host number
 *
 * OUT PARAMS    : None
 * RETURN VALUE  : Network short
 *--------------------------------------------------------------------------------------*/

uint16_t MxL_MoCA_OsalHtons(uint16_t n)
{
  return htons(n);
}

/*----------------------------------------------------------------------------------------
 *
 * FUNCTION NAME : MxL_MoCA_OsalCheckImage
 *
 *
 *
 * DATE CREATED  : Sep/20/2016
 *
 * DESCRIPTION   : Check the SoC ELF image
 *
 * IN PARAMS     : elfData : Pointer to the ELF image data
 *                 elfSize : Size of the ELF image
 *
 * OUT PARAMS    : None
 *
 * RETURN VALUE  : 
 * \retval MXL_MOCA_OK  Success
 * \retval MXL_MOCA_ERR Failure 
 *
 *--------------------------------------------------------------------------------------*/
 
MXL_MOCA_STATUS_E MxL_MoCA_OsalCheckImage(uint8_t *elfData, uint32_t elfSize)
{
  MXL_MOCA_STATUS_E ret = MXL_MOCA_ERR;

  // MXL_ENTER_FUNCTION(TRACE_OSWP, "elfData = %p, elfSize = %u", elfData, elfSize);
  MxL_MoCA_OsalPrintLog(TRACE_CD, L_DBG, "MxL_MoCA_OsalCheckImage::elfData = %p, elfSize = %u", elfData, elfSize);

  if (elfData != NULL)
  {
    do
    {
      size_t length;
      Elf32_Ehdr *ehdr = (Elf32_Ehdr*) elfData;

      if (elfSize < sizeof(Elf32_Ehdr))
      {
         MxL_MoCA_OsalPrintLog(TRACE_CD, L_DBG, "elfSize %d  < sizeof(Elf32_Ehdr) %d \n", elfSize, sizeof(Elf32_Ehdr) );
         break;
      }

      if (elfSize < MxL_MoCA_OsalNtohl(ehdr->e_phoff))
      {
        MxL_MoCA_OsalPrintLog(TRACE_CD, L_DBG, "elfSize < HTONL(ehdr->e_phoff)\n");
        break;
      }

      length = MxL_MoCA_OsalNtohs(ehdr->e_phentsize) * MxL_MoCA_OsalNtohs(ehdr->e_phnum);

      elfData += MxL_MoCA_OsalNtohl(ehdr->e_phoff);
      elfSize -= MxL_MoCA_OsalNtohl(ehdr->e_phoff);

      if (elfSize < length)
      {
        MxL_MoCA_OsalPrintLog(TRACE_CD, L_ERR, "partial soc image used\n");
        break;
      }
      ret = MXL_MOCA_OK;
    } while(0);
  }
  else
  {
    MxL_MoCA_OsalPrintLog(TRACE_CD, L_DBG, "Invalid elfdata buff \n");
  }

  MXL_EXIT_FUNCTION(TRACE_OSWP, "status = %d", ret);

  return ret;
}

/*----------------------------------------------------------------------------------------
 * FUNCTION NAME : MxL_MoCA_OsalGetFirstPhdr
 *
 *
 * DATE CREATED  : Sep/20/2016
 *
 * DESCRIPTION   : Get the first program header of the ELF image
 *
 * IN PARAMS     : 
 * \param[in] elfData ELF image data
 *
 * OUT PARAMS    : None
 * RETURN VALUE  : Pointer to the first program header
 *--------------------------------------------------------------------------------------*/ 
 
void *MxL_MoCA_OsalGetFirstPhdr(uint8_t * elfData)
{
  void *phdr = 0;
  Elf32_Ehdr *ehdr = (Elf32_Ehdr*) elfData;

  MXL_ENTER_FUNCTION(TRACE_OSWP, "elfData = %p", elfData);

  if (elfData != NULL)
  {
    elfData += MxL_MoCA_OsalNtohl(ehdr->e_phoff);
    phdr = elfData;
  }
  MXL_EXIT_FUNCTION(TRACE_OSWP, "status = %d", MXL_MOCA_OK);

  return phdr;
}

/*----------------------------------------------------------------------------------------
 * FUNCTION NAME : MxL_MoCA_OsalGetNumOfProgHeader
 *
 *
 * DATE CREATED  : Sep/20/2016
 *
 * DESCRIPTION   : Get the first program header of the ELF image
 *
 * IN PARAMS     : 
 * \param[in] elfData ELF image data
 *
 * OUT PARAMS    : None
 * RETURN VALUE  : Number of program header of the ELF image
 *--------------------------------------------------------------------------------------*/ 
 
uint32_t MxL_MoCA_OsalGetNumOfProgHeader(void *data)
{
  Elf32_Ehdr *pdata = (Elf32_Ehdr *)data;
  uint32_t ret = 0;

  DEBUG_PRINT(TRACE_CD, L_DBG, "pdata->e_phnum %d %d\n", pdata->e_phnum, MxL_MoCA_OsalNtohs(pdata->e_phnum));
  if (data != NULL)
  {
    ret =  MxL_MoCA_OsalNtohs(pdata->e_phnum);
  }
  return ret;
}

/*----------------------------------------------------------------------------------------
 * FUNCTION NAME : MxL_MoCA_OsalGetProgHeaderPhyAddr
 *
 *
 * DATE CREATED  : Sep/20/2016
 *
 * DESCRIPTION   : Get the physical address of the program header
 *
 * IN PARAMS     : 
 * \param[in] phdr Program header pointer
 *
 * OUT PARAMS    : None
 * RETURN VALUE  : Physical address of the program header
 *--------------------------------------------------------------------------------------*/  
 
uint32_t MxL_MoCA_OsalGetProgHeaderPhyAddr(void *phdr)
{
  uint32_t ret = 0;
  Elf32_Phdr *pdata = (Elf32_Phdr *)phdr;

  DEBUG_PRINT(TRACE_CD, L_DBG, "pdata->p_paddr %x %x\n", pdata->p_paddr, MxL_MoCA_OsalNtohl(pdata->p_paddr));

  if (phdr != NULL)
  {
    ret =  (uint32_t)MxL_MoCA_OsalNtohl(pdata->p_paddr);
  }

  return ret;
}

/*----------------------------------------------------------------------------------------
 * FUNCTION NAME : MxL_MoCA_OsalGetProgHeaderOffset
 *
 *
 * DATE CREATED  : Sep/20/2016
 *
 * DESCRIPTION   : Get offset of the program header
 *
 * IN PARAMS     : 
 * \param[in] phdr Program header pointer
 *
 * OUT PARAMS    : None
 * RETURN VALUE  : Offset of the program header
 *--------------------------------------------------------------------------------------*/   
 
uint32_t MxL_MoCA_OsalGetProgHeaderOffset(void *phdr)
{
  uint32_t ret = 0;
  Elf32_Phdr * header = (Elf32_Phdr *)phdr;

  DEBUG_PRINT(TRACE_CD, L_DBG, "header->p_offset %x %x\n", header->p_offset, MxL_MoCA_OsalNtohl(header->p_offset));

  if (phdr != NULL)
  {
    ret =  MxL_MoCA_OsalNtohl(header->p_offset);
  }

  return ret;
}

/*----------------------------------------------------------------------------------------
 * FUNCTION NAME : MxL_MoCA_OsalGetProgHeaderFileSZ
 *
 *
 * DATE CREATED  : Sep/20/2016
 *
 * DESCRIPTION   : Get size of the segment
 *
 * IN PARAMS     : 
 * \param[in] hdr Segment header pointer
 *
 * OUT PARAMS    : None
 * RETURN VALUE  : Size of the segment
 *--------------------------------------------------------------------------------------*/  
 
uint32_t MxL_MoCA_OsalGetProgHeaderFileSZ(void *hdr)
{
  uint32_t ret = 0;
  Elf32_Phdr * header = (Elf32_Phdr *)hdr;

  DEBUG_PRINT(TRACE_CD, L_DBG, "header->p_filesz %x %x\n", header->p_filesz, MxL_MoCA_OsalNtohl(header->p_filesz));

  if (hdr != NULL)
  {
    ret = MxL_MoCA_OsalNtohl(header->p_filesz);
  }

  return ret;
}

/*----------------------------------------------------------------------------------------
 * FUNCTION NAME : MxL_MoCA_OsalGetProgHeaderFileSZ
 *
 *
 * DATE CREATED  : Sep/20/2016
 *
 * DESCRIPTION   : Get next program header
 *
 * IN PARAMS     : 
 * \param[in] phdr  Current program header pointer
 *
 * OUT PARAMS    : None
 * RETURN VALUE  : Next program header
 *--------------------------------------------------------------------------------------*/  
 
void *MxL_MoCA_OsalNextProgHeader(void *phdr)
{
  Elf32_Phdr * header = (Elf32_Phdr *)phdr;

  if (phdr != NULL)
  {
    ++header;

    DEBUG_PRINT(TRACE_CD, L_DBG, "next header %x %x\n", header);
  }

  return (void*)header;
}

/*----------------------------------------------------------------------------------------
 * FUNCTION NAME : MxL_MoCA_OsalGetFirstShdr
 *
 *
 * DATE CREATED  : Dec/14/2017
 *
 * DESCRIPTION   : Get the first section header of the ELF image
 *
 * IN PARAMS     : 
 * \param[in] elfData ELF image data
 *
 * OUT PARAMS    : None
 * RETURN VALUE  : Pointer to the first section header
 *--------------------------------------------------------------------------------------*/ 
 
void *MxL_MoCA_OsalGetFirstShdr(uint8_t * elfData)
{
  void *phdr = 0;
  Elf32_Ehdr *ehdr = (Elf32_Ehdr*) elfData;

  MXL_ENTER_FUNCTION(TRACE_OSWP, "elfData = %p", elfData);

  if (elfData != NULL)
  {
    elfData += MxL_MoCA_OsalNtohl(ehdr->e_shoff);
    phdr = elfData;
  }
  
  MXL_EXIT_FUNCTION(TRACE_OSWP, "status = %d", MXL_MOCA_OK);

  return phdr;
}

/*----------------------------------------------------------------------------------------
 * FUNCTION NAME : MxL_MoCA_OsalGetNumOfSecHeaders
 *
 *
 * DATE CREATED  : Dec/14/2017
 *
 * DESCRIPTION   : Get the number of section headers of the ELF image
 *
 * IN PARAMS     : 
 * \param[in] elfData ELF image data
 *
 * OUT PARAMS    : None
 * RETURN VALUE  : Number of program header of the ELF image
 *--------------------------------------------------------------------------------------*/ 
 
uint32_t MxL_MoCA_OsalGetNumOfSecHeaders(void *pData)
{
  Elf32_Ehdr *pdata = (Elf32_Ehdr *)pData;
  uint32_t ret = 0;

  if (pData != NULL)
  {
    ret =  MxL_MoCA_OsalNtohs(pdata->e_shnum);
  }

  DEBUG_PRINT(TRACE_CD, L_DBG, "pdata->e_phnum %d %d\n", pdata->e_shnum, ret);
  
  return ret;
}

/*----------------------------------------------------------------------------------------
 * FUNCTION NAME : MxL_MoCA_OsalGetSecIndexByName
 *
 *
 * DATE CREATED  : Mar/08/2018
 *
 * DESCRIPTION   : Get the matching section index of the ELF image
 *
 * IN PARAMS     : 
 * \param[in] elfData ELF image data
 * \param[in] section name string
 *
 * OUT PARAMS    : None
 * RETURN VALUE  : index of the matching section
 *--------------------------------------------------------------------------------------*/ 
 
uint32_t MxL_MoCA_OsalGetSecIndexByName(void *pData, char* pName)
{
  Elf32_Ehdr *pdata = (Elf32_Ehdr *)pData;
  uint32_t    ret = 0;
  Elf32_Shdr *shdr = NULL;
  uint32_t    shoff = 0;
  uint32_t    shstrtab_off = 0;
  uint32_t    shnum = 0;
  uint8_t     i;
  uint8_t     *pStrData;
  
  if (pData != NULL && pName != NULL)
  {
    shnum = MxL_MoCA_OsalGetNumOfSecHeaders(pData); // number of entries in the section header table
    shoff = MxL_MoCA_OsalNtohl(pdata->e_shoff);     // section header byte offset
    shdr = (Elf32_Shdr*)(pData + shoff);	          // section header table base address
    shstrtab_off = MxL_MoCA_OsalNtohl(shdr[shnum-1].sh_offset);	// offset of the last section (string table)
    
    for (i = 0; i < shnum; i++) 
    {
      // Get the section name for each section and compare with requested name
      pStrData = (uint8_t*)(pData + shstrtab_off + MxL_MoCA_OsalNtohl(shdr[i].sh_name));
      DEBUG_PRINT (TRACE_CD, L_DBG, "%s\n",  pStrData);

      // If a match is found, return the section index. The caller can use the section index to extract the data.
      if (strcmp(pStrData, pName) == 0)
      {
      	DEBUG_PRINT (TRACE_CD, L_DBG, "Requested section index is %d\n", i);    
        ret = i;
        break;
      }
    } 
  }

  DEBUG_PRINT(TRACE_CD, L_DBG, "Requested section index = %d\n", ret);
  
  return ret;
}

/*----------------------------------------------------------------------------------------
 * FUNCTION NAME : MxL_MoCA_OsalGetSecFileOffset
 *
 *
 * DATE CREATED  : Dec/14/2017
 *
 * DESCRIPTION   : Get the byte offset from the beginning of the file to the first byte 
 *                 in the specified section. shdr[sectionId].sh_offset
 *
 * IN PARAMS     : 
 * \param[in] data : ELF image data from Section header table file offset (e_shoff)
 *
 * OUT PARAMS    : None
 * RETURN VALUE  : Section file offset of the specified section.
 *--------------------------------------------------------------------------------------*/ 
 
uint32_t MxL_MoCA_OsalGetSecFileOffset(void *pData, uint32_t sectionId)
{
  Elf32_Shdr *pShdr = (Elf32_Shdr *)pData;
  uint32_t offset = 0;

  if (pData != NULL)
  {
    offset = MxL_MoCA_OsalNtohl(pShdr[sectionId].sh_offset);
  }
  
  DEBUG_PRINT(TRACE_CD, L_DBG, "pShdr[sectionId].sh_offset = %X\n", offset);

  return offset;
}

/*----------------------------------------------------------------------------------------
 * FUNCTION NAME : MxL_MoCA_OsalHtonl
 *
 *
 * DATE CREATED  : Sep/20/2016
 *
 * DESCRIPTION   : Host to network long
 *
 * IN PARAMS     : n  Host number
 *
 * OUT PARAMS    : None
 * RETURN VALUE  : Network long
 *--------------------------------------------------------------------------------------*/

uint32_t MxL_MoCA_OsalHtonl(uint32_t n)
{
  return htonl(n);
}

/*----------------------------------------------------------------------------------------
 * FUNCTION NAME : MxL_MoCA_OsalNtohl
 *
 *
 * DATE CREATED  : Sep/20/2016
 *
 * DESCRIPTION   : Network to host long
 *
 * IN PARAMS     : n  Network long
 *
 * OUT PARAMS    : None
 * RETURN VALUE  : Host long
 *--------------------------------------------------------------------------------------*/

uint32_t MxL_MoCA_OsalNtohl(uint32_t n)
{
  return ntohl(n);
}

/*----------------------------------------------------------------------------------------
 * FUNCTION NAME : MxL_MoCA_OsalNtohs
 *
 *
 * DATE CREATED  : Sep/20/2016
 *
 * DESCRIPTION   : Network to host short
 *
 * IN PARAMS     : n  Network short
 *
 * OUT PARAMS    : None
 * RETURN VALUE  : Host short
 *--------------------------------------------------------------------------------------*/

uint16_t MxL_MoCA_OsalNtohs(uint16_t n)
{
  return ntohs(n);
}

/*----------------------------------------------------------------------------------------
 * FUNCTION NAME : MxL_MoCA_OsalStrstr
 *
 *
 * DATE CREATED  : Sep/20/2016
 *
 * DESCRIPTION   : Find s2 in s1
 *
 * IN PARAMS     : s1 The string expression to search
 *                 s2 The string expression to find
 *
 * OUT PARAMS    : None
 * RETURN VALUE  : 0 for failure, others for pointer in s1
 *--------------------------------------------------------------------------------------*/

int8_t *MxL_MoCA_OsalStrstr(int8_t *s1, int8_t *s2)
{
  return strstr(s1, s2);
}

#if defined(INCLUDE_PCI)

/*----------------------------------------------------------------------------------------
 * FUNCTION NAME : MxL_MoCA_OsalSetMacAddress
 *
 *
 * DATE CREATED  : Sep/20/2016
 *
 * DESCRIPTION   : Sets the MAC address in the OS
 *
 * IN PARAMS     : devIndex - index of device
 *                 mac     - mac address
 *
 * OUT PARAMS    : None
 *--------------------------------------------------------------------------------------*/

void MxL_MoCA_OsalSetMacAddress(uint32_t devIndex, uint32_t *mac)
{
  MxL_MoCA_PCIeSetMgmtMAC(devIndex, mac);
}

/*----------------------------------------------------------------------------------------
 * FUNCTION NAME : MxL_MoCA_OsalPcieAvailable
 *
 *
 *
 * DATE CREATED  : Sep/20/2016
 *
 * DESCRIPTION   : Check if the device is available/ready to handle traffic
 *
 * IN PARAMS     : devIndex : device index
 *
 * OUT PARAMS    : None
 *
 * RETURN VALUE  : MXL_MOCA_STATUS_E code.
 *
 *--------------------------------------------------------------------------------------*/

MXL_MOCA_STATUS_E MxL_MoCA_OsalPcieAvailable(int8_t devIndex)
{
  return MxL_MoCA_PCIeAvailable(devIndex);
}

/*----------------------------------------------------------------------------------------
 * FUNCTION NAME : MxL_MoCA_OsalPcieBusRead
 *
 *
 * DATE CREATED  : Sep/20/2016
 *
 * DESCRIPTION   : Read data via PCIe bus driver
 *
 * IN PARAMS     : devIndex Device index
 *                 addr     Data address
 *                 pData    Pointer to data buffer
 *
 * OUT PARAMS    : None
 * RETURN VALUE  : MXL_MOCA_OK for success, others for failure
 *--------------------------------------------------------------------------------------*/

MXL_MOCA_STATUS_E MxL_MoCA_OsalPcieBusRead(uint32_t devIndex, 
                                           uintptr_t addr, uint32_t *pData)
{
  MXL_MOCA_STATUS_E ret = MXL_MOCA_ERR;

  ret = MxL_MoCA_PCIeRead(devIndex, addr, pData);

  return ret;
}

/*----------------------------------------------------------------------------------------
 * FUNCTION NAME : MxL_MoCA_OsalPcieBusWrite
 *
 *
 * DATE CREATED  : Sep/20/2016
 *
 * DESCRIPTION   : Write data to SoC memory via PCIe bus driver
 *
 * IN PARAMS     : devIndex Device index
 *                 addr    Data address
 *                 data    Pointer to data buffer
 *
 * OUT PARAMS    : None
 * RETURN VALUE  : MXL_MOCA_OK for success, others for failure
 *--------------------------------------------------------------------------------------*/

MXL_MOCA_STATUS_E MxL_MoCA_OsalPcieBusWrite(uint32_t devIndex, 
                                            uintptr_t addr, 
                                            uint32_t data)
{
  MXL_MOCA_STATUS_E ret = MXL_MOCA_ERR;

  ret = MxL_MoCA_PCIeWrite(devIndex, addr, data);

  return ret;
}

/*----------------------------------------------------------------------------------------
 * FUNCTION NAME : MxL_MoCA_OsalDataPlaneIfRestart
 *
 *
 * DATE CREATED  : Sep/20/2016
 *
 * DESCRIPTION   : Restarts the data path
 *
 * IN PARAMS     : devIndex Device index
 *
 * OUT PARAMS    : None
 * RETURN VALUE  : MXL_MOCA_OK for success, others for failure
 *--------------------------------------------------------------------------------------*/

MXL_MOCA_STATUS_E MxL_MoCA_OsalDataPlaneIfRestart(uint32_t devIndex)
{
  MXL_MOCA_STATUS_E ret = MXL_MOCA_ERR;

  ret = MxL_MoCA_PCIeRestart(devIndex); 

  return ret;
}

/*----------------------------------------------------------------------------------------
 * FUNCTION NAME : MxL_MoCA_OsalDataPlaneResetSoC
 *
 *
 * DATE CREATED  : Sep/20/2016
 *
 * DESCRIPTION   : Resets SoC
 *
 * IN PARAMS     : devIndex Device index
 *
 * OUT PARAMS    : None
 * RETURN VALUE  : MXL_MOCA_OK for success, others for failure
 *--------------------------------------------------------------------------------------*/

MXL_MOCA_STATUS_E MxL_MoCA_OsalDataPlaneResetSoC(uint32_t devIndex)
{
  MXL_MOCA_STATUS_E ret = MXL_MOCA_ERR;

  ret = MxL_MoCA_PCIeGlobalResetSoC(devIndex);

  return ret;
}

/*----------------------------------------------------------------------------------------
 *
 * FUNCTION NAME : MxL_MoCA_OsalDetectPCIeDevice
 *
 *
 *
 * DATE CREATED  : Sep/20/2016
 *
 * DESCRIPTION   : Begin searching for a PCI device by vendor/device id. Iterates through
 *                 the list of known PCI devices. If a PCI device is found with a matching
 *                 vendor and device, the reference count to the device is incremented and
 *                 a pointer to its device structure is returned. Otherwise, NULL is
 *                 returned. Before returning true, we will check if the driver was also 
 *                 loaded when the input flag is true.
 *
 * IN PARAMS     : vendor - PCI vendor id to match, or PCI_ANY_ID to match all vendor ids
 *                 device - PCI device id to match, or PCI_ANY_ID to match all device ids
 *                 checkDriverLoad - checks if driver was loaded.
 *
 * OUT PARAMS    : None
 *
 * RETURN VALUE  : MXL_MOCA_OK - found  , MXL_MOCA_ERR - not found
 *
 *--------------------------------------------------------------------------------------*/

MXL_MOCA_STATUS_E MxL_MoCA_OsalDetectPCIeDevice(int32_t vendor, int32_t device, int8_t checkDriverLoad)
{
  struct pci_dev *p_pci_device;
  MXL_MOCA_STATUS_E status = MXL_MOCA_ERR;
  
  p_pci_device = pci_get_device(vendor, device, 0x0);
  
  if (p_pci_device)
  {
    if (checkDriverLoad)
    {
      if (strcmp (p_pci_device->driver->name, MXL_MOCA_PCIE_DRV_NAME) == 0)
      {
        status = MXL_MOCA_OK; 
      }
    }
    else
    {
      status = MXL_MOCA_OK; 
    }
  }
  return status;
}

#endif


/*----------------------------------------------------------------------------------------
 * FUNCTION NAME : MxL_MoCA_OsalCreateLock
 *
 *
 * DATE CREATED  : Oct/06/2016
 *
 * DESCRIPTION   : Create and initilize OSAL lock
 *
 * IN PARAMS     : pLock --> Pointer to user created OSAL lock
 *
 * OUT PARAMS    : pLock --> If return value is success then pLock will be a 
 *                           valid pointer to OSAL lock
 *
 * RETURN VALUE  : MXL_MOCA_OK for success, others for failure
 *--------------------------------------------------------------------------------------*/

MXL_MOCA_STATUS_E MxL_MoCA_OsalCreateLock(MxL_OSAL_LOCK_T *pLock)
{
  MXL_MOCA_STATUS_E status = MXL_MOCA_ERR;

  if (pLock)
  {
    // Initilize semaphore  (binary semaphore)
    sema_init(&pLock->mxlLock, 1);

    status = MXL_MOCA_OK;
    
  }
  else
  {
    status = MXL_MOCA_INVALID_ARGUMENT_ERR;
  }

  return status;
}

/*----------------------------------------------------------------------------------------
 * FUNCTION NAME : MxL_MoCA_OsalDestroyLock
 *
 *
 * DATE CREATED  : Oct/06/2016
 *
 * DESCRIPTION   : Destroy/De-Init OSAL lock
 *
 * IN PARAMS     : pLock --> Pointer to user created OSAL lock
 *
 * OUT PARAMS    : None
 * RETURN VALUE  : None
 *--------------------------------------------------------------------------------------*/

void MxL_MoCA_OsalDestroyLock(MxL_OSAL_LOCK_T *pLock)
{
  // There is no explicit Destroy Semaphore in linux OS.
  // If there is support to de-init or destroy semaphore then
  // please update this function with OS supported API

}

/*----------------------------------------------------------------------------------------
 * FUNCTION NAME : MxL_MoCA_OsalLock
 *
 *
 * DATE CREATED  : Oct/06/2016
 *
 * DESCRIPTION   : Acquire lock. If the Lock is not available, a caller thread will wait 
 *                 forever till the Lock is acquired and it enters sleep state.
 *
 * IN PARAMS     : pLock --> Pointer to user created OSAL lock 
 *
 * OUT PARAMS    : None
 * RETURN VALUE  : None
 *--------------------------------------------------------------------------------------*/

void MxL_MoCA_OsalLock(MxL_OSAL_LOCK_T *pLock)
{
  // down_interruptible would be a better solution since
  // User-mode app can be interrupted by User while waiting on Lock

  if (pLock)
  {
    // Acquire semaphore 
    down(&pLock->mxlLock);
  }
  else
  {
    // Error -- Invalid pointer Add some debug message instead of driver crash
  }

}

/*----------------------------------------------------------------------------------------
 * FUNCTION NAME : MxL_MoCA_OsalUnlock
 *
 *
 * DATE CREATED  : Oct/06/2016
 *
 * DESCRIPTION   : Release previously acquired lock
 *
 * IN PARAMS     : pLock --> Pointer to user created OSAL lock 
 *
 * OUT PARAMS    : None
 * RETURN VALUE  : None
 *--------------------------------------------------------------------------------------*/

void MxL_MoCA_OsalUnlock(MxL_OSAL_LOCK_T *pLock)
{

  if (pLock)
  {
    // Release semaphore
    up(&pLock->mxlLock);
  }
  else
  {
    // Error -- Invalid pointer Add some debug message instead of driver crash
  }

}

