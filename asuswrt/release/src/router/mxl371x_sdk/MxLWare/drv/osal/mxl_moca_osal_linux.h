/*****************************************************************************************
 *
 * FILE NAME          : mxl_moca_osal_linux.h
 *
 *
 *
 * DATE CREATED       : Sep/30/2016
 *
 * LAST MODIFIED      :
 *
 * DESCRIPTION        : Header file for OSAL linux
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

#ifndef __MXL_MOCA_OSAL_LINUX_H__
#define __MXL_MOCA_OSAL_LINUX_H__

/*******************************************************************************
 *      Includes
 ******************************************************************************/
#include <linux/version.h>

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 33)
#include <generated/autoconf.h>
#elif (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 18)) && (LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 33))
#include <linux/autoconf.h>
#else
#include <linux/config.h>
#endif

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/types.h>
#ifdef INCLUDE_PCI
#include <linux/pci.h>
#include <linux/interrupt.h>
#endif
#include <linux/init.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/ethtool.h>
#include <linux/spinlock.h>
#include <linux/timer.h>
#include <linux/delay.h>
#include <linux/time.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/unaligned.h>
#include <asm/uaccess.h>
#include <linux/proc_fs.h>
#include <linux/ctype.h>
#include <linux/list.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/mii.h>

/*****************************************************************************************
 *   Macros
 ****************************************************************************************/

#define USEC_PER_MS 1000

#ifndef init_MUTEX
#define init_MUTEX(sem)     sema_init(sem, 1)
#endif

#ifndef init_MUTEX_LOCKED
#define init_MUTEX_LOCKED(sem)  sema_init(sem, 0)
#endif

#ifndef DECLARE_MUTEX
#define DECLARE_MUTEX(name) \
  struct semaphore name = __SEMAPHORE_INITIALIZER(name, 1)
#endif

#define LOCK_MAGIC      0x4c4f434b       ///< spells "LOCK"
#define KMALLOC_MAX     65536

// Host Print ForMaT string
#define HPFMT "%s: %s"

#if LINUX_VERSION_CODE <  KERNEL_VERSION(2,6,23)
#define UMH_WAIT_PROC  1
#endif

/*****************************************************************************************
 *      Type defines
 ****************************************************************************************/

typedef struct timer_list  timer_list_t;
typedef struct semaphore  semaphore_t;

/*!
 * \brief Log strcture
 */
typedef struct
{
  int32_t log_mod;        ///< used to verify the structure is initialized
  int32_t log_lev;
} osal_log_t ;


/*!
 * \brief Spinlock structure with added bells and whistles
 *
 * For locking purposes.
 */
typedef struct
{
  int32_t       lock_magic;        ///< used to verify the structure is initialized
  unsigned long lock_irq_flags;    ///< irq flags for restoring later
  spinlock_t    lock_spinlock;
} osal_lock_t;

/*!
 * \brief Timer operations
 *
 * For linux this contains a struct timer_list.
 */
typedef struct
{
  struct timer_list ostimer ;
} osal_timer_t ;

/*!
 * \brief Semaphore structure
 *
 * For linux this contains a struct semaphore
 */
typedef struct
{
  struct semaphore ossema ;
} osal_sema_t ;


typedef struct
{

  // Please do not change the variable name "mxlLock" as this 
  // name is used in MxLware. 
  // Just update "struct semaphore" to OS specific locking
  // data structure.
  
  struct semaphore mxlLock;
  
} MxL_OSAL_LOCK_T ;

/*!
 * \brief Wait queue timer
 *
 * This structure contains everything necessary to
 * use timers and wait queues from the !GPL side.
 * You allocate one of these wqt_t things and then
 * pass its pointer back on the various calls.
 */

typedef struct
{
  int32_t           wqt_allocated ; ///< 0 = no, 1 = yes
  struct timer_list wqt_timer;
  wait_queue_head_t wqt_wq;
} osal_wqt_t ;


/*!
 * \brief Thread operations
 *
 * For linux this contains kernel thread related information.
 */
typedef struct
{
  char    name[20];          /*!< thread name */
  int32_t threadID;
  void    (*func)(void *);   /*!< thread internal function */
  void    *arg;              /*!< argument to pass to kernel thread */
} osal_kthread_t;

/**
 * \brief System time value
 *
 */
typedef struct
{
  struct timeval timeVal ;
} apposal_timeval_t;

typedef struct ethtool_cmd apposal_ethtool_cmd_t;

/*****************************************************************************************
 *     Global Variable Declarations
 ****************************************************************************************/

// Defined using DEVICE_ATTR(lof) and DEVICE_ATTR(link_status)
extern struct device_attribute dev_attr_lof;
extern struct device_attribute dev_attr_link_status;
extern struct device_attribute dev_attr_adm_status;
extern struct device_attribute dev_attr_bridge_ctrl_req_status;
extern struct device_attribute dev_attr_net_proxy_req_status;
extern struct device_attribute dev_attr_beacon_power_dist_status;
extern struct device_attribute dev_attr_privacy_update_status;

extern unsigned long volatile jiffies;
extern int32_t major;

/*****************************************************************************************
 *     Function Prototypes
 ****************************************************************************************/

#endif /* __MXL_MOCA_OSAL_LINUX_H__ */

