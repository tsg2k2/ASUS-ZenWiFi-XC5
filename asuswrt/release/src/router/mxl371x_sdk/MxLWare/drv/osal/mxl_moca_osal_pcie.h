/*****************************************************************************************
 *
 * FILE NAME          : mxl_moca_osal_pcie.h
 * 
 *
 *
 * DATE CREATED       : 03/25/2016
 *
 * LAST MODIFIED      : %Name% @ %%/%%/%% 
 *
 * DESCRIPTION        : Linux PCIe/Ethernet Driver. This file contains constants and API  
 *                      definitions.  
 *                      
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

#ifndef __MXL_MOCA_OSAL_PCIE_H__
#define __MXL_MOCA_OSAL_PCIE_H__

/*******************************************************************************
 *      Includes
 ******************************************************************************/
#include <linux/version.h>

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 18)
//#include <linux/autoconf.h>
#else
#include <linux/config.h>
#endif

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/pci.h>
#include <linux/pci_regs.h>
#include <linux/pci-aspm.h>
#include <linux/interrupt.h>
#if LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 31)
#include <linux/pm.h>
#include <linux/pm_wakeup.h>
#include <linux/pm_runtime.h>
#endif
#include <linux/init.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/ethtool.h>
#include <linux/spinlock.h>
#include <linux/timer.h>
#include <linux/delay.h>
#include <linux/aer.h>
#include <linux/pcieport_if.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/unaligned.h>
#include <asm/uaccess.h>
#include <linux/proc_fs.h>
#include <linux/ctype.h>
#include <linux/list.h>
#include <linux/wait.h>
#include <linux/sched.h>


// Macros
/*****************************************************************************************
 *      ECA specific 
 *****************************************************************************************/
#ifndef MXLWARE_CONFIG_BOARD_ECA_9M_L3_NXP
#define MXLWARE_CONFIG_BOARD_ECA_9M_L3_NXP 0
#endif

/*****************************************************************************************
 *      Debug
 *****************************************************************************************/
#define MXL_MOCA_PCIE_DEBUG_DESCR     0
#define MXL_MOCA_PCIE_DEBUG_DROP_TX   0 // Enable this to test TX packet drop feature
#define MXL_MOCA_PCIE_DEBUG_DROP_RX   0 // Enable this to test RX packet drop feature


/*****************************************************************************************
 *      Driver
 *****************************************************************************************/
#define MXL_MOCA_PCIE_DRV_NAME                "mxl_moca_pcie"
#define MXL_MOCA_PCIE_DEV_NAME                "en%d"
#define MXL_MOCA_PCIE_PROC_DIR_NAME           "en%d"
#define MXL_MOCA_PCIE_PROC_ENTRY_NAME         "pcie"
#define MXL_MOCA_MODULE_PARAM_PERM            (0644)  // ( S_IRUSR | S_IWUSR) | S_IRGRP | S_IROTH )

#define MXL_MOCA_PCIE_PROC_NAME_LEN_MAX        30

#define MXL_MOCA_PCIE_NAPI                     1  

#define MXL_MOCA_PCIE_PWR_MGMT                 0

#define MXL_MOCA_ENABLE_DBG_MSG                0  /* Keep this 0 for release builds */

#define MXL_MOCA_PCIE_MAX_NUM_OF_PCIE_DEVICES  1

#define MXL_MOCA_PCIE_MAX_NUM_OF_PCIE_DEV_ID   2

#define MXL_MOCA_PCIE_DMA_MASK_BITS            32

#define MXL_MOCA_PCIE_EN_PROC_DBG              0 /* Keep this 0 for release builds */

/* Enabled feature in CCPU to issue periodic MSIs when there is active traffic. Interval is 160msec.
 * This can be used as a DEBUG feature as well.
 */
#define MXL_MOCA_PCIE_EN_CCPU_TIMER_INTR       1 

#define MXL_MOCA_PROCFS_MAX_SIZE               4096

#define MXL_MOCA_PCIE_VENDOR_ID                0x17e6
#define MXL_MOCA_PCIE_DEV_ID_CARDIFF           0x3700
#define MXL_MOCA_PCIE_DEV_ID_LEUCADIA          0x3710

#define MXL_MOCA_PCIE_CTL_CD_CMD               0x0100000
#define MXL_MOCA_PCIE_CTL_DP_CMD               0x0020000

#define PCIE_INTR_SEL_AUTO                     0 // Detect MSI, if not found, use INTA
#define PCIE_INTR_SEL_MSI                      1 // use MSI
#define PCIE_INTR_SEL_INTA                     2 // use INTA

#define PCIE_INTR_TYPE_MSI                     1 // use MSI
#define PCIE_INTR_TYPE_INTA                    2 // use INTA

// Outbound ATU (RC <-- EP)
#define MXL_MOCA_PCIE_NUM_OUTBOUND_ATU_REGIONS 2  // Maximum num regions are 4
/* If MXL_MOCA_PCIE_NUM_OUTBOUND_ATU_REGIONS is changed, 
 * update MxL_MoCA_PCIeConfigOutboundATU and MxL_MoCA_PCIeCheckOutOfBoundMapping 
 */
/* Example of iATU:
 * Update the numbers in the table below if any updates to related #define constants
 * The region addresses, when programmed, must be in ascending order from region 0 - thru region 3
 * The number of valid regions are defined by MXL_MOCA_PCIE_NUM_OUTBOUND_ATU_REGIONS
 |----------------------------------|
 |            Region 0              |
 |----------------------------------|
 | Target addr   | Region base addr | // Target addr range: 0x0 - 0x3FFF_FFFF
 | 0x0000_0000   | 0x4000_0000      | // MXL_MOCA_PCIE_OUTB_ATU_REGION0_BASE
 |----------------------------------|
 |            Region 1              |
 |----------------------------------|
 | Target addr   | Region base addr | // Target addr range: 0x4000_0000 - 0xBFFF_FFFF
 | 0x4000_0000   | 0x8000_0000      | // MXL_MOCA_PCIE_OUTB_ATU_REGION1_BASE
 |----------------------------------|
 |            Region 2              |
 |----------------------------------|
 | Target addr   | Region base addr | // Unused target or region base addresses will be 0 (default)
 | 0xXXXX_XXXX   | 0xXXXX_XXXX      |
 |----------------------------------|
 |            Region 3              |
 |----------------------------------|
 | Target addr   | Region base addr | // Unused target or region base addresses will be 0 (default)
 | 0xXXXX_XXXX   | 0xXXXX_XXXX      |
 |---------------|------------------|
*/
#define MXL_MOCA_PCIE_REGION0_MASK                            0x7FFFFFFF // 1GB  : 0x40000000 - 1
#define MXL_MOCA_PCIE_REGION1_MASK                            0xFFFFFFFF // 2GB  : 0x80000000 - 1

/* Defaults */
#if defined(__LP64__) || defined(_LP64)
#if MXLWARE_CONFIG_BOARD_ECA_9M_L3_NXP
#define MXL_MOCA_PCIE_OUTB_ATU_TARGET0_ADDR_MASK              0x40000000 // 0x4000_0000 - 0x7FFF_FFFF (1GB)
#define MXL_MOCA_PCIE_OUTB_ATU_TARGET0_ADDR_LIMIT             0x7FFFFFFF
#define MXL_MOCA_PCIE_OUTB_ATU_TARGET1_ADDR_MASK              0x80000000 // 0x8000_0000 - 0xFFFF_FFFF (2GB)
#define MXL_MOCA_PCIE_OUTB_ATU_TARGET1_ADDR_LIMIT             0xFFFFFFFF
#else // PC specific
#define MXL_MOCA_PCIE_OUTB_ATU_TARGET0_ADDR_MASK              0x30000000 // 0x3000_0000 - 0x6FFF_FFFF (1GB)
#define MXL_MOCA_PCIE_OUTB_ATU_TARGET0_ADDR_LIMIT             0x6FFFFFFF
#define MXL_MOCA_PCIE_OUTB_ATU_TARGET1_ADDR_MASK              0x70000000 // 0x7000_0000 - 0xEFFF_FFFF (2GB)
#define MXL_MOCA_PCIE_OUTB_ATU_TARGET1_ADDR_LIMIT             0xEFFFFFFF
#endif
#else // 32-bits
#define MXL_MOCA_PCIE_OUTB_ATU_TARGET0_ADDR_MASK              0x0        // 0x0000_0000 - 0x3FFF_FFFF (1GB)
#define MXL_MOCA_PCIE_OUTB_ATU_TARGET0_ADDR_LIMIT             0x3FFFFFFF
#define MXL_MOCA_PCIE_OUTB_ATU_TARGET1_ADDR_MASK              0x40000000 // 0x4000_0000 - 0xBFFF_FFFF (2GB)
#define MXL_MOCA_PCIE_OUTB_ATU_TARGET1_ADDR_LIMIT             0xBFFFFFFF
#endif

#define MXL_MOCA_PCIE_OUTB_ATU_REGION0_BASE                   0x40000000
#define MXL_MOCA_PCIE_OUTB_ATU_REGION1_BASE                   0x80000000


// Inbound ATU (RC --> EP)
/*
 |---------------------------|
 |            Region 0       |
 |---------------------------|
 | BAR0 match  | Target addr |
 |             | 0x0800_0000 | // MXL_MOCA_PCIE_HIF_TC_DMEM_BASE (Range: 0x800_0000-0x83F_FFFF)
 |---------------------------|
 |            Region 1       |
 |---------------------------|
 | BAR1 match  | Target addr |
 |             | 0x0C00_0000 | // MXL_MOCA_PCIE_CCPU_CORE_BASE   (Range: 0xC00_0000-0xCFF_FFFF)
 |---------------------------|
*/
#define MXL_MOCA_PCIE_BAR0_MASK                               0x003FFFFF // 4MB  : 0x400000  - 1
#define MXL_MOCA_PCIE_BAR1_MASK                               0x00FFFFFF // 16MB : 0x1000000 - 1
// --

// The product family of the Cardiff Chip: Cardiff: 3700, Leucadia: 3710
#define MXL_MOCA_PROD_FAM_ID_ADDR               0x08200000
#define MXL_MOCA_SYS_RES_EXT_CONFIG_STAT_ADDR   0x0820000c
#define MXL_MOCA_BOND_OPT_SKU_MASK              0x003f0000
#define MXL_MOCA_BOND_OPT_OFFSET                16   

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 8, 0)
 #define __devinitdata
 #define __devinit
 #define __devexit

#if defined(MODULE) || defined(CONFIG_HOTPLUG)
  #define __devexit_p(x) x
#else
  #define __devexit_p(x) NULL
#endif

#endif // LINUX_VERSION_CODE >= KERNEL_VERSION(3, 8, 0)

#ifndef IRQF_SHARED
 #define IRQF_SHARED SA_SHIRQ
#endif

#ifndef DMA_BIT_MASK
 #define DMA_BIT_MASK(n)  (((n) == 64) ? ~0ULL : ((1ULL<<(n))-1))
#endif

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,22)
 #undef ETHTOOL_GPERMADDR
 #undef SET_MODULE_OWNER
 #define SET_MODULE_OWNER(dev) do { } while (0)
#endif

#define MXL_MOCA_PCIE_TX_TIMEOUT (4*HZ)     ///< for watchdog
#define MXL_MOCA_PCIE_MAX_ETH_MTU_SIZE        1536
#define MXL_MOCA_PCIE_MIN_ETH_MTU_SIZE_NO_FCS 60

#define MXL_MOCA_PCIE_MAX_TX_PACKETS_PER_INT      32   // do not modify this. 
#define MXL_MOCA_PCIE_MAX_RX_PACKETS_PER_INT      32   // do not modify this. 

#define MXL_MOCA_PCIE_MAX_NAPI_WEIGHT  MXL_MOCA_PCIE_MAX_RX_PACKETS_PER_INT // Change to a value <= MXL_MOCA_PCIE_MAX_RX_PACKETS_PER_INT


// At least +2 due to our Ethernet header misalignment.
#define MXL_MOCA_PCIE_MAX_ETH_RX_BUF_SIZE ((MXL_MOCA_PCIE_MAX_ETH_MTU_SIZE) + 2) // Our NET_IP_ALIGN is 0 for cardiff/leuc

/*----------------------------------------------------------------------------------------
 *      Debug macros (MXL_MOCA_PCIE_L_DBG1 or MXL_MOCA_PCIE_L_DBG2 level must be set)
 *--------------------------------------------------------------------------------------*/ 

#if MXL_MOCA_ENABLE_DBG_MSG

#define MXL_MOCA_DBG1P0(msg)                      MxL_MoCA_PCIeDbgTrace(MXL_MOCA_PCIE_L_DBG1, msg)
#define MXL_MOCA_DBG1P1(msg, p0)                  MxL_MoCA_PCIeDbgTrace(MXL_MOCA_PCIE_L_DBG1, msg, p0)
#define MXL_MOCA_DBG1P2(msg, p0, p1)              MxL_MoCA_PCIeDbgTrace(MXL_MOCA_PCIE_L_DBG1, msg, p0, p1)
#define MXL_MOCA_DBG1P3(msg, p0, p1, p2)          MxL_MoCA_PCIeDbgTrace(MXL_MOCA_PCIE_L_DBG1, msg, p0, p1, p2)
#define MXL_MOCA_DBG1P4(msg, p0, p1, p2, p3)      MxL_MoCA_PCIeDbgTrace(MXL_MOCA_PCIE_L_DBG1, msg, p0, p1, p2, p3)
#define MXL_MOCA_DBG1P5(msg, p0, p1, p2, p3, p4)  MxL_MoCA_PCIeDbgTrace(MXL_MOCA_PCIE_L_DBG1, msg, p0, p1, p2, p3, p4)

#define MXL_MOCA_DBG2P0(msg)                      MxL_MoCA_PCIeDbgTrace(MXL_MOCA_PCIE_L_DBG2, msg)
#define MXL_MOCA_DBG2P1(msg, p0)                  MxL_MoCA_PCIeDbgTrace(MXL_MOCA_PCIE_L_DBG2, msg, p0)
#define MXL_MOCA_DBG2P2(msg, p0, p1)              MxL_MoCA_PCIeDbgTrace(MXL_MOCA_PCIE_L_DBG2, msg, p0, p1)
#define MXL_MOCA_DBG2P3(msg, p0, p1, p2)          MxL_MoCA_PCIeDbgTrace(MXL_MOCA_PCIE_L_DBG2, msg, p0, p1, p2)
#define MXL_MOCA_DBG2P4(msg, p0, p1, p2, p3)      MxL_MoCA_PCIeDbgTrace(MXL_MOCA_PCIE_L_DBG2, msg, p0, p1, p2, p3)
#define MXL_MOCA_DBG2P5(msg, p0, p1, p2, p3, p4)  MxL_MoCA_PCIeDbgTrace(MXL_MOCA_PCIE_L_DBG2, msg, p0, p1, p2, p3, p4)

#else /* Expand to nothing */

#define MXL_MOCA_DBG1P0(msg)
#define MXL_MOCA_DBG1P1(msg, p0)
#define MXL_MOCA_DBG1P2(msg, p0, p1)
#define MXL_MOCA_DBG1P3(msg, p0, p1, p2)
#define MXL_MOCA_DBG1P4(msg, p0, p1, p2, p3)
#define MXL_MOCA_DBG1P5(msg, p0, p1, p2, p3, p4)

#define MXL_MOCA_DBG2P0(msg)
#define MXL_MOCA_DBG2P1(msg, p0)
#define MXL_MOCA_DBG2P2(msg, p0, p1)
#define MXL_MOCA_DBG2P3(msg, p0, p1, p2)
#define MXL_MOCA_DBG2P4(msg, p0, p1, p2, p3)
#define MXL_MOCA_DBG2P5(msg, p0, p1, p2, p3, p4)

#endif


/*******************************************************************************
 *      Constants (Host-to-SoC DMEM interface) : Common to Leuc and Cardiff
 ******************************************************************************/
// DMEM addr offsets
// Datapath descriptors storage 
#define P_PCIE_HOST_RX_DESC_QUEUE           0x00000301
#define P_PCIE_HOST_RX_DESC_Q_LEN           0x00000302
#define P_PCIE_HOST_TX_DESC_QUEUE           0x00000305
#define P_PCIE_HOST_TX_DESC_Q_LEN           0x00000306

#define PCIE_TC_MGMT_IF_MAC_ADDR            0x00000309

// iATU: Outbound settings - region base
#define PCIE_NUM_OUTBOUND_REGIONS           0x0000030b
#define PCIE_OUTBOUND_ATU_REG0_BASE         0x0000030c
#define PCIE_OUTBOUND_ATU_REG1_BASE         0x0000030d
#define PCIE_OUTBOUND_ATU_REG2_BASE         0x0000030e
#define PCIE_OUTBOUND_ATU_REG3_BASE         0x0000030f

// PCIe capabilities settings
#define PCIE_DEVCTL_MAX_PAYLOAD_SIZE        0x00000310 // MTU size. Set LargeWrAXIBurst to ((this_value/4) - 1)
#define PCIE_LNKCTL_RCB                     0x00000311 // Read Completion Boundary. Set LargeRdAXIBurst to ((this_value/4) - 1)

// Handshake 
#define PCIE_HOST_STATUS                    0x00000312 // W_HIF_PCIE_HOST_STATUS in TC

// Periodic CCPU interrupts (Enabled with MXL_MOCA_PCIE_EN_CCPU_TIMER_INTR) option DMEM addr offsets
#define PCIE_EN_CCPU_TIMER_INTR             0x00000313 // W_HIF_PCIE_EN_CCPU_TIMER_INTR in TC : Enables optional periodic timer interrupts (MSI) from CCPU
#define PCIE_CCPU_INTR_COUNT                0x00000314 // W_HIF_PCIE_INTR_TIMER_TICK in TC     : MSI timer tick/counter from Ecl

// iATU: Outbound settings - target base
#define PCIE_OUTBOUND_ATU_TAR0_BASE         0x00000315 
#define PCIE_OUTBOUND_ATU_TAR1_BASE         0x00000316
#define PCIE_OUTBOUND_ATU_TAR2_BASE         0x00000317
#define PCIE_OUTBOUND_ATU_TAR3_BASE         0x00000318

// DMA/transfer config - read/write port access byte order.
#define PCIE_DMA_DATA_ACCESS_BYTE_ORDER     0x00000319 // Location to pass host endianess. 1 = big endian. 0 = little endian (default)
 
#define PCIE_RX_NUM_PKTS                    0x0000031A // No of Eth frames transferred to the  Host from the SHMEM Mem
#define PCIE_TX_NUM_PKTS                    0x0000031B // No of Eth frames transferred to the  SHMEM from the Host Mem

#define PCIE_DEBUG_MEM_ADDR                 0x0000031C // Temporary debug

// Status value
#define PCIE_HOST_STATUS_RUNNING            0x00000001

#define MXL_MOCA_PCIE_KMALLOC_MAX           65536

/*******************************************************************************
 *      Macros
 ******************************************************************************/

#undef  MXL_MOCA_PCIE_TX_MAPPING_SIZE

#define MXL_MOCA_PCIE_TX_QUEUE_SIZE               1024
#define MXL_MOCA_PCIE_RX_QUEUE_SIZE               256

#define MXL_MOCA_PCIE_TX_MAPPING_SIZE             MXL_MOCA_PCIE_TX_QUEUE_SIZE //Must match the desc queue size in SoC
#define MXL_MOCA_PCIE_RX_RING_SIZE                MXL_MOCA_PCIE_RX_QUEUE_SIZE //Must match the desc queue size in SoC

#define MXL_MOCA_PCIE_HIF_TC0_DMEM_BASE           0x08000000       // Cardiff
#define MXL_MOCA_PCIE_HIF_TC1_DMEM_BASE           0x08040000       // Leucadia

// Match the TX/RX descriptor sizes in SoC
#define MXL_MOCA_PCIE_TX_PTR_QUEUE_SIZE           MXL_MOCA_PCIE_TX_QUEUE_SIZE 
#define MXL_MOCA_PCIE_RX_PTR_QUEUE_SIZE           MXL_MOCA_PCIE_RX_QUEUE_SIZE 

// Match the TX/RX descriptor sizes in SoC
#define MXL_MOCA_PCIE_TX_DESC_QUEUE_SIZE          MXL_MOCA_PCIE_TX_QUEUE_SIZE 
#define MXL_MOCA_PCIE_RX_DESC_QUEUE_SIZE          MXL_MOCA_PCIE_RX_QUEUE_SIZE 

#define MXL_MOCA_PCIE_DESC_OWNER_HOST             0 /* host */
#define MXL_MOCA_PCIE_DESC_OWNER_SOC              1
#define MXL_MOCA_PCIE_DESC_OWNER_PKT_DONE         2 

#define MXL_MOCA_PCIE_LOG_PRINT_BUFSIZE           1024


/*******************************************************************************
 *      Public Function Definitions
 ******************************************************************************/
typedef enum
{
  MXL_MOCA_PCIE_DEVICE_ID_UNKNOWN  = 0x0,
  MXL_MOCA_PCIE_DEVICE_ID_CARDIFF  = MXL_MOCA_PCIE_DEV_ID_CARDIFF,
  MXL_MOCA_PCIE_DEVICE_ID_LEUCADIA = MXL_MOCA_PCIE_DEV_ID_LEUCADIA
} MXL_MOCA_PCIE__DEVICE_ID_E;

typedef enum
{
  MXL_MOCA_PCIE_ERROR_OUT_NONE        = 0x0,
  MXL_MOCA_PCIE_ERROR_OUT_FREE_ALLOC  = 0x1,
  MXL_MOCA_PCIE_ERROR_OUT_FREE_UNMAP  = 0x2,
  MXL_MOCA_PCIE_ERROR_OUT_FREE_NETDEV = 0x3  
} MXL_MOCA_PCIE_ERROR_OUT_E;

enum
{
  MXL_MOCA_PCIE_L_DEF  = 0, /* same as level 2 (include only err and warn) */
  MXL_MOCA_PCIE_L_ERR  = 1,
  MXL_MOCA_PCIE_L_WARN = 2,
  MXL_MOCA_PCIE_L_INFO = 3,
  MXL_MOCA_PCIE_L_DBG1 = 4, /* Primarily used to print entry, exits to/from functions */
  MXL_MOCA_PCIE_L_DBG2 = 5, /* Used for messages inside tx/rx packet handlers */
  MXL_MOCA_PCIE_L_END  = 6
};

#define MXL_MOCA_PCIE_HIF_TC_DMEM_BASE            0x08000000
#define MXL_MOCA_PCIE_CCPU_CORE_BASE              0x0c000000 
#define MXL_MOCA_PCIE_CCPU_CORE_BASE_MAX          0x0cffffff

#define MXL_MOCA_PCIE_EP_BASE                     0x08204000 // 0x08204700 + 0x200

#define MXL_MOCA_PCIE_EHI_CTRL_STAT_REG0          0x08205000
#define MXL_MOCA_PCIE_EHI_CTRL_STAT_REG1          0x08205004
#define MXL_MOCA_PCIE_EHI_CTRL_STAT_REG2          0x08205008


#define MXL_MOCA_PCIE_EHI_CTRL_STAT_REG0_INTX_ASSERT      0x0000E809 
#define MXL_MOCA_PCIE_EHI_CTRL_STAT_REG0_INTX_DEASSERT    0x0000E808 


// Global reset
#define MXL_MOCA_PCIE_EP_LEUCADIA_GLOBAL_RESETN               (0x708) // Leucadia
#define MXL_MOCA_PCIE_EP_CARDIFF_GLOBAL_RESETN                (0x814) // Cardiff


// Only offsets are used in config rd/wr
#define MXL_MOCA_PCIE_EP_IATU_INDEX_OFFSET                    (0x900)
#define MXL_MOCA_PCIE_EP_IATU_REGION_CTRL_1_OFFSET            (0x904)
#define MXL_MOCA_PCIE_EP_IATU_REGION_CTRL_2_OFFSET            (0x908)
#define MXL_MOCA_PCIE_EP_IATU_REGION_LOWER_BASE_ADDR_OFFSET   (0x90C)
#define MXL_MOCA_PCIE_EP_IATU_REGION_LIMIT_ADDR_OFFSET        (0x914)
#define MXL_MOCA_PCIE_EP_IATU_REGION_LOWER_TARGET_ADDR_OFFSET (0x918)


// base address + offsets are used for memory rd/wr
#define MXL_MOCA_PCIE_EP_IATU_INDEX_REG                       (MXL_MOCA_PCIE_EP_BASE + MXL_MOCA_PCIE_EP_IATU_INDEX_OFFSET)
#define MXL_MOCA_PCIE_EP_IATU_REGION_CTRL_1_REG               (MXL_MOCA_PCIE_EP_BASE + MXL_MOCA_PCIE_EP_IATU_REGION_CTRL_1_OFFSET)
#define MXL_MOCA_PCIE_EP_IATU_REGION_CTRL_2_REG               (MXL_MOCA_PCIE_EP_BASE + MXL_MOCA_PCIE_EP_IATU_REGION_CTRL_2_OFFSET)
#define MXL_MOCA_PCIE_EP_IATU_REGION_LOWER_BASE_ADDR_REG      (MXL_MOCA_PCIE_EP_BASE + MXL_MOCA_PCIE_EP_IATU_REGION_LOWER_BASE_ADDR_OFFSET)
#define MXL_MOCA_PCIE_EP_IATU_REGION_LIMIT_ADDR_REG           (MXL_MOCA_PCIE_EP_BASE + MXL_MOCA_PCIE_EP_IATU_REGION_LIMIT_ADDR_OFFSET)
#define MXL_MOCA_PCIE_EP_IATU_REGION_LOWER_TARGET_ADDR_REG    (MXL_MOCA_PCIE_EP_BASE + MXL_MOCA_PCIE_EP_IATU_REGION_LOWER_TARGET_ADDR_OFFSET)

// Global reset
// Leucadia
/*
 pcie_global_resetn - Bit 22: It is 1 by-default and SW needs to write 0 to issue the request for global reset.
 pcie_global_resetn_disable : Bit 23: It is 0 by-default and implies that when pcie_global_resetn bit is set to 0,
 then it will reset the entire SoC including PCIe 
*/
#define MXL_MOCA_PCIE_EP_LEUCADIA_GLOBAL_RESET_NO_LINK_DN         0x07800004
#define MXL_MOCA_PCIE_EP_LEUCADIA_GLOBAL_RESET_NO_LINK_DN_CLEAR   0x07C00004

/*
 pcie_global_resetn - Bit 0: It is 1 by-default and SW needs to write 0 to issue the request for global reset.
 pcie_global_resetn_disable : Bit 1: It is 0 by-default and implies that when pcie_global_resetn bit is set to 0,
 then it will reset the entire SoC including PCIe 
*/
// Cardiff
#define MXL_MOCA_PCIE_EP_CARDIFF_GLOBAL_RESET_NO_LINK_DN          0x00000002
#define MXL_MOCA_PCIE_EP_CARDIFF_GLOBAL_RESET_NO_LINK_DN_CLEAR    0x00000003


/*******************************************************************************
 *      Public/Private Data
 ******************************************************************************/
/*!
 * \brief Spinlock structure with added bells and whistles
 *
 * For locking purposes.
 */
typedef struct
{
  unsigned long   lockIrqFlags;
  spinlock_t      lockSpinLock;
} MXL_MOCA_PCIE_ETH_LOCK_T;

/*!
 * \brief Semaphore structure
 */
typedef struct
{
  struct semaphore osSema;
} MXL_MOCA_PCIE_ETH_SEMA_T;


/*!
 * \brief Packet element structure
 */
typedef struct  
{
  uint32_t   totalLen;             // Packet length. DECLARE_PCI_UNMAP_LEN(totalLen)
  dma_addr_t mapping;              // sk_buff physical addr/mapping. DECLARE_PCI_UNMAP_ADDR(mapping)
} MXL_MOCA_PCIE_QUEUE_PKT_T; 

typedef struct
{
  uint32_t  dword0;  /* length and ownership. length[17:2], ownership[1:0] */
  uint32_t  dword1;  /* dma physical addr of the packet                    */
} MXL_MOCA_PCIE_QUEUE_DESCR_T;

typedef struct
{
  void*                         pPktQueuePa;   // Pointer queue physical addr (MxL_MoCA_PCIeAllocDmaMem-->pa)
  MXL_MOCA_PCIE_QUEUE_PKT_T*    pPktQueue;     // Packet : 32 packets (same as descriptors length) (MxL_MoCA_PCIeAllocDmaMem-->va)

  void*                         pDescQueuePa;
  MXL_MOCA_PCIE_QUEUE_DESCR_T*  pDescQueue;   // Desc Queue in host : 32x2 descriptors

  uint32_t            descQueueAddr;      // Host descr queue addr value: host will write the physical base address of the queue here using MemWrite    
  uint32_t            descQueueSize;      // Desc Queue size (in host memory)

  uint32_t            descQueueReadIdx;   // Read index for the host descr 
  uint32_t            descQueueWriteIdx;  // Write index for the host descr 

  uint32_t            lastPktsCnt;        // last counter for # of pkts processed. There is a proc entry for tx/rx stats.
} MXL_MOCA_PCIE_QUEUE_PAIR_T;

/*!
 * \brief DP Structure
 *
 */
typedef struct
{
  MXL_MOCA_PCIE_QUEUE_PAIR_T    txQueuePair;
  MXL_MOCA_PCIE_QUEUE_PAIR_T    rxQueuePair;
} MXL_MOCA_PCIE_DATAPATH_CONTEXT_T;


// MxL MoCA Ethernet Statistics Structure
typedef struct
{
  // Driver stats
  uint32_t drvSwRevNum;
  uint32_t embSwRevNum;
  uint32_t upTime;
  uint32_t linkUpTime;

  // Transmit and recieve stats
  uint32_t rxPackets;           // Total packets received
  uint32_t txPackets;           // Total packets transmitted
  uint32_t rxBytes;             // Total bytes received
  uint32_t txBytes;             // Total bytes transmitted
  uint32_t rxPacketsGood;       // Packet receive with no errors
  uint32_t txPacketsGood;       // Packet transmitted with no errors
  uint32_t rxPacketErrs;        // Packets received with errors
  uint32_t txPacketErrs;        // Packets transmitted with errors
  uint32_t rxDroppedErrs;       // Packets dropped with no host buffers
  uint32_t txDroppedErrs;       // Packet dropped with no host buffers
  uint32_t rxMulticastPackets;  // Total multicast packets received
  uint32_t txMulticastPackets;  // Total multicast packets transmitted
  uint32_t rxMulticastBytes;    // Total multicast bytes received
  uint32_t txMulticastBytes;    // Total multicast bytes transmitted

  // Detailed receive errors
  uint32_t rxLengthErrs;
  uint32_t rxCrc32Errs;         // Packets rx'd with CRC-32 errors
  uint32_t rxFrameHeaderErrs;   // Packets rx'd with frame header errors
  uint32_t rxFifoFullErrs;      // Packets rx'd with FIFO full
  uint32_t rxListHaltErr[3];

  // Detailed transmit errors
  uint32_t txCrc32Errs;         // Packets tx'd with CRC-32 errors
  uint32_t txFrameHeaderErrs;   // Packets tx'd with frame header errors
  uint32_t txFifoFullErrs;      // Packets tx'd with FIFO full
  uint32_t txFifoHaltErr[8];
} MXL_MOCA_PCIE_ETH_STATS_T; 

/*!
 * \brief Ring info data structure
 *
 */
typedef struct __ring_info
{
  struct sk_buff *skb;
  dma_addr_t mapping;
} MXL_MOCA_PCIE_RING_INFO_T;

/*!
 * \brief eth_dev data structure
 *
 * This structure is instantiated at probe time by allocation of the net_device.
 * It exists at the end of the net_device and is pointed to by the priv member
 * of the net_device.
 */
typedef struct 
{
  // Device data
  struct pci_dev                   *pPCIeDev;   // pointer to pci_dev
  struct net_device                *pNetDev;    // pointer to net_dev
  MXL_MOCA_PCIE_DATAPATH_CONTEXT_T *pDpCtx;     // pointer to data path structure 

  // Identification
  MXL_MOCA_PCIE__DEVICE_ID_E        deviceId;
  uint32_t                          devIndex;  
  uint32_t                          mgmtIfMacAddr[2];

  // Stack interface
  MXL_MOCA_PCIE_RING_INFO_T rxRing[MXL_MOCA_PCIE_RX_RING_SIZE];
  MXL_MOCA_PCIE_RING_INFO_T txRing[MXL_MOCA_PCIE_TX_MAPPING_SIZE];
  
  // skb management
  int32_t       txRingWrIndex;
  int32_t       txRingRdIndex;

  // NAPI support
#if MXL_MOCA_PCIE_NAPI
  struct napi_struct      napi;
  int32_t        napiDisableIntr;
  int32_t        polls;
  int32_t        napiWorkDone;
#endif

  // Device capabilities/control/status
  int32_t       devCapMaxPayloadSz;
  int32_t       devCtlMaxPayloadSz;
  int32_t       devCtlMaxReadReqSz;
  int32_t       lnkCapMaxLnkSpeed;
  int32_t       lnkCapMaxLinkWidth;
  int32_t       lnkCapAspmSupport;   /* ASPM support */
  int32_t       lnkCapPme;           /* PME# support */
  int32_t       lnkCtlAspmL1;
  int32_t       lnkCtlRCB;
  int32_t       lnkStaMaxLnkSpeed;
  int32_t       lnkStaMaxLinkWidth;

  // iATU
  uint32_t      dmemBaseAddr;          // Depends on Chip ID
  uint32_t      region0OutATUAddrBase;
  uint32_t      region1OutATUAddrBase;
  uint32_t      region2OutATUAddrBase; // reserved
  uint32_t      region3OutATUAddrBase; // reserved

  uint32_t      target0OutATUAddrMask;
  uint32_t      target1OutATUAddrMask;
  uint32_t      target2OutATUAddrMask; // reserved
  uint32_t      target3OutATUAddrMask; // reserved

  // Interrupts counter
  int32_t       interrupts; // MSI/INTx issued by SoC FW  
  
  // PCIe config space
  /* We have two BARs */
  // 4MB: Use for L2 space access
  uintptr_t     baseIoAddr0; // physical address
  uint32_t      baseIoLen0;
  uintptr_t     baseIoRemap0;// virtual address
  
  // 16MB: Use for Fusion4 access
  uintptr_t     baseIoAddr1; // physical address
  uint32_t      baseIoLen1;
  uintptr_t     baseIoRemap1;// virtual address

  // flags
  uint32_t      active;      // datapath up/down
  uint32_t      firstTime;   // first-time up indicator
  uint32_t      busy;        // for DEBUG
  uint32_t      inbandIntrType;    // MSI/INTx
#if MXL_MOCA_PCIE_DEBUG_DROP_TX  
  uint32_t      dbg_drop_tx;
#endif
#if MXL_MOCA_PCIE_DEBUG_DROP_RX
  uint32_t      dbg_drop_rx;
#endif
  // rx stats 
  uint32_t      netIfRxDropped;
  uint32_t      rxStats[4]; // for DEBUG). See Proc read/write
  
  // tx stats
  uint32_t      numPktsQueued;
  uint32_t      numPktsXmited;
  uint32_t      txIntrCount; // interrupts from stack
  uint32_t      txStats[6]; // for DEBUG). See Proc read/write

  // errors
  uint32_t      mappingErr0;
  uint32_t      mappingErr1;
  uint32_t      mappingErr2;
  uint32_t      skbAllocErr;
  
  MXL_MOCA_PCIE_ETH_STATS_T   ethStats;       // /sys/class/net/en0/statistics 
  struct net_device_stats stats;              // net device statistics

  // proc entry
  uint32_t      readProcMode;
  int8_t        procDirName[MXL_MOCA_PCIE_PROC_NAME_LEN_MAX+1];
  struct proc_dir_entry *pProcDir;

  // Synchronizing objects
  MXL_MOCA_PCIE_ETH_LOCK_T    ethTxLock; ///< xmit spin lock: avoid recursive interrupts and for net_dev_ops
  MXL_MOCA_PCIE_ETH_LOCK_T    rwLock;    ///< For PCI address read/write spin lock
} MXL_MOCA_PCIE_DEV_DATA_CONTEXT_T;

/*!
 * \brief Driver kernel context.
 *
 */
typedef struct net_device  MXL_MOCA_PCIE_KERNEL_CONTEXT_T; 

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 29)
  #define NETDEV_PRIV(dev) netdev_priv(dev)
#else
  #define NETDEV_PRIV(dev) (dev)->priv
#endif

/*******************************************************************************
 *      Function Definitions
 ******************************************************************************/
MXL_MOCA_STATUS_E MxL_MoCA_PCIeRestart(int8_t devIndex);
void              MxL_MoCA_PCIeSetMgmtMAC(int8_t devIndex, uint32_t* pMac);
MXL_MOCA_STATUS_E MxL_MoCA_PCIeAvailable(int8_t devIndex);
void              MxL_MoCA_PCIeStop (int8_t devIndex);
MXL_MOCA_STATUS_E MxL_MoCA_PCIeRead(int8_t devIndex, uintptr_t addr, uint32_t *pVal);
MXL_MOCA_STATUS_E MxL_MoCA_PCIeWrite(int8_t devIndex, uintptr_t addr, uint32_t pVal);
MXL_MOCA_STATUS_E MxL_MoCA_PCIeGlobalResetSoC(int8_t devIndex);

#endif /* __MXL_MOCA_OSAL_PCIE_H__ */

