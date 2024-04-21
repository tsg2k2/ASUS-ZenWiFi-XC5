/*****************************************************************************************
 *
 * FILE NAME          : mxl_moca_osal_pcie.c
 * 
 *
 *
 * DATE CREATED       : 03/25/2016
 *
 * LAST MODIFIED      : %Name% @ %%/%%/%% 
 *
 * DESCRIPTION        : Linux PCIe/Ethernet Driver. This file contains APIs that 
 *                      interface with the control path driver and to handle datapath
 *                      packets to(tx) and from (rx) SoC. 
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

/*----------------------------------------------------------------------------------------
 *      Includes
 *--------------------------------------------------------------------------------------*/
#include "mxl_moca_host_version.h"
#include "mxl_moca_config.h"
#include "mxl_moca_osal_pcie.h"

/*----------------------------------------------------------------------------------------
 * Command line arguments:
 * Usage: sudo insmod mxl_moca_pcie.ko verbose=3 mac_addr="00:20:30:40:50:60"
 * module_param - typesafe helper for a module/cmdline parameter
 * @value: the variable to alter, and exposed parameter name.
 * @type: the type of the parameter
 * @perm: visibility in sysfs. 
 *--------------------------------------------------------------------------------------*/
char *mac_addr = "00:09:8b:40:50:60";
module_param(mac_addr, charp, 0);
MODULE_PARM_DESC(mac_addr, "mac_addr=\"xx:xx:xx:xx:xx:xx\""); // stored in net_device 

int verbose = MXL_MOCA_PCIE_L_DEF; 
module_param(verbose, int, MXL_MOCA_MODULE_PARAM_PERM); // ( S_IRUSR | S_IWUSR) | S_IRGRP | S_IROTH )
MODULE_PARM_DESC(verbose, "verbose=value where value ranges from 0-5, 0 is default");

int intr_sel_type = PCIE_INTR_SEL_AUTO; // { PCIE_INTR_SEL_AUTO=0; PCIE_INTR_SEL_MSI=1; PCIE_INTR_SEL_INTA=2 }
module_param(intr_sel_type, int, MXL_MOCA_MODULE_PARAM_PERM);
MODULE_PARM_DESC(intr_sel_type, "0:sel AUTO(default); 1:sel MSI; 2:sel INTA.");

// Set CONFIG_PCI_DEBUG=y in kernel config if you want the PCI core to produce a bunch of debug messages to the system log. 
/*----------------------------------------------------------------------------------------
 *      Global Data
 *--------------------------------------------------------------------------------------*/
static int gNumMocaPcieDevices = 0;

// moduleInitError is used for auto-detection of PCIe/SGMII for the control path driver and ECA platforms.
// When the PCIe device detection fails in module_init, we still return success to keep the module loaded.
// Without this, the control path driver (mxl_moca_ctrl.ko) will not load due to static binding 
// to data path interfaces.
#if MXLWARE_CONFIG_BOARD_ECA_9M_L3_NXP 
static uint32_t moduleInitError = 0;
#endif

static MXL_MOCA_PCIE_DEV_DATA_CONTEXT_T* gpDevData[MXL_MOCA_PCIE_MAX_NUM_OF_PCIE_DEVICES];
/* MXL_MOCA_PCIE_L_WARN: minimum level to include errors and warnings */
static uint32_t MXL_MOCA_PCIE_HOST_PRINTLOG_THRESHOLD = MXL_MOCA_PCIE_L_INFO; 

/*----------------------------------------------------------------------------------------
 *      Supported devices, Vendor/Device table
 *--------------------------------------------------------------------------------------*/
static const struct pci_device_id __devinitdata gMxlMocaPciDevTbl[MXL_MOCA_PCIE_MAX_NUM_OF_PCIE_DEV_ID + 1] =
{ // Vendor,                       device,                             subvendor,     subdevice,    class, mask, drv_data
  {  MXL_MOCA_PCIE_VENDOR_ID, MXL_MOCA_PCIE_DEV_ID_CARDIFF,  PCI_ANY_ID, PCI_ANY_ID, 0,    0,   0},
  {  MXL_MOCA_PCIE_VENDOR_ID, MXL_MOCA_PCIE_DEV_ID_LEUCADIA, PCI_ANY_ID, PCI_ANY_ID, 0,    0,   0},
  { }
};

MODULE_DEVICE_TABLE(pci, gMxlMocaPciDevTbl);

/*----------------------------------------------------------------------------------------
 *      PCIe Device init and exit
 *--------------------------------------------------------------------------------------*/
static int  __devinit MxL_MoCA_PCIeProbeOne (struct pci_dev *pDev, const struct pci_device_id *pEntry);
static void __devexit MxL_MoCA_PCIeRemoveOne(struct pci_dev *pDev);

/*----------------------------------------------------------------------------------------
 *      PCIe power management
 *      Set CONFIG_PCIEASPM_POWERSAVE=y and CONFIG_PM_RUNTIME=y in kernel .config
 *--------------------------------------------------------------------------------------*/
 int MxL_MoCA_PCIeDevPmSuspend(struct device *pDev);
 int MxL_MoCA_PCIeDevPmResume(struct device *pDev);
 int MxL_MoCA_PCIeDevPmIdle(struct device *pDev);

/* pm-ops: assignments
  .suspend         = MxL_MoCA_PCIeDevPmSuspend
  .resume          = MxL_MoCA_PCIeDevPmResume
  .freeze          = MxL_MoCA_PCIeDevPmSuspend
  .thaw            = MxL_MoCA_PCIeDevPmResume
  .poweroff        = MxL_MoCA_PCIeDevPmSuspend
  .restore         = MxL_MoCA_PCIeDevPmResume
  .runtime_suspend = MxL_MoCA_PCIeDevPmSuspend
  .runtime_resume  = MxL_MoCA_PCIeDevPmResume
  .runtime_idle    = MxL_MoCA_PCIeDevPmIdle
*/

// Use this ops for runtime PME
#if LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 31)
UNIVERSAL_DEV_PM_OPS(pciDevPmOps, MxL_MoCA_PCIeDevPmSuspend, MxL_MoCA_PCIeDevPmResume, MxL_MoCA_PCIeDevPmIdle);
//SIMPLE_DEV_PM_OPS(pciDevPmOps, MxL_MoCA_PCIeDevPmSuspend, MxL_MoCA_PCIeDevPmResume);
#endif

/*----------------------------------------------------------------------------------------
 *      PCIe Advanced Error Reporting (AER)
 *      Refer: https://lwn.net/Articles/162550/
 *      Set CONFIG_PCIEPORTBUS=y and CONFIG_PCIEAER = y in kernel .config
 *      To test errors, set CONFIG_PCIEAER_INJECT=y
 *--------------------------------------------------------------------------------------*/

/* PCI bus error detected on this device */
pci_ers_result_t MxL_MoCA_PCIeAerErrorDetected(struct pci_dev *dev, enum pci_channel_state error);
/* PCI Express link has been reset */
pci_ers_result_t MxL_MoCA_PCIeAerLinkReset(struct pci_dev *dev);
/* PCI slot has been reset */
pci_ers_result_t MxL_MoCA_PCIeAerSlotReset(struct pci_dev *dev);
/* Device driver may resume normal operations */
void MxL_MoCA_PCIeAerResumeNormalOps(struct pci_dev *dev); 

static const struct pci_error_handlers pciErrHandlers = 
{
  .error_detected = MxL_MoCA_PCIeAerErrorDetected,
  .link_reset     = MxL_MoCA_PCIeAerLinkReset,
  .slot_reset     = MxL_MoCA_PCIeAerSlotReset,
  .resume         = MxL_MoCA_PCIeAerResumeNormalOps,
};

/*----------------------------------------------------------------------------------------
 *      PCIe driver structure
 *--------------------------------------------------------------------------------------*/

static struct pci_driver gMxlMocaPCIeDrv =
{
name:
  MXL_MOCA_PCIE_DRV_NAME,
id_table:
  gMxlMocaPciDevTbl,
probe:
  MxL_MoCA_PCIeProbeOne,
remove:
  __devexit_p(MxL_MoCA_PCIeRemoveOne),
.driver = 
  {
  .name = MXL_MOCA_PCIE_DRV_NAME,
#if LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 31)    
  .pm = &pciDevPmOps,
#endif  
  },
.err_handler = &pciErrHandlers
};

/*----------------------------------------------------------------------------------------
 *      Proc entry section
 *--------------------------------------------------------------------------------------*/

#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 10, 0)

static int MxL_MoCA_PCIeReadProc(char *pUserBuf, char **ppStart, off_t offset, int count, int *pEof, void *pData);
int MxL_MoCA_PCIeWriteProc(struct file *pFile, const char *pBuffer, unsigned long count, void *pData);

#else // LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0)

// Common buffer for all devices
static int8_t   gProcFsBuffer[MXL_MOCA_PROCFS_MAX_SIZE];

static ssize_t MxL_MoCA_PCIeReadProc(struct file *pFile, char __user *pPage,  size_t len, loff_t* pOffset);
ssize_t        MxL_MoCA_PCIeWriteProc(struct file *pFile, const char __user *pBuffer, size_t count, loff_t* pOffset);

static const struct file_operations gMxlMocaProcFops = 
{
  .read  = MxL_MoCA_PCIeReadProc,
  .write = MxL_MoCA_PCIeWriteProc,
};

#endif

/*----------------------------------------------------------------------------------------
 *      Network device structure. Operations for the system to interact with
 *      the device.
 *--------------------------------------------------------------------------------------*/
 
 // net_device_ops
int MxL_MoCA_PCIeNetDevOpen (struct net_device *pNetDev);
int MxL_MoCA_PCIeNetDevClose(struct net_device *pNetDev);
static int  MxL_MoCA_PCIeNetDevStartXmit(struct sk_buff *skb, struct net_device *pNetDev);
static struct net_device_stats *MxL_MoCA_PCIeNetDevGetStats(struct net_device *pNetDev);
static void MxL_MoCA_PCIeNetDevSetRxMode(struct net_device *pNetDev);
static void MxL_MoCA_PCIeNetDevTxTimeout(struct net_device *pNetDev);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 29)
static const struct net_device_ops gMxlMocaPcieNetDevOps = 
{
  .ndo_open         = MxL_MoCA_PCIeNetDevOpen,
  .ndo_stop         = MxL_MoCA_PCIeNetDevClose,
  .ndo_start_xmit   = MxL_MoCA_PCIeNetDevStartXmit,
  .ndo_get_stats    = MxL_MoCA_PCIeNetDevGetStats,
  .ndo_set_rx_mode  = MxL_MoCA_PCIeNetDevSetRxMode,
  .ndo_tx_timeout   = MxL_MoCA_PCIeNetDevTxTimeout,
  .ndo_do_ioctl     = 0,
};
#endif

/*----------------------------------------------------------------------------------------
 *      Ethernet tool ops. 
 *      ethtool -i en0
 *      
 *--------------------------------------------------------------------------------------*/
 
static void  MxL_MoCA_PCIeEthtoolGetDrvInfo(struct net_device *pNetDev, struct ethtool_drvinfo *drvinfo);
unsigned int MxL_MoCA_PCIeEthtoolGetLink   (struct net_device *pNetDev);

// eth_tool_ops
static const struct ethtool_ops gMxlMocaPcieEthtoolOps =
{
  .get_drvinfo            = MxL_MoCA_PCIeEthtoolGetDrvInfo,
  .get_link               = MxL_MoCA_PCIeEthtoolGetLink, 
#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 3, 0)
  .get_sg                 = ethtool_op_get_sg,
  .set_sg                 = ethtool_op_set_sg,
#ifdef NETIF_F_TSO
  .get_tso                = ethtool_op_get_tso,
#endif
#endif
};

/*----------------------------------------------------------------------------------------
 *        - Data path functions - forward declarations
 *--------------------------------------------------------------------------------------*/
 
static void MxL_MoCA_PCIeGetMgmtMAC(MXL_MOCA_PCIE_DEV_DATA_CONTEXT_T *pDdc, unsigned int* mac); 
static MXL_MOCA_STATUS_E  MxL_MoCA_PCIeAllocResources(MXL_MOCA_PCIE_DEV_DATA_CONTEXT_T *pDdc);
static MXL_MOCA_STATUS_E  MxL_MoCA_PCIeAllocRxBuffers(void* pNetDev);
static MXL_MOCA_STATUS_E  MxL_MoCA_PCIeFreeRxBuffers(void* pNetDev);
static MXL_MOCA_STATUS_E  MxL_MoCA_PCIeResetTxQueue(void* pNetDev);
static void MxL_MoCA_PCIeStartXmit(void* arg);
static void MxL_MoCA_PCIeStopXmit(void* arg);
static MXL_MOCA_STATUS_E  MxL_MoCA_PCIeEnqueueRxBuffs(struct net_device *pNetDev);
#if MXL_MOCA_PCIE_NAPI
static int  MxL_MoCA_PCIeNapiPoll(struct napi_struct *p_napi, int budget);
#endif
static MXL_MOCA_STATUS_E MxL_MoCA_PCIeStartXmitSkbs(struct net_device *pNetDev);
static MXL_MOCA_STATUS_E MxL_MoCA_PCIeDataPathReset (MXL_MOCA_PCIE_DEV_DATA_CONTEXT_T *pDdc, int8_t devIndex);
static MXL_MOCA_STATUS_E MxL_MoCA_PCIeDataPathInit(MXL_MOCA_PCIE_DEV_DATA_CONTEXT_T *pDdc, int8_t devIndex);

/*----------------------------------------------------------------------------------------
 *        - ISR triggered by MSI or INTA(ASSERT)TLP from the EP
 *--------------------------------------------------------------------------------------*/
 
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 19)
static irqreturn_t MxL_MoCA_PCIeISR(int irq, void *p_dev);
#else
static irqreturn_t MxL_MoCA_PCIeISR(int irq, void *p_dev, struct pt_regs *regs); 
#endif

/*----------------------------------------------------------------------------------------
 *        - PCIe EP initialization functions
 *--------------------------------------------------------------------------------------*/
 
static int __init MxL_MoCA_PCIeModuleInit(void);
static MXL_MOCA_STATUS_E MxL_MoCA_PCIeInitSoCIfc(unsigned int devIndex);
static MXL_MOCA_STATUS_E MxL_MoCA_PCIeConfigInboundATU(unsigned int devIndex);
static MXL_MOCA_STATUS_E MxL_MoCA_PCIeConfigOutboundATU(unsigned int devIndex);
static void MxL_MoCA_PCIeSetEthToolVar(struct net_device *pNetDev);

/*----------------------------------------------------------------------------------------
 *
 * FUNCTION NAME : MxL_MoCA_PCIeFormatString
 *
 *
 *
 * DATE CREATED  : 12/13/2016
 *
 * DESCRIPTION   : Get parameters from argument buffer
 *
 * IN PARAMS     : buf  : Buffer to return created string
 *                 size : max size in bytes of the buffer
 *                 fmt  : format specifier string
 *
 * OUT PARAMS    : None
 *
 * RETURN VALUE  : Length of created string
 *
 *--------------------------------------------------------------------------------------*/

int32_t MxL_MoCA_PCIeFormatString(int8_t *buf, uint32_t size, const int8_t *fmt, ...)
{
  va_list args;
  int32_t i;

  va_start(args, fmt);
  i=vsnprintf(buf,size,fmt,args);
  va_end(args);

  return i;
}

/*----------------------------------------------------------------------------------------
 * 
 * FUNCTION NAME : MxL_MoCA_PCIeDbgTrace
 * 
 *
 *
 * DATE CREATED  : 03/25/2016
 *
 * DESCRIPTION   : Variable argument OS print
 * 
 *  For reference:  Linux has these
 *
 *   KERN_EMERG   "<0>" Used for emergency messages, usually those that precede a crash.
 *   KERN_ALERT   "<1>" A situation requiring immediate action.
 *   KERN_CRIT    "<2>" Critical conditions, often related to serious hw/sw failures.
 *   KERN_ERR   "<3>" Used to report error conditions; device drivers often use KERN_ERR 
 *                      to report hardware difficulties
 *   KERN_WARNING "<4>" Warnings about problematic situations that do not, in themselves, 
 *                      create serious problems with the system.
 *   KERN_NOTICE  "<5>" Situations that are normal, but still worthy of note. A number 
 *                      of security-related conditions are reported at this level.
 *   KERN_INFO    "<6>" Informational messages. Many drivers print information about 
 *                      the hardware they find at startup time at this level.
 *   KERN_DEBUG   "<7>" Debugging messages.
 *
 * IN PARAMS     : lev Severity level. See the MXL_MOCA_PCIE_L_* enum definitions listed.
 *                     MXL_MOCA_PCIE_L_DEF      "<0>"   : defailt (same as 2)             
 *                     MXL_MOCA_PCIE_L_ERR      "<1>"   : error conditions 
 *                     MXL_MOCA_PCIE_L_WARN     "<2>"   : warning conditions 
 *                     MXL_MOCA_PCIE_L_INFO     "<3>"   : informational                   
 *                     MXL_MOCA_PCIE_L_DBG1     "<4>"   : debug messages (level1)         
 *                     MXL_MOCA_PCIE_L_DBG2      "<5>"  : debug messages (level2)   
 *                     See MXL_MOCA_PCIE_HOST_OS_PRINTLOG_THRESHOLD
 *                 fmt Printf format string. 
 *                 ... Printf format arguments (variable length)
 *
 * OUT PARAMS    : None
 *
 * RETURN VALUE  : None
 *
 *--------------------------------------------------------------------------------------*/

static void MxL_MoCA_PCIeDbgTrace(uint32_t lev, const int8_t *fmt, ...)
{
  va_list args;
  int  printLen;
  static int8_t printkBuf[MXL_MOCA_PCIE_LOG_PRINT_BUFSIZE];

  if (lev <= MXL_MOCA_PCIE_HOST_PRINTLOG_THRESHOLD) 
  {
    /* Print argument data to buffer */
    va_start(args, fmt);
    printLen = vsnprintf(printkBuf, sizeof(printkBuf), fmt, args);
    va_end(args);
    
    if(printLen < 0)
    {
      printk(KERN_ERR     "%s: %s", MXL_MOCA_PCIE_DRV_NAME, "vsnprintf failed!!\n");
      return;
    }

    if (printLen < MXL_MOCA_PCIE_LOG_PRINT_BUFSIZE)
    {
      switch (lev)
      {
      case MXL_MOCA_PCIE_L_ERR:
        printk(KERN_ERR     "%s: %s", MXL_MOCA_PCIE_DRV_NAME, printkBuf); 
        break ;
      case MXL_MOCA_PCIE_L_WARN:
        printk(KERN_WARNING "%s: %s", MXL_MOCA_PCIE_DRV_NAME, printkBuf); 
        break ;
      case MXL_MOCA_PCIE_L_INFO:
        printk(KERN_INFO    "%s: %s", MXL_MOCA_PCIE_DRV_NAME, printkBuf);
        break ;
      case MXL_MOCA_PCIE_L_DBG1:
        printk(KERN_NOTICE  "%s: %s", MXL_MOCA_PCIE_DRV_NAME, printkBuf);
        break ;
      case MXL_MOCA_PCIE_L_DBG2: 
        printk(KERN_DEBUG   "%s: %s", MXL_MOCA_PCIE_DRV_NAME, printkBuf);
        break ;
      }
    }
    else
    {
      printk(KERN_ERR     "%s: %s", MXL_MOCA_PCIE_DRV_NAME, "Not enough DEBUG print buffer!!\n"); 
    }
  }
}

/*----------------------------------------------------------------------------------------
 * 
 * FUNCTION NAME : MxL_MoCA_PCIeAerErrorDetected
 * 
 *
 *
 * DATE CREATED  : 12/07/2016
 *
 * DESCRIPTION   : PCI bus error detected on this device
 *
 * IN PARAMS     : pDev : device pointer
 *
 * OUT PARAMS    : None
 *
 * RETURN VALUE  : pci_ers_result_t
 *
 *--------------------------------------------------------------------------------------*/

pci_ers_result_t MxL_MoCA_PCIeAerErrorDetected(struct pci_dev *pDev, enum pci_channel_state error)
{
  MxL_MoCA_PCIeDbgTrace(MXL_MOCA_PCIE_L_INFO, "PCIe Aer Error Detected handler called\n");
  return PCI_ERS_RESULT_NEED_RESET;
}

/*----------------------------------------------------------------------------------------
 * 
 * FUNCTION NAME : MxL_MoCA_PCIeAerLinkReset
 * 
 *
 *
 * DATE CREATED  : 12/07/2016
 *
 * DESCRIPTION   : PCI Express link has been reset
 *
 * IN PARAMS     : pDev : device pointer
 *
 * OUT PARAMS    : None
 *
 * RETURN VALUE  : pci_ers_result_t
 *
 *--------------------------------------------------------------------------------------*/

pci_ers_result_t MxL_MoCA_PCIeAerLinkReset(struct pci_dev *pDev)
{
  MxL_MoCA_PCIeDbgTrace(MXL_MOCA_PCIE_L_INFO, "PCIe Aer Link Reset handler called\n");
  return PCI_ERS_RESULT_RECOVERED;
}

/*----------------------------------------------------------------------------------------
 * 
 * FUNCTION NAME : MxL_MoCA_PCIeAerSlotReset
 * 
 *
 *
 * DATE CREATED  : 12/07/2016
 *
 * DESCRIPTION   : PCI slot has been reset 
 *
 * IN PARAMS     : pDev : device pointer
 *
 * OUT PARAMS    : None
 *
 * RETURN VALUE  : pci_ers_result_t
 *
 *--------------------------------------------------------------------------------------*/

pci_ers_result_t MxL_MoCA_PCIeAerSlotReset(struct pci_dev *pDev)
{
  MxL_MoCA_PCIeDbgTrace(MXL_MOCA_PCIE_L_INFO, "PCIe Aer Slot Reset handler called\n");
  return PCI_ERS_RESULT_RECOVERED;
}

/*----------------------------------------------------------------------------------------
 * 
 * FUNCTION NAME : MxL_MoCA_PCIeAerResumeNormalOps
 * 
 *
 *
 * DATE CREATED  : 12/07/2016
 *
 * DESCRIPTION   : Device driver may resume normal operations 
 *
 * IN PARAMS     : pDev : device pointer
 *
 * OUT PARAMS    : None
 *
 * RETURN VALUE  : None
 *
 *--------------------------------------------------------------------------------------*/

void MxL_MoCA_PCIeAerResumeNormalOps(struct pci_dev *pDev)
{
  MxL_MoCA_PCIeDbgTrace(MXL_MOCA_PCIE_L_INFO, "PCIe Aer Resume Normal Ops handler called\n");
  pci_cleanup_aer_uncorrect_error_status(pDev);
}

/*----------------------------------------------------------------------------------------
 * 
 * FUNCTION NAME : MxL_MoCA_PCIeDevPmSuspend
 * 
 *
 *
 * DATE CREATED  : 11/29/2016
 *
 * DESCRIPTION   : PCIe power management ops 
 *
 * IN PARAMS     : pDev : device pointer
 *
 * OUT PARAMS    : None
 *
 * RETURN VALUE  : pci_ers_result_t
 *
 *--------------------------------------------------------------------------------------*/

int MxL_MoCA_PCIeDevPmSuspend(struct device *pDev)
{
  struct pci_dev *pPCIeDev = to_pci_dev(pDev);
  /* Other data can be accessed as follows 
     struct net_device *pNetDev = pci_get_drvdata(pPCIeDev);
     struct MXL_MOCA_PCIE_DEV_DATA_CONTEXT_T *pDdc = NETDEV_PRIV(pNetDev);
  */
  
  MxL_MoCA_PCIeDbgTrace(MXL_MOCA_PCIE_L_INFO, "Suspend\n");
  MxL_MoCA_PCIeDbgTrace(MXL_MOCA_PCIE_L_INFO, "current_state = %x\n", pPCIeDev->current_state);
  pci_disable_device(pPCIeDev); // disable bus mastering
  pci_set_power_state(pPCIeDev,PCI_D3hot); // device's power state
  MxL_MoCA_PCIeDbgTrace(MXL_MOCA_PCIE_L_INFO, "current_state = %x\n", pPCIeDev->current_state);
  /* No more memory access after this point until device is brought back to D0 */
  return 0;
}

/*----------------------------------------------------------------------------------------
 * 
 * FUNCTION NAME : MxL_MoCA_PCIeDevPmResume
 * 
 *
 *
 * DATE CREATED  : 11/29/2016
 *
 * DESCRIPTION   : PCIe power management ops 
 *
 * IN PARAMS     : dev : device pointer
 *
 * OUT PARAMS    : None
 *
 * RETURN VALUE  : 0 on success
 *
 *--------------------------------------------------------------------------------------*/

int MxL_MoCA_PCIeDevPmResume(struct device *pDev)
{
  struct pci_dev *pPCIeDev = to_pci_dev(pDev);
  
  MxL_MoCA_PCIeDbgTrace(MXL_MOCA_PCIE_L_INFO, "Resume\n");
  MxL_MoCA_PCIeDbgTrace(MXL_MOCA_PCIE_L_INFO, "current_state = %x\n", pPCIeDev->current_state);
  pci_set_power_state(pPCIeDev,PCI_D0);

  if (pci_enable_device(pPCIeDev))
  { 
    MxL_MoCA_PCIeDbgTrace(MXL_MOCA_PCIE_L_ERR, "pci_enable_device failed\n");
  }

  pci_pme_active (pPCIeDev, true);
  MxL_MoCA_PCIeDbgTrace(MXL_MOCA_PCIE_L_INFO, "current_state = %x\n", pPCIeDev->current_state);
  
  return 0;
}

/*----------------------------------------------------------------------------------------
 * 
 * FUNCTION NAME : MxL_MoCA_PCIeDevPmIdle
 * 
 *
 *
 * DATE CREATED  : 11/29/2016
 *
 * DESCRIPTION   : PCIe power management ops 
 *
 * IN PARAMS     : dev : device pointer
 *
 * OUT PARAMS    : None
 *
 * RETURN VALUE  : 0 on success
 *
 *--------------------------------------------------------------------------------------*/

int MxL_MoCA_PCIeDevPmIdle(struct device *pDev)
{
  MxL_MoCA_PCIeDbgTrace(MXL_MOCA_PCIE_L_INFO, "Idle\n");
  return 0;
}

/*----------------------------------------------------------------------------------------
 * 
 * FUNCTION NAME : MxL_MoCA_PCIeLockInit
 * 
 *
 *
 * DATE CREATED  : 03/25/2016
 *
 * DESCRIPTION   : Initializes a spinlock
 *
 * IN PARAMS     : pLock Pointer to a lock structure
 *
 * OUT PARAMS    : None
 *
 * RETURN VALUE  : None
 *
 *--------------------------------------------------------------------------------------*/
 
static void MxL_MoCA_PCIeLockInit(void *pLock)
{
  MXL_MOCA_PCIE_ETH_LOCK_T *lk = (MXL_MOCA_PCIE_ETH_LOCK_T *)pLock ;

  if (!lk)
  {
    MxL_MoCA_PCIeDbgTrace(MXL_MOCA_PCIE_L_ERR, "Error: invalid lock pointer\n");
  }
  else
  {
    spin_lock_init(&lk->lockSpinLock);
  }
}

/*----------------------------------------------------------------------------------------
 * 
 * FUNCTION NAME : MxL_MoCA_PCIeLockIrqSave
 * 
 *
 *
 * DATE CREATED  : 03/25/2016
 *
 * DESCRIPTION   : Saves current interrupt context and waits for the lock.
 *                 Pre-condition: The lock must be initialized.
 *
 * IN PARAMS     : pLock Pointer to a lock structure
 *
 * OUT PARAMS    : None
 *
 * RETURN VALUE  : None
 *
 *--------------------------------------------------------------------------------------*/
 
static void MxL_MoCA_PCIeLockIrqSave(void *pLock)
{
  MXL_MOCA_PCIE_ETH_LOCK_T *lk = (MXL_MOCA_PCIE_ETH_LOCK_T *)pLock ;
  spin_lock_irqsave(&lk->lockSpinLock, lk->lockIrqFlags);
}

/*----------------------------------------------------------------------------------------
 * 
 * FUNCTION NAME : MxL_MoCA_PCIeLock
 * 
 *
 *
 * DATE CREATED  : 03/25/2016
 *
 * DESCRIPTION   : Waits for the lock.
 *                 Pre-condition: The lock must be initialized.
 *
 * IN PARAMS     : pLock Pointer to an initialized lock structure
 *
 * OUT PARAMS    : None
 *
 * RETURN VALUE  : None
 *
 *--------------------------------------------------------------------------------------*/
 
static void MxL_MoCA_PCIeLock(MXL_MOCA_PCIE_ETH_LOCK_T *pLock)
{
  MXL_MOCA_PCIE_ETH_LOCK_T *lk = (MXL_MOCA_PCIE_ETH_LOCK_T *)pLock ;
  
  if (in_irq())
  {
    spin_lock(&lk->lockSpinLock);
  }
  else
  {
    spin_lock_irqsave(&lk->lockSpinLock, lk->lockIrqFlags);
  }
}

/*----------------------------------------------------------------------------------------
 * 
 * FUNCTION NAME : MxL_MoCA_PCIeUnLockIrqRestore
 * 
 *
 *
 * DATE CREATED  : 03/25/2016
 *
 * DESCRIPTION   : Unlocks a lock and restores interrupt context
 *                 Pre-condition: The lock must be initialized.
 *
 * IN PARAMS     : pLock Pointer to an initialized lock structure
 *
 * OUT PARAMS    : None
 *
 * RETURN VALUE  : None
 *
 *--------------------------------------------------------------------------------------*/
 
static void MxL_MoCA_PCIeUnLockIrqRestore(void *pLock)
{
  MXL_MOCA_PCIE_ETH_LOCK_T *lk = (MXL_MOCA_PCIE_ETH_LOCK_T *)pLock ;
  spin_unlock_irqrestore(&lk->lockSpinLock, lk->lockIrqFlags);
}

/*----------------------------------------------------------------------------------------
 * 
 * FUNCTION NAME : MxL_MoCA_PCIeUnLock
 * 
 *
 *
 * DATE CREATED  : 03/25/2016
 *
 * DESCRIPTION   : Unlocks a lock
 *                 Pre-condition: The lock must be initialized.
 *
 * IN PARAMS     : pLock Pointer to an initialized lock structure
 *
 * OUT PARAMS    : None
 *
 * RETURN VALUE  : None
 *
 *--------------------------------------------------------------------------------------*/
 
static void MxL_MoCA_PCIeUnLock(MXL_MOCA_PCIE_ETH_LOCK_T *pLock)
{
  MXL_MOCA_PCIE_ETH_LOCK_T *lk = (MXL_MOCA_PCIE_ETH_LOCK_T *)pLock ;

  if (in_irq())
  {
    spin_unlock(&lk->lockSpinLock);
  }
  else
  {
    spin_unlock_irqrestore(&lk->lockSpinLock, lk->lockIrqFlags);
  }
}

/*----------------------------------------------------------------------------------------
 *      Low level PCIe read/write routines 
 *--------------------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------------------
 * 
 * FUNCTION NAME : MxL_MoCA_PCIeBusRead
 * 
 *
 *
 * DATE CREATED  : 03/25/2016
 *
 * DESCRIPTION   : Low level PCIe read routine
 *
 * IN PARAMS     : [addr]  : address of SoC memory to read.
 *
 * OUT PARAMS    : [*data] : pointer to read data.
 *
 * RETURN VALUE  : None
 *
 *--------------------------------------------------------------------------------------*/
 
static void MxL_MoCA_PCIeBusRead(uintptr_t addr, uint32_t *pData)
{
  uint32_t rv ;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 0)
  rv = readl( (const volatile void *)addr );
#else
  rv = readl( addr ) ;
#endif  
  *pData = rv;
}

/*----------------------------------------------------------------------------------------
 * 
 * FUNCTION NAME : MxL_MoCA_PCIeBusWrite
 * 
 *
 *
 * DATE CREATED  : 03/25/2016
 *
 * DESCRIPTION   : Low level PCIe write routine
 *
 * IN PARAMS     : [addr] : address of SoC memory.
 *               : [data] : data to write.
 *
 * OUT PARAMS    : None.
 *
 * RETURN VALUE  : None
 *
 *--------------------------------------------------------------------------------------*/
 
static void MxL_MoCA_PCIeBusWrite(uintptr_t addr, uint32_t data)
{
//  MXL_MOCA_DBG2P3("%s: address = %x, data = %x\n", __FUNCTION__, addr, data);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 0)
  writel( data, (volatile void *)addr );
#else
  writel( data, addr );
#endif  
}

/*---------------------------------------------------------------------------------------
 * 
 * FUNCTION NAME : MxL_MoCA_PCIeCheckAddr
 * 
 *
 *
 * DATE CREATED  : 03/25/2016
 *
 * DESCRIPTION   : Checks for validity of the memory address
 *
 * IN PARAMS     : [addr] : address to validate
 *
 * OUT PARAMS    : None
 *
 * RETURN VALUE  : 0 if invalid address and 1 if valid address
 *
 *--------------------------------------------------------------------------------------*/
 
static bool MxL_MoCA_PCIeCheckAddr(uint32_t addr)
{
  if (addr < MXL_MOCA_PCIE_HIF_TC_DMEM_BASE)
  {
    MXL_MOCA_DBG2P2("%s: Invalid address %#010x (low)!!!\n", __FUNCTION__, addr);
    return false;
  }
  
  if (addr > MXL_MOCA_PCIE_CCPU_CORE_BASE_MAX)
  {
    MXL_MOCA_DBG2P2("%s: Invalid address %#010x (high)!!!\n", __FUNCTION__, addr);
    return false;
  }
  return true;
}

/*----------------------------------------------------------------------------------------
 * 
 * FUNCTION NAME : MxL_MoCA_PCIeApplyInboundBaseAddr
 * 
 *
 *
 * DATE CREATED  : 03/25/2016
 *
 * DESCRIPTION   : Translates the addresses for datapath read/write routines. Host to SoC 
 *                 access
 *
 * IN PARAMS     : [*pDdc]  : driver data context
 *               : [addr]   : data to apply the right mask.
 *
 * OUT PARAMS    : None.
 *
 * RETURN VALUE  : translated address
 *
 *--------------------------------------------------------------------------------------*/  

static void* MxL_MoCA_PCIeApplyInboundBaseAddr(MXL_MOCA_PCIE_DEV_DATA_CONTEXT_T* pDdc, uint32_t addr) 
{
  uintptr_t base = (uintptr_t)pDdc->baseIoRemap1; // BAR1
  void *pDst = 0;

  uint32_t addrBase = (addr & MXL_MOCA_PCIE_CCPU_CORE_BASE); 
  uint32_t addrMask = (addr & MXL_MOCA_PCIE_BAR1_MASK); 

  if (addrBase != MXL_MOCA_PCIE_CCPU_CORE_BASE) // Did not match BAR1
  {
   // MXL_MOCA_DBG2P0( "did not match bar1\n");

    base = (uintptr_t)pDdc->baseIoRemap0; // BAR0
    
//    MXL_MOCA_DBG2P1("base = 0x%08x\n", base);

    addrBase = (addr & MXL_MOCA_PCIE_HIF_TC_DMEM_BASE); 
    addrMask = (addr & MXL_MOCA_PCIE_BAR0_MASK);  

    if (addrBase != MXL_MOCA_PCIE_HIF_TC_DMEM_BASE)
    {
      MXL_MOCA_DBG2P4("%s: address invalid: addr=0x%08x (CCPU BASE: 0x%08x, HIFTC BASE: 0x%08x)\n", 
                      __FUNCTION__, addr, MXL_MOCA_PCIE_CCPU_CORE_BASE, MXL_MOCA_PCIE_HIF_TC_DMEM_BASE);
      return 0; 
    }
    else
    {
  //    MXL_MOCA_DBG2P1("%s: Matches BAR0\n", __FUNCTION__);
    }
  }
  else
  {
  //  MXL_MOCA_DBG2P1("%s: Matches BAR1\n", __FUNCTION__);
  }
  if (base == 0)
  {
    MXL_MOCA_DBG2P1("%s: base is 0 (abort)\n", __FUNCTION__);
    return 0;
  }
  pDst = (void*)(( addr & addrMask ) + base);

//  MXL_MOCA_DBG2P5("addr = 0x%x, base = 0x%x, addrBase = 0x%x, addrMask = 0x%x, pDst = 0x%x\n", 
//                  addr, base, addrBase, addrMask, pDst); 

  return(pDst);
} 

/*----------------------------------------------------------------------------------------
 * 
 * FUNCTION NAME : MxL_MoCA_PCIeRead
 * 
 *
 *
 * DATE CREATED  : 03/25/2016
 *
 * DESCRIPTION   : Reads from memory in the SoC hardware 
 *
 * IN PARAMS     : [devIndex]  : device index
 *               : [addr]      : The memory address to be read.
 *
 * OUT PARAMS    : [*va]]      : Pointer to return the data.
 *
 * RETURN VALUE  : MXL_MOCA_STATUS_E
 *
 *--------------------------------------------------------------------------------------*/
  
 // interface
MXL_MOCA_STATUS_E MxL_MoCA_PCIeRead(int8_t devIndex, uintptr_t addr, uint32_t *val) //<<<
{ 
  bool goodAddr;
  uint32_t *src;
  MXL_MOCA_PCIE_DEV_DATA_CONTEXT_T *pDdc = (MXL_MOCA_PCIE_DEV_DATA_CONTEXT_T*)gpDevData[devIndex];
  MXL_MOCA_STATUS_E ret = MXL_MOCA_OK;

  goodAddr = MxL_MoCA_PCIeCheckAddr(addr);

  if (goodAddr == true) 
  {
    src = MxL_MoCA_PCIeApplyInboundBaseAddr(pDdc, addr);
    if (src != 0) 
    {
      MxL_MoCA_PCIeLock(&pDdc->rwLock);
      MxL_MoCA_PCIeBusRead((uintptr_t)src, val);
      MxL_MoCA_PCIeUnLock(&pDdc->rwLock);
    }
    else
    {
      ret = MXL_MOCA_ERR;
    }
// MxL_MoCA_PCIeDbgTrace (MXL_MOCA_PCIE_L_INFO, "%s: addr = 0x%08x, val = 0x%08x\n", __FUNCTION__, addr, *val);
  }
  else
  {
    ret = MXL_MOCA_ERR;
  }
  return ret;
}

/*----------------------------------------------------------------------------------------
 * 
 * FUNCTION NAME : MxL_MoCA_PCIeWrite
 * 
 *
 *
 * DATE CREATED  : 03/25/2016
 *
 * DESCRIPTION   : Reads from memory in the SoC hardware 
 *
 * IN PARAMS     : [devIndex]  : device index
 *               : [addr]      : The memory address to write
 *               : [va]]       : data to write.  
 *
 * OUT PARAMS    : None
 *
 * RETURN VALUE  : MXL_MOCA_STATUS_E
 *
 *--------------------------------------------------------------------------------------*/
  
 // interface
MXL_MOCA_STATUS_E MxL_MoCA_PCIeWrite(int8_t devIndex, uintptr_t addr, uint32_t val) //<<<
{ 
  uint32_t goodAddr;
  uint32_t *pDst;
  MXL_MOCA_STATUS_E ret = MXL_MOCA_OK;
  MXL_MOCA_PCIE_DEV_DATA_CONTEXT_T *pDdc = (MXL_MOCA_PCIE_DEV_DATA_CONTEXT_T *)gpDevData[devIndex];

  goodAddr = MxL_MoCA_PCIeCheckAddr(addr);
//  MXL_MOCA_DBG2P2 ("%s: goodAddr = 0x%x\n", __FUNCTION__, goodAddr);
  
  if (goodAddr) 
  {
    pDst = MxL_MoCA_PCIeApplyInboundBaseAddr(pDdc, addr);
    if (pDst != 0) 
    {
      MxL_MoCA_PCIeLock(&pDdc->rwLock);
      MxL_MoCA_PCIeBusWrite((uintptr_t)pDst, val);
      MxL_MoCA_PCIeUnLock(&pDdc->rwLock);
    }
    else
    {
      ret = MXL_MOCA_ERR;
    }
 //   MXL_MOCA_DBG2P3 ("%s: addr = 0x%08x, val = 0x%08x\n", __FUNCTION__, addr, val);
  }
  else
  {
    ret = MXL_MOCA_ERR;
  }

  return ret;
}

/*----------------------------------------------------------------------------------------
 * 
 * FUNCTION NAME : MxL_MoCA_PCIeConfigOutboundATU
 * 
 *
 *
 * DATE CREATED  : 03/25/2016
 *
 * DESCRIPTION   : Program the upstream (outbound : to RC) iATU 
 *
 * IN PARAMS     : [devIndex] : device index  
 *
 * OUT PARAMS    : None
 *
 * RETURN VALUE  : MXL_MOCA_STATUS_E code
 *
 *--------------------------------------------------------------------------------------*/
  
static MXL_MOCA_STATUS_E MxL_MoCA_PCIeConfigOutboundATU(unsigned int devIndex)
{
  MXL_MOCA_PCIE_DEV_DATA_CONTEXT_T *pDdc = (MXL_MOCA_PCIE_DEV_DATA_CONTEXT_T *)gpDevData[devIndex];
  MXL_MOCA_STATUS_E ret = MXL_MOCA_OK;

  // Program the upstream (outbound : to RC) iATU
  // There will be MXL_MOCA_PCIE_NUM_OUTBOUND_ATU_REGIONS
  
  // REGION0 --
  // 08204900 : iATU Index Register. REGION_DIR 0: outbound; 1 inbound
  do
  {
    ret = MxL_MoCA_PCIeWrite(devIndex, (uintptr_t)(MXL_MOCA_PCIE_EP_IATU_INDEX_REG), 0x00000000); // 0x900: set outbound region 0 as the current region
    if (MXL_MOCA_ERR == ret)
    {
      break;
    }  

    // 0820490c : iATU Region Lower Base Address Register
    ret = MxL_MoCA_PCIeWrite(devIndex, (uintptr_t)(MXL_MOCA_PCIE_EP_IATU_REGION_LOWER_BASE_ADDR_REG), MXL_MOCA_PCIE_OUTB_ATU_REGION0_BASE);
    if (MXL_MOCA_ERR == ret)
    {
      break;
    }  
    
    // 08204914 : iATU Region Limit Address Register (inbound)
    ret = MxL_MoCA_PCIeWrite(devIndex, (uintptr_t)(MXL_MOCA_PCIE_EP_IATU_REGION_LIMIT_ADDR_REG), MXL_MOCA_PCIE_REGION0_MASK);
    if (MXL_MOCA_ERR == ret)
    {
      break;
    }  

    // 08204918 : iATU Region Lower Target Address Register. Correct target address register need to be specified by host 
    ret = MxL_MoCA_PCIeWrite(devIndex, (uintptr_t)(MXL_MOCA_PCIE_EP_IATU_REGION_LOWER_TARGET_ADDR_REG), pDdc->target0OutATUAddrMask);
    if (MXL_MOCA_ERR == ret)
    {
      break;
    }  

    // 08204908 : iATU Region Control 2 Register - enable for addr match
    ret = MxL_MoCA_PCIeWrite(devIndex, (uintptr_t)(MXL_MOCA_PCIE_EP_IATU_REGION_CTRL_2_REG), 0x80000000);// REGION_EN = 1
    if (MXL_MOCA_ERR == ret)
    {
      break;
    }  

    // REGION1 --
    // 08204900 : iATU Index Register
    ret = MxL_MoCA_PCIeWrite(devIndex, (uintptr_t)(MXL_MOCA_PCIE_EP_IATU_INDEX_REG), 0x00000001); // set outbound region 1 as the current region
    if (MXL_MOCA_ERR == ret)
    {
      break;
    }  

    // 0820490c : iATU Region Lower Base Address Register
    ret = MxL_MoCA_PCIeWrite(devIndex, (uintptr_t)(MXL_MOCA_PCIE_EP_IATU_REGION_LOWER_BASE_ADDR_REG), MXL_MOCA_PCIE_OUTB_ATU_REGION1_BASE);
    if (MXL_MOCA_ERR == ret)
    {
      break;
    }  

    // 08204914 : iATU Region Limit Address Register 
    ret = MxL_MoCA_PCIeWrite(devIndex, (uintptr_t)(MXL_MOCA_PCIE_EP_IATU_REGION_LIMIT_ADDR_REG), MXL_MOCA_PCIE_REGION1_MASK);
    if (MXL_MOCA_ERR == ret)
    {
      break;
    }  

    // 08204918 : iATU Region Lower Target Address Register. Correct target address register need to be specified by host 
    ret = MxL_MoCA_PCIeWrite(devIndex, (uintptr_t)(MXL_MOCA_PCIE_EP_IATU_REGION_LOWER_TARGET_ADDR_REG), pDdc->target1OutATUAddrMask);
    if (MXL_MOCA_ERR == ret)
    {
      break;
    }  

    // 08204908 : iATU Region Control 2 Register - enable for addr match
    ret = MxL_MoCA_PCIeWrite(devIndex, (uintptr_t)(MXL_MOCA_PCIE_EP_IATU_REGION_CTRL_2_REG), 0x80000000);  // REGION_EN = 1 
    if (MXL_MOCA_ERR == ret)
    {
      break;
    }      
  } while (0);

  if (MXL_MOCA_ERR == ret)
  {
    MxL_MoCA_PCIeDbgTrace(MXL_MOCA_PCIE_L_ERR, "PCIe Config Outbound ATU: Read/Write error!\n");
  }
  else
  {
    MxL_MoCA_PCIeDbgTrace(MXL_MOCA_PCIE_L_INFO, "Configuring outbound (SoC to Host mem access) iATUs...OK\n");
  }
  
  return ret;
}

/*----------------------------------------------------------------------------------------
 * 
 * FUNCTION NAME : MxL_MoCA_PCIeConfigInboundATU
 * 
 *
 *
 * DATE CREATED  : 03/25/2016
 *
 * DESCRIPTION   : Program the downstream (inbound : from RC) iATU 
 *
 * IN PARAMS     : [devIndex] : device index  
 *
 * OUT PARAMS    : None
 *
 * RETURN VALUE  : MXL_MOCA_STATUS_E code
 *
 *--------------------------------------------------------------------------------------*/
  
static MXL_MOCA_STATUS_E MxL_MoCA_PCIeConfigInboundATU(unsigned int devIndex)
{
  // Program the downstream (inbound : from RC) iATU
  // For downstream iATU: port logic registers start at base address 0x0820_4700 (PCI EP Core datasheet has offsets). 
  // MXL_MOCA_PCIE_EP_IATU_BASE    0x08204000 + (0x700 + 0x200)
  // This has to be CfgWr since we cannot do Memory writes until the inbound ATU is setup.
  // BAR0 --
  uint32_t  val = 0;
  uint32_t  ret = 0;
  MXL_MOCA_STATUS_E err = MXL_MOCA_OK;

  MXL_MOCA_PCIE_DEV_DATA_CONTEXT_T *pDevData = (MXL_MOCA_PCIE_DEV_DATA_CONTEXT_T *)gpDevData[devIndex];

  do
  {
    // 08204900 : iATU Index Register. REGION_DIR 0: outbound; 1 inbound
    if ((ret = pci_write_config_dword(pDevData->pPCIeDev, MXL_MOCA_PCIE_EP_IATU_INDEX_OFFSET, 0x80000000)) != 0) // set inbound region 0 as the current region
    {
      MxL_MoCA_PCIeDbgTrace(MXL_MOCA_PCIE_L_ERR, "pci_write_config_dword failed: 0x%08x\n", MXL_MOCA_PCIE_EP_IATU_INDEX_OFFSET);
      err = MXL_MOCA_ERR;
      break;
    }
    if ((ret = pci_read_config_dword(pDevData->pPCIeDev, MXL_MOCA_PCIE_EP_IATU_INDEX_OFFSET, &val)) == 0)
    {
      MXL_MOCA_DBG1P1( "BAR0:::::PCIE_EP_IATU_INDEX_OFFSET: 0x%08x\n", val);
    }

    // 08204904 : iATU Region Control 1 Register (inbound)
    if ((ret = pci_write_config_dword(pDevData->pPCIeDev, MXL_MOCA_PCIE_EP_IATU_REGION_CTRL_1_OFFSET, 0)) != 0) // to define the type of the region to be MEM.
    {
      MxL_MoCA_PCIeDbgTrace(MXL_MOCA_PCIE_L_ERR, "pci_write_config_dword failed: 0x%08x\n", MXL_MOCA_PCIE_EP_IATU_REGION_CTRL_1_OFFSET);
      err = MXL_MOCA_ERR;
      break;      
    } 
    if ((ret = pci_read_config_dword(pDevData->pPCIeDev, MXL_MOCA_PCIE_EP_IATU_REGION_CTRL_1_OFFSET, &val)) == 0)
    {
      MXL_MOCA_DBG1P1( "PCIE_EP_IATU_REGION_CTRL_1_OFFSET: 0x%08x\n", val);
    }

    // 0820490c : iATU Region Lower Base Address Register
    if ((ret = pci_write_config_dword(pDevData->pPCIeDev, MXL_MOCA_PCIE_EP_IATU_REGION_LOWER_BASE_ADDR_OFFSET, pDevData->baseIoAddr0)) != 0) 
    {
      MxL_MoCA_PCIeDbgTrace(MXL_MOCA_PCIE_L_ERR, "pci_write_config_dword failed: 0x%08x\n", MXL_MOCA_PCIE_EP_IATU_REGION_LOWER_BASE_ADDR_OFFSET);
      err = MXL_MOCA_ERR;
      break;      
    } 
    if ((ret = pci_read_config_dword(pDevData->pPCIeDev, MXL_MOCA_PCIE_EP_IATU_REGION_LOWER_BASE_ADDR_OFFSET, &val)) == 0)
    {
      MXL_MOCA_DBG1P1( "PCIE_EP_IATU_REGION_LOWER_BASE_ADDR_OFFSET: 0x%08x\n", val);
    }
    
    // 08204914 : iATU Region Limit Address Register (inbound)
    if ((ret = pci_write_config_dword(pDevData->pPCIeDev, MXL_MOCA_PCIE_EP_IATU_REGION_LIMIT_ADDR_OFFSET, (pDevData->baseIoAddr0 | MXL_MOCA_PCIE_BAR0_MASK))) != 0) 
    {
      MxL_MoCA_PCIeDbgTrace(MXL_MOCA_PCIE_L_ERR, "pci_write_config_dword failed: 0x%08x\n", MXL_MOCA_PCIE_EP_IATU_REGION_LIMIT_ADDR_OFFSET);
      err = MXL_MOCA_ERR;
      break;      
    }
    if ((ret = pci_read_config_dword(pDevData->pPCIeDev, MXL_MOCA_PCIE_EP_IATU_REGION_LIMIT_ADDR_OFFSET, &val)) == 0)
    {
      MXL_MOCA_DBG1P1( "PCIE_EP_IATU_REGION_LIMIT_ADDR_OFFSET: 0x%08x\n", val);
    }
    
    // 08204918 : iATU Region Lower Target Address Register
    if ((ret = pci_write_config_dword(pDevData->pPCIeDev, MXL_MOCA_PCIE_EP_IATU_REGION_LOWER_TARGET_ADDR_OFFSET, MXL_MOCA_PCIE_HIF_TC_DMEM_BASE)) != 0) 
    {
      MxL_MoCA_PCIeDbgTrace(MXL_MOCA_PCIE_L_ERR, "pci_write_config_dword failed: 0x%08x\n", MXL_MOCA_PCIE_EP_IATU_REGION_LOWER_TARGET_ADDR_OFFSET);
      err = MXL_MOCA_ERR;
      break;      
    }
    if ((ret = pci_read_config_dword(pDevData->pPCIeDev, MXL_MOCA_PCIE_EP_IATU_REGION_LOWER_TARGET_ADDR_OFFSET, &val)) == 0)
    {
      MXL_MOCA_DBG1P1( "PCIE_EP_IATU_REGION_LOWER_TARGET_ADDR_OFFSET: 0x%08x\n", val);
    }
    
    // 08204908 : iATU Region Control 2 Register - enable for BAR match
    if ((ret = pci_write_config_dword(pDevData->pPCIeDev, MXL_MOCA_PCIE_EP_IATU_REGION_CTRL_2_OFFSET, 0xC0000000)) != 0) // REGION_EN = 1, BAR0
    {
      MxL_MoCA_PCIeDbgTrace(MXL_MOCA_PCIE_L_ERR, "pci_write_config_dword failed: 0x%08x\n", MXL_MOCA_PCIE_EP_IATU_REGION_CTRL_2_OFFSET);
      err = MXL_MOCA_ERR;
      break;      
    }
    if ((ret = pci_read_config_dword(pDevData->pPCIeDev, MXL_MOCA_PCIE_EP_IATU_REGION_CTRL_2_OFFSET, &val)) == 0)
    {
      MXL_MOCA_DBG1P1( "PCIE_EP_IATU_REGION_CTRL_2_OFFSET: 0x%08x\n", val);
    }

    // BAR1 --
    // 08204900 : iATU Index Register
    if ((ret = pci_write_config_dword(pDevData->pPCIeDev, MXL_MOCA_PCIE_EP_IATU_INDEX_OFFSET, 0x80000001)) != 0) // set inbound region 1 as the current region. 1 for second region
    {
      MxL_MoCA_PCIeDbgTrace(MXL_MOCA_PCIE_L_ERR, "pci_write_config_dword failed: 0x%08x\n", MXL_MOCA_PCIE_EP_IATU_INDEX_OFFSET);
      err = MXL_MOCA_ERR;  
      break;      
    } 
    if ((ret = pci_read_config_dword(pDevData->pPCIeDev, MXL_MOCA_PCIE_EP_IATU_INDEX_OFFSET, &val)) == 0)
    {
      MXL_MOCA_DBG1P1( "BAR1:::::PCIE_EP_IATU_INDEX_OFFSET: 0x%08x\n", val);
    }

    // 08204904 : iATU Region Control 1 Register  
    if ((ret = pci_write_config_dword(pDevData->pPCIeDev, MXL_MOCA_PCIE_EP_IATU_REGION_CTRL_1_OFFSET, 0)) != 0) // to define the type of the region to be MEM.
    {
      MxL_MoCA_PCIeDbgTrace(MXL_MOCA_PCIE_L_ERR, "pci_write_config_dword failed: 0x%08x\n", MXL_MOCA_PCIE_EP_IATU_REGION_CTRL_1_OFFSET);
      err = MXL_MOCA_ERR;    
      break;      
    } 
    if ((ret = pci_read_config_dword(pDevData->pPCIeDev, MXL_MOCA_PCIE_EP_IATU_REGION_CTRL_1_OFFSET, &val)) == 0)
    {
      MXL_MOCA_DBG1P1( "PCIE_EP_IATU_REGION_CTRL_1_OFFSET: 0x%08x\n", val);
    }
    
    // 0820490c : iATU Region Lower Base Address Register
    if ((ret = pci_write_config_dword(pDevData->pPCIeDev, MXL_MOCA_PCIE_EP_IATU_REGION_LOWER_BASE_ADDR_OFFSET, pDevData->baseIoAddr1)) != 0) 
    {
      MxL_MoCA_PCIeDbgTrace(MXL_MOCA_PCIE_L_ERR, "pci_write_config_dword failed: 0x%08x\n", MXL_MOCA_PCIE_EP_IATU_REGION_LOWER_BASE_ADDR_OFFSET);
      err = MXL_MOCA_ERR; 
      break;      
    } 
    if ((ret = pci_read_config_dword(pDevData->pPCIeDev, MXL_MOCA_PCIE_EP_IATU_REGION_LOWER_BASE_ADDR_OFFSET, &val)) == 0)
    {
      MXL_MOCA_DBG1P1( "PCIE_EP_IATU_REGION_LOWER_BASE_ADDR_OFFSET: 0x%08x\n", val);
    }
    
    // 08204914 : iATU Region Limit Address Register 
    if ((ret = pci_write_config_dword(pDevData->pPCIeDev, MXL_MOCA_PCIE_EP_IATU_REGION_LIMIT_ADDR_OFFSET, (pDevData->baseIoAddr1 | MXL_MOCA_PCIE_BAR1_MASK))) != 0) 
    {
      MxL_MoCA_PCIeDbgTrace(MXL_MOCA_PCIE_L_ERR, "pci_write_config_dword failed: 0x%08x\n", MXL_MOCA_PCIE_EP_IATU_REGION_LIMIT_ADDR_OFFSET);
      err = MXL_MOCA_ERR;   
      break;      
    } 
    if ((ret = pci_read_config_dword(pDevData->pPCIeDev, MXL_MOCA_PCIE_EP_IATU_REGION_LIMIT_ADDR_OFFSET, &val)) == 0)
    {
      MXL_MOCA_DBG1P1( "PCIE_EP_IATU_REGION_LIMIT_ADDR_OFFSET: 0x%08x\n", val);
    }
    
    // 08204918 : iATU Region Lower Target Address Register 
    if ((ret = pci_write_config_dword(pDevData->pPCIeDev, MXL_MOCA_PCIE_EP_IATU_REGION_LOWER_TARGET_ADDR_OFFSET, MXL_MOCA_PCIE_CCPU_CORE_BASE)) != 0) 
    {
      MxL_MoCA_PCIeDbgTrace(MXL_MOCA_PCIE_L_ERR, "pci_write_config_dword failed: 0x%08x\n", MXL_MOCA_PCIE_EP_IATU_REGION_LOWER_TARGET_ADDR_OFFSET);
      err = MXL_MOCA_ERR;  
      break;      
    } 
    if ((ret = pci_read_config_dword(pDevData->pPCIeDev, MXL_MOCA_PCIE_EP_IATU_REGION_LOWER_TARGET_ADDR_OFFSET, &val)) == 0)
    {
      MXL_MOCA_DBG1P1( "PCIE_EP_IATU_REGION_LOWER_TARGET_ADDR_OFFSET: 0x%08x\n", val);
    }

    // 08204908 : iATU Region Control 2 Register - enable for BAR match
    if ((ret = pci_write_config_dword(pDevData->pPCIeDev, MXL_MOCA_PCIE_EP_IATU_REGION_CTRL_2_OFFSET, 0xC0000100)) != 0) // REGION_EN = 1, BAR1
    {
      MxL_MoCA_PCIeDbgTrace(MXL_MOCA_PCIE_L_ERR, "pci_write_config_dword failed: 0x%08x\n", MXL_MOCA_PCIE_EP_IATU_REGION_CTRL_2_OFFSET);
      err = MXL_MOCA_ERR;  
      break;      
    } 
    if ((ret = pci_read_config_dword(pDevData->pPCIeDev, MXL_MOCA_PCIE_EP_IATU_REGION_CTRL_2_OFFSET, &val)) == 0)
    {
      MXL_MOCA_DBG1P1( "PCIE_EP_IATU_REGION_CTRL_2_OFFSET: 0x%08x\n", val);
    }
  } while (0);

  if (ret != 0)
  {
    MxL_MoCA_PCIeDbgTrace(MXL_MOCA_PCIE_L_ERR, "%s failed!!!\n",  __FUNCTION__);
    err = MXL_MOCA_ERR;      
  }
  else
  {
    MxL_MoCA_PCIeDbgTrace(MXL_MOCA_PCIE_L_INFO, "Configuring inbound (Host to SoC mem access) iATUs...OK\n");
  }

  return err;
} 

/*----------------------------------------------------------------------------------------
 * 
 * FUNCTION NAME : MxL_MoCA_PCIeGlobalResetSoC
 * 
 *
 *
 * DATE CREATED  : 03/25/2016
 * 
 * DESCRIPTION   : Generates a reset pulse to reset the chip via config space, link will
 *                 not go down. 
 *
 * IN PARAMS     : [devIndex] : device index  
 *
 * OUT PARAMS    : None
 *
 * RETURN VALUE  : MXL_MOCA_OK or MXL_MOCA_ERR
 *
 *--------------------------------------------------------------------------------------*/
 
 // interface
MXL_MOCA_STATUS_E MxL_MoCA_PCIeGlobalResetSoC(int8_t devIndex) 
{
  MXL_MOCA_STATUS_E  status = MXL_MOCA_OK;
  MXL_MOCA_PCIE_DEV_DATA_CONTEXT_T *pDevData = (MXL_MOCA_PCIE_DEV_DATA_CONTEXT_T *)gpDevData[devIndex];

  MXL_MOCA_DBG2P1("%s...\n", __FUNCTION__);

  do
  {
    if (pDevData->deviceId == MXL_MOCA_PCIE_DEVICE_ID_CARDIFF)
    {
      if (pci_write_config_dword(pDevData->pPCIeDev, MXL_MOCA_PCIE_EP_CARDIFF_GLOBAL_RESETN,
                                  MXL_MOCA_PCIE_EP_CARDIFF_GLOBAL_RESET_NO_LINK_DN) != 0)
      {
        MxL_MoCA_PCIeDbgTrace(MXL_MOCA_PCIE_L_ERR, "%s failed: 0x%08x\n",
                              __FUNCTION__, MXL_MOCA_PCIE_EP_CARDIFF_GLOBAL_RESETN);
        status = MXL_MOCA_ERR;
        break;
      } 
      if (pci_write_config_dword(pDevData->pPCIeDev, MXL_MOCA_PCIE_EP_CARDIFF_GLOBAL_RESETN,
                                  MXL_MOCA_PCIE_EP_CARDIFF_GLOBAL_RESET_NO_LINK_DN_CLEAR) != 0)
      {
        MxL_MoCA_PCIeDbgTrace(MXL_MOCA_PCIE_L_ERR, "%s failed: 0x%08x\n",
                              __FUNCTION__, MXL_MOCA_PCIE_EP_CARDIFF_GLOBAL_RESETN);
        status = MXL_MOCA_ERR;
        break;
      }
    }
    else if (pDevData->deviceId == MXL_MOCA_PCIE_DEVICE_ID_LEUCADIA)
    {
      if (pci_write_config_dword(pDevData->pPCIeDev, MXL_MOCA_PCIE_EP_LEUCADIA_GLOBAL_RESETN,
                                  MXL_MOCA_PCIE_EP_LEUCADIA_GLOBAL_RESET_NO_LINK_DN) != 0) 
      {
        MxL_MoCA_PCIeDbgTrace(MXL_MOCA_PCIE_L_ERR, "%s failed: 0x%08x\n",
                              __FUNCTION__, MXL_MOCA_PCIE_EP_LEUCADIA_GLOBAL_RESETN);
        status = MXL_MOCA_ERR;
        break;
      } 
      if (pci_write_config_dword(pDevData->pPCIeDev, MXL_MOCA_PCIE_EP_LEUCADIA_GLOBAL_RESETN,
                                  MXL_MOCA_PCIE_EP_LEUCADIA_GLOBAL_RESET_NO_LINK_DN_CLEAR) != 0)
      {
        MxL_MoCA_PCIeDbgTrace(MXL_MOCA_PCIE_L_ERR, "%s failed: 0x%08x\n",
                              __FUNCTION__, MXL_MOCA_PCIE_EP_LEUCADIA_GLOBAL_RESETN);
        status = MXL_MOCA_ERR;
        break;
      }
    }
  }while (0);
  
  return status;
}

/*----------------------------------------------------------------------------------------
 *      utility functions
 *--------------------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------------------
 * 
 * FUNCTION NAME : MxL_MoCA_PCIeAllocDmaMem
 * 
 *
 *
 * DATE CREATED  : 03/25/2016
 * 
 * DESCRIPTION   : Generates a reset pulse to reset the chip via config space, link will
 *                 not go down. 
 *
 * IN PARAMS     : [pDdc]    : Pointer to data context  
 *               : [size]    : Requested block size
 *               : [ppMemPa] : Pointer to place to return the Physical memory address.
 *
 * OUT PARAMS    : None
 *
 * RETURN VALUE  : Virtual memory address (and Phys address), or NULL on failure
 *
 *--------------------------------------------------------------------------------------*/
 
static void* MxL_MoCA_PCIeAllocDmaMem(void *pDdc, size_t size, void **ppMemPa)
{
  struct pci_dev *pDev= ((MXL_MOCA_PCIE_DEV_DATA_CONTEXT_T *)pDdc)->pPCIeDev;
  dma_addr_t     dma_buf;
  void           *pMemVa;

  pMemVa   = pci_alloc_consistent(pDev, size, &dma_buf);
  *ppMemPa = (void *)(uintptr_t)dma_buf;

  return (pMemVa);
}

/*----------------------------------------------------------------------------------------
 * 
 * FUNCTION NAME : MxL_MoCA_PCIeFreeDmaMem
 * 
 *
 *
 * DATE CREATED  : 03/25/2016
 * 
 * DESCRIPTION   : Free DMA-addressable memory.
 *
 * IN PARAMS     : [pDdc]   : Pointer to data context  
 *               : [size]   : Requested block size
 *               : [pMemVa] : Physical memory address returned by MxL_MoCA_PCIeAllocDmaMem()
 *               : [pMemPa] : Virtual memory address returned by MxL_MoCA_PCIeAllocDmaMem() 
 *
 * OUT PARAMS    : None
 *
 * RETURN VALUE  : Virtual memory address (and Phys address), or NULL on failure
 *
 *--------------------------------------------------------------------------------------*/
 
static void MxL_MoCA_PCIeFreeDmaMem(void *pDdc, size_t size, void *pMemVa, void *pMemPa)
{
  struct pci_dev *pDev = ((MXL_MOCA_PCIE_DEV_DATA_CONTEXT_T *)pDdc)->pPCIeDev;
  
  pci_free_consistent(pDev, size, pMemVa, (dma_addr_t)(uintptr_t)pMemPa);
}

/*----------------------------------------------------------------------------------------
 * 
 * FUNCTION NAME : MxL_MoCA_PCIeKMemAlloc
 * 
 *
 *
 * DATE CREATED  : 03/25/2016
 * 
 * DESCRIPTION   : Allocates memory. Function allocates size bytes of memory. If the 
 *                 allocation succeeds, a pointer to the block of memory is returned, 
 *                 otherwise a null pointer is returned.
 *
 * IN PARAMS     : [size]   : Size to allocate  
 *
 * OUT PARAMS    : None
 *
 * RETURN VALUE  : Pointer to allocated block if successful, Null pointer otherwise
 *
 *--------------------------------------------------------------------------------------*/
 
static void* MxL_MoCA_PCIeKMemAlloc(int size)
{
  void *pMem;

  if (size < MXL_MOCA_PCIE_KMALLOC_MAX)
  {
    pMem = kmalloc(size, GFP_KERNEL /* | __GFP_DMA */);
  }
  else
  {
    pMem = (void *)__get_free_pages(GFP_KERNEL, get_order(size));
  }
  return (pMem);
}

/*----------------------------------------------------------------------------------------
 * 
 * FUNCTION NAME : MxL_MoCA_PCIeKMemFree
 *  
 *
 *
 * DATE CREATED  : 03/25/2016
 * 
 * DESCRIPTION   : Free memory.
 *
 * IN PARAMS     : [pMem]  : Allocation pointer from get free pages. 
 *               : [size]  : Requested block size.
 *
 * OUT PARAMS    : None
 *
 * RETURN VALUE  : None
 *
 *--------------------------------------------------------------------------------------*/
 
static void MxL_MoCA_PCIeKMemFree(void *pMem, int size)
{
  if (size < MXL_MOCA_PCIE_KMALLOC_MAX)
      kfree(pMem);
  else
      free_pages((unsigned long)pMem, get_order(size));
}

/*----------------------------------------------------------------------------------------
 * 
 * FUNCTION NAME : MxL_MoCA_PCIeFreeResources
 *  
 *
 *
 * DATE CREATED  : 03/25/2016
 * 
 * DESCRIPTION   : Free all allocated resources.
 *
 * IN PARAMS     : [pDdc] : Pointer to data context  
 *
 * OUT PARAMS    : None
 *
 * RETURN VALUE  : MXL_MOCA_OK or MXL_MOCA_ERR
 *
 *--------------------------------------------------------------------------------------*/
 
static MXL_MOCA_STATUS_E MxL_MoCA_PCIeFreeResources(MXL_MOCA_PCIE_DEV_DATA_CONTEXT_T *pDdc)
{
  uint32_t sz;
  MXL_MOCA_PCIE_DATAPATH_CONTEXT_T *pDataPath;

  pDataPath = (MXL_MOCA_PCIE_DATAPATH_CONTEXT_T *)pDdc->pDpCtx;
  
  if (!pDataPath)
  {
      MXL_MOCA_DBG1P1("%s NULL pointer\n",  __FUNCTION__);
      return MXL_MOCA_ERR;
  }
///    MXL_MOCA_DBG2P1("%s ...\n",  __FUNCTION__);
  if (pDataPath->txQueuePair.pDescQueue)
  {
    sz = MXL_MOCA_PCIE_TX_PTR_QUEUE_SIZE * sizeof(MXL_MOCA_PCIE_QUEUE_DESCR_T),
    MxL_MoCA_PCIeFreeDmaMem(pDdc, sz,
                        pDataPath->txQueuePair.pDescQueue,
                        pDataPath->txQueuePair.pDescQueuePa);
  }

  if (pDataPath->rxQueuePair.pDescQueue)
  {
    sz = MXL_MOCA_PCIE_RX_PTR_QUEUE_SIZE * sizeof(MXL_MOCA_PCIE_QUEUE_DESCR_T),
    MxL_MoCA_PCIeFreeDmaMem(pDdc, sz,
                        pDataPath->rxQueuePair.pDescQueue,
                        pDataPath->rxQueuePair.pDescQueuePa);
  }

  if (pDataPath->txQueuePair.pPktQueue)
  {
    sz = MXL_MOCA_PCIE_TX_PTR_QUEUE_SIZE * sizeof(MXL_MOCA_PCIE_QUEUE_PKT_T),
    MxL_MoCA_PCIeFreeDmaMem(pDdc, sz,
                        pDataPath->txQueuePair.pPktQueue,
                        pDataPath->txQueuePair.pPktQueuePa);
  }

  if (pDataPath->rxQueuePair.pPktQueue)
  {
    sz = MXL_MOCA_PCIE_RX_PTR_QUEUE_SIZE * sizeof(MXL_MOCA_PCIE_QUEUE_PKT_T),
    MxL_MoCA_PCIeFreeDmaMem(pDdc, sz,
                        pDataPath->rxQueuePair.pPktQueue,
                        pDataPath->rxQueuePair.pPktQueuePa);
  }

  MxL_MoCA_PCIeKMemFree(pDataPath, sizeof(MXL_MOCA_PCIE_DATAPATH_CONTEXT_T)); 
  pDdc->pDpCtx = NULL;

  return MXL_MOCA_OK;
}

/*----------------------------------------------------------------------------------------
 * 
 * FUNCTION NAME : MxL_MoCA_PCIeOpen
 *  
 *
 *
 * DATE CREATED  : 03/25/2016
 * 
 * DESCRIPTION   : Brings up the Ethernet interface.
 *
 * IN PARAMS     : [kdev]  : context structure with device pointer  
 *
 * OUT PARAMS    : None
 *
 * RETURN VALUE  : None
 *
 *--------------------------------------------------------------------------------------*/
 
void MxL_MoCA_PCIeOpen(void *kdev)
{
  struct net_device *pNetDev = (struct net_device *)kdev;

  if ((pNetDev->flags & IFF_UP) == IFF_UP)
  {
    MxL_MoCA_PCIeNetDevOpen(pNetDev);
  }
}

/*----------------------------------------------------------------------------------------
 * 
 * FUNCTION NAME : MxL_MoCA_PCIeClose
 *  
 *
 *
 * DATE CREATED  : 03/25/2016
 * 
 * DESCRIPTION   : Shuts down the Ethernet interface.
 *
 * IN PARAMS     : [*kdev]   : context structure with device pointer  
 *
 * OUT PARAMS    : None
 *
 * RETURN VALUE  : None
 *
 *--------------------------------------------------------------------------------------*/
 
void MxL_MoCA_PCIeClose(void *kdev)
{
  struct net_device *pNetDev = (struct net_device *)kdev;

  if ((pNetDev->flags & IFF_UP) == IFF_UP)
  {
    MxL_MoCA_PCIeNetDevClose(pNetDev);
  }
}

/*----------------------------------------------------------------------------------------
 * 
 * FUNCTION NAME : MxL_MoCA_PCIeDataPathInit
 *  
 *
 *
 * DATE CREATED  : 03/25/2016
 * 
 * DESCRIPTION   : Initializes the DP
 *
 * IN PARAMS     : [*pDdc] : driver data context 
 *
 * OUT PARAMS    : None
 *
 * RETURN VALUE  : MXL_MOCA_OK for success, or MXL_MOCA_ERR on failure
 *
 *--------------------------------------------------------------------------------------*/
 
static MXL_MOCA_STATUS_E  MxL_MoCA_PCIeDataPathInit(MXL_MOCA_PCIE_DEV_DATA_CONTEXT_T *pDdc, int8_t devIndex) 
{
  MXL_MOCA_STATUS_E ret = MXL_MOCA_OK;

  MXL_MOCA_DBG2P1("%s: -- Data path init --\n", __FUNCTION__);

  // Allocate and initialize host tx/rx lists and descriptors
  ret = MxL_MoCA_PCIeAllocResources(pDdc); // memory alloc/init

  if (MXL_MOCA_OK != ret)
  {
    MxL_MoCA_PCIeDbgTrace(MXL_MOCA_PCIE_L_ERR, "%s: Alloc resources failed!\n", __FUNCTION__);
  }
  else
  {
    // init the rxList and rxFreeList before MxL_MoCA_PCIeNetDevOpen is called
    ret = MxL_MoCA_PCIeDataPathReset(pDdc, devIndex);
    if (MXL_MOCA_OK != ret)
    {
      MxL_MoCA_PCIeDbgTrace(MXL_MOCA_PCIE_L_ERR, "%s: Data Path Reset failed!\n", __FUNCTION__);
    }
    MXL_MOCA_DBG2P2("NetDev, 0x%x, DevCtx: 0x%x\n", pDdc->pNetDev, pDdc);
    MXL_MOCA_DBG2P1("line(%d): Datapath init done! \n", __LINE__);
  }
 
  return ret;
}


/*----------------------------------------------------------------------------------------
 * 
 * FUNCTION NAME : MxL_MoCA_PCIeTerminate
 *  
 *
 *
 * DATE CREATED  : 03/25/2016
 * 
 * DESCRIPTION   : Terminates the DP
 *
 * IN PARAMS     : [pddp] : Pointer to driver data plane context. 
 *
 * OUT PARAMS    : None
 *
 * RETURN VALUE  : None
 *
 *--------------------------------------------------------------------------------------*/
 
void MxL_MoCA_PCIeTerminate(MXL_MOCA_PCIE_DEV_DATA_CONTEXT_T *pddp)
{
  if (pddp == NULL)
  {
    MXL_MOCA_DBG1P0( "pddp is NULL\n");
    return;
  }
  // Initialize hardware registers to reset device
  // perform graceful shutdown if possible
  MxL_MoCA_PCIeDbgTrace(MXL_MOCA_PCIE_L_INFO, "MoCA PCIe device stopping\n");
  MxL_MoCA_PCIeClose(pddp->pNetDev);
  MXL_MOCA_DBG2P0( "MxL_MoCA_PCIeClose done\n");
  
  if (MxL_MoCA_PCIeFreeResources(pddp) == MXL_MOCA_ERR)
  {
    MxL_MoCA_PCIeDbgTrace(MXL_MOCA_PCIE_L_ERR, "MxL_MoCA_PCIeTerminate failed\n");
  }
}

/*----------------------------------------------------------------------------------------
 * 
 * FUNCTION NAME : MxL_MoCA_PCIeDataPathClear
 *  
 *
 *
 * DATE CREATED  : 03/25/2016
 * 
 * DESCRIPTION   : Resets the DP (clears counters, if any; and statistics)
 *
 * IN PARAMS     : [pddp] : Pointer to driver data plane context. 
 *
 * OUT PARAMS    : None
 *
 * RETURN VALUE  : None
 *
 *--------------------------------------------------------------------------------------*/
 
void MxL_MoCA_PCIeDataPathClear(MXL_MOCA_PCIE_DEV_DATA_CONTEXT_T *pDdc)
{
  memset(&pDdc->ethStats, 0, sizeof(MXL_MOCA_PCIE_ETH_STATS_T));
}

/*----------------------------------------------------------------------------------------
 * 
 * FUNCTION NAME : MxL_MoCA_PCIeDataPathControl
 *  
 *
 *
 * DATE CREATED  : 03/25/2016
 * 
 * DESCRIPTION   : Controls the operation of the data path (place holder)
 *
 * IN PARAMS     : [pddp] : Pointer to driver data plane context. 
 *               : [cmd]  : MxL MoCA IO command.   
 *               : [val]  : IO block pointer. 
 *
 * OUT PARAMS    : None
 *
 * RETURN VALUE  : MXL_MOCA_OK or MXL_MOCA_ERR
 *
 *--------------------------------------------------------------------------------------*/
 
MXL_MOCA_STATUS_E MxL_MoCA_PCIeDataPathControl(MXL_MOCA_PCIE_DEV_DATA_CONTEXT_T *pDdc, unsigned int cmd, uintptr_t val) 
{
  return (MXL_MOCA_OK);
}

#if MXL_MOCA_PCIE_PWR_MGMT
/*----------------------------------------------------------------------------------------
 * 
 * FUNCTION NAME : MxL_MoCA_PCIePowerManagementInit
 *  
 *
 *
 * DATE CREATED  : 03/25/2016
 * 
 * DESCRIPTION   : Function to init power management capabilities and control
 *
 * IN PARAMS     : [devIndex] : device index 
 *
 * OUT PARAMS    : None
 *
 * RETURN VALUE  : false or true
 *
 *--------------------------------------------------------------------------------------*/

MXL_MOCA_STATUS_E MxL_MoCA_PCIePowerManagementInit(uint8_t devIndex)
{
  MXL_MOCA_STATUS_E ret = MXL_MOCA_OK;
  MXL_MOCA_PCIE_DEV_DATA_CONTEXT_T *pDdc = (MXL_MOCA_PCIE_DEV_DATA_CONTEXT_T*)gpDevData[devIndex];
 
  struct pci_dev  *pDev = pDdc->pPCIeDev;
  uint32_t        pos, pos1;
  uint16_t        capability, control;
  uint32_t        status, lnkCap, lnkCtl, lnkStat, pmCap, pmCtrl;
  uint16_t        lnkCtrlWr;
  bool            bStatus;

  // Power management capabilities
  if ((pos = pci_find_capability(pDev, PCI_CAP_ID_EXP)) != 0) 
  {
    pci_read_config_word(pDev, pos + PCI_EXP_DEVCAP, &capability);
    pci_read_config_word(pDev, pos + PCI_EXP_DEVCTL, &control);
    
    pci_read_config_dword(pDev, pos + PCI_EXP_DEVSTA, &status);
    pci_read_config_dword(pDev, pos + PCI_EXP_LNKCAP, &lnkCap);
    pci_read_config_dword(pDev, pos + PCI_EXP_LNKCTL, &lnkCtl);
    pci_read_config_dword(pDev, pos + PCI_EXP_LNKSTA, &lnkStat);

    if ((pos1 = pci_find_capability(pDev, PCI_CAP_ID_PM)) != 0) 
    {
      pci_read_config_dword(pDev, pos1 + PCI_PM_PMC, &pmCap);
      pci_read_config_dword(pDev, pos1 + PCI_PM_CTRL, &pmCtrl);
    }  
    /* 
      PCIE_LINK_STATE_L0S     1
      PCIE_LINK_STATE_L1      2
    */
    // Link Cap: ASPM (Active State Power Management) states 
    pDdc->lnkCapAspmSupport  = (lnkCap & PCI_EXP_LNKCAP_ASPMS) >> 10;
    MXL_MOCA_DBG1P2("LNKCAP:ASPM Support: L0s(%d); L1(%d)\n", \
                    (pDdc->lnkCapAspmSupport & PCIE_LINK_STATE_L0S) == PCIE_LINK_STATE_L0S,\
                    (pDdc->lnkCapAspmSupport & PCIE_LINK_STATE_L1)  == PCIE_LINK_STATE_L1);

    // Dev Cap: PME (Power Management Events) supported 
    pDdc->lnkCapPme = 0;
    
    bStatus = pci_pme_capable(pDev, PCI_D0);
    pDdc->lnkCapPme |= (bStatus == true) ? 1:0;

    bStatus = pci_pme_capable(pDev, PCI_D1);    
    pDdc->lnkCapPme |= (bStatus == true) ? 1<<1:0;

    bStatus = pci_pme_capable(pDev, PCI_D2);    
    pDdc->lnkCapPme |= (bStatus == true) ? 1<<2:0;

    bStatus = pci_pme_capable(pDev, PCI_D3hot);    
    pDdc->lnkCapPme |= (bStatus == true) ? 1<<3:0;

    bStatus = pci_pme_capable(pDev, PCI_D3cold);
    pDdc->lnkCapPme |= (bStatus == true) ? 1<<4:0;
    
    MXL_MOCA_DBG1P1("PME Capability(D3c,D3h,D2,D1,D0 = 0x%x)\n", pDdc->lnkCapPme);

    /* Link Ctrl: ASPM (enable if needed)
       PCI_EXP_LNKCTL_ASPMC   0x0003    // ASPM Control 
       - PCI_EXP_LNKCTL_ASPM_L0S 0x0001 // L0s Enable 
       - PCI_EXP_LNKCTL_ASPM_L1  0x0002 // L1 Enable 
    */
    pDdc->lnkCtlAspmL1 = ((lnkCtl & PCI_EXP_LNKCTL_ASPM_L1) == PCI_EXP_LNKCTL_ASPM_L1);
    if (PCI_EXP_LNKCTL_ASPM_L1 == (pDdc->lnkCapAspmSupport & PCI_EXP_LNKCTL_ASPM_L1)) /* ASPM Capable (L1) */
    {
      if (0 == pDdc->lnkCtlAspmL1)
      {
        MXL_MOCA_DBG1P0("ASPM Disabled!\n");
        lnkCtrlWr = lnkCtl | PCI_EXP_LNKCTL_ASPM_L1; /* Enable */
        pci_write_config_dword(pDev, pos +  PCI_EXP_LNKCTL, lnkCtrlWr);
        pci_read_config_dword(pDev, pos + PCI_EXP_LNKCTL, &lnkCtl); /* Update */
        pDdc->lnkCtlAspmL1 = ((lnkCtl & PCI_EXP_LNKCTL_ASPM_L1) == PCI_EXP_LNKCTL_ASPM_L1);
      }
    }
    else
    {
      MXL_MOCA_DBG1P0("ASPM: Link not capable of L1!\n");
    } 
   
   ///* For TEST: */pci_write_config_dword(pDev, pos1 + PCI_PM_CTRL, pmCtrl | 0x3); // Capabilities::Status: D3

    MxL_MoCA_PCIeDbgTrace(MXL_MOCA_PCIE_L_INFO, "ASPM Enabled\n");
    
    /* Enable or disable PCI device's PME# function */
    if (pDdc->lnkCapPme)
    {
      pci_pme_active (pDev, true);
      MxL_MoCA_PCIeDbgTrace(MXL_MOCA_PCIE_L_INFO, "Enabled PME generation.\n");
    }
    else
    {
      MxL_MoCA_PCIeDbgTrace(MXL_MOCA_PCIE_L_WARN, "Not PME capable\n");
    }

    pci_read_config_dword(pDev, pos + PCI_EXP_DEVSTA, &status);
    pci_read_config_dword(pDev, pos + PCI_EXP_LNKCAP, &lnkCap);
    pci_read_config_dword(pDev, pos + PCI_EXP_LNKCTL, &lnkCtl);
    pci_read_config_dword(pDev, pos + PCI_EXP_LNKSTA, &lnkStat);

    MxL_MoCA_PCIeDbgTrace(MXL_MOCA_PCIE_L_INFO, "pDev->pm_cap=%d\n", pDev->pm_cap);
    MxL_MoCA_PCIeDbgTrace(MXL_MOCA_PCIE_L_DBG2, "capability=%x, control=%x, status=%x, lnkCap=%x, lnkCtl=%x, lnkStat=%x, pmCap=%x, pmCtrl=%x\n",
                          capability, control, status, lnkCap, lnkCtl, lnkStat, pmCap, pmCtrl);
  }
  return ret;
}
#endif 

/*----------------------------------------------------------------------------------------
 * 
 * FUNCTION NAME : MxL_MoCA_PCIeDataPathReset
 *  
 *
 *
 * DATE CREATED  : 03/25/2016
 * 
 * DESCRIPTION   : Resets the SoC hardware for the data path
 *
 * IN PARAMS     : [*pDdc] : driver data context 
 *
 * OUT PARAMS    : None
 *
 * RETURN VALUE  : MXL_MOCA_OK or MXL_MOCA_ERR
 *
 *--------------------------------------------------------------------------------------*/
 
static MXL_MOCA_STATUS_E MxL_MoCA_PCIeDataPathReset(MXL_MOCA_PCIE_DEV_DATA_CONTEXT_T *pDdc, int8_t devIndex)
{
  uint32_t mac[2] = {0,0};
  uint32_t addr, i, v;
  MXL_MOCA_STATUS_E ret = MXL_MOCA_OK;
  MXL_MOCA_PCIE_DATAPATH_CONTEXT_T *pDataPath = (MXL_MOCA_PCIE_DATAPATH_CONTEXT_T *)pDdc->pDpCtx;

  //Also called after SoC firmware is downloaded.

  MXL_MOCA_DBG1P1( "%s ...\n",  __FUNCTION__);

  do
  {
    MxL_MoCA_PCIeStopXmit(pDdc->pNetDev);

    MxL_MoCA_PCIeGetMgmtMAC(pDdc, mac);

    addr = pDdc->dmemBaseAddr + (PCIE_TC_MGMT_IF_MAC_ADDR * 4);
    ret = MxL_MoCA_PCIeWrite(devIndex, addr, mac[0]);
    if (MXL_MOCA_OK != ret)
    {
      break;
    }    
    
    ret = MxL_MoCA_PCIeWrite(devIndex, addr+4, mac[1]);
    if (MXL_MOCA_OK != ret)
    {
      break;
    }

    // RX section
    v = pDdc->dmemBaseAddr + (P_PCIE_HOST_RX_DESC_QUEUE * 4);
    pDataPath->rxQueuePair.descQueueAddr = v;
    addr = v;

    ret = MxL_MoCA_PCIeWrite(devIndex, addr, (uintptr_t)pDataPath->rxQueuePair.pDescQueuePa);
    if (MXL_MOCA_OK != ret)
    {
      break;
    }

    addr = pDdc->dmemBaseAddr + (P_PCIE_HOST_RX_DESC_Q_LEN * 4); // Rx desc queue len
    ret = MxL_MoCA_PCIeWrite(devIndex, addr, MXL_MOCA_PCIE_RX_DESC_QUEUE_SIZE);
    if (MXL_MOCA_OK != ret)
    {
      break;
    }

    // TX section
    v = pDdc->dmemBaseAddr + (P_PCIE_HOST_TX_DESC_QUEUE * 4);
    pDataPath->txQueuePair.descQueueAddr = v;
    addr = v;

    ret = MxL_MoCA_PCIeWrite(devIndex, addr, (uintptr_t)pDataPath->txQueuePair.pDescQueuePa);
    if (MXL_MOCA_OK != ret)
    {
      break;
    }

    addr = pDdc->dmemBaseAddr + (P_PCIE_HOST_TX_DESC_Q_LEN * 4); // Tx desc queue len    
    ret = MxL_MoCA_PCIeWrite(devIndex, addr, MXL_MOCA_PCIE_TX_DESC_QUEUE_SIZE);
    if (MXL_MOCA_OK != ret)
    {
      break;
    }

    pDataPath->txQueuePair.descQueueReadIdx  = 0;
    pDataPath->rxQueuePair.descQueueReadIdx  = 0;
    pDataPath->txQueuePair.descQueueWriteIdx = 0;
    pDataPath->rxQueuePair.descQueueWriteIdx = 0;

    pDataPath->rxQueuePair.descQueueSize = MXL_MOCA_PCIE_RX_DESC_QUEUE_SIZE;
    pDataPath->txQueuePair.descQueueSize = MXL_MOCA_PCIE_TX_DESC_QUEUE_SIZE;

    MxL_MoCA_PCIeFreeRxBuffers(pDdc->pNetDev);

    for (i = 0; i < MXL_MOCA_PCIE_RX_PTR_QUEUE_SIZE; i++)
        pDataPath->rxQueuePair.pDescQueue[i].dword0 = MXL_MOCA_PCIE_DESC_OWNER_HOST;  

    ret = MxL_MoCA_PCIeAllocRxBuffers(pDdc->pNetDev);
    if (MXL_MOCA_OK != ret)
    {
      break;
    }
    
    MxL_MoCA_PCIeResetTxQueue(pDdc->pNetDev);

    for(i = 0; i < MXL_MOCA_PCIE_TX_PTR_QUEUE_SIZE; i++)
        pDataPath->txQueuePair.pDescQueue[i].dword0 = MXL_MOCA_PCIE_DESC_OWNER_HOST;

    ret = MxL_MoCA_PCIeInitSoCIfc(devIndex);  
    if (MXL_MOCA_OK != ret)
    {
      break;
    }    

    MxL_MoCA_PCIeStartXmit(pDdc->pNetDev); 
  } while (0);
  
  return ret;
}

/*----------------------------------------------------------------------------------------
 * 
 * FUNCTION NAME : MxL_MoCA_PCIeRestart
 *  
 *
 *
 * DATE CREATED  : 03/25/2016
 * 
 * DESCRIPTION   : Restarts the data path
 *
 * IN PARAMS     : [devIndex] : device index   
 *
 * OUT PARAMS    : None
 *
 * RETURN VALUE  : MXL_MOCA_OK or MXL_MOCA_ERR
 *
 *--------------------------------------------------------------------------------------*/
 
 // interface
MXL_MOCA_STATUS_E MxL_MoCA_PCIeRestart(int8_t devIndex) 
{
  MXL_MOCA_PCIE_DEV_DATA_CONTEXT_T *pDdc = (MXL_MOCA_PCIE_DEV_DATA_CONTEXT_T*)gpDevData[devIndex];
  MXL_MOCA_STATUS_E ret = MXL_MOCA_OK;

  MXL_MOCA_DBG1P1( "%s...\n",  __FUNCTION__);
  ret = MxL_MoCA_PCIeConfigInboundATU(devIndex);
  if (MXL_MOCA_OK != ret)
  {
    return MXL_MOCA_ERR;
  }
  return MxL_MoCA_PCIeDataPathReset(pDdc, devIndex);
}

/*----------------------------------------------------------------------------------------
 * 
 * FUNCTION NAME : MxL_MoCA_PCIeStop
 *  
 *
 *
 * DATE CREATED  : 03/25/2016
 * 
 * DESCRIPTION   : Stops the data path
 *
 * IN PARAMS     : [devIndex] : device index
 *
 * OUT PARAMS    : None
 *
 * RETURN VALUE  : None
 *
 *--------------------------------------------------------------------------------------*/
 
 // interface
void MxL_MoCA_PCIeStop (int8_t devIndex) 
{
  MXL_MOCA_PCIE_DEV_DATA_CONTEXT_T *pDdc = (MXL_MOCA_PCIE_DEV_DATA_CONTEXT_T*)gpDevData[devIndex];

  MxL_MoCA_PCIeStopXmit(pDdc->pNetDev);
}


/*----------------------------------------------------------------------------------------
 * 
 * FUNCTION NAME : MxL_MoCA_PCIeQueuePkt
 *  
 *
 *
 * DATE CREATED  : 03/25/2016
 * 
 * DESCRIPTION   : Queue packet in the descriptor (common for Tx and Rx queues)
 *
 * IN PARAMS     : [*queuePair]  : descriptor queue.  
 *               : [*pPktElement]  : physical address.
 *
 * OUT PARAMS    : None
 *
 * RETURN VALUE  : MXL_MOCA_OK or MXL_MOCA_ERR
 *
 *--------------------------------------------------------------------------------------*/

static MXL_MOCA_STATUS_E MxL_MoCA_PCIeQueuePkt(MXL_MOCA_PCIE_QUEUE_PAIR_T* queuePair, uint32_t pktLen, dma_addr_t mapping)
{
  uint32_t widx, descrDword0, descrDword1;

  widx = queuePair->descQueueWriteIdx;

  // Each PCIe Tx/Rx Desc Queue entry is 2dwords long.
  if ((queuePair->pDescQueue[widx].dword0 & 0x03) != MXL_MOCA_PCIE_DESC_OWNER_HOST)
  {
    MXL_MOCA_DBG2P1("MXL_MOCA_NO_HOST_DESC_ERR, own=%x\n", (queuePair->pDescQueue[widx].dword0 & 0x03));
    return MXL_MOCA_NO_HOST_DESC_ERR;
  }

  /* ToDo: We don't need the packet queue, just the ring buffer is enough  Or use this and remove the ring */
  queuePair->pPktQueue[widx].mapping  = mapping;
  queuePair->pPktQueue[widx].totalLen = pktLen;

  /*****************************************************************************
   *  dword[1]::phy_addr    : 32  //!< 31:0  - physical addr of the packet
   *  dword[0]::totalLen    : 16  //!< 17:2  - total length of packet
   *  dword[0]::ownership   : 2;  //!<  1:0  - ownership: 1=SOC, 0=HOST, 2=PKT_DONE
   ****************************************************************************/
  descrDword0 = (pktLen << 2) | MXL_MOCA_PCIE_DESC_OWNER_SOC;
  descrDword1 =  mapping; /* physical addr (mapping) */

  queuePair->pDescQueue[widx].dword1 = descrDword1;
  queuePair->pDescQueue[widx].dword0 = descrDword0;
  
  widx++;

  if (widx >= queuePair->descQueueSize) // descriptors are 2 dwords
  {
    MXL_MOCA_DBG2P0( "wrap around!\n");
    widx = 0;
  }
  queuePair->descQueueWriteIdx = widx;
  return MXL_MOCA_OK;
}

/*----------------------------------------------------------------------------------------
 * 
 * FUNCTION NAME : MxL_MoCA_PCIeCheckOutOfBoundMapping
 *  
 *
 *
 * DATE CREATED  : 01/03/2017
 * 
 * DESCRIPTION   : Checks if the physical address mapping is outside the iATU window.
 *                 This function must be effecient.
 *
 * IN PARAMS     : [mapping]  : physical address provided by the OS (pci_map_single call)
 *
 * OUT PARAMS    : None
 *
 * RETURN VALUE  : MXL_MOCA_OK or MXL_MOCA_ERR
 *
 *--------------------------------------------------------------------------------------*/

static MXL_MOCA_STATUS_E MxL_MoCA_PCIeCheckOutOfBoundMapping(dma_addr_t mapping)
{
  /* NOTE: start_addr must be lowest of the target base addresses */
  if (mapping >= MXL_MOCA_PCIE_OUTB_ATU_TARGET0_ADDR_MASK && mapping <= MXL_MOCA_PCIE_OUTB_ATU_TARGET1_ADDR_LIMIT)
  {
    return MXL_MOCA_OK;
  }
  
  //MxL_MoCA_PCIeDbgTrace(MXL_MOCA_PCIE_L_ERR, "mapping error (0x%p), out of range (%x, %x)\n", 
  //                      mapping,
  //                      MXL_MOCA_PCIE_OUTB_ATU_TARGET0_ADDR_MASK, 
  //                      MXL_MOCA_PCIE_OUTB_ATU_TARGET1_ADDR_LIMIT);
  
  return MXL_MOCA_ERR;
}

/*----------------------------------------------------------------------------------------
 * 
 * FUNCTION NAME : MxL_MoCA_PCIeDebugHandleRxReturnOwnership
 *  
 *
 *
 * DATE CREATED  : 10/20/2017
 * 
 * DESCRIPTION   : This is a DEBUG function. When enabled, it will not hand off the 
 *                 received packets to upper layers. See echo 'r 2' > /proc/%s/pcie 
 *                 command usage. MXL_MOCA_PCIE_DEBUG_DROP_RX must be enabled.
 *
 * IN PARAMS     : [pDdc] : Pointer to driver data plane context. 
 *
 * OUT PARAMS    : None
 *
 * RETURN VALUE  : None
 *
 *--------------------------------------------------------------------------------------*/

 #if MXL_MOCA_PCIE_DEBUG_DROP_RX 
static void MxL_MoCA_PCIeDebugHandleRxReturnOwnership(MXL_MOCA_PCIE_DEV_DATA_CONTEXT_T *pDdc)
{
  MXL_MOCA_PCIE_DATAPATH_CONTEXT_T *pDataPath = (MXL_MOCA_PCIE_DATAPATH_CONTEXT_T *)pDdc->pDpCtx;
  uint16_t index, num=0;
  MXL_MOCA_PCIE_QUEUE_PAIR_T* queuePair;
  uint32_t descrDword0 = 0;

  queuePair = &pDataPath->rxQueuePair;

  for (index = 0; index < queuePair->descQueueSize; index++)
  {
    if ((queuePair->pDescQueue[index].dword0 & 0x03) == MXL_MOCA_PCIE_DESC_OWNER_HOST)
    { 
      num++;
      pDdc->rxStats[0]++;
      /*****************************************************************************
       *  dword[1]::phy_addr    : 32  //!< 31:0  - physical addr of the packet
       *  dword[0]::totalLen    : 16  //!< 17:2  - total length of packet
       *  dword[0]::ownership   : 2;  //!<  1:0  - ownership: 1=SOC, 0=HOST, 2=PKT_DONE
       ****************************************************************************/    
      pDdc->rxStats[2]++;    // proc stats
      pDdc->netIfRxDropped++; // ifconfig
      // Give the ownership back to SoC
      descrDword0 = queuePair->pDescQueue[index].dword0;
      descrDword0 &= 0xFFFFFFFC;
      descrDword0 |= MXL_MOCA_PCIE_DESC_OWNER_SOC;
      queuePair->pDescQueue[index].dword0 = descrDword0;
    }
  }
  if (num)
  {
    pDdc->ethStats.rxPackets     += num;
    pDdc->ethStats.rxPacketsGood += num;
  }  
}
#endif

/*----------------------------------------------------------------------------------------
 *      Interrupt handlers
 *--------------------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------------------
 * 
 * FUNCTION NAME : MxL_MoCA_PCIeHandleRxData
 *  
 *
 *
 * DATE CREATED  : 03/25/2016
 * 
 * DESCRIPTION   : Handles the receive interrupt for the SoC hardware.
 *                 Keep this function simple. No DBG print statements in release version
 *                 since it will impact performance at high data rate.
 *
 * IN PARAMS     : [pDdc] : Pointer to driver data plane context.  
 *
 * OUT PARAMS    : None
 *
 * RETURN VALUE  : None
 *
 *--------------------------------------------------------------------------------------*/
 
void MxL_MoCA_PCIeHandleRxData(MXL_MOCA_PCIE_DEV_DATA_CONTEXT_T *pDdc)
{
  MXL_MOCA_PCIE_DATAPATH_CONTEXT_T *pDataPath = (MXL_MOCA_PCIE_DATAPATH_CONTEXT_T *)pDdc->pDpCtx;

  uint32_t num, max;
  uint32_t ridx, pktLen, netif_rx_status;
  MXL_MOCA_PCIE_QUEUE_PKT_T* pPktInfo;
  struct net_device* pNetDev = pDdc->pNetDev;
  dma_addr_t         mapping;
  struct sk_buff    *skb;
  MXL_MOCA_STATUS_E  status;

  MXL_MOCA_PCIE_QUEUE_PAIR_T* queuePair;

  //MXL_MOCA_DBG2P1("%s\n", __FUNCTION__);

  num    = 0;
  queuePair = &pDataPath->rxQueuePair;
  ridx   = queuePair->descQueueReadIdx;

  max = MXL_MOCA_PCIE_MAX_RX_PACKETS_PER_INT; // Go one cycle of the descriptor array length
  MXL_MOCA_DBG2P4("%s:queuePair->pDescQueue[%d].dword0/dword1 %x, %x\n",
    __FUNCTION__, ridx, queuePair->pDescQueue[ridx].dword0, queuePair->pDescQueue[ridx].dword1);
  for (num=0; num < max; num++)
  {
    // MXL_MOCA_PCIE_DESC_OWNER_HOST is the initial state (no allocations). DESC_OWNER_SOC is when packets are ready to be sent.
    // MXL_MOCA_PCIE_DESC_OWNER_PKT_DONE is the state the SoC returns when packets are transferred and descriptor returned
    MXL_MOCA_DBG2P4("queuePair->pDescQueue[%d].dword0 = %d, %x, %x\n",  \
                     ridx, (queuePair->pDescQueue[ridx].dword0 & 0x03), \
                     queuePair->pDescQueue[ridx].dword0, queuePair->pDescQueue[ridx].dword1);
    
    if ((queuePair->pDescQueue[ridx].dword0 & 0x03) == MXL_MOCA_PCIE_DESC_OWNER_HOST) 
    {
      MXL_MOCA_DBG2P2("%s:*** (rx) ridx = %d\n",  __FUNCTION__, ridx);
    }
    else if ( (queuePair->pDescQueue[ridx].dword0 & 0x03) == MXL_MOCA_PCIE_DESC_OWNER_SOC ) /* unused descriptor */
    {
      break;
    }

    pPktInfo = &queuePair->pPktQueue[ridx];

    if (pPktInfo == NULL)
    {
      MXL_MOCA_DBG2P2("%s:pPktInfo is NULL at %d\n",  __FUNCTION__, ridx);
    }
    else
    {
      pDdc->ethStats.rxBytes += pPktInfo->totalLen;
    }
    pDdc->rxStats[0]++;

    if (ridx >= MXL_MOCA_PCIE_RX_RING_SIZE)
    {
      MxL_MoCA_PCIeDbgTrace(MXL_MOCA_PCIE_L_WARN, "%s invalid packet id %d\n", pNetDev->name, ridx);
      BUG();
      continue;
    }

    if (pDdc->rxRing[ridx].skb != NULL)
    {
      skb = (struct sk_buff *)pDdc->rxRing[ridx].skb;
      pktLen = queuePair->pDescQueue[ridx].dword0 >> 2;
      
      MXL_MOCA_DBG2P2("rx callback (alloc): packetID = %3d, skb = %p\n", ridx, skb);

  #if MXL_MOCA_ENABLE_DBG_MSG
      MxL_MoCA_PCIeDbgTrace(MXL_MOCA_PCIE_L_DBG2, "RX:(skb=%x) few pkt bytes(to stack) length=%x [%2x %2x %2x %2x %2x %2x %2x %2x\n %2x %2x %2x %2x %2x %2x %2x %2x\n %2x %2x %2x %2x %2x %2x %2x %2x\n %2x %2x %2x %2x %2x %2x %2x %2x\n %2x %2x]\n", 
                            skb->data, skb->len, 
                            skb->data[0],  skb->data[1],  skb->data[2],  skb->data[3],
                            skb->data[4],  skb->data[5],  skb->data[6],  skb->data[7],
                            skb->data[8],  skb->data[9],  skb->data[10], skb->data[11],
                            skb->data[12], skb->data[13], skb->data[14], skb->data[15],
                            skb->data[16], skb->data[17], skb->data[18], skb->data[19],
                            skb->data[20], skb->data[21], skb->data[22], skb->data[23],
                            skb->data[24], skb->data[25], skb->data[26], skb->data[27],
                            skb->data[28], skb->data[29], skb->data[30], skb->data[31],
                            skb->data[32], skb->data[33]);
  #endif
  
      /// unlock memory; may improve eg checksum speed
      pci_unmap_single(pDdc->pPCIeDev, (dma_addr_t)pDdc->rxRing[ridx].mapping, MXL_MOCA_PCIE_MAX_ETH_RX_BUF_SIZE, PCI_DMA_FROMDEVICE);
  
      if (pktLen > MXL_MOCA_PCIE_MAX_ETH_MTU_SIZE)
      {
        MxL_MoCA_PCIeDbgTrace(MXL_MOCA_PCIE_L_WARN, "%s X large packet size %d(%#x)\n", pNetDev->name, pktLen, pktLen);
  
        dev_kfree_skb_any(skb);
        pDdc->netIfRxDropped++;
      }
      else
      {
        // we do not have CRC: pktLen -= 4;            //strip 4B CRC
        skb_put(skb,pktLen);
  
        skb->protocol = eth_type_trans(skb, pNetDev);
      
#if MXL_MOCA_PCIE_DEBUG_DROP_RX 
        if (1 == pDdc->dbg_drop_rx) 
        {
          dev_kfree_skb_any(skb);
          netif_rx_status = NET_RX_DROP;
        }
        else
#endif // MXL_MOCA_PCIE_DEBUG_DROP_RX
        {
          // hand off the skb to upper layers
#if MXL_MOCA_PCIE_NAPI
          netif_rx_status = netif_receive_skb(skb);
#else
          netif_rx_status = netif_rx(skb); 
#endif
        }

        if (netif_rx_status == NET_RX_DROP) // DO NOT USE PRINT STATEMENTS
        {
          pDdc->rxStats[2]++;    // proc stats
          pDdc->netIfRxDropped++; // ifconfig
        }
        else
        {
          pDdc->rxStats[1]++;
        }
        pNetDev->last_rx = jiffies;
      } /* end if !status */  
    }
    
    // Alloc new skb
    skb = dev_alloc_skb(MXL_MOCA_PCIE_MAX_ETH_RX_BUF_SIZE);
    if (skb != NULL)
    {
      skb->dev = pNetDev;

      pDdc->rxRing[ridx].skb = skb;

      /// Ethernet header alignment.
      skb_reserve(skb,0); // Do not use NET_IP_ALIGN 

      mapping = pci_map_single(pDdc->pPCIeDev, skb->data, MXL_MOCA_PCIE_MAX_ETH_RX_BUF_SIZE, PCI_DMA_FROMDEVICE);
      if (unlikely(pci_dma_mapping_error(pDdc->pPCIeDev, mapping))) 
      {
        MxL_MoCA_PCIeDbgTrace(MXL_MOCA_PCIE_L_ERR, "HandleRxData: dma_mapping error!\n");
        pDdc->mappingErr0++;
        dev_kfree_skb_any(skb);
        pDdc->rxRing[ridx].skb = NULL;
        return;
      }
      else
      {
        if (MxL_MoCA_PCIeCheckOutOfBoundMapping(mapping) == MXL_MOCA_ERR)
        {
          MxL_MoCA_PCIeDbgTrace(MXL_MOCA_PCIE_L_ERR, "(0x%x)HandleRxData:out-of-bounds mapping error!\n", mapping);
          pDdc->mappingErr0++;
          dev_kfree_skb_any(skb);
          pDdc->rxRing[ridx].skb = NULL;          
          return;
        }
      }
      pDdc->rxRing[ridx].mapping = mapping;

      status = MxL_MoCA_PCIeQueuePkt(&pDdc->pDpCtx->rxQueuePair, MXL_MOCA_PCIE_MAX_ETH_MTU_SIZE, mapping);

      if (status != MXL_MOCA_OK)
      {
        MxL_MoCA_PCIeDbgTrace(MXL_MOCA_PCIE_L_WARN, "%s attach rx failed packet id:%d\n", pNetDev->name, ridx);
        dev_kfree_skb_any(skb);
      }
    }
    else
    {
#if MXL_MOCA_ENABLE_DBG_MSG    
      MxL_MoCA_PCIeDbgTrace(MXL_MOCA_PCIE_L_WARN, "************** cannot alloc a skb\n");
#endif  
      pDdc->skbAllocErr++;
      pDdc->rxRing[ridx].skb = NULL; // Upadte RX RingBuf SKB pointer to be NULL.
      // Do not increment ridx, exit loop. Breaks out of the loop to reduce ISR execution time.
      // This section will enter only when oversubscribed and the host is resource limited.
      break; 
    }
    ridx++;

    if (ridx >= MXL_MOCA_PCIE_RX_DESC_QUEUE_SIZE) 
      ridx = 0;
  }

  queuePair->descQueueReadIdx = ridx;

  MXL_MOCA_DBG2P2("%s:num = %d\n",  __FUNCTION__, num);
  
#if MXL_MOCA_PCIE_NAPI
  pDdc->napiWorkDone += num;
#endif

  if (num)
  {
    pDdc->ethStats.rxPackets     += num;
    pDdc->ethStats.rxPacketsGood += num;
  }
}

/*----------------------------------------------------------------------------------------
 * 
 * FUNCTION NAME : MxL_MoCA_PCIeHandleTxData
 *  
 *
 *
 * DATE CREATED  : 03/25/2016
 * 
 * DESCRIPTION   : Handles the transmit interrupt for the SoC hardware.
 *                 Keep this function simple. No DBG print statements in release version
 *                 since it will impact performance at high data rate.
 *
 * IN PARAMS     : [pDdc] : Pointer to driver data plane context. 
 *
 * OUT PARAMS    : None
 *
 * RETURN VALUE  : None
 *
 *--------------------------------------------------------------------------------------*/
 
void MxL_MoCA_PCIeHandleTxData(MXL_MOCA_PCIE_DEV_DATA_CONTEXT_T *pDdc)
{
  uint32_t ridx;
  MXL_MOCA_PCIE_DATAPATH_CONTEXT_T *pDataPath = (MXL_MOCA_PCIE_DATAPATH_CONTEXT_T *)pDdc->pDpCtx;
  MXL_MOCA_PCIE_QUEUE_PAIR_T* queuePair;
  MXL_MOCA_PCIE_QUEUE_PKT_T* pPktInfo;
  uint32_t  num, max;
  struct sk_buff *skb;
  dma_addr_t mapping;

  queuePair = &pDataPath->txQueuePair;

  num    = 0;
  ridx   = queuePair->descQueueReadIdx;

  MXL_MOCA_DBG2P2("%s:txQueuePair::descQueueReadIdx at %d \n",  __FUNCTION__, ridx);

  max = MXL_MOCA_PCIE_MAX_TX_PACKETS_PER_INT; // Go once cycle of the descriptor array length

  MXL_MOCA_DBG2P4("%s:queuePair->pDescQueue[%d].dword0/dword1 %x, %x\n",
    __FUNCTION__, ridx, queuePair->pDescQueue[ridx].dword0, queuePair->pDescQueue[ridx].dword1);
  
  for (num = 0; num < max; num++)
  {
    // MXL_MOCA_PCIE_DESC_OWNER_HOST is the initial state (no allocations). MXL_MOCA_PCIE_MXL_MOCA_PCIE_DESC_OWNER_SOC is when packets are ready to be sent.
    // MXL_MOCA_PCIE_DESC_OWNER_PKT_DONE is the state the SoC returns when packets are transferred and descriptor returned
    if ((queuePair->pDescQueue[ridx].dword0 & 0x03) != MXL_MOCA_PCIE_DESC_OWNER_PKT_DONE) 
    {
      MXL_MOCA_DBG2P2("%s:*** DESC_OWNER_PKT_DONE::ridx (break) = %d\n",  __FUNCTION__, ridx);
      break;
    }
    pPktInfo = &queuePair->pPktQueue[ridx]; 
    //  MxL_MoCA_PCIeDbgTrace(MXL_MOCA_PCIE_L_ERR, "%s:%d,%d\n",  __FUNCTION__, pPktInfo->totalLen, queuePair->pDescQueue[ridx].dword0 >> 2); 

    queuePair->pDescQueue[ridx].dword1 = 0;
    queuePair->pDescQueue[ridx].dword0 = MXL_MOCA_PCIE_DESC_OWNER_HOST; 

    pDdc->ethStats.txBytes += pPktInfo->totalLen;

    if (ridx >= MXL_MOCA_PCIE_TX_MAPPING_SIZE)
    {
      MxL_MoCA_PCIeDbgTrace(MXL_MOCA_PCIE_L_ERR, "invalid xmitted packet id:0x%x\n", ridx);
    }
    else if ((skb = (struct sk_buff *)pDdc->txRing[ridx].skb) == NULL)
    {
      MxL_MoCA_PCIeDbgTrace(MXL_MOCA_PCIE_L_ERR, "invalid tx skb for packet id:%d\n", ridx);
    }
    else
    {
      pDdc->txStats[5]++;
      pDdc->numPktsXmited++;
      mapping = (dma_addr_t)pDdc->txRing[ridx].mapping;
      pci_unmap_single(pDdc->pPCIeDev, mapping, skb->len, PCI_DMA_TODEVICE);
      MXL_MOCA_DBG2P2("%s:freeing ring = %x\n",  __FUNCTION__, ridx);
      dev_kfree_skb_any(skb);
      pDdc->txRing[ridx].skb = NULL;
    }

    ridx++;

    if (ridx >= MXL_MOCA_PCIE_TX_DESC_QUEUE_SIZE) 
      ridx = 0;
    
  }

  queuePair->descQueueReadIdx = ridx;
  queuePair->lastPktsCnt += num; // Increment tx stats for ifconfig
  MXL_MOCA_DBG2P3("%s ... num pkts = %d , descQueueReadIdx = %d\n",  __FUNCTION__, num, queuePair->descQueueReadIdx);

  if (num)
  {
    pDdc->ethStats.txPackets     += num;
    pDdc->ethStats.txPacketsGood += num;
  }

  netif_wake_queue(pDdc->pNetDev);

  //transmit pkts if already queued.
  MxL_MoCA_PCIeStartXmitSkbs(pDdc->pNetDev);
}

/*----------------------------------------------------------------------------------------
 * 
 * FUNCTION NAME : MxL_MoCA_PCIeAllocResources
 *  
 *
 *
 * DATE CREATED  : 03/25/2016
 * 
 * DESCRIPTION   : Initializes the data path (mem and queues)
 *
 * IN PARAMS     : [pDdc] : Pointer to driver data plane context. 
 *
 * OUT PARAMS    : None
 *
 * RETURN VALUE  : MXL_MOCA_OK for success, or MXL_MOCA_MEM_ALLOC_ERR for failure
 *
 *--------------------------------------------------------------------------------------*/
 
static MXL_MOCA_STATUS_E MxL_MoCA_PCIeAllocResources(MXL_MOCA_PCIE_DEV_DATA_CONTEXT_T *pDdc)
{
  void *pMem, *pa;
  uint32_t sz;
  MXL_MOCA_PCIE_DATAPATH_CONTEXT_T *pData;
  uint32_t addr;
  uint32_t i;
  MXL_MOCA_STATUS_E ret = MXL_MOCA_OK;
#if MXL_MOCA_PCIE_DEBUG_DESCR 
  MXL_MOCA_PCIE_QUEUE_DESCR_T* pDescr;
#endif

  MXL_MOCA_DBG2P1("%s: -- Alloc resources --\n", __FUNCTION__);

  // Allocate the contexts
  pDdc->pDpCtx = (MXL_MOCA_PCIE_DATAPATH_CONTEXT_T *)MxL_MoCA_PCIeKMemAlloc(sizeof(MXL_MOCA_PCIE_DATAPATH_CONTEXT_T));
  if (!pDdc->pDpCtx)
  {
    MxL_MoCA_PCIeDbgTrace(MXL_MOCA_PCIE_L_ERR, "Failed to allocate MXL_MOCA_PCIE_DATAPATH_CONTEXT_T ... \n");
    return MXL_MOCA_MEM_ALLOC_ERR;
  }

  do
  {
    pData = pDdc->pDpCtx;
    pData->txQueuePair.descQueueReadIdx  = 0;
    pData->rxQueuePair.descQueueReadIdx  = 0;
    pData->txQueuePair.descQueueWriteIdx = 0;
    pData->rxQueuePair.descQueueWriteIdx = 0;

    pData->txQueuePair.pPktQueue = NULL;
    pData->rxQueuePair.pPktQueue = NULL;

    pData->txQueuePair.pDescQueue = NULL;
    pData->rxQueuePair.pDescQueue = NULL;

    sz = MXL_MOCA_PCIE_TX_PTR_QUEUE_SIZE * sizeof(MXL_MOCA_PCIE_QUEUE_DESCR_T),
    pMem = MxL_MoCA_PCIeAllocDmaMem(pDdc, sz, (void **) &pa);
    if (!pMem)
    {
      MxL_MoCA_PCIeDbgTrace(MXL_MOCA_PCIE_L_ERR, "Failed to allocate PCIe Tx Desc Queue ... \n");
      MxL_MoCA_PCIeTerminate(pDdc);
      ret = MXL_MOCA_MEM_ALLOC_ERR;
      break;
    }
    
    memset(pMem, 0, sz);
    pData->txQueuePair.pDescQueue = (MXL_MOCA_PCIE_QUEUE_DESCR_T*) pMem;
    pData->txQueuePair.pDescQueuePa = pa; 

    for (i = 0; i < MXL_MOCA_PCIE_TX_PTR_QUEUE_SIZE; i++)
    {
      pData->txQueuePair.pDescQueue[i].dword0 = MXL_MOCA_PCIE_DESC_OWNER_HOST;
    }
    
    sz = MXL_MOCA_PCIE_RX_PTR_QUEUE_SIZE * sizeof(MXL_MOCA_PCIE_QUEUE_DESCR_T),
    pMem = MxL_MoCA_PCIeAllocDmaMem(pDdc, sz, (void **) &pa);
    if (!pMem)
    {
      MxL_MoCA_PCIeDbgTrace(MXL_MOCA_PCIE_L_ERR, "Failed to allocate PCIe Rx Desc Queue ... \n");
      MxL_MoCA_PCIeTerminate(pDdc);
      ret = MXL_MOCA_MEM_ALLOC_ERR;
      break;
    }
    
    memset(pMem, 0, sz);
    pData->rxQueuePair.pDescQueue = (MXL_MOCA_PCIE_QUEUE_DESCR_T*) pMem;
    pData->rxQueuePair.pDescQueuePa = pa; 

    for(i = 0; i < MXL_MOCA_PCIE_RX_PTR_QUEUE_SIZE; i++)
    {
      pData->rxQueuePair.pDescQueue[i].dword0 = MXL_MOCA_PCIE_DESC_OWNER_HOST;
    }
    
    sz = MXL_MOCA_PCIE_TX_PTR_QUEUE_SIZE * sizeof(MXL_MOCA_PCIE_QUEUE_PKT_T);
    pMem = MxL_MoCA_PCIeAllocDmaMem(pDdc, sz, (void **) &pa);

    if (!pMem)
    {
      MxL_MoCA_PCIeDbgTrace(MXL_MOCA_PCIE_L_ERR, "Failed to allocate PCIe Tx Ptr Queue ... \n");
      MxL_MoCA_PCIeTerminate(pDdc);
      ret = MXL_MOCA_MEM_ALLOC_ERR;
      break;
    }

    memset(pMem, 0, sz);
    pData->txQueuePair.pPktQueue = (MXL_MOCA_PCIE_QUEUE_PKT_T*) pMem;
    pData->txQueuePair.pPktQueuePa = pa;

    sz = MXL_MOCA_PCIE_RX_PTR_QUEUE_SIZE * sizeof(MXL_MOCA_PCIE_QUEUE_PKT_T);
    pMem = MxL_MoCA_PCIeAllocDmaMem(pDdc, sz, (void **) &pa);
    if (!pMem)
    {
      MxL_MoCA_PCIeDbgTrace(MXL_MOCA_PCIE_L_ERR, "Failed to allocate PCIe Rx Ptr Queue ... \n");
      MxL_MoCA_PCIeTerminate(pDdc);
      ret = MXL_MOCA_MEM_ALLOC_ERR;
      break;
    }

    memset(pMem, 0, sz);
    pData->rxQueuePair.pPktQueue = (MXL_MOCA_PCIE_QUEUE_PKT_T*) pMem;
    pData->rxQueuePair.pPktQueuePa = pa;

    MXL_MOCA_DBG1P1("PCIe Rx Descr Queue Host Addr: 0x%x\n",  pData->rxQueuePair.pDescQueuePa);
    MXL_MOCA_DBG1P1("PCIe Tx Descr Queue Host Addr: 0x%x\n",  pData->txQueuePair.pDescQueuePa);

    MXL_MOCA_DBG1P1("PCIe Rx Packet Queue Host Addr: 0x%x\n", pData->rxQueuePair.pPktQueuePa);
    MXL_MOCA_DBG1P1("PCIe Tx Packet Queue Host Addr: 0x%x\n", pData->txQueuePair.pPktQueuePa);

#if MXL_MOCA_PCIE_DEBUG_DESCR
    // print descriptors  
    pDescr = pData->rxQueuePair.pDescQueue; 
    for (i = 0; i < (MXL_MOCA_PCIE_RX_DESC_QUEUE_SIZE); i++) 
    {
      MXL_MOCA_DBG1P3("rx descr[0x%x] = {0x%x, 0x%x}\n", i, pDescr[i].dword0, pDescr[i].dword1);
    }
    pDescr = pData->txQueuePair.pDescQueue;
    for (i = 0; i < (MXL_MOCA_PCIE_TX_DESC_QUEUE_SIZE); i++) 
    {
      MXL_MOCA_DBG1P3("tx descr[0x%x] = {0x%x, 0x%x}\n", i, pDescr[i].dword0, pDescr[i].dword1);
    }
#endif

    addr = pDdc->dmemBaseAddr + (PCIE_NUM_OUTBOUND_REGIONS * 4);
    ret = MxL_MoCA_PCIeWrite(0, addr, MXL_MOCA_PCIE_NUM_OUTBOUND_ATU_REGIONS);
    if (MXL_MOCA_OK != ret)
    {
      break;
    }

    // Region
    // ToDo: Change this to use region0/1OutATUAddrBase when CAR-1141 if fixed
    addr = pDdc->dmemBaseAddr + (PCIE_OUTBOUND_ATU_REG0_BASE * 4);
    pDdc->region0OutATUAddrBase = MXL_MOCA_PCIE_OUTB_ATU_REGION0_BASE; 
    ret = MxL_MoCA_PCIeWrite(0, addr, pDdc->region0OutATUAddrBase);  
    if (MXL_MOCA_OK != ret)
    {
      break;
    }
    
    addr = pDdc->dmemBaseAddr + (PCIE_OUTBOUND_ATU_REG1_BASE * 4); 
    pDdc->region1OutATUAddrBase = MXL_MOCA_PCIE_OUTB_ATU_REGION1_BASE; 
    ret = MxL_MoCA_PCIeWrite(0, addr, pDdc->region1OutATUAddrBase); 
    if (MXL_MOCA_OK != ret)
    {
      break;
    }

    // Target
    addr = pDdc->dmemBaseAddr + (PCIE_OUTBOUND_ATU_TAR0_BASE * 4);
    pDdc->target0OutATUAddrMask = MXL_MOCA_PCIE_OUTB_ATU_TARGET0_ADDR_MASK;
    ret = MxL_MoCA_PCIeWrite(0, addr, pDdc->target0OutATUAddrMask);
    if (MXL_MOCA_OK != ret)
    {
      break;
    }

    addr = pDdc->dmemBaseAddr + (PCIE_OUTBOUND_ATU_TAR1_BASE * 4); 
    pDdc->target1OutATUAddrMask = MXL_MOCA_PCIE_OUTB_ATU_TARGET1_ADDR_MASK;
    ret = MxL_MoCA_PCIeWrite(0, addr, pDdc->target1OutATUAddrMask);
    if (MXL_MOCA_OK != ret)
    {
      break;
    }
  } while(0);
  
  return ret;
}

/*----------------------------------------------------------------------------------------
 * 
 * FUNCTION NAME : MxL_MoCA_PCIeIsXmitOn
 *  
 *
 *
 * DATE CREATED  : 03/25/2016
 * 
 * DESCRIPTION   : Return TRUE if the device is available/ready to handle traffic
 *
 * IN PARAMS     : [devIndex] : device index 
 *
 * OUT PARAMS    : None
 *
 * RETURN VALUE  : false or true
 *
 *--------------------------------------------------------------------------------------*/
 
 // interface
int MxL_MoCA_PCIeIsXmitOn(unsigned int devIndex) 
{
  int ret = false;
  MXL_MOCA_PCIE_DEV_DATA_CONTEXT_T *pDdc = (MXL_MOCA_PCIE_DEV_DATA_CONTEXT_T *)gpDevData[devIndex];

  if (pDdc) 
    ret = pDdc->active;

  return ret;
}

/*----------------------------------------------------------------------------------------
 * 
 * FUNCTION NAME : MxL_MoCA_PCIeAvailable
 *  
 *
 *
 * DATE CREATED  : 03/25/2016
 * 
 * DESCRIPTION   : Return TRUE if the device is available/ready 
 *
 * IN PARAMS     : [devIndex] : device index 
 *
 * OUT PARAMS    : None
 *
 * RETURN VALUE  : MXL_MOCA_OK or MXL_MOCA_ERR
 *
 *--------------------------------------------------------------------------------------*/
 
 // interface
MXL_MOCA_STATUS_E MxL_MoCA_PCIeAvailable(int8_t devIndex) 
{
  uint32_t val;
  MXL_MOCA_STATUS_E ret = MXL_MOCA_OK;
  
  MXL_MOCA_PCIE_DEV_DATA_CONTEXT_T *pDdc = (MXL_MOCA_PCIE_DEV_DATA_CONTEXT_T *)gpDevData[devIndex];

  ret = MxL_MoCA_PCIeRead(devIndex, MXL_MOCA_PROD_FAM_ID_ADDR, &val);
  
  if (MXL_MOCA_OK == ret)
  {
    if (pDdc->deviceId != val) 
    {
      ret = MXL_MOCA_ERR;
    }
  }
  return ret;
}

/*----------------------------------------------------------------------------------------
 * 
 * FUNCTION NAME : MxL_MoCA_PCIeInitSoCIfc
 *  
 *
 *
 * DATE CREATED  : 03/25/2016
 * 
 * DESCRIPTION   : Local function to init the SoC ifc
 *
 * IN PARAMS     : [devIndex] : device index 
 *
 * OUT PARAMS    : None
 *
 * RETURN VALUE  : MXL_MOCA_STATUS_E code
 *
 *--------------------------------------------------------------------------------------*/
 
static MXL_MOCA_STATUS_E MxL_MoCA_PCIeInitSoCIfc(unsigned int devIndex) // need to check for failures
{
  MXL_MOCA_PCIE_DEV_DATA_CONTEXT_T *pDdc = (MXL_MOCA_PCIE_DEV_DATA_CONTEXT_T *)gpDevData[devIndex];  
  uint32_t val = 0, addr;
  int8_t   loop;
  uint32_t endianCheckVal = 1; 
  MXL_MOCA_STATUS_E ret = MXL_MOCA_OK;

  uint8_t *pByte = (uint8_t*)&endianCheckVal;
  uint32_t hostEndian = (pByte[0] == 1)?0:1; // hostEndian result: 0 = little; 1 = big

  MXL_MOCA_DBG1P0("#### Outbound ATU Regions (DMEM) ####\n");

  do 
  {
    ret = MxL_MoCA_PCIeRead (devIndex, pDdc->dmemBaseAddr + (PCIE_OUTBOUND_ATU_REG0_BASE * 4), &val); 
    MXL_MOCA_DBG1P2("PCIE_OUTBOUND_ATU_REG0_BASE(0x%x)=0x%8x\n", pDdc->dmemBaseAddr + (PCIE_OUTBOUND_ATU_REG0_BASE * 4), val);
    if (ret == MXL_MOCA_ERR)
    {
      break;
    }

    ret = MxL_MoCA_PCIeRead (devIndex, pDdc->dmemBaseAddr + (PCIE_OUTBOUND_ATU_REG1_BASE * 4), &val); 
    MXL_MOCA_DBG1P2("PCIE_OUTBOUND_ATU_REG1_BASE(0x%x)=0x%8x\n", pDdc->dmemBaseAddr + (PCIE_OUTBOUND_ATU_REG1_BASE * 4), val);
    if (ret == MXL_MOCA_ERR)
    {
      break;
    }

    /* Address translation for outbound */
    ret = MxL_MoCA_PCIeConfigOutboundATU(devIndex);
    if (ret == MXL_MOCA_ERR)
    {
      break;
    }    
  } while (0);
  
  // DEBUG Info
  MXL_MOCA_DBG1P0("#### Outbound ATU values (EP) ####\n");

  for (loop = 0; loop < MXL_MOCA_PCIE_NUM_OUTBOUND_ATU_REGIONS; loop++)
  {
    if (ret == MXL_MOCA_ERR)
    {
      break;
    }
    
    ret = MxL_MoCA_PCIeWrite(devIndex, (uintptr_t)(MXL_MOCA_PCIE_EP_IATU_INDEX_REG), loop);
    MXL_MOCA_DBG1P1("EP_IATU_INDEX_REG(0x08204900)=0x%x\n", loop);
    if (ret == MXL_MOCA_ERR)
    {
      break;
    }
    
    ret = MxL_MoCA_PCIeRead(devIndex, (uintptr_t)MXL_MOCA_PCIE_EP_IATU_REGION_CTRL_2_REG, &val); 
    MXL_MOCA_DBG1P1("EP_IATU_REGION_CTRL_2_REG(0x08204908)=0x%x\n", val);
    if (ret == MXL_MOCA_ERR)
    {
      break;
    }
    
    ret = MxL_MoCA_PCIeRead(devIndex, (uintptr_t)MXL_MOCA_PCIE_EP_IATU_REGION_LOWER_BASE_ADDR_REG, &val); 
    MXL_MOCA_DBG1P1("EP_IATU_REGION_LOWER_BASE_ADDR_REG(0x0820490C)=0x%x\n", val);
    if (ret == MXL_MOCA_ERR)
    {
      break;
    }
    
    ret = MxL_MoCA_PCIeRead(devIndex, (uintptr_t)MXL_MOCA_PCIE_EP_IATU_REGION_LIMIT_ADDR_REG, &val); 
    MXL_MOCA_DBG1P1("EP_IATU_REGION_LIMIT_ADDR_REG(0x08204914)=0x%x\n", val);
    if (ret == MXL_MOCA_ERR)
    {
      break;
    }
    
    ret = MxL_MoCA_PCIeRead(devIndex, (uintptr_t)MXL_MOCA_PCIE_EP_IATU_REGION_LOWER_TARGET_ADDR_REG, &val); 
    MXL_MOCA_DBG1P1("EP_IATU_REGION_LOWER_TARGET_ADDR_REG(0x08204918)=0x%x\n\n", val);
    if (ret == MXL_MOCA_ERR)
    {
      break;
    }    
  }

  do
  {
    if (ret == MXL_MOCA_ERR)
    {
      break;
    }
    
    // Update DMEM region:
    addr = pDdc->dmemBaseAddr + (PCIE_NUM_OUTBOUND_REGIONS * 4);   
    ret = MxL_MoCA_PCIeWrite(devIndex, addr, MXL_MOCA_PCIE_NUM_OUTBOUND_ATU_REGIONS);
    if (ret == MXL_MOCA_ERR)
    {
      break;
    }
    // ToDo: Change these to region0OutATUAddrBase when CAR-1141 if fixed
    // Region
    addr = pDdc->dmemBaseAddr + (PCIE_OUTBOUND_ATU_REG0_BASE * 4); 
    ret = MxL_MoCA_PCIeWrite(devIndex, addr, pDdc->region0OutATUAddrBase);
    if (ret == MXL_MOCA_ERR)
    {
      break;
    }

    addr = pDdc->dmemBaseAddr + (PCIE_OUTBOUND_ATU_REG1_BASE * 4); 
    ret = MxL_MoCA_PCIeWrite(devIndex, addr, pDdc->region1OutATUAddrBase);
    if (ret == MXL_MOCA_ERR)
    {
      break;
    }

    // target
    // Target
    addr = pDdc->dmemBaseAddr + (PCIE_OUTBOUND_ATU_TAR0_BASE * 4);
    ret = MxL_MoCA_PCIeWrite(0, addr, pDdc->target0OutATUAddrMask);
    if (ret == MXL_MOCA_ERR)
    {
      break;
    }

    addr = pDdc->dmemBaseAddr + (PCIE_OUTBOUND_ATU_TAR1_BASE * 4); 
    ret = MxL_MoCA_PCIeWrite(0, addr, pDdc->target1OutATUAddrMask);
    if (ret == MXL_MOCA_ERR)
    {
      break;
    }
    
    ret = MxL_MoCA_PCIeWrite(devIndex,(pDdc->dmemBaseAddr + (PCIE_DEVCTL_MAX_PAYLOAD_SIZE * 4)), pDdc->devCtlMaxPayloadSz);
    /* NOTE: The RC is sending completion TLPs larger than the RCB setting (not conforming to spec.). 
       This is ok as long as there are no issues seen on the SoC side. If a host system is always going 
       to send TLPs of RCB length (64 or 128 bytes), we should set the LargeRdAXIBurst to RCB length 
       instead of MaxReadReq. This is done by enabling pDdc->lnkCtlRCB below. 
    */
    ret = MxL_MoCA_PCIeWrite(devIndex,(pDdc->dmemBaseAddr + (PCIE_LNKCTL_RCB * 4)), pDdc->devCtlMaxReadReqSz/*pDdc->lnkCtlRCB*/);
    if (ret == MXL_MOCA_ERR)
    {
      break;
    }

    // DEBUG Info
    MXL_MOCA_DBG1P0("#### DMEM values (SoC) ####\n");
    ret = MxL_MoCA_PCIeRead (devIndex, pDdc->dmemBaseAddr + (PCIE_HOST_STATUS * 4), &val); 
    MXL_MOCA_DBG1P2("W_HIF_PCIE_HOST_STATUS(0x%x)=0x%x\n", pDdc->dmemBaseAddr + (PCIE_HOST_STATUS * 4), val);
    if (ret == MXL_MOCA_ERR)
    {
      break;
    }

    ret = MxL_MoCA_PCIeRead (devIndex, pDdc->dmemBaseAddr + (PCIE_OUTBOUND_ATU_REG0_BASE * 4), &val); 
    MXL_MOCA_DBG1P2("PCIE_OUTBOUND_ATU_REG0_BASE(0x%x)=0x%x\n", pDdc->dmemBaseAddr + (PCIE_OUTBOUND_ATU_REG0_BASE * 4), val);
    if (ret == MXL_MOCA_ERR)
    {
      break;
    }
    
    ret = MxL_MoCA_PCIeRead (devIndex, pDdc->dmemBaseAddr + (PCIE_OUTBOUND_ATU_REG1_BASE * 4), &val); 
    MXL_MOCA_DBG1P2("PCIE_OUTBOUND_ATU_REG1_BASE(0x%x)=0x%x\n", pDdc->dmemBaseAddr + (PCIE_OUTBOUND_ATU_REG1_BASE * 4), val);
    if (ret == MXL_MOCA_ERR)
    {
      break;
    }

    ret = MxL_MoCA_PCIeRead (devIndex, (pDdc->dmemBaseAddr + (PCIE_DEVCTL_MAX_PAYLOAD_SIZE * 4)), &val); 
    MXL_MOCA_DBG1P2("PCIE_DEVCTL_MAX_PAYLOAD_SIZE(0x%x)=0x%x\n", (pDdc->dmemBaseAddr + (PCIE_DEVCTL_MAX_PAYLOAD_SIZE * 4)), val);
    if (ret == MXL_MOCA_ERR)
    {
      break;
    }

    ret = MxL_MoCA_PCIeRead (devIndex, (pDdc->dmemBaseAddr + (PCIE_LNKCTL_RCB * 4)), &val); 
    MXL_MOCA_DBG1P2("PCIE_LNKCTL_RCB(0x%x)=0x%x\n", (pDdc->dmemBaseAddr + (PCIE_LNKCTL_RCB * 4)), val);
    if (ret == MXL_MOCA_ERR)
    {
      break;
    }

    // Activate CCPU periodic interrupts to prevent missed/overlap interrupt scenarios
    MXL_MOCA_DBG1P2("Setting PCIE_EN_CCPU_TIMER_INTR (0x%x) to %d \n", (pDdc->dmemBaseAddr + (PCIE_EN_CCPU_TIMER_INTR * 4)), MXL_MOCA_PCIE_EN_CCPU_TIMER_INTR);
    ret = MxL_MoCA_PCIeWrite(devIndex,(pDdc->dmemBaseAddr + (PCIE_EN_CCPU_TIMER_INTR * 4)), (uint32_t)MXL_MOCA_PCIE_EN_CCPU_TIMER_INTR);
    if (ret == MXL_MOCA_ERR)
    {
      break;
    }

    // DMA access byte order
    MXL_MOCA_DBG1P2("Setting PCIE_DMA_DATA_ACCESS_BYTE_ORDER (0x%x) to %d \n", (pDdc->dmemBaseAddr + (PCIE_DMA_DATA_ACCESS_BYTE_ORDER * 4)), hostEndian);
    ret = MxL_MoCA_PCIeWrite(devIndex,(pDdc->dmemBaseAddr + (PCIE_DMA_DATA_ACCESS_BYTE_ORDER * 4)), hostEndian);
    MxL_MoCA_PCIeDbgTrace(MXL_MOCA_PCIE_L_INFO, "Host byte order:%s\n", (hostEndian==1)?"big-endian":"little-endian");  
    if (ret == MXL_MOCA_ERR)
    {
      break;
    }

    // Done with Init: TC code can start running now.
    MXL_MOCA_DBG1P1("Setting PCIE_HOST_STATUS (0x%x) to RUNNING \n", (pDdc->dmemBaseAddr + (PCIE_HOST_STATUS * 4)));
    ret = MxL_MoCA_PCIeWrite(devIndex,(pDdc->dmemBaseAddr + (PCIE_HOST_STATUS * 4)), (uint32_t)PCIE_HOST_STATUS_RUNNING);
  } while (0);
  
  if (ret == MXL_MOCA_ERR)
  {
    MxL_MoCA_PCIeDbgTrace(MXL_MOCA_PCIE_L_ERR, "PCIe Init SoC Ifc: Read/Write error!\n");
  }
  else
  {
    MxL_MoCA_PCIeDbgTrace(MXL_MOCA_PCIE_L_INFO, "Initializing PCIe interface of MoCA SoC...OK\n");
  }
  
  return ret;
}

/*----------------------------------------------------------------------------------------
 * 
 * FUNCTION NAME : MxL_MoCA_PCIeGetSocName
 *  
 *
 *
 * DATE CREATED  : 07/26/2017
 * 
 * DESCRIPTION   : Queries the family ID and bond option and return the chip name string
 *
 * IN PARAMS     : [devIndex] : Device index of the PCI device.  
 *
 * OUT PARAMS    : [pChipName]: pointer to return MoCA SoC skuName string or 'Unknown'.
 *
 * RETURN VALUE  : return MXL_MOCA_INVALID_PARAM_ERR, MXL_MOCA_OK or MXL_MOCA_ERR
 *
 *--------------------------------------------------------------------------------------*/

static MXL_MOCA_STATUS_E MxL_MoCA_PCIeGetSocName(int8_t devIndex, char **pChipName)
{
  uint32_t productFamilyId;
  uint32_t bondOpt;
  uint32_t arrayLen;
  uint32_t i = 0;
  MXL_MOCA_STATUS_E ret = MXL_MOCA_OK;
  static const char const *UnknownStr = "Unknown";
  static const struct skuNameArray
  {
    uint32_t familyId;
    uint32_t bondOption;
    char *skuName;
  } skuArray[] = {
    {0x3700, 0x2f, "MXL3700"},
    {0x3700, 0x3f, "MXL3701"},
    {0x3700, 0x2e, "MXL3702"},
    {0x3700, 0x2d, "MXL3705"},
    {0x3700, 0x3d, "MXL3706"},
    {0x3700, 0x2c, "MXL3707"},
    {0x3710, 0x2d, "MXL3710"},
    {0x3710, 0x3d, "MXL3711"},
    {0x3710, 0x2c, "MXL3712"},
  };
  
  if (NULL == pChipName)
  { 
    MxL_MoCA_PCIeDbgTrace(MXL_MOCA_PCIE_L_ERR, "Invalid pointer passed for chip name\n");
    return MXL_MOCA_INVALID_PARAM_ERR;
  }

  arrayLen = sizeof(skuArray)/sizeof(skuArray[0]);

  do
  {
    ret = MxL_MoCA_PCIeRead(devIndex, MXL_MOCA_PROD_FAM_ID_ADDR, &productFamilyId);
    if (MXL_MOCA_OK == ret)
    {
      ret = MxL_MoCA_PCIeRead(devIndex, MXL_MOCA_SYS_RES_EXT_CONFIG_STAT_ADDR, &bondOpt);
      if (MXL_MOCA_OK == ret)
      {
        bondOpt = (bondOpt & MXL_MOCA_BOND_OPT_SKU_MASK) >> MXL_MOCA_BOND_OPT_OFFSET;        

        for (i = 0; i < arrayLen; i++)
        {
          if ((skuArray[i].familyId == productFamilyId) &&
              (skuArray[i].bondOption == bondOpt))
          {
            *pChipName = (char *)skuArray[i].skuName;
            break;
          }
        }                
      }
    }

    // Can't find the valid SKU from skuArray
    if (MXL_MOCA_ERR == ret || (i == arrayLen))
    {
      *pChipName = (char*)UnknownStr;
      if (MXL_MOCA_ERR == ret)
      {
        MxL_MoCA_PCIeDbgTrace(MXL_MOCA_PCIE_L_ERR, "Failed to read device ID of MoCA SoC.\n");
      }     
    }
  } while (0);

  return ret;
}

/*----------------------------------------------------------------------------------------
 * 
 * FUNCTION NAME : MxL_MoCA_PCIeProbeOne
 *  
 *
 *
 * DATE CREATED  : 03/25/2016
 * 
 * DESCRIPTION   : Initializes an instance of clink ethernet driver.
 *                 Initializes an adapter identified by a pci_dev structure.
 *                 The OS initialization, configuring of the adapter private structure,
 *                 and a hardware reset occur.
 *
 * IN PARAMS     : [pdev]    : Pointer to the PCI device.  
 *               : [pEntry]  : Pointer to the PCI device ID. 
 *
 * OUT PARAMS  : None
 *
 * RETURN VALUE  : return 0 if successful, <0 (Linux error code) if error
 *
 *--------------------------------------------------------------------------------------*/
 
static int __devinit MxL_MoCA_PCIeProbeOne(struct pci_dev *pdev, const struct pci_device_id *pEntry)
{
  struct net_device                *pNetDev;
  MXL_MOCA_PCIE_DEV_DATA_CONTEXT_T *pDrvData, *pDdc;
  uint16_t                          err, index;
  struct proc_dir_entry            *pProcEntry;
  uint32_t         val, bar0;
  uint16_t         valShort;

  uint32_t        bar1, pos; 
  uint16_t        control, capability, status;
  uint32_t        configValue; 
  uint32_t        tmp[2]; 
  uint32_t        seq;
  uint8_t         myMacAddr[6];
  MXL_MOCA_STATUS_E ret = MXL_MOCA_OK;
  MXL_MOCA_PCIE_ERROR_OUT_E errorOut = MXL_MOCA_PCIE_ERROR_OUT_NONE;
  char*           pChipName = NULL; 
  static const int8_t *pLinkSpeed[4] = {
                                         "Invalid",
                                         "2.5GT/s",
                                         "5.0GT/s",
                                         "8.0GT/s"
                                         };

  MXL_MOCA_DBG1P1("pdev: 0x%p\n", pdev);

  do
  {
    /* Enable PCIe Device: Initialize device before it's used by a driver.
     * Ask low-level code to enable I/O and memory
     */
    if (pci_enable_device(pdev)) 
    {
      MXL_MOCA_DBG2P0("pci_enable_device failed\n");
      return -ENODEV;
    }

    // Set DMA Mask so we work on 32-bit and 64-bit systems  
    err = pci_set_dma_mask(pdev, 0xffffffff);
    if (err) 
    {
      MxL_MoCA_PCIeDbgTrace(MXL_MOCA_PCIE_L_ERR, "pci_set_dma_mask failed: %d\n", err);
      pci_disable_device(pdev);
      return -ENODEV;
    }

    err = pci_set_consistent_dma_mask(pdev, 0xffffffff);
    if (err) 
    {
      MxL_MoCA_PCIeDbgTrace(MXL_MOCA_PCIE_L_ERR, "pci_set_consistent_dma_mask failed: %d\n", err);
      pci_disable_device(pdev);
      return -ENODEV;
    }

    /// allocate an aligned and zeroed private driver structure
    /// the structure is
    ///    net_device with eth_dev tacked on the end
    ///    net_device->priv points at the eth_dev
    pNetDev = alloc_etherdev(sizeof(MXL_MOCA_PCIE_DEV_DATA_CONTEXT_T));

    if (!pNetDev) 
    {
      MxL_MoCA_PCIeDbgTrace(MXL_MOCA_PCIE_L_ERR, "alloc_etherdev failed\n");
      pci_disable_device(pdev);
      return -ENOMEM;
    }   
    memset(pNetDev->broadcast, 0xFF, MAX_ADDR_LEN); 

    strcpy(pNetDev->name, MXL_MOCA_PCIE_DEV_NAME); // en%d

    /// Initialize the owner field in net device
    SET_MODULE_OWNER(pNetDev);

    /* Reserve PCI I/O and memory resources (owner is MXL_MOCA_PCIE_DRV_NAME) */
    if (pci_request_regions(pdev, MXL_MOCA_PCIE_DRV_NAME)) 
    {
      MxL_MoCA_PCIeDbgTrace(MXL_MOCA_PCIE_L_ERR, "pci_request_regions failed\n");
      errorOut = MXL_MOCA_PCIE_ERROR_OUT_FREE_NETDEV;
      break;
    }
    
    /// Store net device in PCI device structure
    pci_set_drvdata(pdev, pNetDev);      /// pci_dev -> driver_data = net_device : Set private driver data pointer for a pci_dev

    /// init the driver data context
    pDrvData = NETDEV_PRIV(pNetDev);

    MxL_MoCA_PCIeLockInit(&pDrvData->ethTxLock);

    // BAR0
    pDrvData->baseIoAddr0 = pci_resource_start(pdev, 0);
    pDrvData->baseIoLen0  = pci_resource_len(pdev, 0);
    
    /* map bus memory into CPU space */
    pDrvData->baseIoRemap0 = (uintptr_t)ioremap_nocache(pDrvData->baseIoAddr0, pDrvData->baseIoLen0);
    if (pDrvData->baseIoRemap0 == 0)
    {
      MxL_MoCA_PCIeDbgTrace(MXL_MOCA_PCIE_L_ERR, "ioremap_nocache failed for BAR1 (vmalloc too small) - abort!\n");
      return -ENOMEM;
    }

    // BAR1
    pDrvData->baseIoAddr1 = pci_resource_start(pdev, 1);
    pDrvData->baseIoLen1  = pci_resource_len(pdev, 1);
    
    /* map bus memory into CPU space */
    pDrvData->baseIoRemap1 = (uintptr_t)ioremap_nocache(pDrvData->baseIoAddr1, pDrvData->baseIoLen1);
    
    if (pDrvData->baseIoRemap1 == 0)
    {
      MxL_MoCA_PCIeDbgTrace(MXL_MOCA_PCIE_L_ERR, "ioremap_nocache failed for BAR1 (vmalloc too small) - abort!\n");
      return -ENOMEM;
    }

    MxL_MoCA_PCIeDbgTrace(MXL_MOCA_PCIE_L_DBG1, "baseIoAddr0: 0x%08x\n", pDrvData->baseIoAddr0);
    
    MXL_MOCA_DBG1P1("baseIoLen0  : 0x%08x\n",  pDrvData->baseIoLen0);
    MXL_MOCA_DBG1P1("baseIoRemap0: 0x%x\n",    pDrvData->baseIoRemap0);

    MxL_MoCA_PCIeDbgTrace(MXL_MOCA_PCIE_L_DBG1, "baseIoAddr1: 0x%08x\n", pDrvData->baseIoAddr1);
    
    MXL_MOCA_DBG1P1("baseIoLen1  : 0x%08x\n",  pDrvData->baseIoLen1);
    MXL_MOCA_DBG1P1("baseIoRemap1: 0x%x\n",    pDrvData->baseIoRemap1);

    if (pDrvData->baseIoAddr0 == pDrvData->baseIoAddr1)
    {
      MxL_MoCA_PCIeDbgTrace(MXL_MOCA_PCIE_L_ERR, "Error: baseIoAddr0(0x%x) and baseIoAddr1(0x%x) are same!\n", pDrvData->baseIoAddr0, pDrvData->baseIoAddr1);    
    }

    pDrvData->pPCIeDev   = pdev; 
    pNetDev->base_addr   = pDrvData->baseIoAddr0;

    pDrvData->deviceId = MXL_MOCA_PCIE_DEVICE_ID_UNKNOWN;

    if (pci_read_config_word(pdev, PCI_DEVICE_ID, &valShort))
    {
      MxL_MoCA_PCIeDbgTrace(MXL_MOCA_PCIE_L_ERR, "Cannot read vendor ID from the PCIe device!\n");
      errorOut = MXL_MOCA_PCIE_ERROR_OUT_FREE_ALLOC;
      break;      
    }
    else
    {
      pDrvData->deviceId = (MXL_MOCA_PCIE__DEVICE_ID_E)valShort;
      
      if (pDrvData->deviceId == MXL_MOCA_PCIE_DEVICE_ID_CARDIFF)
      {
        MxL_MoCA_PCIeDbgTrace(MXL_MOCA_PCIE_L_INFO, "Reading config space(PCI_DEVICE_ID) = 0x%08x (Cardiff)\n", valShort);
        pDrvData->dmemBaseAddr = MXL_MOCA_PCIE_HIF_TC0_DMEM_BASE;
      }
      else if (pDrvData->deviceId == MXL_MOCA_PCIE_DEVICE_ID_LEUCADIA)
      {
        MxL_MoCA_PCIeDbgTrace(MXL_MOCA_PCIE_L_INFO, "Reading config space(PCI_DEVICE_ID) = 0x%08x (Leucadia)\n", valShort);
        pDrvData->dmemBaseAddr = MXL_MOCA_PCIE_HIF_TC1_DMEM_BASE;
      }
      else
      {
        MxL_MoCA_PCIeDbgTrace(MXL_MOCA_PCIE_L_ERR, "Device unknown!\n");
        pDrvData->dmemBaseAddr = 0;
      }
    }

    // A few debug prints   
    // Let us read a few config registers here for debug.
    pci_read_config_dword(pdev, PCI_VENDOR_ID, &val);
    MXL_MOCA_DBG2P1("Config rd(0): 0x%08x\n", val);
    pci_read_config_word(pdev, PCI_COMMAND, &valShort);
    MXL_MOCA_DBG2P1("Config rd(4): 0x%08x\n", valShort);
    val |= 0x400;
    pci_write_config_word(pdev, PCI_COMMAND, valShort);
    pci_read_config_word(pdev, PCI_COMMAND, &valShort);
    MXL_MOCA_DBG2P1("Config rd(4) - disable INT_A: 0x%08x\n", val);

    // Read the BAR0 size.
    pci_read_config_dword(pdev, PCI_BASE_ADDRESS_0, &bar0);
    MXL_MOCA_DBG2P1("Config rd(0x10) BAR0: 0x%08x\n", bar0);
    if (0 == bar0)
    {
      MxL_MoCA_PCIeDbgTrace(MXL_MOCA_PCIE_L_ERR, "BAR0 value(0x%x) is invalid!\n", bar0);     
    }        
    pci_write_config_dword(pdev, PCI_BASE_ADDRESS_0, 0xFFFFFFFF); 
    pci_read_config_dword(pdev, PCI_BASE_ADDRESS_0, &val);
    MXL_MOCA_DBG2P1("Config rd(0x10) BAR0 size = 0x%08x\n", (~(val & 0xFFFFFFF0)) + 1 );
    pci_write_config_dword(pdev, PCI_BASE_ADDRESS_0, bar0); 
        
    // Read the BAR1 size.
    pci_read_config_dword(pdev, PCI_BASE_ADDRESS_1, &bar1);
    MXL_MOCA_DBG2P1("Config rd(0x10) BAR1: 0x%08x\n", bar1);  
    if (0 == bar1)
    {
      MxL_MoCA_PCIeDbgTrace(MXL_MOCA_PCIE_L_ERR, "BAR1 value(0x%x) is invalid!\n", bar1);
    }
    pci_write_config_dword(pdev, PCI_BASE_ADDRESS_1, 0xFFFFFFFF); 
    pci_read_config_dword(pdev, PCI_BASE_ADDRESS_1, &val);
    MXL_MOCA_DBG2P1("Config rd(0x10) BAR1 size = 0x%08x\n", (~(val & 0xFFFFFFF0)) + 1 );
    pci_write_config_dword(pdev, PCI_BASE_ADDRESS_1, bar1); 

    for (index = 0; index < MXL_MOCA_PCIE_MAX_NUM_OF_PCIE_DEVICES; index++)
    {
      if (gpDevData[index] == NULL)
      {
        pDrvData->devIndex = index;
        gpDevData[index] = pDrvData;
        break;
      }
    }

    index++;
    if (index > gNumMocaPcieDevices)
        gNumMocaPcieDevices = index;

    MxL_MoCA_PCIeDbgTrace(MXL_MOCA_PCIE_L_DBG1, "MoCA PCIe Device 0x%02x\n", gNumMocaPcieDevices);

    /// create dcp contexts
    pDdc = pDrvData;
    MxL_MoCA_PCIeLockInit(&pDdc->rwLock); 
    pDdc->busy = 0;

    /* Program Inbound ATU */
    if (MXL_MOCA_OK != MxL_MoCA_PCIeConfigInboundATU(0))
    {
      MxL_MoCA_PCIeDbgTrace(MXL_MOCA_PCIE_L_ERR, "Inbound ATU not configured\n");
      errorOut = MXL_MOCA_PCIE_ERROR_OUT_FREE_NETDEV;
      break;          
    }

    /* Inbound ATU programmed: Host can access SoC memory */
    ret = MxL_MoCA_PCIeGetSocName(pDrvData->devIndex, &pChipName);
    if (MXL_MOCA_ERR == ret)
    {
      MxL_MoCA_PCIeDbgTrace(MXL_MOCA_PCIE_L_ERR, "Cannot read from chip\n");
      errorOut = MXL_MOCA_PCIE_ERROR_OUT_FREE_NETDEV;
      break;          
    }
    else 
    {
      MxL_MoCA_PCIeDbgTrace(MXL_MOCA_PCIE_L_INFO, "MoCA SoC: %s\n", pChipName);        
    }
    
    // -------------------------------- MSI/INTA Section ----------------------------------------
    // NOTE: MSI is the preferred inband interrupt type. There are several advantages such as interupt not being shared, 
    // Single TLP is sent from EP to RC for interrupt. With INTx, there are 3: ASSERT, DEASSERT_REQ(from host), DEASSERT TLP.        
    pDdc->inbandIntrType = PCIE_INTR_TYPE_MSI;

    // { Command line options(intr_sel_type=): PCIE_INTR_SEL_AUTO=0; PCIE_INTR_SEL_MSI=1; PCIE_INTR_SEL_INTA=2 }  
    // If MSI has not been disabled by the command-line option pci=nomsi or CONFIG_PCI_MSI is defined and enabled,
    // pci_msi_enabled() will return TRUE. MSI section is also entered, if command line param = PCIE_INTR_SEL_MSI
    // If PCIE_INTR_SEL_AUTO, and the host device is capable of MSI and INTx, MSI has preference and driver will 
    // configure endpoint (EP) for MSI.
    // Run: modinfo mxl_moca_pcie.ko to print command line options.

    // If system is capable of both MSI and INTx, specify PCIE_INTR_SEL_INTA in cmd line to prefer INTx over MSI for interrupts.
    // If system is NOT capable of MSI, make sure the kernel is built without CONFIG_PCI_MSI option. 
    // When CONFIG_PCI_MSI is not defined by kernel, 
    // specifying either PCIE_INTR_SEL_AUTO or PCIE_INTR_SEL_INTA in cmd line will configure the EP for INTx.

    if ( (PCIE_INTR_SEL_MSI == intr_sel_type || pci_msi_enabled()) && 
          (PCIE_INTR_SEL_INTA != intr_sel_type) ) 
    {
      // MSI section
      if ((pos = pci_find_capability(pdev, PCI_CAP_ID_MSI)) != 0) // MSI capable
      {
        pci_read_config_word(pdev, pos + PCI_MSI_FLAGS, &control);
        MXL_MOCA_DBG1P1("PCI_MSI_FLAGS = %x\n", control);
        if ((control & PCI_MSI_FLAGS_ENABLE) != 1);
        {
          MXL_MOCA_DBG1P0("setting PCI_MSI_FLAGS_ENABLE\n");
          control |= PCI_MSI_FLAGS_ENABLE;
        }
        pci_write_config_word(pdev, pos + PCI_MSI_FLAGS, control);  
        MXL_MOCA_DBG1P1("pos + PCI_MSI_FLAGS = %x\n", pos+PCI_MSI_FLAGS);

        // verify
        pci_read_config_word(pdev, pos + PCI_MSI_FLAGS, &control);
        MXL_MOCA_DBG1P1("PCI_MSI_FLAGS = %x\n", control);

        if (!pci_enable_msi(pdev)) // allocates ONE interrupt to the device
        {                
          // Register interrupt handler
          MXL_MOCA_DBG1P2("%s: IRQ: %i\n", __FUNCTION__, pdev->irq);
          pNetDev->irq = pdev->irq;
          err = request_irq(pdev->irq, MxL_MoCA_PCIeISR, IRQF_SHARED, pNetDev->name, pNetDev); 
          
          if (err)
          {
            MxL_MoCA_PCIeDbgTrace(MXL_MOCA_PCIE_L_ERR, "MSI: IRQ request failed!\n");
          }
          else
          {
            pci_intx(pdev, 0); // lspci:DisINTx+
            pDdc->inbandIntrType = PCIE_INTR_TYPE_MSI;
            MxL_MoCA_PCIeDbgTrace(MXL_MOCA_PCIE_L_INFO, "Configuring interrupt(MSI)...OK\n");
          }
        }
        else
        {
          MxL_MoCA_PCIeDbgTrace(MXL_MOCA_PCIE_L_INFO, "Configuring interrupt(MSI)...failed!\n");          
        }

        pci_read_config_dword(pdev, pos + PCI_MSI_ADDRESS_LO, &configValue); 
        MXL_MOCA_DBG2P1("pos + PCI_MSI_ADDRESS_LO val = %x\n", configValue); 
        MXL_MOCA_DBG2P1("pos + PCI_MSI_DATA_32 = %x\n", pos+PCI_MSI_DATA_64);
      }
    }  
    else if ( (PCIE_INTR_SEL_INTA == intr_sel_type || (!pci_msi_enabled())) && 
               (PCIE_INTR_SEL_MSI != intr_sel_type) ) 
    { 
      // INTA section
      // Kernel not built with CONFIG_PCI_MSI (pci_msi_enabled will return 0) or user specified to use INTA only
      // INTA section is also entered if command line param was specified with PCIE_INTR_SEL_INTA
      pci_disable_msi(pdev); // Resets cfg_msi_en in EHI reg space for SoC FW. lspci: Capability::MSI: Enable-
      pci_intx(pdev, 1);    // Enable INTx. lspci:DisINTx-

      // Configure for INTx     
      // Register interrupt handler
      MXL_MOCA_DBG1P2("%s: IRQ: %i\n", __FUNCTION__, pdev->irq);    
      pNetDev->irq = pdev->irq;
      // Uses pre-assigned IOAPIC
      err = request_irq(pdev->irq, MxL_MoCA_PCIeISR, IRQF_SHARED, pNetDev->name, pNetDev);    
      if (err)
      {
        MxL_MoCA_PCIeDbgTrace(MXL_MOCA_PCIE_L_ERR, "INT_A: IRQ request failed!\n");
      } 
      else
      {      
        pDdc->inbandIntrType = PCIE_INTR_TYPE_INTA;
        MxL_MoCA_PCIeDbgTrace(MXL_MOCA_PCIE_L_INFO, "Configuring interrupt(INT_A)...OK\n");          
      }
    }  
    else
    {
      MxL_MoCA_PCIeDbgTrace(MXL_MOCA_PCIE_L_ERR, "Interrupts are not configured!!!");
    }

    // -------------------------------------------------------------------------------------
    // Clear TX/RX ring buffers
    for (index = 0; index < MXL_MOCA_PCIE_TX_MAPPING_SIZE; index++)
    {
      pDdc->txRing[index].skb = NULL;
    }

    pDdc->txRingWrIndex = 0;
    pDdc->txRingRdIndex = 0;

    for (index = 0; index < MXL_MOCA_PCIE_RX_RING_SIZE; index++)
    {
      pDdc->rxRing[index].skb = NULL;
    }
  
    // -------------------------------- PCIe capabilities ----------------------------------
    // PCIe express capabilities
    MXL_MOCA_DBG1P0("-- PCI Express cap/ctl/sta--\n");
    
    if ((pos = pci_find_capability(pdev, PCI_CAP_ID_EXP)) != 0) 
    {
      /* Read Device MaxPayload capability and setting */
      pci_read_config_word(pdev, pos + PCI_EXP_DEVCTL, &control);
      pci_read_config_word(pdev, pos + PCI_EXP_DEVCAP, &capability);
      /*    
        PCI_EXP_DEVCTL_READRQ_128B  0x0000 // 128 Bytes 
        PCI_EXP_DEVCTL_READRQ_256B  0x1000 // 256 Bytes 
        PCI_EXP_DEVCTL_READRQ_512B  0x2000 // 512 Bytes 
        PCI_EXP_DEVCTL_READRQ_1024B 0x3000 // 1024 Bytes 
      */  
      pDdc->devCapMaxPayloadSz  = 1 << ( (capability & PCI_EXP_DEVCAP_PAYLOAD) + 7 );
      MXL_MOCA_DBG1P2("PCI_EXP_DEVCAP[0x%x]MaxPayLoadSize = %d\n", capability, pDdc->devCapMaxPayloadSz);
      
      pDdc->devCtlMaxPayloadSz  = 1 << ( ((control & PCI_EXP_DEVCTL_PAYLOAD)>> 5)  + 7 );
      pDdc->devCtlMaxReadReqSz = 1 << ( ((control & PCI_EXP_DEVCTL_READRQ) >> 12) + 7 );
      MXL_MOCA_DBG1P3("PCI_EXP_DEVCTL[0x%x]MaxPayLoadSize = %d, MaxReadReq = %d\n", \
                       control, pDdc->devCtlMaxPayloadSz, pDdc->devCtlMaxReadReqSz);
      /*
        PCI_EXP_LNKCAP_SLS_2_5GB 0x00000001 // LNKCAP2 SLS Vector bit 0 
        PCI_EXP_LNKCAP_SLS_5_0GB 0x00000002 // LNKCAP2 SLS Vector bit 1 
      */
      pci_read_config_word(pdev, pos +  PCI_EXP_LNKCTL, &control);
      pci_read_config_word(pdev, pos +  PCI_EXP_LNKCAP, &capability);
      /* Supported Link Speeds, Maximum Link Width 
       * PCI_EXP_LNKCAP_SLS_2_5GB 0x00000001 // LNKCAP2 SLS Vector bit 0 
       * PCI_EXP_LNKCAP_SLS_5_0GB 0x00000002 // LNKCAP2 SLS Vector bit 1 
      */
      pDdc->lnkCapMaxLnkSpeed  = (capability & PCI_EXP_LNKCAP_SLS); // SLS = Supported Link Speed       
      pDdc->lnkCapMaxLinkWidth = (capability & PCI_EXP_LNKCAP_MLW) >> 4;  

      if (pDdc->lnkCapMaxLnkSpeed > 3)
      {
        pDdc->lnkCapMaxLnkSpeed = 0;
      }

      MXL_MOCA_DBG1P3("PCI_EXP_LNKCAP[0x%x]MaxLinkSpeed(1=2.5GT/s, 2=5GT/s) = %d, MaxLinkWidth = x%d\n", \
          capability, pDdc->lnkCapMaxLnkSpeed, pDdc->lnkCapMaxLinkWidth);
      // Read Completion Boundary 
      pDdc->lnkCtlRCB = 64 * ( ((control & PCI_EXP_LNKCTL_RCB) >> 3) + 1 );  
      MXL_MOCA_DBG1P2("PCI_EXP_LNKCTL[0x%x]RCB = %d\n", control, pDdc->lnkCtlRCB);
      
      pci_read_config_word(pdev, pos +  PCI_EXP_LNKSTA, &status);
      
      // Current Link Speed, Negotiated Link Width
      pDdc->lnkStaMaxLnkSpeed  = (status & PCI_EXP_LNKSTA_CLS);    // CLS = Current Link Speed
      pDdc->lnkStaMaxLinkWidth = (status & PCI_EXP_LNKSTA_NLW) >> 4;    

      if (pDdc->lnkStaMaxLnkSpeed > 3)
      {
        pDdc->lnkStaMaxLnkSpeed = 0;
      }
      
      MxL_MoCA_PCIeDbgTrace(MXL_MOCA_PCIE_L_INFO, "Supported Link Speed : %s, Current Link Speed : %s\n", pLinkSpeed[pDdc->lnkCapMaxLnkSpeed], pLinkSpeed[pDdc->lnkStaMaxLnkSpeed]);
      
      MXL_MOCA_DBG1P3("PCI_EXP_LNKSTA[0x%x]MaxLinkSpeed(1=2.5GT/s, 2=5GT/s, 3=8GT/s) = %d, MaxLinkWidth = x%d\n", \
          capability, pDdc->lnkStaMaxLnkSpeed, pDdc->lnkStaMaxLinkWidth);
    }
    pDrvData->pPCIeDev = pdev;
#if MXL_MOCA_PCIE_PWR_MGMT  
    MxL_MoCA_PCIePowerManagementInit(gNumMocaPcieDevices-1);
#endif
    if (pci_enable_pcie_error_reporting(pdev) == 0)
    {
      MxL_MoCA_PCIeDbgTrace(MXL_MOCA_PCIE_L_INFO,"Advanced Error Reporting (AER) enable...OK\n");
    }
    else
    {
      MxL_MoCA_PCIeDbgTrace(MXL_MOCA_PCIE_L_ERR, "Advanced Error Reporting (AER) enable...failed!\n");
    }
  
    // -------------------------------------------------------------------------------------
    /// initialize default handlers
    ether_setup(pNetDev);

    /// Initialize net device for Ethernet
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 29)
    pNetDev->netdev_ops = &gMxlMocaPcieNetDevOps;
#else
    pNetDev->open               = MxL_MoCA_PCIeNetDevOpen; 
    pNetDev->stop               = MxL_MoCA_PCIeNetDevClose; 
    pNetDev->hard_start_xmit    = MxL_MoCA_PCIeNetDevStartXmit; 
    pNetDev->do_ioctl           = 0;     
    pNetDev->get_stats          = MxL_MoCA_PCIeNetDevGetStats;
    pNetDev->set_multicast_list = MxL_MoCA_PCIeNetDevSetRxMode;
    pNetDev->tx_timeout         = MxL_MoCA_PCIeNetDevTxTimeout;
#endif
    pNetDev->watchdog_timeo     = MXL_MOCA_PCIE_TX_TIMEOUT;
    pNetDev->tx_queue_len       = MXL_MOCA_PCIE_TX_MAPPING_SIZE; /* reflects in ifconfig */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 4)
    MxL_MoCA_PCIeSetEthToolVar(pNetDev); 
#endif

#if MXL_MOCA_PCIE_NAPI
    MxL_MoCA_PCIeDbgTrace(MXL_MOCA_PCIE_L_INFO, "Config NAPI...\n");
    netif_napi_add(pNetDev, &pDrvData->napi, MxL_MoCA_PCIeNapiPoll, MXL_MOCA_PCIE_MAX_NAPI_WEIGHT);
    napi_enable(&pDdc->napi);
#endif

    for (val = 0; val < 6; val++)
    {
      int8_t c1 = mac_addr[val*3];
      int8_t c2 = mac_addr[val*3+1];

      if (val != 5 && mac_addr[val*3+2] != ':') break;

      if (!isxdigit(c1) || !isxdigit(c2)) break;

      c1 = isdigit(c1) ? c1 - '0' : tolower(c1) - 'a' + 10;
      c2 = isdigit(c2) ? c2 - '0' : tolower(c2) - 'a' + 10;
      myMacAddr[val] = (uint8_t) c1 * 16 + (uint8_t) c2;
    }
    // 00:20:30:40:50:60
    //  tmp[0] = 0x00203040;
    //  tmp[1] = 0x50600000;  
    tmp[0] =  (uint32_t)myMacAddr[0];
    tmp[0] <<= 8;
    tmp[0] |= (uint32_t)myMacAddr[1];
    tmp[0] <<= 8;
    tmp[0] |= (uint32_t)myMacAddr[2];
    tmp[0] <<= 8;
    tmp[0] |= (uint32_t)myMacAddr[3];
    //--
    tmp[1] =  (uint32_t)myMacAddr[4];
    tmp[1] <<= 8;
    tmp[1] |= (uint32_t)myMacAddr[5];
    tmp[1] <<= 16;

    MxL_MoCA_PCIeSetMgmtMAC(0, tmp);

    pci_set_master(pdev);
    /// Register Linux net device
    /// Note: MxL_MoCA_PCIeNetDevGetStats (and possibly others) is called from inside register_netdev
    if (register_netdev(pNetDev)) 
    {
      MxL_MoCA_PCIeDbgTrace(MXL_MOCA_PCIE_L_ERR, "register_netdev failed\n");
      errorOut = MXL_MOCA_PCIE_ERROR_OUT_FREE_UNMAP;
      break;        
    }

    MxL_MoCA_PCIeDbgTrace(MXL_MOCA_PCIE_L_INFO, "Registering network device(%s)...OK\n", pNetDev->name);    
    
    seq = simple_strtoul(pNetDev->name+2,0, 10);

    MxL_MoCA_PCIeDbgTrace(MXL_MOCA_PCIE_L_DBG1, "mac_addr: %02x %02x %02x %02x %02x %02x\n",
                          pNetDev->dev_addr[0], pNetDev->dev_addr[1], pNetDev->dev_addr[2],
                          pNetDev->dev_addr[3], pNetDev->dev_addr[4], pNetDev->dev_addr[5]);

    pDrvData->netIfRxDropped = 0;
    pDrvData->txIntrCount    = 0;
    pDrvData->readProcMode   = 0;
#if MXL_MOCA_PCIE_DEBUG_DROP_RX    
    pDrvData->dbg_drop_rx    = 0;
#endif
#if MXL_MOCA_PCIE_DEBUG_DROP_TX
    pDrvData->dbg_drop_tx    = 0;
#endif

    pDrvData->skbAllocErr    = 0;

#if MXL_MOCA_PCIE_NAPI  
    pDrvData->napiDisableIntr = 0;
#endif

    if (gNumMocaPcieDevices > 0)
    {
      MxL_MoCA_PCIeFormatString(pDrvData->procDirName,
                                MXL_MOCA_PCIE_PROC_NAME_LEN_MAX,
                                MXL_MOCA_PCIE_PROC_DIR_NAME,
                                gNumMocaPcieDevices-1); // en%d
     
      pDrvData->pProcDir = proc_mkdir(pDrvData->procDirName, NULL);

      if (pDrvData->pProcDir)
      {
#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 10, 0)
        pProcEntry = create_proc_entry(MXL_MOCA_PCIE_PROC_ENTRY_NAME, (S_IFREG|S_IRUGO|S_IWUGO), pDrvData->pProcDir);
        if (pProcEntry != NULL)
        {
          pProcEntry->read_proc  = MxL_MoCA_PCIeReadProc;
          pProcEntry->write_proc = MxL_MoCA_PCIeWriteProc; 
        }
#else 
        pProcEntry = proc_create(MXL_MOCA_PCIE_PROC_ENTRY_NAME, (S_IFREG|S_IRUGO|S_IWUGO), pDrvData->pProcDir, &gMxlMocaProcFops);
#endif
        if (pProcEntry == NULL)
        {
          MxL_MoCA_PCIeDbgTrace(MXL_MOCA_PCIE_L_ERR, "Creating proc failed!\n");
        }
        else
        {
          MxL_MoCA_PCIeDbgTrace(MXL_MOCA_PCIE_L_INFO, "Creating proc(directory: %s, entry: %s)...OK\n", pDrvData->procDirName, MXL_MOCA_PCIE_PROC_ENTRY_NAME);
        }
      }
      else
      {
        MxL_MoCA_PCIeDbgTrace(MXL_MOCA_PCIE_L_ERR, "proc_mkdir failed!\n");
      }
    }
  
    pDdc->pNetDev = pNetDev; 
    
    if (MxL_MoCA_PCIeDataPathInit(pDdc, gNumMocaPcieDevices-1) != MXL_MOCA_OK) 
    { 
      MxL_MoCA_PCIeDbgTrace(MXL_MOCA_PCIE_L_ERR, "Data path init failed\n");
      errorOut = MXL_MOCA_PCIE_ERROR_OUT_FREE_ALLOC;
      break;       
    }     
    /* Outbound ATU configured: SoC can access host memory via PCIe link for DMA transfers */
    printk("%s: PCIe driver init done!\n\n", MXL_MOCA_PCIE_DRV_NAME);
  }while (0);

  // Handle error conditions before returning
  switch (errorOut)
  {
    case MXL_MOCA_PCIE_ERROR_OUT_NONE:
      return MXL_MOCA_OK;
      
    case MXL_MOCA_PCIE_ERROR_OUT_FREE_ALLOC:
      MXL_MOCA_DBG2P0( "err_out_free_alloc\n");
      pDrvData = NETDEV_PRIV(pNetDev);      
      /* Free any allocations here */
      // fall-through
    
    case MXL_MOCA_PCIE_ERROR_OUT_FREE_UNMAP:
      // unmap/un-register
      MXL_MOCA_DBG2P0( "err_out_free_umap\n");
      /// Unregister Linux net device
      free_irq(pdev->irq, pNetDev);
      pci_disable_msi(pdev); 
      unregister_netdev(pNetDev);
      
      // Free the net device
      pci_disable_device(pdev);
      
      // Release reserved PCI I/O and memory resources
      pci_release_regions(pdev);
      
      iounmap((void*)pDrvData->baseIoRemap0);
      iounmap((void*)pDrvData->baseIoRemap1);      
      // fall-through
    
    case MXL_MOCA_PCIE_ERROR_OUT_FREE_NETDEV:
      // release the net device and data
      MXL_MOCA_DBG2P0( "err_out_free_netdev\n");
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 0)
      /// pNetDev is not the allocation address anymore, can't use kfree() directly.
      free_netdev(pNetDev);
#else
      kfree(pNetDev);
#endif
      pci_set_drvdata(pdev, NULL);    
      break;
      
    default:
      break;
  }

  return -ENODEV;
}

/*----------------------------------------------------------------------------------------
 * 
 * FUNCTION NAME : MxL_MoCA_PCIeRemoveOne
 *  
 *
 *
 * DATE CREATED  : 03/25/2016
 * 
 * DESCRIPTION   : Remove or uninstall the driver.
 *
 * IN PARAMS     : [pdev] : Pointer to the PCI device.  
 *
 * OUT PARAMS    : None
 *
 * RETURN VALUE  : return 0 if successful, <0 (Linux error code) if error
 *
 *--------------------------------------------------------------------------------------*/
 
static void __devexit MxL_MoCA_PCIeRemoveOne(struct pci_dev *pdev)
{
  int8_t i;
  struct net_device *pNetDev = pci_get_drvdata(pdev);

  MXL_MOCA_PCIE_DEV_DATA_CONTEXT_T *pDdc = NETDEV_PRIV(pNetDev);

  MXL_MOCA_DBG1P2("%s: pdev: 0x%p\n", __FUNCTION__, pdev);

#if MXL_MOCA_PCIE_NAPI
  napi_disable(&pDdc->napi);
#endif

  if (pDdc->pProcDir)
  {
    MxL_MoCA_PCIeDbgTrace(MXL_MOCA_PCIE_L_INFO, "Removing proc entry\n");
    remove_proc_entry(MXL_MOCA_PCIE_PROC_ENTRY_NAME, pDdc->pProcDir);
    remove_proc_entry(pDdc->procDirName, NULL);
#if LINUX_VERSION_CODE > KERNEL_VERSION(3, 10, 0)    
    proc_remove(pDdc->pProcDir);
#endif
    pDdc->pProcDir = NULL;
  }

  for (i = 0; i < gNumMocaPcieDevices; i++)
  {
    if (gpDevData[i] == pDdc)
    {
      gpDevData[i] = NULL;
      MXL_MOCA_DBG1P1("Removed MoCA-Ethernet board #%d.\n", i);

      break;
    }
  }

  /// Get the net device from the PCI device structure
  if (!pNetDev)
  {
    MxL_MoCA_PCIeDbgTrace(MXL_MOCA_PCIE_L_ERR, "Net device is NULL\n");
    return;
  }

  MXL_MOCA_DBG2P0( "Un-register IRQ handler\n");

  /// Unregister interrupt handler
  free_irq(pdev->irq, pNetDev);
  //MXL_MOCA_DBG2P0( "done\n");

  // Disable MSI
  pci_disable_msi(pdev); // moved up // kernel BUG at /build/buildd/linux-2.6.31/drivers/pci/msi.c:668!
  MXL_MOCA_DBG2P0( "pci_disable_msi done\n");
  
  /// Unregister Linux net device
  unregister_netdev(pNetDev);
  MXL_MOCA_DBG2P0( "unregister_netdev done\n");
  
  /// Free the net device   
  pci_disable_device(pdev);
  MXL_MOCA_DBG2P0( "pci_disable_device done\n");
  
  iounmap((void *)pDdc->baseIoRemap0);
  iounmap((void *)pDdc->baseIoRemap1);
  MXL_MOCA_DBG2P0( "iounmap done\n"); 
  
  pci_release_regions(pdev);
  MXL_MOCA_DBG2P0( "pci_release_regions done\n");
  
  /// Terminate the Common Ethernet Module
  MxL_MoCA_PCIeTerminate(pDdc); 

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 0)
  /// pNetDev is not the allocation address anymore, can't use kfree() directly.
  free_netdev(pNetDev);
  MXL_MOCA_DBG2P0( "free_netdev done\n");
#else
  kfree(pNetDev);
  MXL_MOCA_DBG2P0( "kfree done\n");
#endif

 MxL_MoCA_PCIeDbgTrace(MXL_MOCA_PCIE_L_INFO, "Done.\n\n");

 //////   pci_set_drvdata(pdev, NULL);
}

/*----------------------------------------------------------------------------------------
 * 
 * FUNCTION NAME : MxL_MoCA_PCIeDumpQueues
 *  
 *
 *
 * DATE CREATED  : 03/25/2016
 * 
 * DESCRIPTION   : Dumps the rx or tx descriptor wueues. Useful for debugging.
 *
 * IN PARAMS     : [idx]  : device index. 
 *                 [type] : 0 = rx only; 1 = tx only; 2 = rx and tx .
 *
 * OUT PARAMS    : None
 *
 * RETURN VALUE  : None
 *
 *--------------------------------------------------------------------------------------*/
 
static void MxL_MoCA_PCIeDumpQueues(unsigned int idx, unsigned int type) 
{
  int16_t i;
  MXL_MOCA_PCIE_DATAPATH_CONTEXT_T*  pDataPath;
  MXL_MOCA_PCIE_QUEUE_PAIR_T* queuePair;
  MXL_MOCA_PCIE_DEV_DATA_CONTEXT_T*   pDdc;

  pDdc = gpDevData[idx];
  pDataPath  = (MXL_MOCA_PCIE_DATAPATH_CONTEXT_T *)pDdc->pDpCtx;

  if (type == 0 || type == 2)
  {
    printk("============================ RX Queue ======================\n");
    queuePair = &pDataPath->rxQueuePair;

    printk("ridx = %d, widx = %d, lastPktsCnt = %d\n",
            queuePair->descQueueReadIdx,
            queuePair->descQueueWriteIdx,
            queuePair->lastPktsCnt);

    for (i = 0; i < (queuePair->descQueueSize); i++)
    {
        printk("[%04d]: w0=0x%x w1=0x%x plen=0x%x\n", i, (queuePair->pDescQueue[i].dword0 & 0x03), 
                queuePair->pDescQueue[i].dword1,
                queuePair->pPktQueue[i].totalLen);

    }
  }
  
  if (type == 1 || type == 2)
  {
    printk("============================ TX Queue ======================\n");
    queuePair = &pDataPath->txQueuePair;

    printk("ridx = %d, widx = %d, lastPktsCnt = %d\n",
            queuePair->descQueueReadIdx,
            queuePair->descQueueWriteIdx,
            queuePair->lastPktsCnt);
        
    for (i = 0; i < (queuePair->descQueueSize); i++) // descriptors are two dwords
    {
      printk("[%04d]: w0=0x%x w1=0x%x plen=0x%x\n", i, (queuePair->pDescQueue[i].dword0 & 0x03), 
              queuePair->pDescQueue[i].dword1,
              queuePair->pPktQueue[i].totalLen);

    }
  }
}

/*----------------------------------------------------------------------------------------
 * 
 * FUNCTION NAME : MxL_MoCA_PCIeWriteProc
 *  
 *
 *
 * DATE CREATED  : 03/25/2016
 * 
 * DESCRIPTION   : Proc Read operations.
 *  Proc read/write commands (usage from user space):
 *
 *  DEBUG:
 *  ======
 *  0)
 *  > echo 'h' > /proc/en%d/pcie
 *  Display usage
 *
 *  1) 
 *  > echo 'd 0' > /proc/en%d/pcie
 *  Revert back to displaying tx and rx counters
 *  > cat /proc/en%d/pcie
 *  Display the tx/rx counters
 *  
 *  2) 
 *  > echo 'd 1' > /proc/en%d/pcie
 *  To clear rx counters
 *  > cat /proc/en%d/pcie
 *  Display the tx/rx counters (rx counts are reset)
 *
 *  3)
 *  > echo 'd 2' > /proc/en%d/pcie
 *  To clear tx counters
 *  > cat /proc/en%d/pcie
 *  Display the tx/rx counters (tx counts are reset)
 *
 *  4)
 *  > echo 'd 3' > /proc/en%d/pcie
 *  To clear tx and rx counters
 *  > cat /proc/en%d/pcie
 *  Display the tx/rx counters (tx and rx counts are reset)
 *
 *  5) 
 *  > echo 'd 4' > /proc/en%d/pcie
 *  To output Rx descr queue info.
 *  > cat /proc/en%d/pcie
 *  > dmesg
 *  Displays the Rx descr queue 
 *
 *  6) 
 *  > echo 'd 5' > /proc/en%d/pcie
 *  To output Tx descr queue info.
 *  > cat /proc/en%d/pcie
 *  > dmesg
 *  Displays the Tx descr queue
 *
 *  7) 
 *  > echo 'd 6' > /proc/en%d/pcie
 *  To displays the DMEM fixed locations
 *  > cat /proc/en%d/pcie
 *  > dmesg
 *  Displays the DMEM fixed locations (Host-to-SoC DMEM interface)
 *
 *  8) 
 *  > echo 'd 7' > /proc/en%d/pcie
 *  Output outbound (EP to RC) iATU configurations
 *  > cat /proc/en%d/pcie
 *  > dmesg
 *  Displays the outbound iATU configurations
 *
 *  << values 8 and 9 are reserved >>
 *
 *  INFO:
 *  =====
 *  1)
 *  > echo 'i 1' > /proc/en%d/pcie
 *  Display EP info.
 *  > dmesg
 *  Displays the capabilities and key config info (vendor/device ID). 
 *
 *
 *--------------------------------------------------------------------------------------*/

#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 10, 0)
static int MxL_MoCA_PCIeReadProc(char *pUserBuf, char **ppStart, off_t offset, int count, int *pEof, void *data) // old kernel
{
#else
static ssize_t MxL_MoCA_PCIeReadProc(struct file *pFile, char __user *pPage,  size_t len, loff_t* pOffset)
{
#endif
  int8_t idx, loop;
  uint32_t regions;
  unsigned int*    p;
  MXL_MOCA_PCIE_DATAPATH_CONTEXT_T* pDataPath;
  MXL_MOCA_PCIE_QUEUE_PAIR_T* queuePair;
  MXL_MOCA_PCIE_DEV_DATA_CONTEXT_T*   pDdc;
  uint32_t val = 0;
  int16_t  num = 0;
#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 10, 0)  
#else
  char* pUserBuf = gProcFsBuffer;
#endif
    
#if MXL_MOCA_PCIE_NAPI
  const int8_t intrModeStr[] = "[NAPI]";
  const int8_t intrHandlerStr[] = "[NAPI Polls]";
#else
  const int8_t intrModeStr[] = "      "; // non-NAPI
  const int8_t intrHandlerStr[] = "Interrupts]";
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0)
  static int finished = 0;
#endif

  idx=0;
  for (idx = 0; idx < gNumMocaPcieDevices; idx++)
  {
    pDdc = gpDevData[idx];
    pDataPath  = (MXL_MOCA_PCIE_DATAPATH_CONTEXT_T *)pDdc->pDpCtx;   

    switch (pDdc->readProcMode)
    {
      case 0:
      case 1:
      case 2:
      case 3:
        {
          p = (unsigned int*) &pDdc->rxStats;

          num += sprintf(pUserBuf + num, "\n                  === ingress ==>\n");          
          num += sprintf(pUserBuf + num, "|--------|       |----------------|        |-----------|\n");
          num += sprintf(pUserBuf + num, "|        |------>|RX    Host    TX|------->|           |\n");
          num += sprintf(pUserBuf + num, "| Upper  |       |    MxL PCIe    |  PCIe  |  MxL37xx  |\n");
          num += sprintf(pUserBuf + num, "| layers |       |     EP Drv     |  Link  | (PCIe EP) | <=Coax=>\n");
          num += sprintf(pUserBuf + num, "|        |<------|TX            RX|<-------|           |\n");          
          num += sprintf(pUserBuf + num, "|--------|       |----------------|        |-----------|\n");   
          num += sprintf(pUserBuf + num, "                  <== egress ====\n\n");          
          num += sprintf(pUserBuf + num, "RX Stats:-----------------------------\n");
          num += sprintf(pUserBuf + num, "%sRX(egress)  Pkts Good   : %u\n%sTX(egress)  Pkts Good   : %u\n%sTX(egress)  Pkts Dropped: %u\n", 
                          intrModeStr, p[0], "      ", p[1], "      ", p[2]);

          num += sprintf(pUserBuf + num, "TX Stats:-----------------------------\n");

          
          p = (unsigned int*) &pDdc->txStats;
          val = 0;
          MxL_MoCA_PCIeRead (idx, pDdc->dmemBaseAddr + (PCIE_TX_NUM_PKTS * 4), &val);
          num += sprintf(pUserBuf + num, "%sRX(ingress) Pkts Good   : %u\n%sTX(ingress) Pkts Good   : %u\n%sTX(ingress) Pkts Dropped: %u\n", "      ", p[0], "      ", val, "      ", p[2]);
          num += sprintf(pUserBuf + num, "--------------------------------------\n");                              
#if MXL_MOCA_PCIE_NAPI
          num += sprintf(pUserBuf + num, "%s: %u \n", intrHandlerStr, pDdc->polls);
          num += sprintf(pUserBuf + num, "%s: %u \n", (pDdc->inbandIntrType == PCIE_INTR_TYPE_MSI)?"[MSI]":"[INTA]", pDdc->interrupts);
#else
          num += sprintf(pUserBuf + num, "%s%s: %u \n", (pDdc->inbandIntrType == PCIE_INTR_TYPE_MSI)?"[MSI/":"[INTA/", intrHandlerStr, pDdc->interrupts);
#endif
          queuePair = &pDataPath->rxQueuePair;
          num += sprintf(pUserBuf + num, "RX (ridx, widx) = (%d, %d) \n", 
                          queuePair->descQueueReadIdx, queuePair->descQueueWriteIdx);

          queuePair = &pDataPath->txQueuePair;
          num += sprintf(pUserBuf + num, "TX (ridx, widx) = (%d, %d) \n", 
                          queuePair->descQueueReadIdx, queuePair->descQueueWriteIdx);
          num += sprintf(pUserBuf + num, "Mapping errors: Rx(%u), Q(%u), Tx(%u). skb alloc errors: %u\n", 
                          pDdc->mappingErr0, pDdc->mappingErr1, pDdc->mappingErr2, pDdc->skbAllocErr);
        }
        break;
        
      case 4:
        MxL_MoCA_PCIeDumpQueues(idx, 0);
        break;
        
      case 5:
        MxL_MoCA_PCIeDumpQueues(idx, 1);
        break;      

      case 6:
      {
        MxL_MoCA_PCIeRead (idx, pDdc->dmemBaseAddr + (P_PCIE_HOST_RX_DESC_QUEUE * 4), &val);
        num += sprintf(pUserBuf + num, "P_PCIE_HOST_RX_DESC_QUEUE[0x%x](0x%x)=0x%x\n", P_PCIE_HOST_RX_DESC_QUEUE, pDdc->dmemBaseAddr + (P_PCIE_HOST_RX_DESC_QUEUE * 4), val);

        MxL_MoCA_PCIeRead (idx, pDdc->dmemBaseAddr + (P_PCIE_HOST_RX_DESC_Q_LEN * 4), &val);
        num += sprintf(pUserBuf + num, "P_PCIE_HOST_RX_DESC_Q_LEN[0x%x](0x%x)=0x%x\n", P_PCIE_HOST_RX_DESC_Q_LEN, pDdc->dmemBaseAddr + (P_PCIE_HOST_RX_DESC_Q_LEN * 4), val);

        MxL_MoCA_PCIeRead (idx, pDdc->dmemBaseAddr + (P_PCIE_HOST_TX_DESC_QUEUE * 4), &val);
        num += sprintf(pUserBuf + num, "P_PCIE_HOST_TX_DESC_QUEUE[0x%x](0x%x)=0x%x\n", P_PCIE_HOST_TX_DESC_QUEUE, pDdc->dmemBaseAddr + (P_PCIE_HOST_TX_DESC_QUEUE * 4), val);
        
        MxL_MoCA_PCIeRead (idx, pDdc->dmemBaseAddr + (P_PCIE_HOST_TX_DESC_Q_LEN * 4), &val);
        num += sprintf(pUserBuf + num, "P_PCIE_HOST_TX_DESC_Q_LEN[0x%x](0x%x)=0x%x\n", P_PCIE_HOST_TX_DESC_Q_LEN, pDdc->dmemBaseAddr + (P_PCIE_HOST_TX_DESC_Q_LEN * 4), val);

        MxL_MoCA_PCIeRead (idx, pDdc->dmemBaseAddr + (PCIE_TC_MGMT_IF_MAC_ADDR * 4), &val);
        num += sprintf(pUserBuf + num, "PCIE_TC_MGMT_IF_MAC_ADDR0[0x%x](0x%x)=0x%x\n", PCIE_TC_MGMT_IF_MAC_ADDR, pDdc->dmemBaseAddr + (PCIE_TC_MGMT_IF_MAC_ADDR * 4), val);

        MxL_MoCA_PCIeRead (idx, pDdc->dmemBaseAddr + ((PCIE_TC_MGMT_IF_MAC_ADDR+1) * 4), &val);
        num += sprintf(pUserBuf + num, "PCIE_TC_MGMT_IF_MAC_ADDR1[0x%x](0x%x)=0x%x\n", PCIE_TC_MGMT_IF_MAC_ADDR, pDdc->dmemBaseAddr + ((PCIE_TC_MGMT_IF_MAC_ADDR+1) * 4), val);

        MxL_MoCA_PCIeRead (idx, pDdc->dmemBaseAddr + (PCIE_NUM_OUTBOUND_REGIONS * 4), &val);
        num += sprintf(pUserBuf + num, "PCIE_NUM_OUTBOUND_REGIONS[0x%x](0x%x)=0x%x\n", PCIE_NUM_OUTBOUND_REGIONS, pDdc->dmemBaseAddr + (PCIE_NUM_OUTBOUND_REGIONS * 4), val);
        
        MxL_MoCA_PCIeRead (idx, pDdc->dmemBaseAddr + (PCIE_OUTBOUND_ATU_REG0_BASE * 4), &val);
        num += sprintf(pUserBuf + num, "PCIE_OUTBOUND_ATU_REG0_BASE[0x%x](0x%x)=0x%x\n", PCIE_OUTBOUND_ATU_REG0_BASE, pDdc->dmemBaseAddr + (PCIE_OUTBOUND_ATU_REG0_BASE * 4), val);
        
        MxL_MoCA_PCIeRead (idx, pDdc->dmemBaseAddr + (PCIE_OUTBOUND_ATU_REG1_BASE * 4), &val);
        num += sprintf(pUserBuf + num, "PCIE_OUTBOUND_ATU_REG1_BASE[0x%x](0x%x)=0x%x\n", PCIE_OUTBOUND_ATU_REG1_BASE, pDdc->dmemBaseAddr + (PCIE_OUTBOUND_ATU_REG1_BASE * 4), val);

        MxL_MoCA_PCIeRead (idx, pDdc->dmemBaseAddr + (PCIE_OUTBOUND_ATU_REG2_BASE * 4), &val);
        num += sprintf(pUserBuf + num, "PCIE_OUTBOUND_ATU_REG2_BASE[0x%x](0x%x)=0x%x\n", PCIE_OUTBOUND_ATU_REG2_BASE, pDdc->dmemBaseAddr + (PCIE_OUTBOUND_ATU_REG2_BASE * 4), val);
        
        MxL_MoCA_PCIeRead (idx, pDdc->dmemBaseAddr + (PCIE_OUTBOUND_ATU_REG3_BASE * 4), &val);
        num += sprintf(pUserBuf + num, "PCIE_OUTBOUND_ATU_REG3_BASE[0x%x](0x%x)=0x%x\n", PCIE_OUTBOUND_ATU_REG3_BASE, pDdc->dmemBaseAddr + (PCIE_OUTBOUND_ATU_REG3_BASE * 4), val);

        // ToDo: Add TARGET addresses here when CAR-1141 is fixed.
        MxL_MoCA_PCIeRead (idx, pDdc->dmemBaseAddr + (PCIE_OUTBOUND_ATU_TAR0_BASE * 4), &val);
        num += sprintf(pUserBuf + num, "PCIE_OUTBOUND_ATU_TAR0_BASE[0x%x](0x%x)=0x%x\n", PCIE_OUTBOUND_ATU_TAR0_BASE, pDdc->dmemBaseAddr + (PCIE_OUTBOUND_ATU_TAR0_BASE * 4), val);
        
        MxL_MoCA_PCIeRead (idx, pDdc->dmemBaseAddr + (PCIE_OUTBOUND_ATU_TAR1_BASE * 4), &val);
        num += sprintf(pUserBuf + num, "PCIE_OUTBOUND_ATU_TAR1_BASE[0x%x](0x%x)=0x%x\n", PCIE_OUTBOUND_ATU_TAR1_BASE, pDdc->dmemBaseAddr + (PCIE_OUTBOUND_ATU_TAR1_BASE * 4), val);

        MxL_MoCA_PCIeRead (idx, pDdc->dmemBaseAddr + (PCIE_OUTBOUND_ATU_TAR2_BASE * 4), &val);
        num += sprintf(pUserBuf + num, "PCIE_OUTBOUND_ATU_TAR2_BASE[0x%x](0x%x)=0x%x\n", PCIE_OUTBOUND_ATU_TAR2_BASE, pDdc->dmemBaseAddr + (PCIE_OUTBOUND_ATU_TAR2_BASE * 4), val);
        
        MxL_MoCA_PCIeRead (idx, pDdc->dmemBaseAddr + (PCIE_OUTBOUND_ATU_TAR3_BASE * 4), &val);
        num += sprintf(pUserBuf + num, "PCIE_OUTBOUND_ATU_TAR3_BASE[0x%x](0x%x)=0x%x\n", PCIE_OUTBOUND_ATU_TAR3_BASE, pDdc->dmemBaseAddr + (PCIE_OUTBOUND_ATU_TAR3_BASE * 4), val);

        // Capabilities        
        MxL_MoCA_PCIeRead (idx, (pDdc->dmemBaseAddr + (PCIE_DEVCTL_MAX_PAYLOAD_SIZE * 4)), &val);
        num += sprintf(pUserBuf + num, "PCIE_DEVCTL_MAX_PAYLOAD_SIZE[0x%x](0x%x)=0x%x\n", PCIE_DEVCTL_MAX_PAYLOAD_SIZE, pDdc->dmemBaseAddr + (PCIE_DEVCTL_MAX_PAYLOAD_SIZE * 4), val);
        
        MxL_MoCA_PCIeRead (idx, (pDdc->dmemBaseAddr + (PCIE_LNKCTL_RCB * 4)), &val);
        num += sprintf(pUserBuf + num, "PCIE_LNKCTL_RCB[0x%x](0x%x)=0x%x\n", PCIE_LNKCTL_RCB, pDdc->dmemBaseAddr + (PCIE_LNKCTL_RCB * 4), val);

        MxL_MoCA_PCIeRead (idx, (pDdc->dmemBaseAddr + (PCIE_EN_CCPU_TIMER_INTR * 4)), &val);
        num += sprintf(pUserBuf + num, "PCIE_EN_CCPU_TIMER_INTR[0x%x](0x%x)=0x%x\n", PCIE_EN_CCPU_TIMER_INTR, pDdc->dmemBaseAddr + (PCIE_EN_CCPU_TIMER_INTR * 4), val);

        MxL_MoCA_PCIeRead (idx, (pDdc->dmemBaseAddr + (PCIE_DMA_DATA_ACCESS_BYTE_ORDER * 4)), &val);
        num += sprintf(pUserBuf + num, "PCIE_DMA_DATA_ACCESS_BYTE_ORDER[0x%x](0x%x)=0x%x\n", PCIE_DMA_DATA_ACCESS_BYTE_ORDER, pDdc->dmemBaseAddr + (PCIE_DMA_DATA_ACCESS_BYTE_ORDER * 4), val);
       
        MxL_MoCA_PCIeRead (idx, pDdc->dmemBaseAddr + (PCIE_HOST_STATUS * 4), &val);
        num += sprintf(pUserBuf + num, "PCIE_HOST_STATUS[0x%x](0x%x)=0x%x\n", PCIE_HOST_STATUS, pDdc->dmemBaseAddr + (PCIE_HOST_STATUS * 4), val);
      }
      break;

      case 7:
      {
        MxL_MoCA_PCIeRead (idx, pDdc->dmemBaseAddr + (PCIE_NUM_OUTBOUND_REGIONS * 4), &regions);

        if (regions > 4)
        {
          regions = 4;
        }
        
        for (loop = 0; loop < regions; loop++)
        {          
          MxL_MoCA_PCIeWrite(idx, (uintptr_t)(MXL_MOCA_PCIE_EP_IATU_INDEX_REG), loop); // 0x900
          num += sprintf(pUserBuf + num, "PCIE_EP_IATU_INDEX_REG=0x%x\n", loop);

          // 0820490c : iATU Region Lower Base Address Register
          MxL_MoCA_PCIeRead(idx, (uintptr_t)(MXL_MOCA_PCIE_EP_IATU_REGION_LOWER_BASE_ADDR_REG), &val);
          num += sprintf(pUserBuf + num, "PCIE_EP_IATU_REGION_LOWER_BASE_ADDR_REG=0x%x\n", val);
          
          // 08204914 : iATU Region Limit Address Register (inbound)
          MxL_MoCA_PCIeRead(idx, (uintptr_t)(MXL_MOCA_PCIE_EP_IATU_REGION_LIMIT_ADDR_REG), &val);
          num += sprintf(pUserBuf + num, "PCIE_EP_IATU_REGION_LIMIT_ADDR_REG=0x%x\n", val);

          // 08204918 : iATU Region Lower Target Address Register. Correct target address register need to be specified by host 
          MxL_MoCA_PCIeRead(idx, (uintptr_t)(MXL_MOCA_PCIE_EP_IATU_REGION_LOWER_TARGET_ADDR_REG), &val);
          num += sprintf(pUserBuf + num, "PCIE_EP_IATU_REGION_LOWER_TARGET_ADDR_REG=0x%x\n", val);

          // 08204908 : iATU Region Control 2 Register - enable for addr match
          MxL_MoCA_PCIeRead(idx, (uintptr_t)(MXL_MOCA_PCIE_EP_IATU_REGION_CTRL_2_REG), &val);// REGION_EN = 1
          num += sprintf(pUserBuf + num, "PCIE_EP_IATU_REGION_CTRL_2_REG=0x%x\n", val);
        }
        // Constants
        num += sprintf(pUserBuf + num, "iATU region addresses:\nOUTB_ATU_TARGET0_ADDR_START =0x%x\n", MXL_MOCA_PCIE_OUTB_ATU_TARGET0_ADDR_MASK);
        num += sprintf(pUserBuf + num, "OUTB_ATU_TARGET0_ADDR_END  =0x%x\n", MXL_MOCA_PCIE_OUTB_ATU_TARGET0_ADDR_LIMIT);
        num += sprintf(pUserBuf + num, "OUTB_ATU_TARGET1_ADDR_START=0x%x\n", MXL_MOCA_PCIE_OUTB_ATU_TARGET1_ADDR_MASK);
        num += sprintf(pUserBuf + num, "OUTB_ATU_TARGET1_ADDR_END  =0x%x\n", MXL_MOCA_PCIE_OUTB_ATU_TARGET1_ADDR_LIMIT);        
      }
      break;
        
#if MXL_MOCA_PCIE_EN_PROC_DBG        
    case 8: /* for debug - add any function call or print statement here. See MxL_MoCA_PCIeWriteProc() to enable this case */
        MxL_MoCA_PCIeDevPmSuspend(&(pDdc->pPCIeDev->dev));
        break;
        
    case 9:
        MxL_MoCA_PCIeDevPmResume(&(pDdc->pPCIeDev->dev));
        break;                      
#endif
       
      default:
        break;
    }
  }
  
#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 10, 0)
  return num;
#else
  /* 
   * We return 0 to indicate end of file, that we have
   * no more information. Otherwise, processes will
   * continue to read from us in an endless loop.
   */
  if ( finished ) 
  {
    finished = 0;
    return 0;
  }

  finished = 1;

 /* 
  * We use put_to_user to copy the string from the kernel's
  * memory segment to the memory segment of the process
  * that called us. get_from_user, BTW, is used for the reverse.
  */
  if ( copy_to_user(pPage, gProcFsBuffer, num) ) // pDst, src
  {
    return -EFAULT;
  }
  return num;  /* Return the number of bytes "read" */
#endif
}


/*----------------------------------------------------------------------------------------
 * 
 * FUNCTION NAME : MxL_MoCA_PCIeWriteProc
 *  
 *
 *
 * DATE CREATED  : 03/25/2016
 * 
 * DESCRIPTION   : Proc Write operations.
 *  Proc commands (usage from user space):
 *  ( See description section of MxL_MoCA_PCIeReadProc)
 *
 *
 *--------------------------------------------------------------------------------------*/

#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 10, 0)
int MxL_MoCA_PCIeWriteProc(struct file *pFile, const char *pBuffer, unsigned long count, void *data) // old kernel
{
#else
ssize_t MxL_MoCA_PCIeWriteProc(struct file *pFile, const char __user *pBuffer, size_t count, loff_t* pOffset) // new kernel
{
  int num = 0;
#endif
  int8_t  debug;
  MXL_MOCA_PCIE_DEV_DATA_CONTEXT_T *pDdc = (MXL_MOCA_PCIE_DEV_DATA_CONTEXT_T*)gpDevData[0];
  MXL_MOCA_PCIE_DATAPATH_CONTEXT_T* pDataPath;
  uint32_t*    p; 

#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 10, 0)
  if (!count)
    return 0;
#else
  if ( count > MXL_MOCA_PROCFS_MAX_SIZE )
  {
    num = MXL_MOCA_PROCFS_MAX_SIZE;
  }
  else  
  {
    num = count;
  }
#endif

  if (pBuffer[0] == 'd') /* Debug commands */
  {
    debug = simple_strtol(pBuffer+2, 0, 10);

#if MXL_MOCA_PCIE_EN_PROC_DBG
    if (debug > 9) 
#else    
    if (debug > 13) 
#endif
    {
      printk("invalid debug param: %d\n", debug);
      debug = 0;
    }
    pDdc->readProcMode  = debug; 

    pDataPath  = (MXL_MOCA_PCIE_DATAPATH_CONTEXT_T *)pDdc->pDpCtx;

    switch(debug)
    {
    case 1:
      p = (uint32_t*) &pDdc->rxStats;
      p[0] = p[1] = p[2] = p[3] = 0;
      break;
      
    case 2:
      p = (uint32_t*) &pDdc->txStats;
      p[0] = p[1] = p[2] = p[3] = p[4] = p[5] = 0;
      MxL_MoCA_PCIeWrite(0, pDdc->dmemBaseAddr + (PCIE_TX_NUM_PKTS * 4), 0); // devIndex = 0, assuming a single device
      break;
      
    case 3:
      p = (uint32_t*) &pDdc->rxStats;
      p[0] = p[1] = p[2] = p[3] = 0;
      p = (uint32_t*) &pDdc->txStats; // was unsigned int
      p[0] = p[1] = p[2] = p[3] = p[4] = p[5] = 0;
      MxL_MoCA_PCIeWrite(0, pDdc->dmemBaseAddr + (PCIE_TX_NUM_PKTS * 4), 0);
      pDdc->interrupts = 0;      
#if MXL_MOCA_PCIE_NAPI
      pDdc->polls = 0;
#endif
      break;

    default:
      break;
    }
  }
  else if (pBuffer[0] == 'i') /* Info commands */
  {
    printk ("PCI_EXP_DEVCAP:MaxPayLoadSize = %d\n", pDdc->devCapMaxPayloadSz);
    printk ("PCI_EXP_DEVCTL:MaxPayLoadSize = %d, MaxReadReq = %d\n", pDdc->devCtlMaxPayloadSz, pDdc->devCtlMaxReadReqSz);
    
    /*  Supported Link Speeds, Maximum Link Width */  
    printk ("PCI_EXP_LNKCAP:MaxLinkSpeed(1=2.5GT/s, 2=5GT/s, 4=8GT/s) = %d, MaxLinkWidth = x%d\n",
            pDdc->lnkCapMaxLnkSpeed, pDdc->lnkCapMaxLinkWidth); 
    
    /* Read Completion Boundary */
    printk ("PCI_EXP_LNKCTL:RCB = %d\n", pDdc->lnkCtlRCB);
    
    /* Current Link Speed, Negotiated Link Width */
    printk ("PCI_EXP_LNKSTA:MaxLinkSpeed(1=2.5GT/s, 2=5GT/s, 3=8GT/s) = %d, MaxLinkWidth = x%d\n\n",
            pDdc->lnkStaMaxLnkSpeed, pDdc->lnkStaMaxLinkWidth);
  }
  else if (pBuffer[0] == 't') /* tx */
  {
    debug = simple_strtol(pBuffer+2, 0, 10);
    switch (debug)
    {
    case 0:
    case 1:      
#if MXL_MOCA_PCIE_DEBUG_DROP_TX      
        printk ("Drop TX flag = %d:\n", debug);  
        pDdc->dbg_drop_tx = debug;
#else
        printk("TX pkt drop debug code is not enabled!\n"); 
#endif
        break;
        
     default:
        break;
    }  

  }
  else if (pBuffer[0] == 'r') /* rx */
  {
    debug = simple_strtol(pBuffer+2, 0, 10);
    switch (debug)
    {
    case 0:
    case 1:      
    case 2: 
#if MXL_MOCA_PCIE_DEBUG_DROP_RX        
        printk ("Drop RX flag = %d:\n", debug);
        pDdc->dbg_drop_rx = debug;
#else
        printk("RX pkt drop debug code is not enabled!\n"); 
#endif
        break;
        
     default:
#if MXL_MOCA_PCIE_DEBUG_DROP_RX      
        pDdc->dbg_drop_rx = 0;
#endif
        break;
    }  
  }
  else if (pBuffer[0] == 'h') /* help */
  {
    printk ("DEBUG:\n");
    printk ("======\n");
    printk ("> echo 'd 0' > /proc/%s/pcie\n" \
    "Revert back to displaying tx and rx counters\n" \
    "> cat /proc/%s/pcie\n" \
    "Display the tx/rx counters\n\n", pDdc->procDirName, pDdc->procDirName);

    printk ("> echo 'd 1' > /proc/%s/pcie\n" \
    "To clear rx counters\n" \
    "> cat /proc/%s/pcie\n" \
    "Display the tx/rx counters (rx counts are reset)\n\n", pDdc->procDirName, pDdc->procDirName);

    printk ("> echo 'd 2' > /proc/%s/pcie\n" \
    "To clear tx counters\n" \
    "> cat /proc/%s/pcie\n" \
    "Display the tx/rx counters (tx counts are reset)\n\n", pDdc->procDirName, pDdc->procDirName);

    printk ("> echo 'd 3' > /proc/%s/pcie\n" \
    "To clear tx and rx counters\n" \
    "> cat /proc/%s/pcie\n" \
    "Display the tx/rx counters (tx and rx counts are reset)\n\n", pDdc->procDirName, pDdc->procDirName);

    printk ("> echo 'd 4' > /proc/%s/pcie\n" \
    "To output Rx descr queue info.\n" \
    "> cat /proc/%s/pcie\n" \
    "> dmesg\n" \
    "Displays the Rx descr queue\n\n", pDdc->procDirName, pDdc->procDirName);

    printk ("> echo 'd 5' > /proc/%s/pcie\n" \
    "To output Tx descr queue info.\n" \
    "> cat /proc/%s/pcie\n" \
    "> dmesg\n" \
    "Displays the Tx descr queue\n\n", pDdc->procDirName, pDdc->procDirName);

    printk ("> echo 'd 6' > /proc/%s/pcie\n" \
    "To displays the DMEM fixed locations\n" \
    "> cat /proc/%s/pcie\n" \
    "> dmesg\n" \
    "Displays the DMEM fixed locations (Host-to-SoC DMEM interface)\n\n", pDdc->procDirName, pDdc->procDirName);

    printk ("> echo 'd 7' > /proc/%s/pcie\n" \
    "Output outbound (EP to RC) iATU configurations\n" \
    "> cat /proc/%s/pcie\n" \
    "> dmesg\n" \
    "Displays the outbound iATU configurations\n\n", pDdc->procDirName, pDdc->procDirName);

    // << values 8 and 9 are reserved for internal use >>

    //    "Allow 1 packet every 32 packets (received from upper layers) in TX (Host to SoC) direction\n" 
    //    "Set the second param to 1 to enable drop packets\n" 
    printk ("> echo 't 0' > /proc/%s/pcie\nor\n" \
    "> echo 't 1' > /proc/%s/pcie\n" \
    "0: (default)Allow all packets in TX (received from upper layers) in TX (Host to SoC) direction\n" \
    "1: Drop all packets in TX (received from upper layers) in TX (Host to SoC) direction\n" \
    "> dmesg\n" \
    "Displays the flag was set\n\n", pDdc->procDirName, pDdc->procDirName);

    //    "Allow 1 packet every 32 packets in RX (SoC to Host) direction (not sending to upper layers)\n" 
    //    "Set the second param to 1 to enable drop packets\n"
    printk ("> echo 'r 0' > /proc/%s/pcie\nor\n" \
    "> echo 'r 1' > /proc/%s/pcie\n" \
    "> echo 'r 2' > /proc/%s/pcie\n" \
    "0: (default)Allow all packets in RX (SoC to Host) direction (not sending to upper layers)\n" \
    "1: Drop all packets in RX (SoC to Host) direction (not sending to upper layers but freeing and re-allocating skb)\n" \
    "2: Drop all packets in RX (SoC to Host) direction (not sending to upper layers and not releasing the skb)\n" \
    "> dmesg\n" \
    "Displays the flag was set\n\n", pDdc->procDirName, pDdc->procDirName, pDdc->procDirName); 

    printk ("INFO:\n");
    printk ("=====\n");
    printk ("> echo 'i 1' > /proc/%s/pcie\n" \
    "Display EP info.\n" \
    "> dmesg\n" \
    "Displays the capabilities and key config info (vendor/device ID).\n\n", pDdc->procDirName);
   }
  
#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 10, 0)  
  return count;
#else
  printk(KERN_INFO "procfs_write: write %d bytes\n", num);
  return num;
#endif
}

/*----------------------------------------------------------------------------------------
 * 
 * FUNCTION NAME : MxL_MoCA_PCIeGetMgmtMAC
 *  
 *
 *
 * DATE CREATED  : 03/25/2016
 * 
 * DESCRIPTION   : Get the stored management MAC address from the net_device
 *
 * IN PARAMS     : [*pDdc]: Device driver context.
 *
 * OUT PARAMS    : [*mac] : pointer to array of 2 DWORDs to return the MAC address.
 *
 * RETURN VALUE  : None
 *
 *--------------------------------------------------------------------------------------*/
 
static void MxL_MoCA_PCIeGetMgmtMAC(MXL_MOCA_PCIE_DEV_DATA_CONTEXT_T *pDdc, uint32_t* pMac)
{
  if (pDdc != 0)
  {
    pMac[0] = pDdc->mgmtIfMacAddr[0];
    pMac[1] = pDdc->mgmtIfMacAddr[1];
  }
  else
  {
    MxL_MoCA_PCIeDbgTrace(MXL_MOCA_PCIE_L_ERR, "NULL Pointer (pDdc)\n");
  }
}

/*----------------------------------------------------------------------------------------
 * 
 * FUNCTION NAME : MxL_MoCA_PCIeSetMgmtMAC
 *  
 *
 *
 * DATE CREATED  : 03/25/2016
 * 
 * DESCRIPTION   : Specify the device MAC address
 *
 * IN PARAMS     : [devIndex] : device index.  
 *                 [*mac]     : array of 2 DWORDs to provide the MAC address.
 *
 * OUT PARAMS    : None
 *
 * RETURN VALUE  : None
 *
 *--------------------------------------------------------------------------------------*/
 
 // interface
void MxL_MoCA_PCIeSetMgmtMAC(int8_t devIndex, uint32_t* pMac) 
{
  MXL_MOCA_PCIE_DEV_DATA_CONTEXT_T *pDdc = (MXL_MOCA_PCIE_DEV_DATA_CONTEXT_T*)gpDevData[devIndex];
  struct net_device *pNetDev = pci_get_drvdata(pDdc->pPCIeDev); // pNetDev ?
  unsigned int macHigh, macLow;

  macHigh = pDdc->mgmtIfMacAddr[0] = pMac[0];
  macLow  = pDdc->mgmtIfMacAddr[1] = pMac[1];

  // shift out the hi and low words to be endian-agnostic
  pNetDev->dev_addr[0] = (uint8_t) ((macHigh >> 24) & 0xff);
  pNetDev->dev_addr[1] = (uint8_t) ((macHigh >> 16) & 0xff);
  pNetDev->dev_addr[2] = (uint8_t) ((macHigh >>  8) & 0xff);
  pNetDev->dev_addr[3] = (uint8_t) ((macHigh      ) & 0xff);
  pNetDev->dev_addr[4] = (uint8_t) ((macLow >> 24) & 0xff);
  pNetDev->dev_addr[5] = (uint8_t) ((macLow >> 16) & 0xff);
}

/*----------------------------------------------------------------------------------------
 * 
 * FUNCTION NAME : MxL_MoCA_PCIeStopXmit
 *  
 *
 *
 * DATE CREATED  : 03/25/2016
 * 
 * DESCRIPTION   : Stop transmit
 *
 * IN PARAMS     : [*arg] : pointer to net_device.  
 *
 * OUT PARAMS    : None
 *
 * RETURN VALUE  : None
 *
 *--------------------------------------------------------------------------------------*/
 
static void MxL_MoCA_PCIeStopXmit(void* arg)
{
  struct net_device *pNetDev;
  MXL_MOCA_PCIE_DEV_DATA_CONTEXT_T *pDdc;
  MXL_MOCA_DBG1P0("**** MxL_MoCA_PCIeStopXmit ....\n");

  pNetDev = (struct net_device *)arg;

  pDdc = NETDEV_PRIV(pNetDev);

  MxL_MoCA_PCIeLockIrqSave(&pDdc->ethTxLock);
  pDdc->active = 0;
  MxL_MoCA_PCIeUnLockIrqRestore(&pDdc->ethTxLock);
}

/*----------------------------------------------------------------------------------------
 * 
 * FUNCTION NAME : MxL_MoCA_PCIeStartXmit
 *  
 *
 *
 * DATE CREATED  : 03/25/2016
 * 
 * DESCRIPTION   : Start transmit
 *
 * IN PARAMS     : [*arg] : pointer to net_device.  
 *
 * OUT PARAMS    : None
 *
 * RETURN VALUE  : None
 *
 *--------------------------------------------------------------------------------------*/
 
static void MxL_MoCA_PCIeStartXmit(void* arg)
{
  struct net_device *pNetDev;
  MXL_MOCA_PCIE_DEV_DATA_CONTEXT_T *pDdc;
  pNetDev = (struct net_device *)arg;

  pDdc = NETDEV_PRIV(pNetDev);

  MxL_MoCA_PCIeLockIrqSave(&pDdc->ethTxLock);
  pDdc->active = 1;
  MxL_MoCA_PCIeUnLockIrqRestore(&pDdc->ethTxLock);
}

/*----------------------------------------------------------------------------------------
 * 
 * FUNCTION NAME : MxL_MoCA_PCIeResetTxQueue
 *  
 *
 *
 * DATE CREATED  : 03/25/2016
 * 
 * DESCRIPTION   : Reset Tx queue
 *
 * IN PARAMS     : [*arg] : pointer to net_device.
 *
 * OUT PARAMS    : None
 *
 * RETURN VALUE  : MXL_MOCA_STATUS_E
 *
 *--------------------------------------------------------------------------------------*/
 
static MXL_MOCA_STATUS_E MxL_MoCA_PCIeResetTxQueue(void* arg)
{
  struct net_device *pNetDev;
  MXL_MOCA_PCIE_DEV_DATA_CONTEXT_T *pDdc;
  uint16_t index;

  pNetDev = (struct net_device *)arg;

  MXL_MOCA_DBG1P0("Reseting TxQueue ...\n");

  pDdc = NETDEV_PRIV(pNetDev);

  MxL_MoCA_PCIeLockIrqSave(&pDdc->ethTxLock);

  pDdc->txRingWrIndex = 0;
  pDdc->txRingRdIndex = 0;

  for (index = 0; index < MXL_MOCA_PCIE_TX_MAPPING_SIZE; index++)
  {
    if (pDdc->txRing[index].skb)
    {
      pci_unmap_single(pDdc->pPCIeDev, pDdc->txRing[index].mapping,
                       MXL_MOCA_PCIE_MAX_ETH_MTU_SIZE, PCI_DMA_TODEVICE);
      //dev_kfree_skb(pDdc->txRing[index].skb);
      dev_kfree_skb_any(pDdc->txRing[index].skb);
      pDdc->txRing[index].skb = NULL;
    }
  }

  MxL_MoCA_PCIeUnLockIrqRestore(&pDdc->ethTxLock);

  return MXL_MOCA_OK;
}

/*----------------------------------------------------------------------------------------
 * 
 * FUNCTION NAME : MxL_MoCA_PCIeAllocRxBuffers
 *  
 *
 *
 * DATE CREATED  : 03/25/2016
 * 
 * DESCRIPTION   : Allocate static Rx buffers
 *
 * IN PARAMS     : [*arg] : pointer to the struct net_device. 
 *
 * OUT PARAMS    : None
 *
 * RETURN VALUE  : MXL_MOCA_STATUS_E
 *
 *--------------------------------------------------------------------------------------*/
 
static MXL_MOCA_STATUS_E MxL_MoCA_PCIeAllocRxBuffers(void* arg)
{
  //MXL_MOCA_DBG2P0( "======= MxL_MoCA_PCIeAllocRxBuffers ...\n");
  return MxL_MoCA_PCIeEnqueueRxBuffs((struct net_device *)arg);
}

/*----------------------------------------------------------------------------------------
 * 
 * FUNCTION NAME : MxL_MoCA_PCIeFreeRxBuffers
 *  
 *
 *
 * DATE CREATED  : 03/25/2016
 * 
 * DESCRIPTION   : Free RX buffers
 *
 * IN PARAMS     : [*arg] : pointer to the struct net_device. 
 *
 * OUT PARAMS    : None
 *
 * RETURN VALUE  : None
 *
 *--------------------------------------------------------------------------------------*/
 
static MXL_MOCA_STATUS_E MxL_MoCA_PCIeFreeRxBuffers(void* arg)
{
  struct net_device *pNetDev = (struct net_device *) arg;

  MXL_MOCA_PCIE_DEV_DATA_CONTEXT_T *pDdc = NETDEV_PRIV(pNetDev);

  unsigned int packetID;

  //MXL_MOCA_DBG2P0( "======= MxL_MoCA_PCIeFreeRxBuffers ...\n");
  for (packetID = 0; packetID < MXL_MOCA_PCIE_RX_RING_SIZE; packetID++)
  {
    if (pDdc->rxRing[packetID].skb)
    {
      pci_unmap_single(pDdc->pPCIeDev, pDdc->rxRing[packetID].mapping,
                       MXL_MOCA_PCIE_MAX_ETH_MTU_SIZE, PCI_DMA_FROMDEVICE);
      //dev_kfree_skb(pDdc->rxRing[packetID].skb);
      dev_kfree_skb_any(pDdc->rxRing[packetID].skb);
      pDdc->rxRing[packetID].skb = NULL;
    }
  }

  return MXL_MOCA_OK;
}

/*----------------------------------------------------------------------------------------
 * 
 * FUNCTION NAME : MxL_MoCA_PCIeEnqueueRxBuffs
 *  
 *
 *
 * DATE CREATED  : 03/25/2016
 * 
 * DESCRIPTION   : Allocates initial RX Queue buffers. These are required to be available
 *                 since the packets are pushed from the SoC side (egress) as soon as the
 *                 firmware starts running and packets are pending.
 *
 * IN PARAMS     : [*pNetDev] : pointer to the struct net_device. 
 *
 * OUT PARAMS    : None
 *
 * RETURN VALUE  : MXL_MOCA_STATUS_E (MXL_MOCA_MEM_ALLOC_ERR, MXL_MOCA_OK, MXL_MOCA_ERR)
 *
 *--------------------------------------------------------------------------------------*/
 
static MXL_MOCA_STATUS_E MxL_MoCA_PCIeEnqueueRxBuffs(struct net_device *pNetDev)
{
  MXL_MOCA_PCIE_DEV_DATA_CONTEXT_T *pDdc = NETDEV_PRIV(pNetDev);
  uint16_t loop;
  struct sk_buff *skb;
  dma_addr_t mapping;
  int packetID;
  MXL_MOCA_STATUS_E ret = MXL_MOCA_OK;

  packetID = 0;

  MxL_MoCA_PCIeDbgTrace(MXL_MOCA_PCIE_L_DBG1, "Allocating Rx Queue buffers...\n");

  for (loop = 0; loop < MXL_MOCA_PCIE_RX_RING_SIZE; loop++)
  {
    if (!(skb = dev_alloc_skb(MXL_MOCA_PCIE_MAX_ETH_RX_BUF_SIZE))) 
    {
      MxL_MoCA_PCIeDbgTrace(MXL_MOCA_PCIE_L_ERR, "%s: dev_alloc_skb failed\n", __FUNCTION__);
      return MXL_MOCA_MEM_ALLOC_ERR; // Cause: -ENOMEM
    }

    pDdc->rxRing[packetID].skb = skb;
    skb->dev = pNetDev;

    /// Ethernet header alignment
    skb_reserve(skb,0); // Do not use NET_IP_ALIGN

    mapping = pci_map_single(pDdc->pPCIeDev, skb->data, MXL_MOCA_PCIE_MAX_ETH_RX_BUF_SIZE, PCI_DMA_FROMDEVICE);
    if (unlikely(pci_dma_mapping_error(pDdc->pPCIeDev, mapping))) 
    {
      MxL_MoCA_PCIeDbgTrace(MXL_MOCA_PCIE_L_ERR, "EnqueueRxBuffs: dma_mapping error!\n");
      pDdc->mappingErr1++;
      dev_kfree_skb_any(skb);
      pDdc->rxRing[packetID].skb = NULL;
      ret = MXL_MOCA_ERR;
      break;
    }

    pDdc->rxRing[packetID].mapping = mapping;
    ret = MxL_MoCA_PCIeQueuePkt(&pDdc->pDpCtx->rxQueuePair, MXL_MOCA_PCIE_MAX_ETH_MTU_SIZE, mapping);

    if (ret != MXL_MOCA_OK) 
    {
      pci_unmap_single(pDdc->pPCIeDev, mapping, MXL_MOCA_PCIE_MAX_ETH_MTU_SIZE, PCI_DMA_FROMDEVICE);

      dev_kfree_skb_any(skb);
      pDdc->rxRing[packetID].skb = NULL;

      break;
    }
    packetID++;
  }
  return ret;
}

/*----------------------------------------------------------------------------------------
 * 
 * FUNCTION NAME : MxL_MoCA_PCIeNetDevOpen
 *  
 *
 *
 * DATE CREATED  : 03/25/2016
 * 
 * DESCRIPTION   : Brings up the Ethernet interface. This is one of the net_device_ops.
 *
 * IN PARAMS     : [*pNetDev] : pointer to the struct net_device. 
 *
 * OUT PARAMS    : None
 *
 * RETURN VALUE  : return 0 if successful, <0 (Linux error code) if error
 *
 *--------------------------------------------------------------------------------------*/
 
int MxL_MoCA_PCIeNetDevOpen(struct net_device *pNetDev)
{
  MXL_MOCA_PCIE_DEV_DATA_CONTEXT_T *pDdc = NETDEV_PRIV(pNetDev);

  MXL_MOCA_DBG2P2("%s: %s\n", __FUNCTION__, pNetDev->name);

  MxL_MoCA_PCIeLockIrqSave(&pDdc->ethTxLock);

  /// Start Linux net interface
  netif_carrier_on(pNetDev);
  netif_start_queue(pNetDev);

  pDdc->firstTime = true;
  MxL_MoCA_PCIeUnLockIrqRestore(&pDdc->ethTxLock);

  return (0);
}

/*----------------------------------------------------------------------------------------
 * 
 * FUNCTION NAME : MxL_MoCA_PCIeNetDevClose
 *  
 *
 *
 * DATE CREATED  : 03/25/2016
 * 
 * DESCRIPTION   : Shuts down the Ethernet interface. This is one of the net_device_ops
 *
 * IN PARAMS     : [*pNetDev] : pointer to the struct net_device. 
 *
 * OUT PARAMS    : None
 *
 * RETURN VALUE  : return 0 if successful, <0 (Linux error code) if error
 *
 *--------------------------------------------------------------------------------------*/
 
int MxL_MoCA_PCIeNetDevClose(struct net_device *pNetDev)
{
  MXL_MOCA_PCIE_DEV_DATA_CONTEXT_T *pDdc = NETDEV_PRIV(pNetDev);
  uint16_t packetID;
  uint16_t index;

  /*************************************************************************
  * NEED TO STOP SOC FIRST
  ************************************************************************/

  MXL_MOCA_DBG2P2("%s: %s\n", __FUNCTION__, pNetDev->name);

  MxL_MoCA_PCIeLockIrqSave(&pDdc->ethTxLock);
  /// Stop Linux net interface
  netif_stop_queue(pNetDev);

  for(packetID = 0; packetID < MXL_MOCA_PCIE_RX_RING_SIZE; packetID++)
  {
    if (pDdc->rxRing[packetID].skb)
    {
      pci_unmap_single(pDdc->pPCIeDev, pDdc->rxRing[packetID].mapping,
                       MXL_MOCA_PCIE_MAX_ETH_MTU_SIZE, PCI_DMA_FROMDEVICE);
      dev_kfree_skb_any(pDdc->rxRing[packetID].skb);
      pDdc->rxRing[packetID].skb = NULL;
    }
  }

  for(index = 0; index < MXL_MOCA_PCIE_TX_MAPPING_SIZE; index++)
  {
    if (pDdc->txRing[index].skb)
    {
      pci_unmap_single(pDdc->pPCIeDev, pDdc->txRing[index].mapping,
                       MXL_MOCA_PCIE_MAX_ETH_MTU_SIZE, PCI_DMA_TODEVICE);
      //dev_kfree_skb(pDdc->txRing[index].skb);
      dev_kfree_skb_any(pDdc->txRing[index].skb);
      pDdc->txRing[index].skb = NULL;
    }
  } 

  MxL_MoCA_PCIeUnLockIrqRestore(&pDdc->ethTxLock);
  return (0);
}

/*----------------------------------------------------------------------------------------
 * 
 * FUNCTION NAME : MxL_MoCA_PCIeNapiPoll
 *  
 *
 *
 * DATE CREATED  : 03/25/2016
 * 
 * DESCRIPTION   : NAPI Poll function. Called by Kernel.
 *
 * IN PARAMS     : [*p_napi]  : pointer to the mapi struct. 
 *               : [budget]   : budget or unit of work to handle.
 *
 * OUT PARAMS    : None
 *
 * RETURN VALUE  : pDdc->napiWorkDone
 *
 *--------------------------------------------------------------------------------------*/
 
#if MXL_MOCA_PCIE_NAPI
static int MxL_MoCA_PCIeNapiPoll(struct napi_struct *p_napi, int budget)
{
  MXL_MOCA_PCIE_DEV_DATA_CONTEXT_T *pDdc;

  pDdc = container_of(p_napi, MXL_MOCA_PCIE_DEV_DATA_CONTEXT_T, napi);

  pDdc->napiWorkDone = 0;

#if MXL_MOCA_PCIE_DEBUG_DROP_RX 
  if (2 == pDdc->dbg_drop_rx) 
  {
    MxL_MoCA_PCIeDebugHandleRxReturnOwnership(pDdc);
  }
  else
#endif // MXL_MOCA_PCIE_DEBUG_DROP_RX 
  {
    MxL_MoCA_PCIeHandleRxData(pDdc); /* will update pDdc->napiWorkDone */
  }
  MXL_MOCA_DBG2P3("%s %d %d\n", __FUNCTION__, budget, pDdc->napiWorkDone);

  pDdc->polls++;

  if (pDdc->napiWorkDone < budget)
  {
    /* Turn off polling */
    napi_complete(p_napi);
    pDdc->napiDisableIntr = 0;
  }
  else
  {
    MXL_MOCA_DBG2P0( "********* pDdc->napiWorkDone >= budget ********\n");
  }
  return pDdc->napiWorkDone;
}
#endif

/*----------------------------------------------------------------------------------------
 * 
 * FUNCTION NAME : MxL_MoCA_PCIeStartXmitSkbs
 *  
 *
 *
 * DATE CREATED  : 03/25/2016
 * 
 * DESCRIPTION   : Sends a packet over the Ethernet interface. Invoked by stack.
 *
 * IN PARAMS     : [*pNetDev] : Pointer to the net device.
 *
 * OUT PARAMS    : None
 *
 * RETURN VALUE  : MXL_MOCA_STATUS_E code.
 *
 *--------------------------------------------------------------------------------------*/
 
static MXL_MOCA_STATUS_E MxL_MoCA_PCIeStartXmitSkbs(struct net_device *pNetDev)
{
  MXL_MOCA_PCIE_DEV_DATA_CONTEXT_T *pDdc = NETDEV_PRIV(pNetDev);
  dma_addr_t mapping;
  uint16_t i, j;
  MXL_MOCA_STATUS_E status = MXL_MOCA_OK;
  struct sk_buff *skb;

  MXL_MOCA_DBG2P2("%s: %s\n", __FUNCTION__, pNetDev->name); 

  i = pDdc->txRingRdIndex;
  for (j = 0; j < MXL_MOCA_PCIE_TX_MAPPING_SIZE; i++, j++)
  {
    if (i >= MXL_MOCA_PCIE_TX_MAPPING_SIZE) i = 0;

    if (i == pDdc->txRingWrIndex) break;

    skb     = pDdc->txRing[i].skb;
    mapping = pDdc->txRing[i].mapping;

    //MXL_MOCA_DBG2P3("xmit_skbs: packetID = %d, txRingWrIndex = %d, skb = %p\n", i, pDdc->txRingWrIndex, (long)skb);

    // Should not happen.
    if (skb == NULL)
    {
      MXL_MOCA_DBG2P2("*** MxL_MoCA_PCIeStartXmitSkbs: i = %d, txRingIndex = %d\n", i, pDdc->txRingWrIndex);

      BUG();
      break;
    }

    if (skb->len < MXL_MOCA_PCIE_MIN_ETH_MTU_SIZE_NO_FCS)
    {
      memset(skb->data+skb->len, 0, (MXL_MOCA_PCIE_MIN_ETH_MTU_SIZE_NO_FCS-skb->len)); 
      status = MxL_MoCA_PCIeQueuePkt(&pDdc->pDpCtx->txQueuePair, MXL_MOCA_PCIE_MIN_ETH_MTU_SIZE_NO_FCS, mapping);
    }
    else
    {
      status = MxL_MoCA_PCIeQueuePkt(&pDdc->pDpCtx->txQueuePair, skb->len, mapping);
    }

    MXL_MOCA_DBG2P3("MxL_MoCA_PCIeStartXmitSkbs: packetID = %d, txRingWrIndex = %d, status = %d\n", i, pDdc->txRingWrIndex, status);

    if (status == MXL_MOCA_NO_HOST_DESC_ERR)
    {
      MxL_MoCA_PCIeDbgTrace(MXL_MOCA_PCIE_L_ERR, "StartXmitSkbs: Out-of-descriptors\n");
      break;
    }
    
    pDdc->txIntrCount++;
    pDdc->txStats[4]++;
  }

  pDdc->txRingRdIndex = i;

  pNetDev->trans_start = jiffies;

  return status;
}

/*----------------------------------------------------------------------------------------
 * 
 * FUNCTION NAME : MxL_MoCA_PCIeNetDevStartXmit
 *  
 *
 *
 * DATE CREATED  : 03/25/2016
 * 
 * DESCRIPTION   : Sends a packet over the Ethernet interface. Called by the protocol stack.
 *                 Declared by attaching to the netdev structure. This is one of the 
 *                 net_device_ops.
 *
 * IN PARAMS     : [*skb]     : Pointer to the socket buffer to send. 
 *               : [*pNetDev] : Pointer to the net device.
 *
 * OUT PARAMS    : None
 *
 * RETURN VALUE  : return 0 if successful, <0 (Linux error code) if error
 *
 *--------------------------------------------------------------------------------------*/
 
static int MxL_MoCA_PCIeNetDevStartXmit(struct sk_buff *skb, struct net_device *pNetDev)
{
  MXL_MOCA_PCIE_DEV_DATA_CONTEXT_T *pDdc = NETDEV_PRIV(pNetDev);
  dma_addr_t mapping;

#if MXL_MOCA_PCIE_DEBUG_DROP_TX  
  if (1 == pDdc->dbg_drop_tx) 
  {
    pDdc->txStats[0]++; // recieved from stack
    pDdc->txStats[1]++;
    pDdc->txStats[2]++; // dropped
    dev_kfree_skb_any(skb);
    return NETDEV_TX_OK;
  }
  else
#endif
  {
    if (unlikely(skb->len <= 0) || (pDdc->busy == 1))
    {
      pDdc->txStats[1]++;
      dev_kfree_skb_any(skb);
      pDdc->txStats[2]++; // dropped
      return NETDEV_TX_OK;
    }
  }

  pDdc->txStats[0]++;

  // Map a tx packet ready for DMA
  mapping = pci_map_single(pDdc->pPCIeDev, skb->data, skb->len, PCI_DMA_TODEVICE);
  if (unlikely(pci_dma_mapping_error(pDdc->pPCIeDev, mapping))) 
  {
    MxL_MoCA_PCIeDbgTrace(MXL_MOCA_PCIE_L_ERR, "DevStartXmit: dma_mapping error!\n");
    pDdc->mappingErr2++;
    dev_kfree_skb_any(skb);
    pDdc->txStats[2]++; // dropped
    return NETDEV_TX_OK;
  }
  else
  {
    if (MxL_MoCA_PCIeCheckOutOfBoundMapping(mapping) == MXL_MOCA_ERR)
    {
      MxL_MoCA_PCIeDbgTrace(MXL_MOCA_PCIE_L_ERR, "(0x%x)DevStartXmit:out-of-bounds mapping error!\n", mapping);
      pDdc->mappingErr2++;
      dev_kfree_skb_any(skb);
      pDdc->txStats[2]++; // dropped
      return NETDEV_TX_OK;
    }
  }

#if MXL_MOCA_ENABLE_DBG_MSG
  MxL_MoCA_PCIeDbgTrace(MXL_MOCA_PCIE_L_DBG2,  "TX:(skb=%x) mapping = %x, few pkt bytes(from stack) length=%x [%2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x]\n", 
                        skb->data, mapping, skb->len,
                        skb->data[0],  skb->data[1],  skb->data[2],  skb->data[3],
                        skb->data[4],  skb->data[5],  skb->data[6],  skb->data[7],
                        skb->data[8],  skb->data[9],  skb->data[10], skb->data[11],
                        skb->data[12], skb->data[13], skb->data[14], skb->data[14],
                        skb->data[15], skb->data[15], skb->data[17], skb->data[18]);
#endif

  MxL_MoCA_PCIeLockIrqSave(&pDdc->ethTxLock);

  if (unlikely(pDdc->txRing[pDdc->txRingWrIndex].skb != NULL) ||
      unlikely(!pDdc->active))
  {
    pDdc->txStats[2]++;
    pDdc->ethStats.txDroppedErrs++;
    pci_unmap_single(pDdc->pPCIeDev, mapping, skb->len, PCI_DMA_TODEVICE);
    dev_kfree_skb(skb);
    MxL_MoCA_PCIeUnLockIrqRestore(&pDdc->ethTxLock);

    MXL_MOCA_DBG2P1("NETDEV_TX_OK txRingWrIndex (busy) = %d\n", pDdc->txRingWrIndex);
    //     pDdc->busy = 1; // For DEBUG
    
    return NETDEV_TX_OK;
  }
  pDdc->txRing[pDdc->txRingWrIndex].skb     = skb;
  pDdc->txRing[pDdc->txRingWrIndex].mapping = mapping;

  MXL_MOCA_DBG2P2("MxL_MoCA_PCIeNetDevStartXmit: txRingIndex = %d, skb = %p\n", pDdc->txRingWrIndex, (long)skb);

  pDdc->txRingWrIndex++;
  if (pDdc->txRingWrIndex >= MXL_MOCA_PCIE_TX_MAPPING_SIZE)
      pDdc->txRingWrIndex = 0;

  //MXL_MOCA_DBG2P4("%s: index = %d, txRingIndex = %d, skb = %p\n",  __FUNCTION__, index, pDdc->txRingWrIndex, (long)skb);

  pDdc->numPktsQueued++;
  pDdc->txStats[3]++;

  MxL_MoCA_PCIeStartXmitSkbs(pNetDev);
  MxL_MoCA_PCIeUnLockIrqRestore(&pDdc->ethTxLock);

  return NETDEV_TX_OK;
}


/*----------------------------------------------------------------------------------------
 * 
 * FUNCTION NAME : MxL_MoCA_PCIeNetDevTxTimeout
 *  
 *
 *
 * DATE CREATED  : 03/25/2016
 * 
 * DESCRIPTION   : Transmission Timeouts callback place holder. This is one of the 
 *                 net_device_ops
 *
 * IN PARAMS     : [*skb]     : Pointer to the socket buffer to send. 
 *               : [*pNetDev] : Pointer to the net device.
 *
 * OUT PARAMS    : None
 *
 * RETURN VALUE  : None
 *
 *--------------------------------------------------------------------------------------*/

static void MxL_MoCA_PCIeNetDevTxTimeout(struct net_device *pNetDev)
{
  MXL_MOCA_DBG2P2("%s: %s\n", __FUNCTION__, pNetDev->name);
}

/*----------------------------------------------------------------------------------------
 * 
 * FUNCTION NAME : MxL_MoCA_PCIeISR
 *  
 *
 *
 * DATE CREATED  : 03/25/2016
 * 
 * DESCRIPTION   : Interrupt service routine (MSI type) called by the kernel when 
 *                 a MSI is issued by the EP. Keep this function simple. No print statements.
 *
 * IN PARAMS     : [irq]            : Interrrupt number.  
 *               : [p_dev_instance] : Pointer to a network interface device structure.
 *
 * OUT PARAMS    : None
 *
 * RETURN VALUE  : None
 *
 *--------------------------------------------------------------------------------------*/
 
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 19)
static irqreturn_t MxL_MoCA_PCIeISR(int irq, void *pNetDev)
#else
static irqreturn_t MxL_MoCA_PCIeISR(int irq, void *pNetDev, struct pt_regs *regs)
#endif
{
  MXL_MOCA_PCIE_DEV_DATA_CONTEXT_T *pDdc = NETDEV_PRIV(pNetDev);
  uint16_t                          valShort;

  MXL_MOCA_PCIE_ETH_LOCK_T *lk = &pDdc->ethTxLock;

  // MXL_MOCA_DBG2P1("%s: IRQ recieved!!!!\n", __FUNCTION__);

  spin_lock(&lk->lockSpinLock);

  /* Check if it is INTx and skip if it was not ASSERTED by our device */
  if (pDdc->inbandIntrType == PCIE_INTR_TYPE_INTA)
  {
    pci_read_config_word(pDdc->pPCIeDev, PCI_STATUS, &valShort);
    if (0 == (valShort &  PCI_STATUS_INTERRUPT))
    {
      spin_unlock(&lk->lockSpinLock);
      return IRQ_NONE;
    } 
  }

#if MXL_MOCA_PCIE_NAPI
  if (pDdc->active)
  {
    MxL_MoCA_PCIeHandleTxData(pDdc);
    if (pDdc->napiDisableIntr == 0)
    {
      // MXL_MOCA_DBG1P0("napi_schedule\n");
      napi_schedule(&pDdc->napi);
      // We increment polls count in MxL_MoCA_PCIeNapiPoll
      pDdc->napiDisableIntr = 1;
    }
  }
#else
  /// call RX interrupt routine
  if (pDdc->active)
  {
    // MXL_MOCA_DBG2P1( "No NAPI %d\n", pDdc->active);
#if MXL_MOCA_PCIE_DEBUG_DROP_RX 
    if (2 == pDdc->dbg_drop_rx) 
    {
      MxL_MoCA_PCIeDebugHandleRxReturnOwnership(pDdc);
    }
    else
#endif // MXL_MOCA_PCIE_DEBUG_DROP_RX 
    {
      MxL_MoCA_PCIeHandleRxData(pDdc);
    }
    MxL_MoCA_PCIeHandleTxData(pDdc);
  }
#endif
  pDdc->interrupts++;

  if (pDdc->inbandIntrType == PCIE_INTR_TYPE_INTA)
  {
    MxL_MoCA_PCIeWrite(pDdc->devIndex, MXL_MOCA_PCIE_EHI_CTRL_STAT_REG0, MXL_MOCA_PCIE_EHI_CTRL_STAT_REG0_INTX_DEASSERT);
  }

  spin_unlock(&lk->lockSpinLock);
  
  return IRQ_HANDLED;
}


/*----------------------------------------------------------------------------------------
 * 
 * FUNCTION NAME : MxL_MoCA_PCIeNetDevSetRxMode
 *  
 *
 *
 * DATE CREATED  : 03/25/2016
 * 
 * DESCRIPTION   : Network rx mode setting function. This is one of the net_device_ops
 *
 * IN PARAMS     : [*pNetDev] : Pointer to the struct net_device. 
 *
 * OUT PARAMS    : None
 *
 * RETURN VALUE  : None
 *
 *--------------------------------------------------------------------------------------*/

static void MxL_MoCA_PCIeNetDevSetRxMode(struct net_device *pNetDev)
{
  pNetDev = pNetDev; 
}

/*----------------------------------------------------------------------------------------
 * 
 * FUNCTION NAME : MxL_MoCA_PCIeNetDevGetStats
 *  
 *
 *
 * DATE CREATED  : 03/25/2016
 * 
 * DESCRIPTION   : Network device statistics gatherer. ifconfig from user space calls this.
 *                 This is one of the net_device_ops.
 *
 * IN PARAMS     : [*pNetDev] : Pointer to the struct net_device. 
 *
 * OUT PARAMS    : None
 *
 * RETURN VALUE  : None
 *
 *--------------------------------------------------------------------------------------*/
 
static struct net_device_stats* MxL_MoCA_PCIeNetDevGetStats(struct net_device *pNetDev)
{
  MXL_MOCA_PCIE_DEV_DATA_CONTEXT_T *pDdc = NETDEV_PRIV(pNetDev);
  MXL_MOCA_PCIE_ETH_STATS_T ethStats;

  memcpy((void *)&ethStats, (void *)&pDdc->ethStats, sizeof(MXL_MOCA_PCIE_ETH_STATS_T));

  /// then, update driver stats structure
  pDdc->stats.rx_packets = ethStats.rxPackets;       /** total packets received    */
  pDdc->stats.tx_packets = ethStats.txPackets;       /** total packets transmitted */
  pDdc->stats.rx_bytes   = ethStats.rxBytes;         /** total bytes received      */
  pDdc->stats.tx_bytes   = ethStats.txBytes;         /** total bytes transmitted   */
  pDdc->stats.rx_errors  = ethStats.rxPacketErrs;
  //+ ethStats.rxCrc32Errs;     /* bad packets received  */
  pDdc->stats.tx_errors  = ethStats.txPacketErrs;        /** packet transmit problems   */
  //pDdc->stats.rx_dropped = ethStats.rxDroppedErrs;     /** no space in linux buffers  */
  pDdc->stats.rx_dropped = pDdc->netIfRxDropped;         /** no space in linux buffers  */
  pDdc->stats.tx_dropped = ethStats.txDroppedErrs;
  pDdc->stats.multicast  = ethStats.rxMulticastPackets;  /** multicast packets received */
  pDdc->stats.rx_length_errors  = ethStats.rxLengthErrs;
  pDdc->stats.rx_over_errors    = 0;
  pDdc->stats.rx_crc_errors     = ethStats.rxCrc32Errs;       /** recved pkt with crc error */
  pDdc->stats.rx_frame_errors   = ethStats.rxFrameHeaderErrs; /** recv'd frame alignment error */
  pDdc->stats.rx_fifo_errors    = ethStats.rxFifoFullErrs;    /** recv'r fifo overrun */
  pDdc->stats.tx_aborted_errors = ethStats.txCrc32Errs;
  pDdc->stats.tx_fifo_errors    = ethStats.txFifoFullErrs; 

  return &pDdc->stats;
}

/* ethtool ops */
/*----------------------------------------------------------------------------------------
 * 
 * FUNCTION NAME : netdev_ethtool_ioctl
 *  
 *
 *
 * DATE CREATED  : 03/25/2016
 * 
 * DESCRIPTION   : ethtool handler
 *
 * IN PARAMS     : [*pNetDev] : Pointer to the struct net_device. 
 *
 * OUT PARAMS    : None
 *
 * RETURN VALUE  : None
 *
 *--------------------------------------------------------------------------------------*/
 
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 4)
int netdev_ethtool_ioctl(struct net_device *pNetDev, void *pUserAddr) 
{
  u32 ethCmd;

  MXL_MOCA_DBG2P0("ioctl codes here\n");
  
  if (copy_from_user(&ethCmd, pUserAddr, sizeof(ethCmd)))
      return -EFAULT;

    switch (ethCmd)
    {
      case ETHTOOL_GDRVINFO:
      {
        struct ethtool_drvinfo info = {ETHTOOL_GDRVINFO};
        memcpy(info.driver, MXL_MOCA_PCIE_DRV_NAME, sizeof(MXL_MOCA_PCIE_DRV_NAME));
        memcpy(info.version, VERSION_HOST_STR, sizeof(VERSION_HOST_STR));
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 0)
        /* slot_name does not exist on 2.6 */
        info.bus_info[0] = 0;
#else
        {
          MXL_MOCA_PCIE_DEV_DATA_CONTEXT_T *pDdc = NETDEV_PRIV(pNetDev);
          memcpy(info.bus_info, pDdc->pdev->slot_name, sizeof(pDdc->pdev->slot_name));
        }
#endif
        if (copy_to_user(pUserAddr, &info, sizeof(info)))
            return -EFAULT;

        return 0;
    }
  } /* end switch */

  return -EOPNOTSUPP;
}

#else /* 2.6.4 */

/*----------------------------------------------------------------------------------------
 * 
 * FUNCTION NAME : MxL_MoCA_PCIeEthtoolGetDrvInfo
 *  
 *
 *
 * DATE CREATED  : 03/25/2016
 * 
 * DESCRIPTION   : ethtool get link status
 *
 * IN PARAMS     : [pNetDev]  : Pointer to the struct net_device. 
 *                 [pDrvInfo] : Pointer to return the driver information
 *
 * OUT PARAMS    : None 
 *
 * RETURN VALUE  : None
 *
 *--------------------------------------------------------------------------------------*/

///After 2.6.4, it supports ethtool_ops, ethtool.h to check ethtool_ops struct
// ethtool_ops
static void MxL_MoCA_PCIeEthtoolGetDrvInfo(struct net_device      *pNetDev, 
                                           struct ethtool_drvinfo *pDrvInfo)
{
  strncpy(pDrvInfo->driver,  MXL_MOCA_PCIE_DRV_NAME, 32);
  strncpy(pDrvInfo->version, VERSION_HOST_STR, 32);
  strcpy(pDrvInfo->fw_version, "N/A");
  pDrvInfo->bus_info[0] = 0;
}

/*----------------------------------------------------------------------------------------
 * 
 * FUNCTION NAME : MxL_MoCA_PCIeEthtoolGetLink
 *  
 *
 *
 * DATE CREATED  : 03/25/2016
 * 
 * DESCRIPTION   : ethtool get link status
 *
 * IN PARAMS     : [p_dev] : Pointer to the struct net_device. 
 *
 * OUT PARAMS    : None 
 *
 * RETURN VALUE  : None
 *
 *--------------------------------------------------------------------------------------*/
 
// ethtool_ops
unsigned int MxL_MoCA_PCIeEthtoolGetLink(struct net_device *pNetDev)
{
  return netif_carrier_ok(pNetDev) ? 1 : 0;
}

/*----------------------------------------------------------------------------------------
 * 
 * FUNCTION NAME : MxL_MoCA_PCIeSetEthToolVar
 *  
 *
 *
 * DATE CREATED  : 03/25/2016
 * 
 * DESCRIPTION   : setup default ethtool ops
 *
 * IN PARAMS     : [*pNetDev] : Pointer to the struct net_device. 
 *
 * OUT PARAMS    : None
 *
 * RETURN VALUE  : None
 *
 *--------------------------------------------------------------------------------------*/
 
static void MxL_MoCA_PCIeSetEthToolVar(struct net_device *pNetDev)
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 16, 0)
  SET_ETHTOOL_OPS(pNetDev, (struct ethtool_ops *)&gMxlMocaPcieEthtoolOps);
#else
  netdev_set_default_ethtool_ops(pNetDev, (struct ethtool_ops *)&gMxlMocaPcieEthtoolOps);
#endif
}

#endif /* 2.6.4 */

/*----------------------------------------------------------------------------------------
 * 
 * FUNCTION NAME : MxL_MoCA_PCIeModuleInit
 *  
 *
 *
 * DATE CREATED  : 03/25/2016
 * 
 * DESCRIPTION   : Driver main entry point. Called when driver load (insmod)
 *                 Will call MxL_MoCA_PCIeProbeOne
 *
 *                 Vendor IDs:
 *                 -----------
 *                 MxL3701/MxL3706/MxL3609 : Cardiff 
 *                                           Access - MxL3609. 
 *                                           MoCA - MxL3701/MxL3706
 *
 *                 MxL3711/MxL3619"        : Leucadia
 *                                           Access - MxL3619. 
 *                                           MoCA - MxL3711
 *
 * IN PARAMS     : None 
 *
 * OUT PARAMS    : None
 *
 * RETURN VALUE  : 0  Registration was successful. !0 Registration failed
 *
 *--------------------------------------------------------------------------------------*/
 
static int __init MxL_MoCA_PCIeModuleInit(void)
{
  int ret_code;
  uint8_t index, devNum;
  uint32_t levelBackup = MXL_MOCA_PCIE_HOST_PRINTLOG_THRESHOLD;
  struct pci_dev *p_pci_device = 0; 
  uint8_t found = 0;
  static const int8_t *pErrorLevelStr[MXL_MOCA_PCIE_L_END] = {
                                                               "DEF",
                                                               "ERR",
                                                               "WARN",
                                                               "INFO",
                                                               "DBG_1",
                                                               "DBG_2"
                                                               };

  MXL_MOCA_PCIE_HOST_PRINTLOG_THRESHOLD = MXL_MOCA_PCIE_L_DBG2;

  for(index = 0; index < MXL_MOCA_PCIE_MAX_NUM_OF_PCIE_DEVICES; index++)
      gpDevData[index] = NULL;
  
  for (devNum = 0; devNum < MXL_MOCA_PCIE_MAX_NUM_OF_PCIE_DEVICES; devNum++)
  {
    p_pci_device = pci_get_device(MXL_MOCA_PCIE_VENDOR_ID, PCI_ANY_ID, p_pci_device);

    if (p_pci_device == NULL)
    {
      break;
    }
    else
    {
      found++;
    }
  }
  
  if (0 == found)
  {
    MxL_MoCA_PCIeDbgTrace( MXL_MOCA_PCIE_L_ERR, "No MoCA PCIe device(s) found!\n");
#if MXLWARE_CONFIG_BOARD_ECA_9M_L3_NXP    
    moduleInitError = 1;
    // NOTE: Return 0 to indicate success to avoid unknown symbols issue when control path driver is loaded.
    //       This is essential to support runtime ECA type detection.

    return 0; 
#else // MVL platform: returning -1 will cause insmod to abort loading the driver
    return -1;
#endif
  }
  else
  {
    MxL_MoCA_PCIeDbgTrace(MXL_MOCA_PCIE_L_INFO, "%d device(s) found\n", found);
  }  

#if defined(__LP64__) || defined(_LP64)
  MxL_MoCA_PCIeDbgTrace(MXL_MOCA_PCIE_L_INFO, "Host system type: 64 bits\n");
#else
  MxL_MoCA_PCIeDbgTrace(MXL_MOCA_PCIE_L_INFO, "Host system type: 32 bits\n");
#endif
  
#if MXL_MOCA_PCIE_NAPI
  MxL_MoCA_PCIeDbgTrace(MXL_MOCA_PCIE_L_INFO, "NAPI enabled\n");
#else
  MxL_MoCA_PCIeDbgTrace(MXL_MOCA_PCIE_L_INFO, "NAPI not enabled\n");
#endif

  /* Command line set verbose level takes effect from here */
  if (verbose >= MXL_MOCA_PCIE_L_END)
  {
    MxL_MoCA_PCIeDbgTrace(MXL_MOCA_PCIE_L_INFO, "Verbose option range: %d - %d\n", 
                          MXL_MOCA_PCIE_L_DEF, MXL_MOCA_PCIE_L_DBG2);
    MxL_MoCA_PCIeDbgTrace(MXL_MOCA_PCIE_L_INFO, "Setting level=%s\n", pErrorLevelStr[MXL_MOCA_PCIE_L_INFO]);
    MXL_MOCA_PCIE_HOST_PRINTLOG_THRESHOLD = levelBackup;
  }
  else if (verbose == MXL_MOCA_PCIE_L_DEF)          /* Only errors and warnings */
  {
    MxL_MoCA_PCIeDbgTrace(MXL_MOCA_PCIE_L_INFO, "Setting default debug level(%s)\n", pErrorLevelStr[MXL_MOCA_PCIE_L_INFO]);
    MXL_MOCA_PCIE_HOST_PRINTLOG_THRESHOLD = MXL_MOCA_PCIE_L_INFO;
  }
  else
  {
    MxL_MoCA_PCIeDbgTrace(MXL_MOCA_PCIE_L_INFO, "Setting debug level(%s)\n", pErrorLevelStr[verbose]);
    MXL_MOCA_PCIE_HOST_PRINTLOG_THRESHOLD = verbose;
  }

  ret_code = pci_register_driver(&gMxlMocaPCIeDrv); /* Calls MxL_MoCA_PCIeProbeOne */
  
  if (ret_code)
  {
    MxL_MoCA_PCIeDbgTrace(MXL_MOCA_PCIE_L_ERR, "%s: Module init failed: %d\n", MXL_MOCA_PCIE_DRV_NAME, ret_code);
  }

  return ret_code;
}

/*----------------------------------------------------------------------------------------
 * 
 * FUNCTION NAME : MxL_MoCA_PCIeModuleExit
 *  
 *
 *
 * DATE CREATED  : 03/25/2016
 * 
 * DESCRIPTION   : Driver exit point. Called when driver unload (rmmod).
 *                 Will call MxL_MoCA_PCIeRemoveOne
 *
 * IN PARAMS     : None 
 *
 * OUT PARAMS    : None
 *
 * RETURN VALUE  : None
 *
 *--------------------------------------------------------------------------------------*/
 
static void __exit MxL_MoCA_PCIeModuleExit(void)
{
  if (MXL_MOCA_PCIE_HOST_PRINTLOG_THRESHOLD == MXL_MOCA_PCIE_L_DEF)
  {
    /* Report errors and warnings */
    MXL_MOCA_PCIE_HOST_PRINTLOG_THRESHOLD = MXL_MOCA_PCIE_L_INFO;
  }

#if MXLWARE_CONFIG_BOARD_ECA_9M_L3_NXP 
  if (1 == moduleInitError) // Nothing to unregister
  {
    MxL_MoCA_PCIeDbgTrace(MXL_MOCA_PCIE_L_INFO, "%s %s Unloaded\n", MXL_MOCA_PCIE_DRV_NAME, VERSION_HOST_STR);
    return;
  }
#endif

  MxL_MoCA_PCIeDbgTrace(MXL_MOCA_PCIE_L_INFO, "Removing PCIe driver\n");
  pci_unregister_driver(&gMxlMocaPCIeDrv);
}

/*******************************************************************************
 *      Control Plane Interface Function
 ******************************************************************************/
EXPORT_SYMBOL(MxL_MoCA_PCIeRead);
EXPORT_SYMBOL(MxL_MoCA_PCIeWrite);
EXPORT_SYMBOL(MxL_MoCA_PCIeAvailable);
EXPORT_SYMBOL(MxL_MoCA_PCIeRestart);
EXPORT_SYMBOL(MxL_MoCA_PCIeStop);
EXPORT_SYMBOL(MxL_MoCA_PCIeSetMgmtMAC);
EXPORT_SYMBOL(MxL_MoCA_PCIeGlobalResetSoC);

/*******************************************************************************
 *      Kernel mode driver init and exit
 ******************************************************************************/
module_init(MxL_MoCA_PCIeModuleInit);
module_exit(MxL_MoCA_PCIeModuleExit);

/*******************************************************************************
 *      Kernel module identifications
 ******************************************************************************/
MODULE_LICENSE("GPL");
MODULE_AUTHOR("MaxLinear SW Development Team");
MODULE_DESCRIPTION("MxL37xx PCIe Driver"); // MxL3701 is Cardiff, MxL3711 is Leucadia

