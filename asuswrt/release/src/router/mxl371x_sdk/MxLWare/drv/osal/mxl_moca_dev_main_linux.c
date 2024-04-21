/*****************************************************************************************
 *
 * FILE NAME          : mxl_moca_dev_main_linux.c
 *
 *
 *
 * DATE CREATED       : Sep/21/2016
 *
 * LAST MODIFIED      :
 *
 * DESCRIPTION        : This file include the linux related oswp functions
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

#include <linux/init.h>
#include <linux/version.h>
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 33)
#include <generated/autoconf.h>
#elif (LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 33) ) && ( LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 18))
#include <linux/autoconf.h>
#else
#include <linux/config.h>
#endif
#include <linux/string.h>

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/proc_fs.h>
#include <linux/fcntl.h>
#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 4, 0)
#include <asm/system.h>
#endif
#include <asm/uaccess.h>
#include <asm/io.h>
#include <linux/ioport.h>
#include <linux/sockios.h>
#include <linux/ethtool.h>
#include <linux/mtd/mtd.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/device.h>
#include <linux/netdevice.h>
#include <linux/firmware.h>


#include "mxl_moca_host_version.h"
#include "mxl_data_types.h"
#include "mxl_moca_config.h"
#include "mxl_moca_soc_cmd.h"
#include "mxl_moca_osal.h"
#include "mxl_moca_drv_mdio.h"
#include "mxl_moca_drv_soc.h"
#include "mxl_moca_drv_main.h"
#include "mxl_moca_dev_mgr.h"
#include "mxl_moca_hal_mdio_ctrl.h"

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("MaxLinear MoCA Driver");
MODULE_AUTHOR("MaxLinear SW Development Team");

int32_t major;

MXL_MOCA_STATUS_E MxL_MoCA_DrvProcessEthToolCmd(void *pEthToolCmdBuff);
MXL_MOCA_STATUS_E MxL_MoCA_DrvProcessSIOCSMIIREG(int32_t mocaDevId, void *pSioCmdBuff);
MXL_MOCA_STATUS_E MxL_MoCA_DrvProcessSIOCGMIIREG(int32_t mocaDevId, void *pSioCmdBuff);

/*----------------------------------------------------------------------------------------
 * FUNCTION NAME : MxL_MoCA_DbgShowMocaCmdInfo
 *
 *
 * DATE CREATED  : Oct/4/2016
 *
 * DESCRIPTION   : Display CmdInfo
 *
 *
 * IN PARAMS     : ioctlCmd   Ioctl request command.
 *                 pMocaOpCmd Pointer to MoCA CMD structure
 *
 * OUT PARAMS    : None
 *
 * RETURN VALUE  : None
 *--------------------------------------------------------------------------------------*/

static void MxL_MoCA_DbgShowMocaCmdInfo(uint32_t ioctlCmd, MXL_MOCA_OP_CMD_T *pMocaOpCmd)
{
  uint32_t *pCmdBuf;
  uint32_t cnt;
  
  pCmdBuf = (uint32_t *)pMocaOpCmd;

  // Display some cmd buf 
  for (cnt = 0; cnt < 5; cnt++)
  {
      DEBUG_PRINT(TRACE_OSWP, L_INFO,"CmdBuf[%d]: %08x \n", cnt, *pCmdBuf);
      pCmdBuf++;
  }

  // Display CMD, can add more for other command debug in the future
  switch (ioctlCmd)
  {
    case SIOC_MXL_MOCA_START_DAEMON:
      DEBUG_PRINT(TRACE_OSWP, L_INFO, ": START DAEMON, cmdId = %#x, len=%x (%d) \n", 
                                      pMocaOpCmd->mocaCmdId, 
                                      pMocaOpCmd->mocaCmdLenInBytes,
                                      pMocaOpCmd->mocaCmdLenInBytes);
      for (cnt = 0; cnt < (pMocaOpCmd->mocaCmdLenInBytes/4); cnt++)
        DEBUG_PRINT(TRACE_OSWP, L_INFO,"startDaemonCmdBuf[%d]: %08x \n", cnt, *pCmdBuf);
      break;

    case SIOC_MXL_MOCA_DBG_GET_MEM:
      DEBUG_PRINT(TRACE_OSWP, L_INFO, ": GET mem, cmdId = %#x, len=%x, addr=0x%x \n", 
                                      pMocaOpCmd->mocaCmdId, 
                                      pMocaOpCmd->mocaCmdLenInBytes,
                                      pMocaOpCmd->uCmd.drvGetDbgMemCmd.socMemAddr);
      break;
  
    case SIOC_MXL_MOCA_DBG_SET_MEM:
      DEBUG_PRINT(TRACE_OSWP, L_INFO, ": SET mem, cmdId = %#x, len=%x, addr=0x%x\n", 
                                      pMocaOpCmd->mocaCmdId, 
                                      pMocaOpCmd->mocaCmdLenInBytes,
                                      pMocaOpCmd->uCmd.drvSetDbgMemCmd.socMemAddr[0]);
      break;
  
    default:    
      DEBUG_PRINT(TRACE_OSWP, L_INFO, "SIO command :%#010x \n", pMocaOpCmd->mocaCmdId);    
    break;
  }

  return ;
}


/*----------------------------------------------------------------------------------------
 * FUNCTION NAME : MxL_MoCA_DbgShowMocaCmdRspInfo
 *
 *
 * DATE CREATED  : Oct/4/2016
 *
 * DESCRIPTION   : Display RspInfo
 *
 *
 * IN PARAMS     : ioctlCmd   Ioctl request command.
 *                 pMocaOpRsp Pointer to MoCA CMD Rsp structure
 *
 * OUT PARAMS    : None
 *
 * RETURN VALUE  : None
 *--------------------------------------------------------------------------------------*/

static void MxL_MoCA_DbgShowMocaCmdRspInfo(uint32_t ioctlCmd, MXL_MOCA_OP_RSP_T *pMocaOpRsp)
{
  // Can add more for debug in the future
  switch (ioctlCmd)
  {
    case SIOC_MXL_MOCA_DBG_GET_MEM:
      DEBUG_PRINT(TRACE_OSWP, L_INFO, ": GET mem, Rsp, cmdId = %#x, len=%x, addr=0x%x \n", 
                                      pMocaOpRsp->mocaCmdId, 
                                      pMocaOpRsp->mocaRspLenInBytes,
                                      pMocaOpRsp->uRsp.drvGetDbgMemRsp.usrData[0]);
      break;
  
    case SIOC_MXL_MOCA_DBG_SET_MEM:
      DEBUG_PRINT(TRACE_OSWP, L_INFO, ": SET mem, Rsp, cmdId = %#x, len=%x\n", 
                                      pMocaOpRsp->mocaCmdId, 
                                      pMocaOpRsp->mocaRspLenInBytes);
      break;
  
    default:    
      DEBUG_PRINT(TRACE_OSWP, L_INFO, "MoCA command :%#010x \n", pMocaOpRsp->mocaCmdId);    
    break;
  }

  return ;
}


/*----------------------------------------------------------------------------------------
 * FUNCTION NAME : MxL_MoCA_DevIoctl
 *
 *
 * DATE CREATED  : Sep/21/2016
 *
 * DESCRIPTION   : add Mdio protocol
 *
 *
 * IN PARAMS     : inode      The standard device node link.
 *                 filp       Driver file descriptor, all the dynamic resource saved
 *                            in the private_data area.
 *                 ioctlCmd   Ioctl request command.
 *                 pMocaParam The ioctl parameters
 *
 * OUT PARAMS    : None
 *
 * RETURN VALUE  : status of success or failure
 *--------------------------------------------------------------------------------------*/
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 36)
static long    MxL_MoCA_DevIoctl(struct file *filp,
                                 uint32_t ioctlCmd,
                                 unsigned long pMocaParam)
#else
static int32_t MxL_MoCA_DevIoctl(struct inode *inode,
                                 struct file *filp,
                                 uint32_t ioctlCmd,
                                 unsigned long pMocaParam)
#endif
{
  MXL_MOCA_DEV_CTRL_MGR_T *pDevCtrlMgr;
  uint32_t mocaDevId = 0;
  uint32_t mocaSubCmdId = 0;
  struct   ifreq ifr;
  MXL_MOCA_CMD_PARAMS ioctlMocaCmdParam;  // MoCA comamnd parameter pointers from User Mode

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 36)
  struct inode *inode = filp->f_path.dentry->d_inode;
  long            ret = MXL_MOCA_ERR;
#else
  int32_t         ret = MXL_MOCA_ERR;
#endif

  MXL_ENTER_FUNCTION(TRACE_OSWP, "inode = %p, filp = %p, ioctlCmd = %#x, pMocaParam = %#x",
                                  inode, filp, ioctlCmd, pMocaParam);

  // Get MoCA control device ID
  mocaDevId = iminor(inode);  
  
  // Get MoCA devMgr global data for this control device
  pDevCtrlMgr = MxL_MoCA_DevGetCtrlMgrInstanceHandle(mocaDevId);

  if (NULL == pDevCtrlMgr)
  {
    MxL_MoCA_OsalPrintLog(TRACE_OSWP, L_ERR, "NULL pointer in %s line %d\n", __func__, __LINE__);
    return MXL_MOCA_ERR;
  }

  // Lock for MoCA processing, TODO: add lock timeout processing
  MxL_MoCA_OsalLock(&pDevCtrlMgr->mocaDevLock);
  
  // CMD processing
  do
  {
    // Copy ifreq structure data from user space (use ioctl system param)
    ret = MxL_MoCA_OsalCopyFromUser(&ifr, (void*)pMocaParam, sizeof(ifr));
    
    if (MXL_MOCA_OK != ret)
    {
      MxL_MoCA_OsalPrintLog(TRACE_OSWP, L_ERR, "Line %d, cmd 0x%x: copy data from user FAILED!\n",
                                                __LINE__, ioctlCmd);
      break;
    }

    // MoCA sub command processing
    if (ioctlCmd >= SIOC_MXL_MOCA_CMD && ioctlCmd <= SIOC_MXL_MOCA_START_DAEMON)
    {
      // Copy moca cmd data from user space (use ifreq data pointer)
      ret = MxL_MoCA_OsalCopyFromUser(&ioctlMocaCmdParam,
                                      (void*)ifr.ifr_data, 
                                      sizeof(MXL_MOCA_CMD_PARAMS));
      if (MXL_MOCA_OK != ret)
      {
        MxL_MoCA_OsalPrintLog(TRACE_OSWP, L_ERR, "Line %d, cmd 0x%x: copy data from user FAILED!\n",
                                                  __LINE__, ioctlCmd);
        break;
      }
      
      // Copy moca command data (mocaCmdId and mocaCmdLenInBytes) from user space 
      ret = MxL_MoCA_OsalCopyFromUser(&pDevCtrlMgr->mocaOpCmd,
                                      (void*)ioctlMocaCmdParam.pParam1,
                                      sizeof(pDevCtrlMgr->mocaOpCmd.mocaCmdId) + 
                                      sizeof(pDevCtrlMgr->mocaOpCmd.mocaCmdLenInBytes));
      if (MXL_MOCA_OK != ret)
      {
        MxL_MoCA_OsalPrintLog(TRACE_OSWP, L_ERR, "Line %d, cmd 0x%x: copy data from user FAILED!\n",
                                                  __LINE__, ioctlCmd);
        break;
      }

      // Check mocaCmdLenInBytes is valid
      if (pDevCtrlMgr->mocaOpCmd.mocaCmdLenInBytes > sizeof(pDevCtrlMgr->mocaOpCmd.uCmd))
      {
        MxL_MoCA_OsalPrintLog(TRACE_OSWP, L_ERR, "Line %d, cmd 0x%x, inLen 0x%x is large than cmdLen 0x%x\n", 
                                                  __LINE__, ioctlCmd, pDevCtrlMgr->mocaOpCmd.mocaCmdLenInBytes,
                                                  sizeof(pDevCtrlMgr->mocaOpCmd.uCmd));
        break;
      }

      // Copy moca command data (uCmd) from user space 
      ret = MxL_MoCA_OsalCopyFromUser(&pDevCtrlMgr->mocaOpCmd.uCmd,
                                      (void*)(ioctlMocaCmdParam.pParam1 +
                                              sizeof(pDevCtrlMgr->mocaOpCmd.mocaCmdId) + 
                                              sizeof(pDevCtrlMgr->mocaOpCmd.mocaCmdLenInBytes)),
                                      pDevCtrlMgr->mocaOpCmd.mocaCmdLenInBytes);
      if (MXL_MOCA_OK != ret)
      {
        MxL_MoCA_OsalPrintLog(TRACE_OSWP, L_ERR, "Line %d, cmd 0x%x: copy data from user FAILED!\n",
                                                  __LINE__, ioctlCmd);
        break;
      }

      // Display cmd info for debug
      MxL_MoCA_DbgShowMocaCmdInfo(ioctlCmd, &pDevCtrlMgr->mocaOpCmd);      

      // Get and check the expected response length from user space, and will stop
      // this command processing if the rsp len size is bigger than max rsp len size
      ret = MxL_MoCA_OsalCopyFromUser(&pDevCtrlMgr->mocaOpRsp,
                                      (void*)ioctlMocaCmdParam.pParam2,
                                      sizeof(pDevCtrlMgr->mocaOpRsp.mocaCmdId) + 
                                      sizeof(pDevCtrlMgr->mocaOpRsp.mocaRspLenInBytes));
      if (MXL_MOCA_OK != ret)
      {
        MxL_MoCA_OsalPrintLog(TRACE_OSWP, L_ERR, "Line %d, cmd 0x%x: copy data from user FAILED!\n",
                                                  __LINE__, ioctlCmd);
        break;
      }
      if (pDevCtrlMgr->mocaOpRsp.mocaRspLenInBytes > sizeof(pDevCtrlMgr->mocaOpRsp.uRsp))
      {
        MxL_MoCA_OsalPrintLog(TRACE_OSWP, L_ERR, "Line %d, cmd 0x%x, outLen 0x%x is large than rspLen 0x%x\n",
                                                  __LINE__, ioctlCmd, pDevCtrlMgr->mocaOpRsp.mocaRspLenInBytes, 
                                                  sizeof(pDevCtrlMgr->mocaOpRsp.uRsp));
        break;
      }
      
      // Display Rsp info from APP for debug
      MxL_MoCA_DbgShowMocaCmdRspInfo(ioctlCmd, &pDevCtrlMgr->mocaOpRsp);
    }
    
    switch (ioctlCmd)
    {      
      // ### MoCA command processing ###    
      
      case SIOC_MXL_MOCA_START_DAEMON:
        // Save MoCA sub command for cmd status return status check later in this function
        mocaSubCmdId = pDevCtrlMgr->mocaOpCmd.mocaCmdId;

        // Start daemon command handler
        ret = MxL_MoCA_DevProcessDaemonCmd(pDevCtrlMgr);
        break;

      case SIOC_MXL_MOCA_DBG_GET_MEM:   
      case SIOC_MXL_MOCA_DBG_SET_MEM:
        // Save MoCA sub command for cmd status return status check later in this function
        mocaSubCmdId = pDevCtrlMgr->mocaOpCmd.mocaCmdId;
       
        ret = MxL_MoCA_DevProcessDbgCmd(pDevCtrlMgr);
        break;

      case SIOC_MXL_MOCA_CMD:
        // Save MoCA sub command for cmd status return status check later in this function
        mocaSubCmdId = pDevCtrlMgr->mocaOpCmd.mocaCmdId;
        
        // MoCA driver and SoC command handler
        ret = MxL_MoCA_DevProcessCmd(pDevCtrlMgr);
        break;
      
      // ### OS service related command processing ###      
      
      case SIOCETHTOOL:
        // Input/output buffer from appication is passed as part of ifr data structure
        ret = MxL_MoCA_DrvProcessEthToolCmd((void*)ifr.ifr_data);          
        break;
  
      case SIOCSMIIREG:
        // Input/output buffer from appication is passed as part of raw request i.e not using ifr data structure
        ret = MxL_MoCA_DrvProcessSIOCSMIIREG(mocaDevId, (void *)pMocaParam);
        break;
  
      case SIOCGMIIREG:
        // Input/output buffer from appication is passed as part of raw request i.e not using ifr data structure
        ret = MxL_MoCA_DrvProcessSIOCGMIIREG(mocaDevId, (void *)pMocaParam);
        break;      

      default:
        MxL_MoCA_OsalPrintLog(TRACE_OSWP, L_ERR, "ioctlCmd :%#010x is not supported\n",
                                                  ioctlCmd);
        ret = MXL_MOCA_INVALID_CMD_ERR;
        break;
    }
    
    // MoCA command rsp processing
    if (MXL_MOCA_OK == ret && 
        (ioctlCmd >= SIOC_MXL_MOCA_CMD && ioctlCmd <= SIOC_MXL_MOCA_START_DAEMON))
    {
      // Display Rsp info to APP for debug
      MxL_MoCA_DbgShowMocaCmdRspInfo(ioctlCmd, &pDevCtrlMgr->mocaOpRsp);
          
      // Send Rsp back to user mode Apps
      pDevCtrlMgr->mocaOpRsp.mocaCmdId = mocaSubCmdId;
      
      if (MxL_MoCA_OsalCopyToUser((MXL_MOCA_OP_RSP_T *)ioctlMocaCmdParam.pParam2, 
                                  &pDevCtrlMgr->mocaOpRsp,    
                                  pDevCtrlMgr->mocaOpRsp.mocaRspLenInBytes +              
                                  sizeof(pDevCtrlMgr->mocaOpRsp.mocaCmdId) + 
                                  sizeof(pDevCtrlMgr->mocaOpRsp.mocaRspLenInBytes)))
      {
        MxL_MoCA_OsalPrintLog(TRACE_OSWP, L_ERR, 
                              "MxL_MoCA_OsalCopyToUser() failed for ioctlCmd: %#x!\n",
                              ioctlCmd);
      }                                      
    }

  } while (0); 
  
  // Unlock for MoCA CMD processing 
  MxL_MoCA_OsalUnlock(&pDevCtrlMgr->mocaDevLock);

  // Check return status and print info if not OK
  if (MXL_MOCA_OK != ret)
  {
    MxL_MoCA_OsalPrintLog(TRACE_OSWP, L_ERR, "## Failed to handle ioctlCmd = %#x\n", 
                                              ioctlCmd);
  }
                                            
  MXL_EXIT_FUNCTION(TRACE_OSWP, "status = %d", ret);

  return ret;
}

/*----------------------------------------------------------------------------------------
 * FUNCTION NAME : MxL_MoCA_DevRelease
 *
 *
 * DATE CREATED  : Sep/24/2016
 *
 * DESCRIPTION   : Driver release function, judge if the driver has no caller, if it is,
 *                 then release the dynamic resource.
 *
 *
 * IN PARAMS     : inode The standard device node link.
 *               : filp  Driver file descriptor, all the dynamic resource saved
 *                       in the private_data area.
 *
 * OUT PARAMS    : None
 *
 * RETURN VALUE  : 0 for success, others for failure
 *--------------------------------------------------------------------------------------*/

static int32_t MxL_MoCA_DevRelease(struct inode *inode, struct file *filp)
{
  return 0;
}

/*----------------------------------------------------------------------------------------
 * FUNCTION NAME : MxL_MoCA_DevRead
 *
 *
 * DATE CREATED  : Sep/24/2016
 *
 * DESCRIPTION   : Driver read function, provide the standard method to read memory
 *                 from user space.
 *
 *
 * IN PARAMS     : filp  Driver file descriptor, all the dynamic resource saved
 *                       in the private_data area.
 *               : buf   The user space memory pointer.
 *               : count How many bytes to be read.
 *               : f_pos The file pointer to control where to read.
 *
 * OUT PARAMS    : None
 *
 * RETURN VALUE  : The number of bytes read, or -1 for failure.
 *--------------------------------------------------------------------------------------*/

ssize_t MxL_MoCA_DevRead(struct file *filp, char *buf, size_t count, loff_t *f_pos)
{
  ssize_t sz = 0;

  MXL_ENTER_FUNCTION(TRACE_OSWP, "filp = %p, buf = %p, count = %u, f_pos = %d", filp, buf, count, f_pos);

  MXL_EXIT_FUNCTION(TRACE_OSWP, "status = %u", sz);

  return sz;
}

/*----------------------------------------------------------------------------------------
 * FUNCTION NAME : MxL_MoCA_DevWrite
 *
 *
 * DATE CREATED  : Sep/24/2016
 *
 * DESCRIPTION   : Driver write function, provide the standard method to write to device
 *                 from user space.
 *
 *
 * IN PARAMS     : filp  Driver file descriptor, all the dynamic resource saved
 *                       in the private_data area.
 *               : buf   The user space memory pointer.
 *               : count How many bytes to write.
 *               : f_pos The file pointer to control where to write.
 *
 * OUT PARAMS    : None
 *
 * RETURN VALUE  : The number of bytes written, or -1 for failure.
 *--------------------------------------------------------------------------------------*/

ssize_t MxL_MoCA_DevWrite(struct file *filp, const char *buf, size_t count, loff_t *f_pos)
{
  ssize_t sz = 0;

  MXL_ENTER_FUNCTION(TRACE_OSWP, "filp = %p, buf = %p, count = %u, f_pos = %d", filp, buf, count, f_pos);

  MXL_EXIT_FUNCTION(TRACE_OSWP, "status = %u", sz);

  return sz;
}

/*----------------------------------------------------------------------------------------
 * FUNCTION NAME : MxL_MoCA_DevOpen
 *
 *
 * DATE CREATED  : Sep/24/2016
 *
 * DESCRIPTION   : Driver open function, provide the standard method to open the driver.
 *
 *
 * IN PARAMS     : inode The standard device node link.
 *                 filp  Driver file descriptor, all the dynamic resource saved
 *                       in the private_data area.
 *
 * OUT PARAMS    : None
 *
 * RETURN VALUE  : 0 for success, others for failure
 *--------------------------------------------------------------------------------------*/

static int32_t MxL_MoCA_DevOpen(struct inode *inode, struct file *filp)
{
  return MXL_MOCA_OK;
}

/*----------------------------------------------------------------------------------------
 * FUNCTION NAME : MxL_MoCA_DrvProcessEthToolCmd
 *
 *
 * DATE CREATED  : Sep/28/2016
 *
 * DESCRIPTION   : Process the eth tool command
 *
 *
 * IN PARAMS     : pMxlMocaOpCmd The command to process
 *
 * OUT PARAMS    : pMxlMocaOpRsp The response for the command
 *
 * RETURN VALUE  : status of success or failure
 * 
 * NOTES         : Used the linux data struct "ethtool_drvinfo", not put in command handler
 *--------------------------------------------------------------------------------------*/

MXL_MOCA_STATUS_E MxL_MoCA_DrvProcessEthToolCmd(void *pEthToolCmdBuff)
{
  uint32_t ethToolCmd;
  struct ethtool_drvinfo drvInfo = {ETHTOOL_GDRVINFO};
  MXL_MOCA_STATUS_E status = MXL_MOCA_OK;


  if (NULL == pEthToolCmdBuff)
  {
    MxL_MoCA_OsalPrintLog(TRACE_OSWP, L_ERR, " %s : INVALID input parameter\n", __func__);
    return MXL_MOCA_ERR;
  }
  

  if (MxL_MoCA_OsalCopyFromUser(&ethToolCmd,  (void *)pEthToolCmdBuff, sizeof(ethToolCmd)))
  {
    MxL_MoCA_OsalPrintLog(TRACE_OSWP, L_ERR, " %s : MxL_MoCA_OsalCopyFromUser FAILED!\n", __func__);
    status = MXL_MOCA_ERR;
  }
  else
  {
    switch (ethToolCmd)
    {
      case ETHTOOL_GDRVINFO:

        MxL_MoCA_OsalMemcpy(drvInfo.driver, DRV_NAME, sizeof(DRV_NAME));
        MxL_MoCA_OsalMemcpy(drvInfo.version, VERSION_HOST_STR, sizeof(VERSION_HOST_STR));
        
        if(MxL_MoCA_OsalCopyToUser(pEthToolCmdBuff, &drvInfo, sizeof(drvInfo)))
        {
          MxL_MoCA_OsalPrintLog(TRACE_OSWP, L_ERR, " %s : %d ethToolCmd -- MxL_MoCA_OsalCopyToUser FAILED!\n", __func__, __LINE__);
          status = MXL_MOCA_ERR;
        }
        
        break;

      default:
        status = MXL_MOCA_ERR;
        break;
    }
    
  }

  return status;
}

/*----------------------------------------------------------------------------------------
 * FUNCTION NAME : MxL_MoCA_DrvProcessSIOCSMIIREG
 *
 *
 * DATE CREATED  : Oct/04/2016
 *
 * DESCRIPTION   : Process SIOCSMIIREG command - Write MMI Register
 *
 * IN PARAMS     : mocaDevId Moca device ID
 *                 pSioCmdBuff Pointer to command buffer
 *
 * OUT PARAMS    : MXL_MOCA_STATUS_E
 *
 * RETURN VALUE  : status of success or failure
 * 
 *--------------------------------------------------------------------------------------*/

MXL_MOCA_STATUS_E MxL_MoCA_DrvProcessSIOCSMIIREG(int32_t mocaDevId, void *pSioCmdBuff)
{
  MXL_MOCA_DRV_RW_MII_REG_CMD_T sioCmdBuff;
  MXL_MOCA_DEV_CTRL_MGR_T *pDevCtrlMgr = NULL;
  MXL_MOCA_STATUS_E status = MXL_MOCA_OK;
  
  pDevCtrlMgr = MxL_MoCA_DevGetCtrlMgrInstanceHandle(mocaDevId);

  if (NULL == pDevCtrlMgr)
  {
    status = MXL_MOCA_NODEV_ERR;     
    return status;
  }

  // Copy input parameter from host app to (user mode) to kernel memory
  if (MxL_MoCA_OsalCopyFromUser(&sioCmdBuff, pSioCmdBuff, sizeof(sioCmdBuff)))
  {
      MxL_MoCA_OsalPrintLog(TRACE_OSWP, L_ERR, " %s : %d SET MMI -- MxL_MoCA_OsalCopyToUser FAILED!\n", __func__, __LINE__);
      status = MXL_MOCA_ERR;     
  }
  else
  {
   // Call lower level driver to perform mmi write operation
   status = MxL_MoCA_DevProcessWriteMiiCmd(pDevCtrlMgr, &sioCmdBuff);   

   if (MXL_MOCA_OK != status)
   {
     MxL_MoCA_OsalPrintLog(TRACE_OSWP, L_ERR, " %s : %d SET MMI -- MxL_MoCA_DevProcessWriteMiiCmd FAILED!\n", __func__, __LINE__);
   }
  }

  return status;
}

/*----------------------------------------------------------------------------------------
 * FUNCTION NAME : MxL_MoCA_DrvProcessSIOCGMIIREG
 *
 *
 * DATE CREATED  : Oct/04/2016
 *
 * DESCRIPTION   : Process SIOCGMIIREG command - Read MMI Register
 *
 * IN PARAMS     : mocaDevId Moca device ID
 *                 pSioCmdBuff Pointer to command buffer
 *
 * OUT PARAMS    : MXL_MOCA_STATUS_E
 *
 * RETURN VALUE  : status of success or failure
 * 
 *--------------------------------------------------------------------------------------*/
MXL_MOCA_STATUS_E MxL_MoCA_DrvProcessSIOCGMIIREG(int32_t mocaDevId, void *pSioCmdBuff)
{
  MXL_MOCA_DRV_RW_MII_REG_CMD_T sioCmdBuff;
  MXL_MOCA_DEV_CTRL_MGR_T *pDevCtrlMgr = NULL;  
  MXL_MOCA_STATUS_E status = MXL_MOCA_OK;

  pDevCtrlMgr = MxL_MoCA_DevGetCtrlMgrInstanceHandle(mocaDevId);
  if (NULL == pDevCtrlMgr)
  {
    status = MXL_MOCA_NODEV_ERR;     
    return status;
  }

  // Copy input parameter from host app to (user mode) to kernel memory
  if (MxL_MoCA_OsalCopyFromUser(&sioCmdBuff, pSioCmdBuff, sizeof(sioCmdBuff)))
  {
      MxL_MoCA_OsalPrintLog(TRACE_OSWP, L_ERR, " %s : %d GET MMI -- MxL_MoCA_OsalCopyToUser FAILED!\n", __func__, __LINE__);
      status = MXL_MOCA_ERR;     
  }
  else
  {
    // Call lower level driver to perform mmi read operation
    status = MxL_MoCA_DevProcessReadMiiCmd(pDevCtrlMgr, &sioCmdBuff);  

    if (MXL_MOCA_OK != status)
    {
      MxL_MoCA_OsalPrintLog(TRACE_OSWP, L_ERR, " %s : %d GET MMI -- MxL_MoCA_DevProcessWriteMiiCmd FAILED!\n", __func__, __LINE__);
    }

    // Copy back data read from device using mmi interface
    if (MxL_MoCA_OsalCopyToUser(pSioCmdBuff, &sioCmdBuff, sizeof(sioCmdBuff)))
    {
      MxL_MoCA_OsalPrintLog(TRACE_OSWP, L_ERR, " %s : %d GET MMI -- MxL_MoCA_OsalCopyToUser FAILED!\n", __func__, __LINE__);
      status = MXL_MOCA_ERR;
    }
  }

  return status;
}

static struct file_operations fops =
{
  .read  = MxL_MoCA_DevRead,
  .write = MxL_MoCA_DevWrite,
  .open  = MxL_MoCA_DevOpen,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 36)
  /**************************************************************************
   *     Answer: From The new way of ioctl() by Jonathan Corbet:
   *
   *     ioctl() is one of the remaining parts of the kernel which runs under
   *     the Big Kernel Lock (BKL). In the past, the usage of the BKL has made
   *     it possible for long-running ioctl() methods to create long latencies
   *     for unrelated processes.
   *
   *     Follows an explanation of the patch that introduced unlocked_ioctl
   *     and compat_ioctl into 2.6.11. The removal of the ioctl field happened
   *     a lot later, in 2.6.36.
   *
   *     Explanation: When ioctl was executed, it took the Big Kernel Lock
   *     (BKL), so nothing else could execute at the same time. This is very
   *     bad on a multiprocessor machine, so there was a big effort to get rid
   *     of the BKL. First, unlocked_ioctl was introduced. It lets each driver
   *     writer choose what lock to use instead. This can be difficult, so there
   *     was a period of transition during which old drivers still worked
   *     (using ioctl) but new drivers could use the improved interface
   *     (unlocked_ioctl). Eventually all drivers were converted and ioctl could
   *     be removed.
   *
   *     compat_ioctl is actually unrelated, even though it was added at the same
   *     time. Its purpose is to allow 32-bit userland programs to make ioctl calls
   *     on a 64-bit kernel. The meaning of the last argument to ioctl depends on
   *     the driver, so there is no way to do a driver-independent conversion.
   ********************************************************************************/
  .unlocked_ioctl =  MxL_MoCA_DevIoctl,
#else
  .ioctl   = MxL_MoCA_DevIoctl,
#endif
  .release = MxL_MoCA_DevRelease,
};

/*----------------------------------------------------------------------------------------
 * FUNCTION NAME : MxL_MoCA_DevInit
 *
 *
 * DATE CREATED  : Sep/21/2016
 *
 * DESCRIPTION   : System device init function, called when insmod
 *
 *
 * IN PARAMS     : None
 *
 * OUT PARAMS    : None
 *
 * RETURN VALUE  : status of success or failure
 *--------------------------------------------------------------------------------------*/

static MXL_MOCA_STATUS_E MxL_MoCA_DevInit(void)
{
  MXL_MOCA_STATUS_E result = MXL_MOCA_ERR;
    
  MXL_ENTER_FUNCTION(TRACE_OSWP, "void");

  MxL_MoCA_OsalPrintLog(TRACE_OSWP, L_INFO, "MoCA driver version: v%s\n", VERSION_HOST_STR);

  // Register as character device
  major = register_chrdev(major, DRV_NAME, &fops);

  if (major < 0)
  {
    MxL_MoCA_OsalPrintLog(TRACE_OSWP, L_ERR, "%s %s register_chrdev failed\n", DRV_NAME, VERSION_HOST_STR);
  }
  else
  {
    result = MxL_MoCA_DrvInit();
  }

  if (MXL_MOCA_OK != result)
  {
    if (major >= 0)
      unregister_chrdev(major, DRV_NAME);
  }

  MXL_EXIT_FUNCTION(TRACE_OSWP, "status = %d", result);

  return result;
}

/*----------------------------------------------------------------------------------------
 * FUNCTION NAME : MxL_MoCA_DevExit
 *
 *
 * DATE CREATED  : Sep/21/2016
 *
 * DESCRIPTION   : Driver exit function, un-register the driver
 *                 The global resources will be freed
 *
 *
 * IN PARAMS     : None
 *
 * OUT PARAMS    : None
 *
 * RETURN VALUE  : None
 *--------------------------------------------------------------------------------------*/

static void MxL_MoCA_DevExit(void)
{
  MXL_ENTER_FUNCTION(TRACE_OSWP, "void");

  MxL_MoCA_DrvExit();

  unregister_chrdev(major, DRV_NAME);

  MxL_MoCA_OsalPrintLog(TRACE_OSWP, L_INFO, "%s %s Unloaded\n", DRV_NAME, VERSION_HOST_STR);

  MXL_EXIT_FUNCTION(TRACE_OSWP, "status = %d", MXL_MOCA_OK);
}

/*----------------------------------------------------------------------------------------
 * FUNCTION NAME : MxL_MoCA_DevSysfsShowLof
 *
 *
 * DATE CREATED  : Nov/4/2016
 *
 * DESCRIPTION   : Sysfs function to display the lof in driver which will be 
 *                 updated by daemon
 *                 This function will be called by linux kernel when user issue following 
 *                 command (assume the moca device name is en15):
 *                 `cat /sys/devices/virtual/mxl_moca_ctrl/en15/lof` 
 *
 * IN PARAMS     : pDev  Pointer to the device struct
 *                 pAttr Pointer to device attribute 
 *                 pBuf  Buffer for displaying the lof
 *
 * OUT PARAMS    : none
 *
 * RETURN VALUE  : Size of the displayed value
 *--------------------------------------------------------------------------------------*/

static ssize_t MxL_MoCA_DevSysfsShowLof(struct device *pDev, 
                                        struct device_attribute *pAttr, 
                                        char *pBuf)
{
  int32_t  i;
  ssize_t  ret;
  uint32_t lof;
  MXL_MOCA_DEV_CTRL_MGR_T *pDevCtrlMgr = NULL;

  for (i = 0; i < MAX_SOC_NUMBER; i++)
  {
    if (gMxlMocaDevManager[i].pDev == pDev)
    {
      pDevCtrlMgr = &gMxlMocaDevManager[i];
      break;
    }
  }

  // If read before the device manager init, dislay 0 for lof
  if (NULL != pDevCtrlMgr)
    lof = pDevCtrlMgr->daemonContext.lof;
  else
    lof = 0;

  ret = scnprintf(pBuf, PAGE_SIZE, "%d\n", lof);

  return ret;
}

/*----------------------------------------------------------------------------------------
 * FUNCTION NAME : MxL_MoCA_DevSysfsShowLinkStatus
 *
 *
 * DATE CREATED  : Nov/4/2016
 *
 * DESCRIPTION   : Sysfs function to display the link status in driver which will 
 *                 be updated by daemon. 
 *                 This function will be called by linux kernel when user issue following 
 *                 command (assume the moca device name is en15):
 *                 `cat /sys/devices/virtual/mxl_moca_ctrl/en15/link_status` 
 *
 * IN PARAMS     : pDev  Pointer to the device struct
 *                 pAttr Pointer to device attribute 
 *                 pBuf  Buffer for displaying the link status
 *
 * OUT PARAMS    : none
 *
 * RETURN VALUE  : Size of the displayed value
 *--------------------------------------------------------------------------------------*/

static ssize_t MxL_MoCA_DevSysfsShowLinkStatus(struct device *pDev, 
                                               struct device_attribute *pAttr, 
                                               char *pBuf)
{
  int32_t  i;
  ssize_t  ret;
  uint32_t link_status;
  MXL_MOCA_DEV_CTRL_MGR_T *pDevCtrlMgr = NULL;

  for (i = 0; i < MAX_SOC_NUMBER; i++)
  {
    if (gMxlMocaDevManager[i].pDev == pDev)
    {
      pDevCtrlMgr = &gMxlMocaDevManager[i];
      break;
    }
  }

  // If read before the device manager init, dislay 0 (LINK DOWN) for link status
  if (NULL != pDevCtrlMgr)
    link_status = pDevCtrlMgr->daemonContext.linkUpFlag;
  else
    link_status = 0;

  ret = scnprintf(pBuf, PAGE_SIZE, "%d\n", link_status);

  return ret;
}

/*----------------------------------------------------------------------------------------
 * FUNCTION NAME : MxL_MoCA_DevSysfsShowLinkStatus
 *
 *
 * DATE CREATED  : Jan/27/2017
 *
 * DESCRIPTION   : Sysfs function to display the admission status in driver which will 
 *                 be updated by daemon. 
 *                 This function will be called by linux kernel when user issue following 
 *                 command (assume the moca device name is en15):
 *                 `cat /sys/devices/virtual/mxl_moca_ctrl/en15/adm_status` 
 *
 * IN PARAMS     : pDev  Pointer to the device struct
 *                 pAttr Pointer to device attribute 
 *                 pBuf  Buffer for displaying the admission status
 *
 * OUT PARAMS    : none
 *
 * RETURN VALUE  : Size of the displayed value
 *--------------------------------------------------------------------------------------*/

static ssize_t MxL_MoCA_DevSysfsShowAdmStatus(struct device *pDev, 
                                               struct device_attribute *pAttr, 
                                               char *pBuf)
{
  int32_t  i;
  ssize_t  ret;
  uint32_t adm_status;
  MXL_MOCA_DEV_CTRL_MGR_T *pDevCtrlMgr = NULL;

  for (i = 0; i < MAX_SOC_NUMBER; i++)
  {
    if (gMxlMocaDevManager[i].pDev == pDev)
    {
      pDevCtrlMgr = &gMxlMocaDevManager[i];
      break;
    }
  }

  // If read before the device manager init, dislay 6 (ADM_RESULT_ERROR) for link status
  if (NULL != pDevCtrlMgr)
    adm_status = pDevCtrlMgr->daemonContext.lastAdmStatus;
  else
    adm_status = ADM_RESULT_ERROR;

  ret = scnprintf(pBuf, PAGE_SIZE, "%d\n", adm_status);

  return ret;
}

/*----------------------------------------------------------------------------------------
 * FUNCTION NAME : MxL_MoCA_DevSysfsShowBridgeControlReqStatus
 *
 *
 * DATE CREATED  : Mar/02/2017
 *
 * DESCRIPTION   : Sysfs function to display the proxy packet status in driver which will 
 *                 be updated by daemon. 
 *                 This function will be called by linux kernel when user issue following 
 *                 command (assume the moca device name is en15):
 *                 `cat /sys/devices/virtual/mxl_moca_ctrl/en15/bridge_ctrl_status` 
 *
 * IN PARAMS     : pDev  Pointer to the device struct
 *                 pAttr Pointer to device attribute 
 *                 pBuf  Buffer for displaying the admission status
 *
 * OUT PARAMS    : none
 *
 * RETURN VALUE  : Size of the displayed value
 *--------------------------------------------------------------------------------------*/

static ssize_t MxL_MoCA_DevSysfsShowBridgeControlReqStatus(struct device *pDev, 
                                                           struct device_attribute *pAttr, 
                                                           char *pBuf)
{
  int32_t  i;
  ssize_t  ret;
  uint32_t bridge_ctrl_req_status;
  MXL_MOCA_DEV_CTRL_MGR_T *pDevCtrlMgr = NULL;

  for (i = 0; i < MAX_SOC_NUMBER; i++)
  {
    if (gMxlMocaDevManager[i].pDev == pDev)
    {
      pDevCtrlMgr = &gMxlMocaDevManager[i];
      break;
    }
  }

  // If read before the device manager init, dislay 0 (LINK DOWN) for link status
  if (NULL != pDevCtrlMgr)
    bridge_ctrl_req_status = (pDevCtrlMgr->daemonContext.reqStatus & 2) >> 1; /* Bridge Control */
  else
    bridge_ctrl_req_status = 0;

  ret = scnprintf(pBuf, PAGE_SIZE, "%d\n", bridge_ctrl_req_status);

  return ret;
}


/*----------------------------------------------------------------------------------------
 * FUNCTION NAME : MxL_MoCA_DevSysfsShowProxyReqStatus
 *
 *
 * DATE CREATED  : Jan/27/2017
 *
 * DESCRIPTION   : Sysfs function to display the proxy packet status in driver which will 
 *                 be updated by daemon. 
 *                 This function will be called by linux kernel when user issue following 
 *                 command (assume the moca device name is en15):
 *                 `cat /sys/devices/virtual/mxl_moca_ctrl/en15/net_proxy_req_status` 
 *
 * IN PARAMS     : pDev  Pointer to the device struct
 *                 pAttr Pointer to device attribute 
 *                 pBuf  Buffer for displaying the admission status
 *
 * OUT PARAMS    : none
 *
 * RETURN VALUE  : Size of the displayed value
 *--------------------------------------------------------------------------------------*/

static ssize_t MxL_MoCA_DevSysfsShowProxyReqStatus(struct device *pDev, 
                                                   struct device_attribute *pAttr, 
                                                   char *pBuf)
{
  int32_t  i;
  ssize_t  ret;
  uint32_t net_proxy_req_status;
  MXL_MOCA_DEV_CTRL_MGR_T *pDevCtrlMgr = NULL;

  for (i = 0; i < MAX_SOC_NUMBER; i++)
  {
    if (gMxlMocaDevManager[i].pDev == pDev)
    {
      pDevCtrlMgr = &gMxlMocaDevManager[i];
      break;
    }
  }

  // If read before the device manager init, dislay 0 (LINK DOWN) for link status
  if (NULL != pDevCtrlMgr)
    net_proxy_req_status = pDevCtrlMgr->daemonContext.reqStatus & 1; /* Net Proxy */
  else
    net_proxy_req_status = 0;

  ret = scnprintf(pBuf, PAGE_SIZE, "%d\n", net_proxy_req_status);

  return ret;
}

/*----------------------------------------------------------------------------------------
 * FUNCTION NAME : MxL_MoCA_DevSysfsShowBeaconPowerDistStatus
 *
 *
 * DATE CREATED  : Jun/28/2017
 *
 * DESCRIPTION   : Sysfs function to display the beacon power distributed in driver 
 *                 which will be updated by daemon
 *                 This function will be called by linux kernel when user issue following 
 *                 command (assume the moca device name is en15):
 *                 `cat /sys/devices/virtual/mxl_moca_ctrl/en15/beacon_power_dist_status` 
 *
 * IN PARAMS     : pDev  Pointer to the device struct
 *                 pAttr Pointer to device attribute 
 *                 pBuf  Buffer for displaying the lof
 *
 * OUT PARAMS    : none
 *
 * RETURN VALUE  : Size of the displayed value
 *--------------------------------------------------------------------------------------*/

static ssize_t MxL_MoCA_DevSysfsShowBeaconPowerDistStatus(struct device *pDev, 
                                                          struct device_attribute *pAttr, 
                                                          char *pBuf)
{
  int32_t  i;
  ssize_t  ret;
  uint32_t beacon_power_dist_status;
  MXL_MOCA_DEV_CTRL_MGR_T *pDevCtrlMgr = NULL;

  for (i = 0; i < MAX_SOC_NUMBER; i++)
  {
    if (gMxlMocaDevManager[i].pDev == pDev)
    {
      pDevCtrlMgr = &gMxlMocaDevManager[i];
      break;
    }
  }

  // If read before the device manager init, dislay 0 for beacon power dist status
  if (NULL != pDevCtrlMgr)
    beacon_power_dist_status = (pDevCtrlMgr->daemonContext.reqStatus >> 3) & 1; /* Beacon Power */
  else
    beacon_power_dist_status = 0;

  ret = scnprintf(pBuf, PAGE_SIZE, "%d\n", beacon_power_dist_status);

  return ret;
}

/*----------------------------------------------------------------------------------------
 * FUNCTION NAME : MxL_MoCA_DevSysfsShowPrvacyUpdateStatus
 *
 *
 * DATE CREATED  : Mar/15/2017
 *
 * DESCRIPTION   : Sysfs function to display the privacy update status in driver which will 
 *                 be updated by daemon. 
 *                 This function will be called by linux kernel when user issue following 
 *                 command (assume the moca device name is en15):
 *                 `cat /sys/devices/virtual/mxl_moca_ctrl/en15/privacy_update_status` 
 *
 * IN PARAMS     : pDev  Pointer to the device struct
 *                 pAttr Pointer to device attribute 
 *                 pBuf  Buffer for displaying the admission status
 *
 * OUT PARAMS    : none
 *
 * RETURN VALUE  : Size of the displayed value
 *--------------------------------------------------------------------------------------*/

static ssize_t MxL_MoCA_DevSysfsShowPrvacyUpdateStatus(struct device *pDev, 
                                                       struct device_attribute *pAttr, 
                                                       char *pBuf)
{
  int32_t  i;
  ssize_t  ret;
  uint32_t privacy_update_status;
  MXL_MOCA_DEV_CTRL_MGR_T *pDevCtrlMgr = NULL;

  for (i = 0; i < MAX_SOC_NUMBER; i++)
  {
    if (gMxlMocaDevManager[i].pDev == pDev)
    {
      pDevCtrlMgr = &gMxlMocaDevManager[i];
      break;
    }
  }

  // If read before the device manager init, dislay 0 (LINK DOWN) for privacy update status
  if (NULL != pDevCtrlMgr)
    privacy_update_status = (pDevCtrlMgr->daemonContext.reqStatus >> 2) & 1; /* Bit2: Privacy update */
  else
    privacy_update_status = 0;

  ret = scnprintf(pBuf, PAGE_SIZE, "%d\n", privacy_update_status);

  return ret;
}

// dev_attr_lof and dev_attr_link_status used to create sysfs files for displaying lof and link_status
DEVICE_ATTR(lof, S_IRUGO, MxL_MoCA_DevSysfsShowLof, NULL);
DEVICE_ATTR(link_status, S_IRUGO, MxL_MoCA_DevSysfsShowLinkStatus, NULL);
DEVICE_ATTR(adm_status, S_IRUGO, MxL_MoCA_DevSysfsShowAdmStatus, NULL);
DEVICE_ATTR(bridge_ctrl_req_status, S_IRUGO, MxL_MoCA_DevSysfsShowBridgeControlReqStatus, NULL);
DEVICE_ATTR(net_proxy_req_status, S_IRUGO, MxL_MoCA_DevSysfsShowProxyReqStatus, NULL);
DEVICE_ATTR(beacon_power_dist_status, S_IRUGO, MxL_MoCA_DevSysfsShowBeaconPowerDistStatus, NULL);
DEVICE_ATTR(privacy_update_status, S_IRUGO, MxL_MoCA_DevSysfsShowPrvacyUpdateStatus, NULL);


module_init(MxL_MoCA_DevInit);
module_exit(MxL_MoCA_DevExit);
