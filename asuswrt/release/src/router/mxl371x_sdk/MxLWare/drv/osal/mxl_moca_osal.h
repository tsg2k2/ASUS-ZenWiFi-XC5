/*****************************************************************************************   
 *
 * FILE NAME          : mxl_moca_osal.h
 * 
 *
 *
 * DATE CREATED       : Sep/20/2016
 *
 * LAST MODIFIED      : 
 *
 * DESCRIPTION        : This file contains MoCA device related define and data structrue
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

#ifndef __MXL_MOCA_OSAL_H__
#define __MXL_MOCA_OSAL_H__


/*****************************************************************************************
    Include Header Files
    (No absolute paths - paths handled by make file)
 ****************************************************************************************/
#include "mxl_moca_osal_linux.h"
#include "mxl_data_types.h"
#include "mxl_moca_config.h"
#include "mxl_moca_soc_cmd.h"

/*****************************************************************************************
    Macros
 ****************************************************************************************/
#define LOG_PRINT_BUFSIZE  1024

/*****************************************************************************************
    User-Defined Types (Typedefs)
 ****************************************************************************************/
/// OS timer expiration callback function
typedef void (*timer_function_t)(long);

/// Wait Q Timer exit condition function
typedef int32_t (*osal_wqt_condition)(void *vp);

/// Wait Q exit condition function
typedef int32_t (*osal_waitq_condition)(void *vp);

typedef struct
{
  void *dev;
  void *dkcp;
} osal_device_t;

enum
{
    L_ERR = 0,
    L_WARN,
    L_INFO,
    L_VERBOSE,
    L_DBG,
    L_TMP,
};

// HostOS_ control -- define to have console print
#define HOST_OS_PRINTLOG_THRESHOLD            L_INFO
#define MXL_ENABLE_DEBUG_PRINT                0
#define MXL_ENABLE_DEBUG_ENTER_EXIT_FUNCTION  0

#if MXL_ENABLE_DEBUG_PRINT
#define DEBUG_PRINT(mod, lev, fmt, args...)  MxL_MoCA_OsalPrintLog(mod, lev,"(%s)%d " fmt " \n", __func__, __LINE__, ##args)
#else
#define DEBUG_PRINT(mod, lev, fmt, args...)
#endif

#if MXL_ENABLE_DEBUG_ENTER_EXIT_FUNCTION
#define MXL_ENTER_FUNCTION(mod, fmt, args...) MxL_MoCA_OsalPrintLog(mod, L_INFO, "+++ %s(), line: %d " fmt "\n", __func__, __LINE__, ##args)
#define MXL_EXIT_FUNCTION(mod, fmt, args...)  MxL_MoCA_OsalPrintLog(mod, L_INFO, "--- %s(), line: %d " fmt "\n", __func__, __LINE__, ##args)
#else
#define MXL_ENTER_FUNCTION(mod, fmt, args...)
#define MXL_EXIT_FUNCTION(mod, fmt, args...)
#endif

/*****************************************************************************************
    Global Variable Declarations
 ****************************************************************************************/

/*****************************************************************************************
    Function Prototypes
 ****************************************************************************************/
struct MXL_MOCA_DAEMON_STARTUP_OPTIONS_T *MxL_MoCA_OsalGetDsoFromIface(void *iface);
void MxL_MoCA_OsalAllocGetTimeInit(void **tv);
void MxL_MoCA_OsalGetTime(void *tv);
void MxL_MoCA_OsalAllocGetTime(void **tv);
long MxL_MoCA_OsalDiffTimeInUs(void *tv1, void *tv2);
long MxL_MoCA_OsalDiffTime(void *tv1, void *tv2);
void MxL_MoCA_OsalAssignTime(void *tv1, void *tv2);
void MxL_MoCA_OsalFreeGetTime(void *tv);
void *MxL_MoCA_OsalAlloc(int32_t size);
void MxL_MoCA_OsalFree(void *pMem, int32_t size);

void MxL_MoCA_OsalMemcpy(void *pTo, void *pFrom, int32_t size);
void MxL_MoCA_OsalMemset(void *pMem, int32_t val, int32_t size);
void MxL_MoCA_OsalPrintLog(int32_t mod, int32_t lev, const int8_t *fmt, ...);
void MxL_MoCA_OsalPrintString(int8_t* pStr);
void MxL_MoCA_OsalSetLogLev(int32_t mod, int32_t lev);
void MxL_MoCA_OsalDelay(int32_t timeInUsec);
void MxL_MoCA_OsalDelayUsec(uint32_t timeInUsec);
void MxL_MoCA_OsalMSleep(int32_t timeInMilliSec);
void MxL_MoCA_OsalSscanf(const int8_t *buf, const int8_t *fmt, ...);
int32_t MxL_MoCA_OsalTimerMod(void *pTmr, unsigned long timeout);

unsigned long MxL_MoCA_OsalCopyFromUser(void *to, const void *from, unsigned long nbytes);
unsigned long MxL_MoCA_OsalCopyToUser(void *to, const void *from, unsigned long nbytes);

/// Mutex/Semaphore API's used by MxLWare while accessing shared resources like MDIO driver
void MxL_MoCA_OsalMutexInit(void *vmt);
void MxL_MoCA_OsalMutexRelease(void *vmt);
void MxL_MoCA_OsalMutexAcquire(void *vmt);
int32_t MxL_MoCA_OsalMutexAcquireIntr(void *vmt);
void *MxL_MoCA_OsalGetResetMutex(void);

// MXL OSAL Semaphore API's
MXL_MOCA_STATUS_E MxL_MoCA_OsalCreateLock(MxL_OSAL_LOCK_T *pLock);
void MxL_MoCA_OsalDestroyLock(MxL_OSAL_LOCK_T *pLock);
void MxL_MoCA_OsalLock(MxL_OSAL_LOCK_T *pLock);
void MxL_MoCA_OsalUnlock(MxL_OSAL_LOCK_T *pLock);

unsigned long MxL_MoCA_OsalTimerExpireSeconds(uint32_t future);
unsigned long MxL_MoCA_OsalTimerExpireTicks(uint32_t future);
void *MxL_MoCA_OsalWqtAlloc(void);
void MxL_MoCA_OsalWqtFree(void *vwqt);
void MxL_MoCA_OsalWqtTimerInit(void *vwqt);
void MxL_MoCA_OsalWqtTimerDel(void *vwqt);
int32_t MxL_MoCA_OsalWqtTimerDelSync(void *vwqt);
int32_t MxL_MoCA_OsalWqtTimerMod(void *vwqt, unsigned long timeout);
void MxL_MoCA_OsalWqtTimerAdd(void *vwqt);
void MxL_MoCA_OsalWqtTimerSetup(void *vwqt, timer_function_t func, unsigned long data);
void MxL_MoCA_OsalWqtTimerSetTimeout(void *vwqt, unsigned long timeout);
void MxL_MoCA_OsalWqtWaitqInit(void *vwqt);
void MxL_MoCA_OsalWqtWaitqWakeupIntr(void *vwqt);
void MxL_MoCA_OsalWqtWaitqWaitEventIntr(void *vwqt, osal_wqt_condition func, void *vp);
int32_t MxL_MoCA_OsalSignalPending(void *vtask);
void MxL_MoCA_OsalMsleepInterruptible(uint32_t msecs);
MXL_MOCA_STATUS_E MxL_MoCA_OsalThreadCreateAndStart(int32_t (*pThreadFn)(void *pData),
                                                    void *pData,
                                                    const int8_t *pNameFmt,
                                                    void *pArg,
                                                    void **ppTask);
MXL_MOCA_STATUS_E MxL_MoCA_OsalTestPtrErrCondition(void *pTask);

MXL_MOCA_STATUS_E MxL_MoCA_OsalGetDaemonThreadStopStatus(void);
MXL_MOCA_STATUS_E MxL_MoCA_OsalDetectPCIeDevice(int32_t vendor, int32_t device, int8_t checkDriverLoad);
int32_t MxL_MoCA_OsalThreadStart(uint32_t *pThreadID, int8_t *pName, void (*func)(void *), void *pArg);
int32_t MxL_MoCA_OsalThreadStop(unsigned long threadID);
uint32_t MxL_MoCA_OsalReadWord( void *pAddr );
void MxL_MoCA_OsalWriteWord( uint32_t val, void *pAddr);

void MxL_MoCA_OsalLogInfo(int8_t *pFormat, ...);
int32_t MxL_MoCA_OsalKthreadStop(void *pTsk);
void *MxL_MoCA_OsalGetPointerToFirmwareData(MXL_MOCA_FIRMWARE_T *pFw, int32_t index);
int32_t MxL_MoCA_OsalGetFirmwareLen(MXL_MOCA_FIRMWARE_T *pFw, int32_t index);
void MxL_MoCA_OsalFreeFirmwares(MXL_MOCA_FIRMWARE_T *pFw);
MXL_MOCA_STATUS_E MxL_MoCA_OsalGetFirmwareAndConfigData(uint32_t chipTypeIndex, void *pDev, 
                                                        const int8_t *pDevName, 
                                                        MXL_MOCA_FIRMWARE_T *pFw,
                                                        int32_t*  pFwBitMaskRet);
int8_t *MxL_MoCA_OsalStrcpy(int8_t *pDst, const int8_t *pSrc);
int32_t MxL_MoCA_OsalSnprintf(int8_t *pBuf, uint32_t size, const int8_t *pFmt, ...);
uint16_t MxL_MoCA_OsalHtons(uint16_t n);
uint32_t MxL_MoCA_OsalHtonl(uint32_t n);
uint32_t MxL_MoCA_OsalNtohl(uint32_t n);
uint16_t MxL_MoCA_OsalNtohs(uint16_t n);
int8_t *MxL_MoCA_OsalStrstr(int8_t *pStr1, int8_t *pStr2);
//MOCA_UEVENT
#if ENABLE_ADM_STATUS_UEVENT || ENABLE_LINK_UEVENT
void Mxl_MoCA_OsalGenerateUevent(void * pDev);
#endif
//MOCA_UEVENT
MXL_MOCA_STATUS_E MxL_MoCA_OsalCreateMocaDev(uint8_t ePhyAddr, void** ppDev);

MXL_MOCA_STATUS_E MxL_MoCA_OsalCheckImage(uint8_t *pElfData, uint32_t elfSize);
void *MxL_MoCA_OsalGetFirstPhdr(uint8_t *pElfData);
uint32_t MxL_MoCA_OsalGetNumOfProgHeader(void *pData);
uint32_t MxL_MoCA_OsalGetProgHeaderPhyAddr(void *pHdr);
uint32_t MxL_MoCA_OsalGetProgHeaderOffset(void *pHdr);
uint32_t MxL_MoCA_OsalGetProgHeaderFileSZ(void *pHdr);
void *MxL_MoCA_OsalNextProgHeader(void *pHdr);

void *MxL_MoCA_OsalGetFirstShdr(uint8_t * elfData);
uint32_t MxL_MoCA_OsalGetNumOfSecHeaders(void *pData);
uint32_t MxL_MoCA_OsalGetSecIndexByName(void *pData, char* pName);
uint32_t MxL_MoCA_OsalGetSecFileOffset(void *pData, uint32_t sectionId);

#if defined(INCLUDE_PCI)
/* DP interface func */
void MxL_MoCA_OsalSetMacAddress(uint32_t devIndex, uint32_t *pMac);
MXL_MOCA_STATUS_E MxL_MoCA_OsalPcieAvailable(int8_t devIndex);
MXL_MOCA_STATUS_E MxL_MoCA_OsalPcieBusRead(uint32_t devIndex, unsigned long addr, uint32_t *pData);
MXL_MOCA_STATUS_E MxL_MoCA_OsalPcieBusWrite(uint32_t devIndex, unsigned long addr, uint32_t data);
MXL_MOCA_STATUS_E MxL_MoCA_OsalDataPlaneIfRestart(uint32_t devIndex);
MXL_MOCA_STATUS_E MxL_MoCA_OsalDataPlaneResetSoC(uint32_t devIndex);
#endif

void MxL_MoCA_OsalDestroyMocaDev(uint8_t ePhyAddr, void* ppDev);
void MxL_MoCA_OsalDestroyMocaDevClass(void);
#endif // __MXL_MOCA_OSAL_H__
