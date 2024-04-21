// SPDX-License-Identifier: GPL-2.0+
/*
   Copyright (c) 2015 Broadcom
   All Rights Reserved

*/

#ifndef _BCM4912_XPORT_INTR_AG_H_
#define _BCM4912_XPORT_INTR_AG_H_

#include "access_macros.h"
#include "bcmtypes.h"

/**************************************************************************************************/
/* link_down_intr:  - Link down interrupt for P3:P0. Interrupt corresponds to the attached PHY/in */
/*                 terface link down event.                                                       */
/* link_up_intr:  - Link up interrupt for P3:P0. Interrupt corresponds to the attached PHY/interf */
/*               ace link up event.                                                               */
/* tx_timesync_fifo_entry_valid_intr:  - PTP timestamp available for read. Bits [3:0] correspond  */
/*                                    to XLMAC ports 3 down to 0.                                 */
/* xlmac_intr:  - Interrupt generated by XLMAC signaling various events as described in XLMAC_INT */
/*             R_STATUS.Bits [3:0] correspond to XLMAC ports 3 down to 0.                         */
/* mac_reg_err_intr:  - mac_reg_err_intr.
XLMAC register transaction error                        */
/* mab_status_intr:  - Asserted when any of statuses get set in MSBUS Adaptation Block Status Reg */
/*                  ister(s).                                                                     */
/* mib_reg_err_intr:  - mib_reg_err_intr.
MIB register transaction error.                         */
/* ubus_err_intr:  - UBUS transaction error.                                                      */
/**************************************************************************************************/
typedef struct
{
    uint8_t link_down_intr;
    uint8_t link_up_intr;
    uint8_t tx_timesync_fifo_entry_valid_intr;
    uint8_t xlmac_intr;
    uint8_t mac_reg_err_intr;
    uint8_t mab_status_intr;
    uint8_t mib_reg_err_intr;
    uint8_t ubus_err_intr;
} xport_intr_cpu_status;


/**************************************************************************************************/
/* link_down_intr:  - Link down interrupt for P3:P0. Interrupt corresponds to the attached PHY/in */
/*                 terface link down event.                                                       */
/* link_up_intr:  - Link up interrupt for P3:P0. Interrupt corresponds to the attached PHY/interf */
/*               ace link up event.                                                               */
/* tx_timesync_fifo_entry_valid_intr:  - PTP timestamp available for read. Bits [3:0] correspond  */
/*                                    to XLMAC ports 3 down to 0.                                 */
/* xlmac_intr:  - Interrupt generated by XLMAC signaling various events as described in XLMAC_INT */
/*             R_STATUS.Bits [3:0] correspond to XLMAC ports 3 down to 0.                         */
/* mac_reg_err_intr:  - mac_reg_err_intr.
XLMAC register transaction error                        */
/* mab_status_intr:  - Asserted when any of statuses get set in MSBUS Adaptation Block Status Reg */
/*                  ister(s).                                                                     */
/* mib_reg_err_intr:  - mib_reg_err_intr.
MIB register transaction error.                         */
/* ubus_err_intr:  - UBUS transaction error.                                                      */
/**************************************************************************************************/
typedef struct
{
    uint8_t link_down_intr;
    uint8_t link_up_intr;
    uint8_t tx_timesync_fifo_entry_valid_intr;
    uint8_t xlmac_intr;
    uint8_t mac_reg_err_intr;
    uint8_t mab_status_intr;
    uint8_t mib_reg_err_intr;
    uint8_t ubus_err_intr;
} xport_intr_cpu_set;


/**************************************************************************************************/
/* link_down_intr:  - Link down interrupt for P3:P0. Interrupt corresponds to the attached PHY/in */
/*                 terface link down event.                                                       */
/* link_up_intr:  - Link up interrupt for P3:P0. Interrupt corresponds to the attached PHY/interf */
/*               ace link up event.                                                               */
/* tx_timesync_fifo_entry_valid_intr:  - PTP timestamp available for read. Bits [3:0] correspond  */
/*                                    to XLMAC ports 3 down to 0.                                 */
/* xlmac_intr:  - Interrupt generated by XLMAC signaling various events as described in XLMAC_INT */
/*             R_STATUS.Bits [3:0] correspond to XLMAC ports 3 down to 0.                         */
/* mac_reg_err_intr:  - mac_reg_err_intr.
XLMAC register transaction error                        */
/* mab_status_intr:  - Asserted when any of statuses get set in MSBUS Adaptation Block Status Reg */
/*                  ister(s).                                                                     */
/* mib_reg_err_intr:  - mib_reg_err_intr.
MIB register transaction error.                         */
/* ubus_err_intr:  - UBUS transaction error.                                                      */
/**************************************************************************************************/
typedef struct
{
    uint8_t link_down_intr;
    uint8_t link_up_intr;
    uint8_t tx_timesync_fifo_entry_valid_intr;
    uint8_t xlmac_intr;
    uint8_t mac_reg_err_intr;
    uint8_t mab_status_intr;
    uint8_t mib_reg_err_intr;
    uint8_t ubus_err_intr;
} xport_intr_cpu_clear;


/**************************************************************************************************/
/* link_down_intr_mask:  - This bit corresponds to the mask for link down interrupt. See register */
/*                       description above for details of how to use this bit.                    */
/* link_up_intr_mask:  - This bit corresponds to the mask for link up interrupt. See register des */
/*                    cription above for details of how to use this bit.                          */
/* tx_timesync_fifo_entry_valid_intr_mask:  - This bit corresponds to the mask for  tx timesync f */
/*                                         ifo entry valid interrupt. See register description ab */
/*                                         ove for details of how to use this bit.                */
/* xlmac_intr_mask:  - This bit corresponds to the mask for xlmac interrupt. See register descrip */
/*                  tion above for details of how to use this bit.                                */
/* mac_reg_err_intr_mask:  - This bit corresponds to the mask for mac register error interrupt. S */
/*                        ee register description above for details of how to use this bit.       */
/* mab_status_intr_mask:  - This bit corresponds to the mask for msbus adaptation block status in */
/*                       terrupt. See register description above for details of how to use this b */
/*                       it.                                                                      */
/* mib_reg_err_intr_mask:  - This bit corresponds to the mask for mib register error interrupt. S */
/*                        ee register description above for details of how to use this bit.       */
/* ubus_err_intr_mask:  - This bit corresponds to the mask for ubus error interrupt. See register */
/*                      description above for details of how to use this bit.                     */
/**************************************************************************************************/
typedef struct
{
    uint8_t link_down_intr_mask;
    uint8_t link_up_intr_mask;
    uint8_t tx_timesync_fifo_entry_valid_intr_mask;
    uint8_t xlmac_intr_mask;
    uint8_t mac_reg_err_intr_mask;
    uint8_t mab_status_intr_mask;
    uint8_t mib_reg_err_intr_mask;
    uint8_t ubus_err_intr_mask;
} xport_intr_cpu_mask_status;


/**************************************************************************************************/
/* link_down_intr_mask:  - This bit corresponds to the mask for link down interrupt. See register */
/*                       description above for details of how to use this bit.                    */
/* link_up_intr_mask:  - This bit corresponds to the mask for link up interrupt. See register des */
/*                    cription above for details of how to use this bit.                          */
/* tx_timesync_fifo_entry_valid_intr_mask:  - This bit corresponds to the mask for  tx timesync f */
/*                                         ifo entry valid interrupt. See register description ab */
/*                                         ove for details of how to use this bit.                */
/* xlmac_intr_mask:  - This bit corresponds to the mask for xlmac interrupt. See register descrip */
/*                  tion above for details of how to use this bit.                                */
/* mac_reg_err_intr_mask:  - This bit corresponds to the mask for mac register error interrupt. S */
/*                        ee register description above for details of how to use this bit.       */
/* mab_status_intr_mask:  - This bit corresponds to the mask for msbus adaptation block status in */
/*                       terrupt. See register description above for details of how to use this b */
/*                       it.                                                                      */
/* mib_reg_err_intr_mask:  - This bit corresponds to the mask for mib register error interrupt. S */
/*                        ee register description above for details of how to use this bit.       */
/* ubus_err_intr_mask:  - This bit corresponds to the mask for ubus error interrupt. See register */
/*                      description above for details of how to use this bit.                     */
/**************************************************************************************************/
typedef struct
{
    uint8_t link_down_intr_mask;
    uint8_t link_up_intr_mask;
    uint8_t tx_timesync_fifo_entry_valid_intr_mask;
    uint8_t xlmac_intr_mask;
    uint8_t mac_reg_err_intr_mask;
    uint8_t mab_status_intr_mask;
    uint8_t mib_reg_err_intr_mask;
    uint8_t ubus_err_intr_mask;
} xport_intr_cpu_mask_set;


/**************************************************************************************************/
/* link_down_intr_mask:  - This bit corresponds to the mask for link down interrupt. See register */
/*                       description above for details of how to use this bit.                    */
/* link_up_intr_mask:  - This bit corresponds to the mask for link up interrupt. See register des */
/*                    cription above for details of how to use this bit.                          */
/* tx_timesync_fifo_entry_valid_intr_mask:  - This bit corresponds to the mask for  tx timesync f */
/*                                         ifo entry valid interrupt. See register description ab */
/*                                         ove for details of how to use this bit.                */
/* xlmac_intr_mask:  - This bit corresponds to the mask for xlmac interrupt. See register descrip */
/*                  tion above for details of how to use this bit.                                */
/* mac_reg_err_intr_mask:  - This bit corresponds to the mask for mac register error interrupt. S */
/*                        ee register description above for details of how to use this bit.       */
/* mab_status_intr_mask:  - This bit corresponds to the mask for msbus adaptation block status in */
/*                       terrupt. See register description above for details of how to use this b */
/*                       it.                                                                      */
/* mib_reg_err_intr_mask:  - This bit corresponds to the mask for mib register error interrupt. S */
/*                        ee register description above for details of how to use this bit.       */
/* ubus_err_intr_mask:  - This bit corresponds to the mask for ubus error interrupt. See register */
/*                      description above for details of how to use this bit.                     */
/**************************************************************************************************/
typedef struct
{
    uint8_t link_down_intr_mask;
    uint8_t link_up_intr_mask;
    uint8_t tx_timesync_fifo_entry_valid_intr_mask;
    uint8_t xlmac_intr_mask;
    uint8_t mac_reg_err_intr_mask;
    uint8_t mab_status_intr_mask;
    uint8_t mib_reg_err_intr_mask;
    uint8_t ubus_err_intr_mask;
} xport_intr_cpu_mask_clear;


/**************************************************************************************************/
/* link_down_intr:  - Link down interrupt for P3:P0. Interrupt corresponds to the attached PHY/in */
/*                 terface link down event.                                                       */
/* link_up_intr:  - Link up interrupt for P3:P0. Interrupt corresponds to the attached PHY/interf */
/*               ace link up event.                                                               */
/* tx_timesync_fifo_entry_valid_intr:  - PTP timestamp available for read. Bits [3:0] correspond  */
/*                                    to XLMAC ports 3 down to 0.                                 */
/* xlmac_intr:  - Interrupt generated by XLMAC signaling various events as described in XLMAC_INT */
/*             R_STATUS.Bits [3:0] correspond to XLMAC ports 3 down to 0.                         */
/* mac_reg_err_intr:  - mac_reg_err_intr.
XLMAC register transaction error                        */
/* mab_status_intr:  - Asserted when any of statuses get set in MSBUS Adaptation Block Status Reg */
/*                  ister(s).                                                                     */
/* mib_reg_err_intr:  - mib_reg_err_intr.
MIB register transaction error.                         */
/* ubus_err_intr:  - UBUS transaction error.                                                      */
/**************************************************************************************************/
typedef struct
{
    uint8_t link_down_intr;
    uint8_t link_up_intr;
    uint8_t tx_timesync_fifo_entry_valid_intr;
    uint8_t xlmac_intr;
    uint8_t mac_reg_err_intr;
    uint8_t mab_status_intr;
    uint8_t mib_reg_err_intr;
    uint8_t ubus_err_intr;
} xport_intr_pci_status;


/**************************************************************************************************/
/* link_down_intr:  - Link down interrupt for P3:P0. Interrupt corresponds to the attached PHY/in */
/*                 terface link down event.                                                       */
/* link_up_intr:  - Link up interrupt for P3:P0. Interrupt corresponds to the attached PHY/interf */
/*               ace link up event.                                                               */
/* tx_timesync_fifo_entry_valid_intr:  - PTP timestamp available for read. Bits [3:0] correspond  */
/*                                    to XLMAC ports 3 down to 0.                                 */
/* xlmac_intr:  - Interrupt generated by XLMAC signaling various events as described in XLMAC_INT */
/*             R_STATUS.Bits [3:0] correspond to XLMAC ports 3 down to 0.                         */
/* mac_reg_err_intr:  - mac_reg_err_intr.
XLMAC register transaction error                        */
/* mab_status_intr:  - Asserted when any of statuses get set in MSBUS Adaptation Block Status Reg */
/*                  ister(s).                                                                     */
/* mib_reg_err_intr:  - mib_reg_err_intr.
MIB register transaction error.                         */
/* ubus_err_intr:  - UBUS transaction error.                                                      */
/**************************************************************************************************/
typedef struct
{
    uint8_t link_down_intr;
    uint8_t link_up_intr;
    uint8_t tx_timesync_fifo_entry_valid_intr;
    uint8_t xlmac_intr;
    uint8_t mac_reg_err_intr;
    uint8_t mab_status_intr;
    uint8_t mib_reg_err_intr;
    uint8_t ubus_err_intr;
} xport_intr_pci_set;


/**************************************************************************************************/
/* link_down_intr:  - Link down interrupt for P3:P0. Interrupt corresponds to the attached PHY/in */
/*                 terface link down event.                                                       */
/* link_up_intr:  - Link up interrupt for P3:P0. Interrupt corresponds to the attached PHY/interf */
/*               ace link up event.                                                               */
/* tx_timesync_fifo_entry_valid_intr:  - PTP timestamp available for read. Bits [3:0] correspond  */
/*                                    to XLMAC ports 3 down to 0.                                 */
/* xlmac_intr:  - Interrupt generated by XLMAC signaling various events as described in XLMAC_INT */
/*             R_STATUS.Bits [3:0] correspond to XLMAC ports 3 down to 0.                         */
/* mac_reg_err_intr:  - mac_reg_err_intr.
XLMAC register transaction error                        */
/* mab_status_intr:  - Asserted when any of statuses get set in MSBUS Adaptation Block Status Reg */
/*                  ister(s).                                                                     */
/* mib_reg_err_intr:  - mib_reg_err_intr.
MIB register transaction error.                         */
/* ubus_err_intr:  - UBUS transaction error.                                                      */
/**************************************************************************************************/
typedef struct
{
    uint8_t link_down_intr;
    uint8_t link_up_intr;
    uint8_t tx_timesync_fifo_entry_valid_intr;
    uint8_t xlmac_intr;
    uint8_t mac_reg_err_intr;
    uint8_t mab_status_intr;
    uint8_t mib_reg_err_intr;
    uint8_t ubus_err_intr;
} xport_intr_pci_clear;


/**************************************************************************************************/
/* link_down_intr_mask:  - This bit corresponds to the mask for link down interrupt. See register */
/*                       description above for details of how to use this bit.                    */
/* link_up_intr_mask:  - This bit corresponds to the mask for link up interrupt. See register des */
/*                    cription above for details of how to use this bit.                          */
/* tx_timesync_fifo_entry_valid_intr_mask:  - This bit corresponds to the mask for  tx timesync f */
/*                                         ifo entry valid interrupt. See register description ab */
/*                                         ove for details of how to use this bit.                */
/* xlmac_intr_mask:  - This bit corresponds to the mask for xlmac interrupt. See register descrip */
/*                  tion above for details of how to use this bit.                                */
/* mac_reg_err_intr_mask:  - This bit corresponds to the mask for mac register error interrupt. S */
/*                        ee register description above for details of how to use this bit.       */
/* mab_status_intr_mask:  - This bit corresponds to the mask for msbus adaptation block status in */
/*                       terrupt. See register description above for details of how to use this b */
/*                       it.                                                                      */
/* mib_reg_err_intr_mask:  - This bit corresponds to the mask for mib register error interrupt. S */
/*                        ee register description above for details of how to use this bit.       */
/* ubus_err_intr_mask:  - This bit corresponds to the mask for ubus error interrupt. See register */
/*                      description above for details of how to use this bit.                     */
/**************************************************************************************************/
typedef struct
{
    uint8_t link_down_intr_mask;
    uint8_t link_up_intr_mask;
    uint8_t tx_timesync_fifo_entry_valid_intr_mask;
    uint8_t xlmac_intr_mask;
    uint8_t mac_reg_err_intr_mask;
    uint8_t mab_status_intr_mask;
    uint8_t mib_reg_err_intr_mask;
    uint8_t ubus_err_intr_mask;
} xport_intr_pci_mask_status;


/**************************************************************************************************/
/* link_down_intr_mask:  - This bit corresponds to the mask for link down interrupt. See register */
/*                       description above for details of how to use this bit.                    */
/* link_up_intr_mask:  - This bit corresponds to the mask for link up interrupt. See register des */
/*                    cription above for details of how to use this bit.                          */
/* tx_timesync_fifo_entry_valid_intr_mask:  - This bit corresponds to the mask for  tx timesync f */
/*                                         ifo entry valid interrupt. See register description ab */
/*                                         ove for details of how to use this bit.                */
/* xlmac_intr_mask:  - This bit corresponds to the mask for xlmac interrupt. See register descrip */
/*                  tion above for details of how to use this bit.                                */
/* mac_reg_err_intr_mask:  - This bit corresponds to the mask for mac register error interrupt. S */
/*                        ee register description above for details of how to use this bit.       */
/* mab_status_intr_mask:  - This bit corresponds to the mask for msbus adaptation block status in */
/*                       terrupt. See register description above for details of how to use this b */
/*                       it.                                                                      */
/* mib_reg_err_intr_mask:  - This bit corresponds to the mask for mib register error interrupt. S */
/*                        ee register description above for details of how to use this bit.       */
/* ubus_err_intr_mask:  - This bit corresponds to the mask for ubus error interrupt. See register */
/*                      description above for details of how to use this bit.                     */
/**************************************************************************************************/
typedef struct
{
    uint8_t link_down_intr_mask;
    uint8_t link_up_intr_mask;
    uint8_t tx_timesync_fifo_entry_valid_intr_mask;
    uint8_t xlmac_intr_mask;
    uint8_t mac_reg_err_intr_mask;
    uint8_t mab_status_intr_mask;
    uint8_t mib_reg_err_intr_mask;
    uint8_t ubus_err_intr_mask;
} xport_intr_pci_mask_set;


/**************************************************************************************************/
/* link_down_intr_mask:  - This bit corresponds to the mask for link down interrupt. See register */
/*                       description above for details of how to use this bit.                    */
/* link_up_intr_mask:  - This bit corresponds to the mask for link up interrupt. See register des */
/*                    cription above for details of how to use this bit.                          */
/* tx_timesync_fifo_entry_valid_intr_mask:  - This bit corresponds to the mask for  tx timesync f */
/*                                         ifo entry valid interrupt. See register description ab */
/*                                         ove for details of how to use this bit.                */
/* xlmac_intr_mask:  - This bit corresponds to the mask for xlmac interrupt. See register descrip */
/*                  tion above for details of how to use this bit.                                */
/* mac_reg_err_intr_mask:  - This bit corresponds to the mask for mac register error interrupt. S */
/*                        ee register description above for details of how to use this bit.       */
/* mab_status_intr_mask:  - This bit corresponds to the mask for msbus adaptation block status in */
/*                       terrupt. See register description above for details of how to use this b */
/*                       it.                                                                      */
/* mib_reg_err_intr_mask:  - This bit corresponds to the mask for mib register error interrupt. S */
/*                        ee register description above for details of how to use this bit.       */
/* ubus_err_intr_mask:  - This bit corresponds to the mask for ubus error interrupt. See register */
/*                      description above for details of how to use this bit.                     */
/**************************************************************************************************/
typedef struct
{
    uint8_t link_down_intr_mask;
    uint8_t link_up_intr_mask;
    uint8_t tx_timesync_fifo_entry_valid_intr_mask;
    uint8_t xlmac_intr_mask;
    uint8_t mac_reg_err_intr_mask;
    uint8_t mab_status_intr_mask;
    uint8_t mib_reg_err_intr_mask;
    uint8_t ubus_err_intr_mask;
} xport_intr_pci_mask_clear;

int ag_drv_xport_intr_cpu_status_set(uint8_t xlmac_id, const xport_intr_cpu_status *cpu_status);
int ag_drv_xport_intr_cpu_status_get(uint8_t xlmac_id, xport_intr_cpu_status *cpu_status);
int ag_drv_xport_intr_cpu_set_set(uint8_t xlmac_id, const xport_intr_cpu_set *cpu_set);
int ag_drv_xport_intr_cpu_set_get(uint8_t xlmac_id, xport_intr_cpu_set *cpu_set);
int ag_drv_xport_intr_cpu_clear_set(uint8_t xlmac_id, const xport_intr_cpu_clear *cpu_clear);
int ag_drv_xport_intr_cpu_clear_get(uint8_t xlmac_id, xport_intr_cpu_clear *cpu_clear);
int ag_drv_xport_intr_cpu_mask_status_set(uint8_t xlmac_id, const xport_intr_cpu_mask_status *cpu_mask_status);
int ag_drv_xport_intr_cpu_mask_status_get(uint8_t xlmac_id, xport_intr_cpu_mask_status *cpu_mask_status);
int ag_drv_xport_intr_cpu_mask_set_set(uint8_t xlmac_id, const xport_intr_cpu_mask_set *cpu_mask_set);
int ag_drv_xport_intr_cpu_mask_set_get(uint8_t xlmac_id, xport_intr_cpu_mask_set *cpu_mask_set);
int ag_drv_xport_intr_cpu_mask_clear_set(uint8_t xlmac_id, const xport_intr_cpu_mask_clear *cpu_mask_clear);
int ag_drv_xport_intr_cpu_mask_clear_get(uint8_t xlmac_id, xport_intr_cpu_mask_clear *cpu_mask_clear);
int ag_drv_xport_intr_pci_status_set(uint8_t xlmac_id, const xport_intr_pci_status *pci_status);
int ag_drv_xport_intr_pci_status_get(uint8_t xlmac_id, xport_intr_pci_status *pci_status);
int ag_drv_xport_intr_pci_set_set(uint8_t xlmac_id, const xport_intr_pci_set *pci_set);
int ag_drv_xport_intr_pci_set_get(uint8_t xlmac_id, xport_intr_pci_set *pci_set);
int ag_drv_xport_intr_pci_clear_set(uint8_t xlmac_id, const xport_intr_pci_clear *pci_clear);
int ag_drv_xport_intr_pci_clear_get(uint8_t xlmac_id, xport_intr_pci_clear *pci_clear);
int ag_drv_xport_intr_pci_mask_status_set(uint8_t xlmac_id, const xport_intr_pci_mask_status *pci_mask_status);
int ag_drv_xport_intr_pci_mask_status_get(uint8_t xlmac_id, xport_intr_pci_mask_status *pci_mask_status);
int ag_drv_xport_intr_pci_mask_set_set(uint8_t xlmac_id, const xport_intr_pci_mask_set *pci_mask_set);
int ag_drv_xport_intr_pci_mask_set_get(uint8_t xlmac_id, xport_intr_pci_mask_set *pci_mask_set);
int ag_drv_xport_intr_pci_mask_clear_set(uint8_t xlmac_id, const xport_intr_pci_mask_clear *pci_mask_clear);
int ag_drv_xport_intr_pci_mask_clear_get(uint8_t xlmac_id, xport_intr_pci_mask_clear *pci_mask_clear);

#ifdef USE_BDMF_SHELL
bdmfmon_handle_t ag_drv_xport_intr_cli_init(bdmfmon_handle_t driver_dir);
#endif


#endif
