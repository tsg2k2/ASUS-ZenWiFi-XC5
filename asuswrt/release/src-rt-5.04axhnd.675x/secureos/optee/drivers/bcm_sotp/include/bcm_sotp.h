/*
<:copyright-BRCM:2019:DUAL/GPL:standard 

   Copyright (c) 2019 Broadcom 
   All Rights Reserved

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License, version 2, as published by
the Free Software Foundation (the "GPL").

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.


A copy of the GPL is available at http://www.broadcom.com/licenses/GPLv2.php, or by
writing to the Free Software Foundation, Inc., 59 Temple Place - Suite 330,
Boston, MA 02111-1307, USA.

:> 
*/

#ifndef BCM_SOTP_H
#define BCM_SOTP_H

#include <stdint.h>
#include <tee_api.h>

#define ISAO_BITS					(3 << 20)
#define SOTP_DEV_CONFIG					12
#define SOTP_ECC_ERR_DETECT				0x8000000000000000

uint32_t bcm_iproc_sotp_get_status1(void);
TEE_Result bcm_iproc_sotp_mem_read(uint32_t row_addr, uint32_t sotp_add_ecc, uint64_t *rdata);
TEE_Result bcm_iproc_sotp_mem_write(uint32_t addr, uint32_t sotp_add_ecc, uint64_t wdata);
TEE_Result bcm_iproc_sotp_set_temp_rdlock( uint32_t row, uint32_t num_of_rows );
TEE_Result bcm_iproc_sotp_set_temp_wrlock( uint32_t row, uint32_t num_of_rows );
TEE_Result bcm_iproc_sotp_is_accessable(void);
#endif
