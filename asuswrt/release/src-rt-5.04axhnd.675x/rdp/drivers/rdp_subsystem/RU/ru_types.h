/*
<:copyright-BRCM:2015:DUAL/GPL:standard

   Copyright (c) 2015 Broadcom 
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

#ifndef _RU_TYPES_H_
#define _RU_TYPES_H_
/**
 * \brief Register tracking type definitions
 *
 * The register track module provides functionality to comprehensively debug
 * register level transactions.  It is a slow interface that should be used in
 * parallel to standard direct read/write transactions.  The module can parse
 * registers into easy read field format.  Registers may be looked up by name
 * or address.
 *
 */
#ifndef RDP_SIM
#ifdef _CFE_ 
#include "lib_types.h" 
#else
#include <linux/types.h>
#endif
#endif

#include "ru_config.h"

typedef enum
{
    ru_access_read      = 0x01,         /*< Read only */
    ru_access_write     = 0x02,         /*< Write only */
    ru_access_rw        = 0x03          /*< Read/write */
} ru_access;

typedef struct
{
    const char *name;                   /*< Name of field from reg spec */
#if RU_INCLUDE_DESC
    const char *title;                  /*< Short title of the field */
    const char *desc;                   /*< Detail description */
#endif
    uint32_t mask;                      /*< Field bit mask */
    uint32_t align;                     /*< Unknown, used by register macro */
    uint32_t bits;                      /*< Field bit width */
    uint32_t shift;                     /*< Field bit offset */
#if RU_INCLUDE_ACCESS
    ru_access access;                   /*< Field read/write access */
#endif
} ru_field_rec;                         /*< Field info record */

typedef struct
{
    const char *name;                   /*< Name of register from reg spec */
#if RU_INCLUDE_DESC
    const char *title;                  /*< Short title of the register */
    const char *desc;                   /*< Detail description */
#endif
    unsigned long addr;                 /*< Block relative register address */
    uint32_t ram_count;                 /*< RAM addresses, 0 for std register */
    uint32_t offset;                    /*< Offset of next index in RAM types */
    uint32_t log_idx;                   /*< Register ID for debug logging */
#if RU_INCLUDE_ACCESS
    ru_access access;                   /*< Register read/write access */
#endif
#if RU_INCLUDE_FIELD_DB
    uint32_t field_count;               /*< Number of fields, private */
    const ru_field_rec **fields;        /*< All fields for register, private */
#endif
} ru_reg_rec;                           /*< Register info record */

typedef struct
{
    const char *name;                   /*< Name of the block */
    unsigned long *addr;                /*< Block base addresses */
    uint8_t addr_count;                 /*< Number of block instances */
    uint32_t reg_count;                 /*< Number of registers, private */
    const ru_reg_rec **regs;            /*< All registers for block, private */
} ru_block_rec;                         /*< Info for a block instance */

#define RU_BLK(b) b##_BLOCK
#define RU_REG(b,r) b##_##r##_REG
#define RU_REG_OFFSET(b,r) b##_##r##_REG_OFFSET
#define RU_REG_RAM_CNT(b,r) b##_##r##_REG_RAM_CNT
#define RU_FLD(b,r,f) b##_##r##_##f##_FIELD
#define RU_FLD_MASK(b,r,f) b##_##r##_##f##_FIELD_MASK
#define RU_FLD_SHIFT(b,r,f) b##_##r##_##f##_FIELD_SHIFT

typedef uint8_t ru_block_inst;          /*< Multiple block instance index */
typedef uint32_t ru_ram_addr;           /*< Index for RAM mapped registers */

#endif /* End of file _RU_TYPES_H_ */