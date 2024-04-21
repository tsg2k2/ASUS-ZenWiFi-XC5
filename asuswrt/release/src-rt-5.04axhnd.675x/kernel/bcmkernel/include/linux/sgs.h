/*
 * <:copyright-BRCM:2019:DUAL/GPL:standard 
 * 
 *    Copyright (c) 2019 Broadcom 
 *    All Rights Reserved
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License, version 2, as published by
 * the Free Software Foundation (the "GPL").
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * 
 * A copy of the GPL is available at http://www.broadcom.com/licenses/GPLv2.php, or by
 * writing to the Free Software Foundation, Inc., 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 * 
 * :>
 */

#ifndef _LINUX_SGS_H_
#define _LINUX_SGS_H_

#define SGS_CT_ACCEPT_BIT	0
#define SGS_CT_BLOCK_BIT	1
#define SGS_CT_SESSION_BIT	2
#define SGS_CT_TERMINATED_BIT	3

#define SGS_MAGIC		0x600df00d /* Good Food */
struct sgs_info {
	unsigned long		valid;
	unsigned long		flags;
	unsigned long		rcvcnt;
	int			rstcnt;
};

struct nf_conn;

struct sgs_core_hooks {
    void (*delete)(struct nf_conn *ct);
};

int  sgs_core_hooks_register(struct sgs_core_hooks *h);
void sgs_nf_ct_delete_from_lists(struct nf_conn *ct);
void sgs_core_hooks_unregister(void);

#endif /* _LINUX_SGS_H_ */
