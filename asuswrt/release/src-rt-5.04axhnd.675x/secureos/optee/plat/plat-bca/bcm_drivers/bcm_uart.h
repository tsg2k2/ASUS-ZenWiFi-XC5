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

#ifndef BCM_UART_H
#define BCM_UART_H

#include <types_ext.h>
#include <drivers/serial.h>

#define BCM_UART_REG_SIZE	0x20000

struct bcm_uart_data {
	struct io_pa_va base;
	struct serial_chip chip;
};

void bcm_uart_init(struct bcm_uart_data *pd, paddr_t pbase);

#endif /* BCM_UART_H */
