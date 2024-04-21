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
#ifndef TEE_SOTP_SER_H
#define TEE_SOTP_SER_H

#define SOTP_SERVICE_UUID \
		{ 0xd3df7b18, 0x4c02, 0x4051,  \
		{ 0xb8, 0x65, 0xe3, 0x19, 0x34, 0x35, 0x3e, 0xfd } }

typedef enum {
	CMD_SOTP_SERVICE_INIT = 0,
	CMD_SOTP_SERVICE_READ,
	CMD_SOTP_SERVICE_WRITE,
	CMD_SOTP_SERVICE_LOCK,
} cmd_sotp_service;

#endif /* TEE_SOTP_SER_H */
