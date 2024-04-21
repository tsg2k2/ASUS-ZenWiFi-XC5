/***********************************************************************
 *
 *  Copyright (c) 2020  Broadcom Ltd
 *  All Rights Reserved
 *
<:label-BRCM:2020:DUAL/GPL:standard

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
 *
 ************************************************************************/

#ifndef __BCM_GENERIC_HAL_DEFS_H__
#define __BCM_GENERIC_HAL_DEFS_H__

/*!\file bcm_generic_hal_defs.h
 * \brief Various generic hal defines that are used in various places in 
 *        CMS and BDK.  Not owned by any library, since many libraries
 *        use these defines.  Some users: CMS PHL, ZBus, DBus and UBus,
 *        bcm_generic_hal.
 *
 */

#include "os_defs.h"


// This struct is used by various flavors of getParameterNames and getParameterValues.
// If this struct is returned by getParmeterNames, the value field is not set.
typedef struct {
   char   *fullpath;
   char   *type;      // BBF types (MdmParamTypes) as defined in cms_mdm.h and mdm_binaryhelper.c
   char   *value;
   char   *profile;   // BBF profile, e.g. Device2_Baseline:1  Probably only TR69 cares about this.
   UBOOL8  writable;
   UBOOL8  isPassword;
   UINT32  errorCode;    // On return from setParameterValues, will be error code, or 0 if no errors.
} BcmGenericParamInfo;

// This struct is used by various flavors of getParameterAttributes and setParameterAttributes.
// See also MdmNodeAttributes in cms_mdm.h
typedef struct {
   char  *fullpath;
   UBOOL8 setAccess;   // only used during setAttr
   UINT16 access;      // access bitmask (set or get)
   UBOOL8 setNotif;    // only used during setAttr
   UINT16 notif;       // notification setting (set or get)
   UINT16 valueChanged;  // only used during getAttr: this param has notification enabled, and its value has changed.
   UBOOL8 setAltNotif;  // only used during setAttr (TODO: placeholder in struct, not implemented yet)
   UINT16 altNotif;     // TODO: placeholder in struct, not implemented yet
   UBOOL8 clearAltNotifValue;  // only used during setAttr (TODO: placeholder in struct, not implemented yet)
   UINT16 altNotifValue;       // 0 means no change (TODO: placeholder in struct, not implemented yet)
   UINT32 errorCode;   // On return from setParameterAttributes, will be error code, or 0 if no errors. (TODO: placeholder in struct, not implemented yet)
} BcmGenericParamAttr;


// These are the ops given to bcm_generic_databaseOp.
#define BCM_DATABASEOP_SAVECONFIG                     "saveConfig"
#define BCM_DATABASEOP_SAVECONFIG_LOCAL               "saveConfigLocal"
#define BCM_DATABASEOP_INVALIDATECONFIG               "invalidateConfig"
#define BCM_DATABASEOP_INVALIDATECONFIG_LOCAL         "invalidateConfigLocal"
#define BCM_DATABASEOP_READCONFIG                     "readConfig"
#define BCM_DATABASEOP_READCONFIG_LOCAL               "readConfigLocal"
#define BCM_DATABASEOP_READMDM                        "readMdm"
#define BCM_DATABASEOP_READMDM_LOCAL                  "readMdmLocal"
// ValidateConfig and WriteConfig are handled a bit differently, so there is
// no local version.  Both of these ops are local.  Remote_objd breaks up
// the concatenated config file at its level.
#define BCM_DATABASEOP_VALIDATECONFIG                 "validateConfig"
#define BCM_DATABASEOP_WRITECONFIG                    "writeConfig"
// TODO: currently, the BCM Generic HAL only implements the "LOCAL" version of these ops.
// If you want to operate on all of the components, you must use the CMS PHL API.
// For now, to prevent build breakages in existing code, point the global versions to the local versions.
#define BCM_DATABASEOP_GET_NUM_VALUE_CHANGES                "getNumValuesChangesLocal"
#define BCM_DATABASEOP_GET_NUM_VALUE_CHANGES_LOCAL          "getNumValuesChangesLocal"
#define BCM_DATABASEOP_CLEAR_ALL_PARAM_VALUE_CHANGES        "clearAllParamValueChangesLocal"
#define BCM_DATABASEOP_CLEAR_ALL_PARAM_VALUE_CHANGES_LOCAL  "clearAllParamValueChangesLocal"
#define BCM_DATABASEOP_GET_CHANGED_PARAMS                   "getChangedParamsLocal"
#define BCM_DATABASEOP_GET_CHANGED_PARAMS_LOCAL             "getChangedParamsLocal"

#define BCM_DATABASEOP_GET_ALT_NUM_VALUE_CHANGES            "getAltNumValuesChangesLocal"
#define BCM_DATABASEOP_GET_ALT_NUM_VALUE_CHANGES_LOCAL      "getAltNumValuesChangesLocal"
#define BCM_DATABASEOP_CLEAR_ALL_ALT_PARAM_VALUE_CHANGES    "clearAllAltParamValueChangesLocal"
#define BCM_DATABASEOP_CLEAR_ALL_ALT_PARAM_VALUE_CHANGES_LOCAL  "clearAllAltParamValueChangesLocal"
#define BCM_DATABASEOP_GET_ALT_CHANGED_PARAMS               "getAltChangedParamsLocal"
#define BCM_DATABASEOP_GET_ALT_CHANGED_PARAMS_LOCAL         "getAltChangedParamsLocal"


// The next two always go to the devinfo component.
// The add operation does not take input arg, but on successful return, the
// fullpath of the created object is returned in the output arg.
// The del operation takes the fullpath to be deleted as the input arg, but
// does not return output arg.
#define BCM_DATABASEOP_ADD_VENDOR_CONFIG             "addVendorConfig"
#define BCM_DATABASEOP_DEL_VENDOR_CONFIG             "delVendorConfig"


#endif /* __BCM_GENERIC_HAL_DEFS_H__ */
