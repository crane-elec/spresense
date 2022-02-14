/*##############################################################################
 * Copyright 2015 Sony Corporation.
 * This is UNPUBLISHED PROPRIETARY SOURCE CODE of Sony Corporation.
 * No part of this file may be copied, modified, sold, and distributed in any
 * form or by any means without prior explicit permission in writing from
 * Sony Corporation.
 */
/**
 * @file       ble_gattc.c
 * @note       ble gatt client interface source file
 * @attention
 */
/*############################################################################*/

/******************************************************************************
 * Include
 *****************************************************************************/
#include <string.h>
#include <stdint.h>
#include <unistd.h>
#include <ble/ble_comm.h>
#include <ble/ble_gattc.h>
#include "ble.h"
#include "ble_err.h"
#include "ble_comm_internal.h"

//#define BLE_DBGPRT_ENABLE
#ifdef BLE_DBGPRT_ENABLE
#include <stdio.h>
#define BLE_PRT printf
#else
#define BLE_PRT(...)
#endif

/******************************************************************************
 * externs
 *****************************************************************************/
extern int bleConvertErrorCode(uint32_t errCode);
extern bleCommMem commMem;
/******************************************************************************
 * Define
 *****************************************************************************/
#define SRV_DISC_START_HANDLE  0x0001

/******************************************************************************
 * Function
 *****************************************************************************/
int BLE_GattcStartDbDiscovery(uint16_t connHandle)
{
	int ret      = BLE_SUCCESS;
	int errCode = 0;

	errCode = sd_ble_gattc_primary_services_discover(connHandle,SRV_DISC_START_HANDLE,NULL);
	ret = bleConvertErrorCode((uint32_t)errCode);
	return ret;
}
int BLE_GattcRead(uint16_t connHandle, BLE_GattcReadParams const *readParams)
{
#define BLE_GATTC_READ_BUSY_CHECK_MAX 5
	int ret      = BLE_SUCCESS;
	int errCode = 0;

	if (readParams == NULL) {
		return -EINVAL;
	}

	for (int i=0; i<BLE_GATTC_READ_BUSY_CHECK_MAX; i++) {
		errCode = sd_ble_gattc_read(connHandle, readParams->charValHandle, 0);
		if (errCode != NRF_ERROR_BUSY) {
			ret = bleConvertErrorCode((uint32_t)errCode);
			return ret;
		}
		int msec = (commMem.gapMem->connParams.max_conn_interval * 1250 + (1000 - 1)) / 1000;
		BLE_PRT("BLE_GattcRead: delay %d\n", msec);
		usleep(msec * 1000);
	}
	return -EBUSY;
}
int BLE_GattcWrite(uint16_t connHandle, BLE_GattcWriteParams const *writeParams)
{
#define BLE_GATTC_TX_BUFFER_CHECK_MAX 5
	int ret      = BLE_SUCCESS;
	int errCode = 0;
	ble_gattc_write_params_t gattcWriteParams = {0};

	if (writeParams == NULL) {
		return -EINVAL;
	}

	memset(&gattcWriteParams,0,sizeof(gattcWriteParams));
	gattcWriteParams.handle   = writeParams->charValHandle;
	gattcWriteParams.len      = writeParams->charValLen;
	gattcWriteParams.p_value  = writeParams->charValData;
	gattcWriteParams.offset   = 0;
	if (writeParams->writeOp == BLE_GATTC_WRITE_CMD) {
		gattcWriteParams.write_op = BLE_GATT_OP_WRITE_CMD;
	}
	else if (writeParams->writeOp == BLE_GATTC_WRITE_REQ) {
		gattcWriteParams.write_op = BLE_GATT_OP_WRITE_REQ;
	}
	else {
		return -EINVAL;
	}
	gattcWriteParams.flags    = BLE_GATT_EXEC_WRITE_FLAG_PREPARED_CANCEL;
	for (int i=0; i<BLE_GATTC_TX_BUFFER_CHECK_MAX; i++) {
		errCode = sd_ble_gattc_write(connHandle, &gattcWriteParams);
		if (errCode != NRF_ERROR_RESOURCES) {
			ret = bleConvertErrorCode((uint32_t)errCode);
			return ret;
		}
		int msec = (commMem.gapMem->connParams.max_conn_interval * 1250 + (1000 - 1)) / 1000;
		BLE_PRT("BLE_GattcWrite: delay %d\n", msec);
		usleep(msec * 1000);
	}
	return -EBUSY;

}

int BLE_GattcConfirm(uint16_t connHandle, uint16_t attrHandle)
{
	int ret      = BLE_SUCCESS;
	int errCode = 0;

	errCode = sd_ble_gattc_hv_confirm(connHandle, attrHandle);
	ret = bleConvertErrorCode((uint32_t)errCode);
	return ret;
}

#ifdef BLE_ENABLE_NORDIC_ORIGINAL
int BLE_GattcRegisterUuid128(BLE_Uuid128* uuidBase, uint8_t* type)
{
	int ret      = BLE_SUCCESS;
	int errCode = 0;
	*type = BLE_UUID_TYPE_VENDOR_BEGIN;
	errCode  = sd_ble_uuid_vs_add((ble_uuid128_t *)uuidBase, type);
	ret = bleConvertErrorCode((uint32_t)errCode);
	return ret;
}
#endif
