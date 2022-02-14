/*##############################################################################
 * Copyright 2015 Sony Corporation.
 * This is UNPUBLISHED PROPRIETARY SOURCE CODE of Sony Corporation.
 * No part of this file may be copied, modified, sold, and distributed in any
 * form or by any means without prior explicit permission in writing from
 * Sony Corporation.
 */
/**
 * @file       ble_comm.c
 * @note       ble common interface source file
 * @attention
 */
/*############################################################################*/

/******************************************************************************
 * Include
 *****************************************************************************/
#include <string.h>
#include <unistd.h>
#include <ble/ble_comm.h>
#include <ble/ble_gap.h>
#include <ble/ble_gatts.h>
#include <ble/ble_gattc.h>
#include "ble_storage_operations.h"
#include "ble_comm_internal.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include <arch/board/board.h>

/******************************************************************************
 * externs
 *****************************************************************************/
extern bleGapMem *bleGetGapMem(void);
extern void bleMngEvtDispatch(ble_evt_t *nrfEvt);
extern void board_nrf52_initialize(void);
extern void board_nrf52_reset(bool en);
extern void board_nrf52_dfu_mode(bool en);

/******************************************************************************
 * Define
 *****************************************************************************/
#define BLE_DBGPRT_ENABLE
#ifdef BLE_DBGPRT_ENABLE
#include <stdio.h>
#define BLE_PRT printf
#define BLE_PRT2(...)
#define BLE_ERR printf
#else
#define BLE_PRT(...)
#define BLE_PRT2(...)
#define BLE_ERR(...)
#endif

 /******************************************************************************
 * Structure define
 *****************************************************************************/

 /******************************************************************************
 * Function prototype declaration
 *****************************************************************************/
static int blePowerOn(void);
static int bleStackInit(void);
static int bleStackFin(void);
#ifdef BLE_USE_SECTION
static void bleEvtDispatch(ble_evt_t *pBleNrfEvt);
#endif
#if NRF_SD_BLE_API_VERSION > 5
static void onAdvSetTerminate(BLE_Evt *pBleEvent, ble_evt_t *pBleNrfEvt);
#endif
static void onScanReqReport(BLE_Evt *pBleEvent, ble_evt_t *pBleNrfEvt);
static void onPhyUpdateReq(BLE_Evt *pBleEvent, ble_evt_t *pBleNrfEvt);
static void onPhyUpdate(BLE_Evt *pBleEvent, ble_evt_t *pBleNrfEvt);
static void onDataLengthUpdateRequest(BLE_Evt *pBleEvent, ble_evt_t *pBleNrfEvt);
static void onDataLengthUpdate(BLE_Evt *pBleEvent, ble_evt_t *pBleNrfEvt);
static void onExchangeMtuRequest(BLE_Evt *pBleEvent, ble_evt_t *pBleNrfEvt);
static void onTxComplete(BLE_Evt *pBleEvent, ble_evt_t *pBleNrfEvt);
static void onConnect(BLE_Evt *pBleEvent, ble_evt_t *pBleNrfEvt);
static void onConnParamUpdate(BLE_Evt *pBleEvent, ble_evt_t *pBleNrfEvt);
static void onConnParamUpdateRequest(BLE_Evt *pBleEvent, ble_evt_t *pBleNrfEvt);
static void onDisconnect(BLE_Evt *pBleEvent, ble_evt_t *pBleNrfEvt);
static void onSecParamsRequest(BLE_Evt *pBleEvent, ble_evt_t *pBleNrfEvt);
static void onAuthStatus(BLE_Evt *pBleEvent, ble_evt_t *pBleNrfEvt);
static void onDispPasskey(BLE_Evt *pBleEvent, ble_evt_t *pBleNrfEvt);
static void onAdvReport(BLE_Evt *pBleEvent, ble_evt_t *pBleNrfEvt);
static void onAuthKeyRequest(BLE_Evt *pBleEvent, ble_evt_t *pBleNrfEvt);
static void onConnSecUpdate(BLE_Evt *pBleEvent, ble_evt_t *pBleNrfEvt);
static void onSecInfoRequest(BLE_Evt *pBleEvent, ble_evt_t *pBleNrfEvt);
static void onTimeout(BLE_Evt *pBleEvent, ble_evt_t *pBleNrfEvt);
static void onGattsTimeout(BLE_Evt *pBleEvent, ble_evt_t *pBleNrfEvt);
static void onRssiChanged(BLE_Evt *pBleEvent, ble_evt_t *pBleNrfEvt);
static void onGattsWrite(BLE_Evt *pBleEvent, ble_evt_t *pBleNrfEvt);
static void onGattsIndConfirm(BLE_Evt *pBleEvent, ble_evt_t *pBleNrfEvt);
static void onPrimarySrvDiscoveryRsp(bleGattcDb *gattcDbDiscovery, BLE_Evt *pBleEvent, ble_evt_t *pBleNrfEvt);
static void onCharacteristicDiscoveryRsp(bleGattcDb *const gattcDbDiscovery, BLE_Evt *pBleEvent, ble_evt_t *pBleNrfEvt);
static void onDescriptorDiscoveryRsp(bleGattcDb *const gattcDbDiscovery, BLE_Evt *pBleEvent, ble_evt_t *pBleNrfEvt);
static void onHvx(BLE_Evt *pBleEvent, ble_evt_t *pBleNrfEvt);
static void onSysAttrMissing(BLE_Evt *pBleEvent, ble_evt_t *pBleNrfEvt);
static void onReadRsp(BLE_Evt *pBleEvent, ble_evt_t *pBleNrfEvt);
static void onWriteRsp(BLE_Evt *pBleEvent, ble_evt_t *pBleNrfEvt);
static void onSrvDiscCompletion(BLE_Evt *pBleEvent, bleGattcDb *gattcDbDiscovery);
static  int characteristicsDiscover(BLE_Evt *pBleEvent, bleGattcDb *const gattcDbDiscovery);
static bool isCharDiscoveryReqd(bleGattcDb *const gattcDbDiscovery, BLE_GattcChar *afterChar);
static bool isDescDiscoveryReqd(bleGattcDb *gattcDbDiscovery, BLE_GattcDbDiscChar *currChar, BLE_GattcDbDiscChar *nextChar, BLE_GattcHandleRange *handleRange);
static  int descriptorsDiscover(BLE_Evt *pBleEvent, bleGattcDb *const gattcDbDiscovery, bool *raiseDiscovComplete);

/******************************************************************************
 * Variable
 *****************************************************************************/
bleCommMem commMem = {0};
bleGapWrapperBondInfo BondInfoInFlash[BLE_SAVE_BOND_DEVICE_MAX_NUM] = {{0}};
uint32_t bleBondEnableList = 0;
bleBondInfoKey bleKey = {0};

/******************************************************************************
 * Function
 *****************************************************************************/
static int blePowerOff(void)
{
	int ret = 0;
	board_nrf52_reset(true);
	ret = board_power_control(POWER_BTBLE, false);
	if (ret) {
		BLE_PRT("board_power_control(off): NG %d\n", ret);
		return ret;
	}
	board_nrf52_dfu_mode(true);
	BLE_PRT("Power off BLE!!\n");
	return BLE_SUCCESS;
}

static void bleInitBondInfoKey(void)
{
	bleKey.enable_key = BSO_GenerateRegistryKey(BOND_ENABLE_KEY_NAME, 0);
	bleKey.info_key[0] = BSO_GenerateRegistryKey(BOND_INFO_1_KEY_NAME, 0);
	bleKey.info_key[1] = BSO_GenerateRegistryKey(BOND_INFO_2_KEY_NAME, 0);
	bleKey.info_key[2] = BSO_GenerateRegistryKey(BOND_INFO_3_KEY_NAME, 0);
	bleKey.info_key[3] = BSO_GenerateRegistryKey(BOND_INFO_4_KEY_NAME, 0);
	bleKey.info_key[4] = BSO_GenerateRegistryKey(BOND_INFO_5_KEY_NAME, 0);
	bleKey.info_key[5] = BSO_GenerateRegistryKey(BOND_INFO_6_KEY_NAME, 0);
	bleKey.info_key[6] = BSO_GenerateRegistryKey(BOND_INFO_7_KEY_NAME, 0);
	bleKey.info_key[7] = BSO_GenerateRegistryKey(BOND_INFO_8_KEY_NAME, 0);
#ifdef BLE_DBGPRT_ENABLE
	BLE_PRT("bleInitBondInfoKey: enable_key 0x%x\n", bleKey.enable_key);
	for (int i=0; i<BLE_SAVE_BOND_DEVICE_MAX_NUM; i++) {
		BLE_PRT("bleInitBondInfoKey: info_key[%d] 0x%x\n", i, bleKey.info_key[i]);
	}
#endif
}

static int bleGetBondInfo(void)
{
	int ret = 0;
	bleInitBondInfoKey();
	ret = BSO_GetRegistryValue(bleKey.enable_key, (void *)&bleBondEnableList, sizeof(uint32_t));
	if (ret == -ENOENT) {
		BLE_PRT("bleGetBondInfo: Set bleBondEnableList = 0\n");
		bleBondEnableList = 0;
		BSO_SetRegistryValue(bleKey.enable_key, (const void*)&bleBondEnableList, sizeof(uint32_t));
		BSO_Sync();
	} else if (0 != ret) {
		BLE_PRT("bleGetBondInfo: Get bleBondEnableList NG %d\n", ret);
		return ret;
	}
	BLE_PRT("bleGetBondInfo: bleBondEnableList 0x%x\n", bleBondEnableList);
	for (int i=0; i<BLE_SAVE_BOND_DEVICE_MAX_NUM; i++) {
		ret = BSO_GetRegistryValue(bleKey.info_key[i], (void *)&BondInfoInFlash[i], sizeof(bleGapWrapperBondInfo));
		if(ret == -ENOENT) {
			BLE_PRT("bleGetBondInfo: Set bleBondEnableList = 0\n");
			bleBondEnableList = 0;
			BSO_SetRegistryValue(bleKey.info_key[i], (const void*)&BondInfoInFlash[i], sizeof(bleGapWrapperBondInfo));
			BSO_Sync();
		} else if (0 != ret) {
			BLE_PRT("bleGetBondInfo: Get BondInfoInFlash[%d] NG %d\n", i, ret);
			return ret;
		}
	}
	return 0;
}

int BLE_CommonInitializeStack(BLE_InitializeParams *initializeParams)
{
	uint8_t role = 0;
	int ret = BLE_SUCCESS;

	if (initializeParams == NULL) {
		return -EINVAL;
	}
	if (commMem.stackInited) {
		return -EPERM;
	}

	board_nrf52_initialize();
        
        puts("board init");
        
	commMem.callbackFlag = 0;
	commMem.gapMem = bleGetGapMem();

        printf("gapmem %x\n",commMem.gapMem);
        
	role = initializeParams->role;

        printf("role %x\n",role);
        
	ret = BSO_Init(NULL);
	if (ret) {
		BLE_PRT("BSO_Init failed\n");
		return ret;
	}

         puts("BSO init");

         
	ret = bleGetBondInfo();
	if (ret) {
		BLE_PRT("bleGetBondInfo: NG %d\n", ret);
		return ret;
	}
	switch (role) {
	case BLE_ROLE_PERIPHERAL:
	case BLE_ROLE_CENTRAL:
	case BLE_ROLE_PERIPHERAL_AND_CENTRAL:
		ret = blePowerOn();
         puts("power on");
         if (ret) {
			goto errPower;
		}
		ret = bleStackInit();
		if (ret) {
			BLE_PRT("bleStackInit: NG %d\n", ret);
			goto errInit;
		} else {
			commMem.stackInited = true;
		}
		break;
	default:
		ret = -EINVAL;
		break;
	}
	return ret;
errInit:
	bleStackFin();
	blePowerOff();
errPower:
	return ret;

}

int BLE_CommonSetBleEvtCallback(BLE_EfCb funcCb, BLE_EvtCtx *ctx)
{
	if((funcCb == NULL) || (ctx == NULL)) {
		return -EINVAL;
	}
	if (commMem.managerCbFlag) {
		return -EPERM;
	}

	commMem.cbId = funcCb;
	commMem.bleEvtCtx = ctx;
	commMem.callbackFlag = 1;
	return BLE_SUCCESS;
}

int BLE_CommonFinalizeStack(void)
{
	int ret = BLE_SUCCESS;
	if (!commMem.stackInited) {
		return ret;
	}
	commMem.callbackFlag = 0;
	commMem.gapMem = NULL;
	if (bleStackFin()) {
		ret = -EPERM;
	}
	if (blePowerOff()) {
		ret = -EPERM;
	}
	commMem.stackInited = false;
	return ret;
}

static
int blePowerOn(void)
{
	int ret = 0;
	board_nrf52_dfu_mode(false);
	ret = board_power_control(POWER_BTBLE, true);
	if (ret) {
		BLE_PRT("board_power_control(on): NG %d\n", ret);
		goto errPower;
	}
	board_nrf52_reset(false);
	BLE_PRT("Power on BLE!!\n");
	return BLE_SUCCESS;
errPower:
	(void)board_power_control(POWER_BTBLE, false);
	return ret;

}

int bleConvertErrorCode(uint32_t errCode)
{
	int ret = 0;
	switch (errCode) {
	case NRF_SUCCESS:
		//BLE_ERR("BLE_SUCCESS \n");
		ret = BLE_SUCCESS;
		break;
	case NRF_ERROR_INVALID_PARAM:
		BLE_ERR("errno: 0x%x, Invalid parameter.\n", errCode);
		ret = -EINVAL;
		break;
	case NRF_ERROR_INVALID_STATE:
		BLE_ERR("errno: 0x%x, Invalid state.\n", errCode);
		ret = -EINVAL;
		break;
	case NRF_ERROR_INVALID_ADDR:
		BLE_ERR("errno: 0x%x, Bad Memory Address.\n", errCode);
		ret = -EINVAL;
		break;
	case NRF_ERROR_INVALID_FLAGS:
		BLE_ERR("errno: 0x%x, Invalid flags.\n", errCode);
		ret = -EINVAL;
		break;
	case NRF_ERROR_INVALID_DATA:
		BLE_ERR("errno: 0x%x, Invalid data.\n", errCode);
		ret = -EINVAL;
		break;
	case NRF_ERROR_DATA_SIZE:
		BLE_ERR("errno: 0x%x, Data size exceeds limit.\n", errCode);
		ret = -EINVAL;
		break;
	case NRF_ERROR_INVALID_LENGTH:
		BLE_ERR("errno: 0x%x, Invalid length.\n", errCode);
		ret = -EINVAL;
		break;
	case NRF_ERROR_NOT_SUPPORTED:
		BLE_ERR("errno: 0x%x, Not supported.\n", errCode);
		ret = -EINVAL;
		break;
	case NRF_ERROR_BUSY:
		BLE_ERR("errno: 0x%x, Busy.\n", errCode);
		ret = -EPERM;
		break;
	case NRF_ERROR_NO_MEM:
		BLE_ERR("errno: 0x%x, No Memory for operation.\n", errCode);
		ret = -EPERM;
		break;
	case NRF_ERROR_FORBIDDEN:
		BLE_ERR("errno: 0x%x, Forbidden Operation.\n", errCode);
		ret = -EPERM;
		break;
	case BLE_ERROR_INVALID_CONN_HANDLE:
		BLE_ERR("errno: 0x%x, Invalid connection handler.\n", errCode);
		ret = -EPERM;
		break;
	case BLE_ERROR_GAP_UUID_LIST_MISMATCH:
		BLE_ERR("errno: 0x%x, BLE_ERROR_GAP_UUID_LIST_MISMATCH.\n", errCode);
		ret = -EINVAL;
		break;
	case BLE_ERROR_GAP_INVALID_BLE_ADDR:
		BLE_ERR("errno: 0x%x, BLE_ERROR_GAP_INVALID_BLE_ADDR.\n", errCode);
		ret = -EPERM;
		break;
	case BLE_ERROR_GAP_WHITELIST_IN_USE:
		BLE_ERR("errno: 0x%x, BLE_ERROR_GAP_WHITELIST_IN_USE.\n", errCode);
		ret = -EPERM;
		break;
	case BLE_ERROR_GAP_DISCOVERABLE_WITH_WHITELIST:
		BLE_ERR("errno: 0x%x, BLE_ERROR_GAP_DISCOVERABLE_WITH_WHITELIST.\n", errCode);
		ret = -EPERM;
		break;
	case NRF_ERROR_TIMEOUT:
		BLE_ERR("errno: 0x%x, NRF_ERROR_TIMEOUT.\n", errCode);
		ret = -ETIMEDOUT;
		break;
	default:
		if (errCode > 0) {
			BLE_ERR("errno: 0x%x, Stack error.\n", errCode);
			ret = -EPERM;
		}
		break;
	}
	return ret;
}

static
int bleStackInit(void)
{
	int ret;
	BLE_PRT("nrf_sdh_enable_request\n");
	ret = nrf_sdh_enable_request();
	if (ret) {
		return bleConvertErrorCode(ret);
	}
	// Configure the BLE stack using the default settings.
	// Fetch the start address of the application RAM.
	BLE_PRT("nrf_sdh_ble_default_cfg_set\n");
	ret = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, NULL);
	if (ret) {
		goto err;
	}
	// Enable BLE stack.
	BLE_PRT("nrf_sdh_ble_enable\n");
	ret = nrf_sdh_ble_enable(NULL);
	if (ret) {
          puts("koko?");
		goto err;
	}
	// Register a handler for BLE events.
#ifdef BLE_USE_SECTION
	BLE_PRT("NRF_SDH_BLE_OBSERVER\n");
	NRF_SDH_BLE_OBSERVER(m_ble_observer, 0, (void *)bleEvtDispatch, NULL);
#endif
	return ret;
err:
	nrf_sdh_disable_request();
	return bleConvertErrorCode(ret);
}

static
int bleStackFin(void)
{
	int ret = 0;
	ret = nrf_sdh_disable_request();
	return bleConvertErrorCode(ret);
}

void bleNrfEvtHandler(BLE_Evt *bleEvent, ble_evt_t *pBleNrfEvt)
{
	//BLE_PRT("bleNrfEvtHandler 0x%x\n", pBleNrfEvt->header.evt_id);
	memset(bleEvent, 0x00, sizeof(BLE_Evt));
	switch(pBleNrfEvt->header.evt_id) {
	case BLE_GATTC_EVT_WRITE_CMD_TX_COMPLETE:
	case BLE_GATTS_EVT_HVN_TX_COMPLETE:
		onTxComplete(bleEvent, pBleNrfEvt);
		break;
	case BLE_GAP_EVT_CONNECTED:
		onConnect(bleEvent, pBleNrfEvt);
		break;
	case BLE_GAP_EVT_CONN_PARAM_UPDATE:
		onConnParamUpdate(bleEvent, pBleNrfEvt);
		break;
	case BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST:
		onConnParamUpdateRequest(bleEvent, pBleNrfEvt);
		break;
	case BLE_GAP_EVT_DISCONNECTED:
		onDisconnect(bleEvent, pBleNrfEvt);
		break;
	case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
		onSecParamsRequest(bleEvent, pBleNrfEvt);
		break;
	case BLE_GAP_EVT_AUTH_STATUS:
		onAuthStatus(bleEvent, pBleNrfEvt);
		break;
	case BLE_GAP_EVT_PASSKEY_DISPLAY:
		onDispPasskey(bleEvent, pBleNrfEvt);
		break;
	case BLE_GAP_EVT_ADV_REPORT:
		onAdvReport(bleEvent, pBleNrfEvt);
		break;
	case BLE_GAP_EVT_AUTH_KEY_REQUEST:
		onAuthKeyRequest(bleEvent, pBleNrfEvt);
		break;
	case BLE_GAP_EVT_SEC_INFO_REQUEST:
		onSecInfoRequest(bleEvent, pBleNrfEvt);
		break;
	case BLE_GAP_EVT_RSSI_CHANGED:
		onRssiChanged(bleEvent, pBleNrfEvt);
		break;
	case BLE_GATTS_EVT_WRITE:
		onGattsWrite(bleEvent, pBleNrfEvt);
		break;
	case BLE_GATTS_EVT_HVC:
		onGattsIndConfirm(bleEvent, pBleNrfEvt);
		break;
	case BLE_GAP_EVT_CONN_SEC_UPDATE:
		onConnSecUpdate(bleEvent, pBleNrfEvt);
		break;
	case BLE_GAP_EVT_TIMEOUT:
		onTimeout(bleEvent, pBleNrfEvt);
		break;
	case BLE_GAP_EVT_DATA_LENGTH_UPDATE_REQUEST:
		onDataLengthUpdateRequest(bleEvent, pBleNrfEvt);
		break;
	case BLE_GAP_EVT_DATA_LENGTH_UPDATE:
		onDataLengthUpdate(bleEvent, pBleNrfEvt);
		break;
	case BLE_GAP_EVT_SCAN_REQ_REPORT:
		onScanReqReport(bleEvent, pBleNrfEvt);
		break;
	case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
		onPhyUpdateReq(bleEvent, pBleNrfEvt);
		break;
	case BLE_GAP_EVT_PHY_UPDATE:
		onPhyUpdate(bleEvent, pBleNrfEvt);
		break;
	case BLE_GATTC_EVT_PRIM_SRVC_DISC_RSP:
		onPrimarySrvDiscoveryRsp(&commMem.gattcDb, bleEvent, pBleNrfEvt);
		break;
	case BLE_GATTC_EVT_CHAR_DISC_RSP:
		onCharacteristicDiscoveryRsp(&commMem.gattcDb, bleEvent, pBleNrfEvt);
		break;
	case BLE_GATTC_EVT_DESC_DISC_RSP:
		onDescriptorDiscoveryRsp(&commMem.gattcDb, bleEvent, pBleNrfEvt);
		break;
	case BLE_GATTC_EVT_READ_RSP:
		onReadRsp(bleEvent, pBleNrfEvt);
		break;
	case BLE_GATTC_EVT_WRITE_RSP:
		onWriteRsp(bleEvent, pBleNrfEvt);
		break;
	case BLE_GATTC_EVT_HVX:
		onHvx(bleEvent, pBleNrfEvt);
		break;
	case BLE_GATTS_EVT_SYS_ATTR_MISSING:
		onSysAttrMissing(bleEvent, pBleNrfEvt);
		break;
	case BLE_GATTS_EVT_TIMEOUT:
		onGattsTimeout(bleEvent, pBleNrfEvt);
		break;
	case BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST:
		onExchangeMtuRequest(bleEvent, pBleNrfEvt);
		break;
#if NRF_SD_BLE_API_VERSION > 5
	case BLE_GAP_EVT_ADV_SET_TERMINATED:
		onAdvSetTerminate(bleEvent, pBleNrfEvt);
		break;
#endif
	default:
		bleEvent->evtHeader = (BLE_EventTypes)pBleNrfEvt->header.evt_id;
		BLE_ERR("Unknown event 0x%x\n", bleEvent->evtHeader);
		break;
	}
}

static
void bleCommEvtDispatch(ble_evt_t *pBleNrfEvt)
{
	BLE_Evt *bleEvent = &commMem.bleEvtCtx->evt;
	bleNrfEvtHandler(bleEvent, pBleNrfEvt);
	if (bleEvent->evtHeader) {
		commMem.cbId(commMem.bleEvtCtx);
	}
}

#ifdef BLE_USE_SECTION
static
#endif
void bleEvtDispatch(ble_evt_t *pBleNrfEvt)
{
//	BLE_PRT("bleEvtDispatch callbackFlag %d managerCbFlag %d\n",
//		commMem.callbackFlag, commMem.managerCbFlag);
	if (commMem.callbackFlag) {
		bleCommEvtDispatch(pBleNrfEvt);
	}
	if (commMem.managerCbFlag) {
		commMem.managerCb(pBleNrfEvt);
		//bleMngEvtDispatch(pBleNrfEvt);
	}
}

#if NRF_SD_BLE_API_VERSION > 5
static
void onAdvSetTerminate(BLE_Evt *pBleEvent, ble_evt_t *pBleNrfEvt)
{
#ifdef BLE_DBGPRT_ENABLE
	ble_gap_evt_adv_set_terminated_t *set = &pBleNrfEvt->evt.gap_evt.params.adv_set_terminated;
	BLE_PRT("onAdvSetTerminate reason=%d handle=%d num=%d\n",
		set->reason, set->adv_handle, set->num_completed_adv_events);
#endif
	pBleNrfEvt->evt.gap_evt.params.timeout.src = BLE_GAP_TIMEOUT_ADVERTISING;
	onTimeout(pBleEvent, pBleNrfEvt);
}
#endif

static
void onScanReqReport(BLE_Evt *pBleEvent, ble_evt_t *pBleNrfEvt)
{
	BLE_PRT("onScanReqReort\n");
}

static
void onPhyUpdateReq(BLE_Evt *pBleEvent, ble_evt_t *pBleNrfEvt)
{
	pBleEvent->evtHeader = BLE_GAP_EVENT_PHY_UPDATE_REQUEST;
	ble_gap_phys_t *phys = &pBleNrfEvt->evt.gap_evt.params.phy_update_request.peer_preferred_phys;
	BLE_PRT("onPhyUpdateReq tx_phy %d rx_phy %d\n", phys->tx_phys, phys->rx_phys);
	commMem.phyUpdate.txPhy = phys->tx_phys;
	commMem.phyUpdate.rxPhy = phys->rx_phys;
	commMem.phyUpdate.status = 0;
	pBleEvent->evtDataSize = sizeof(BLE_EvtPhyUpdate);
	memcpy(pBleEvent->evtData, &commMem.phyUpdate, pBleEvent->evtDataSize);
}

static
void onPhyUpdate(BLE_Evt *pBleEvent, ble_evt_t *pBleNrfEvt)
{
	pBleEvent->evtHeader = BLE_GAP_EVENT_PHY_UPDATE;
	ble_gap_evt_phy_update_t *phys = &pBleNrfEvt->evt.gap_evt.params.phy_update;
	BLE_PRT("onPhyUpdate stat %d tx_phy %d rx_phy %d\n", phys->status, phys->tx_phy, phys->rx_phy);
	commMem.phyUpdate.txPhy = phys->tx_phy;
	commMem.phyUpdate.rxPhy = phys->rx_phy;
	commMem.phyUpdate.status = phys->status;
	pBleEvent->evtDataSize = sizeof(BLE_EvtPhyUpdate);
	memcpy(pBleEvent->evtData, &commMem.phyUpdate, pBleEvent->evtDataSize);
}

static
void onDataLengthUpdateRequest(BLE_Evt *pBleEvent, ble_evt_t *pBleNrfEvt)
{
	int ret = 0;
#ifdef BLE_DBGPRT_ENABLE
	ble_gap_data_length_params_t *req = &pBleNrfEvt->evt.gap_evt.params.data_length_update_request.peer_params;
	BLE_PRT("onLenUpReq: max_tx=%d\n", req->max_tx_octets);
	BLE_PRT("onLenUpReq: max_rx=%d\n", req->max_rx_octets);
	BLE_PRT("onLenUpReq: tx_time=%d\n", req->max_tx_time_us);
	BLE_PRT("onLenUpReq: rx_time=%d\n", req->max_rx_time_us);
#endif
	ret = sd_ble_gap_data_length_update(pBleNrfEvt->evt.gap_evt.conn_handle, NULL, NULL);
	if (ret) {
		BLE_ERR("onLenUp: sd_ble_gap_data_length_update %d\n", ret);
	}
}

static
void onDataLengthUpdate(BLE_Evt *pBleEvent, ble_evt_t *pBleNrfEvt)
{
	pBleEvent->evtHeader = BLE_GAP_EVENT_DATA_LENGTH_UPDATE;
	ble_gap_data_length_params_t *datalen = &pBleNrfEvt->evt.gap_evt.params.data_length_update.effective_params;
	BLE_PRT("onLenUp: max_tx=%d\n", datalen->max_tx_octets);
	BLE_PRT("onLenUp: max_rx=%d\n", datalen->max_rx_octets);
	BLE_PRT("onLenUp: tx_time=%d\n", datalen->max_tx_time_us);
	BLE_PRT("onLenUp: rx_time=%d\n", datalen->max_rx_time_us);
	commMem.dataLen = *(BLE_EvtDataLengthUpdate *)datalen;
	pBleEvent->evtDataSize = sizeof(BLE_EvtDataLengthUpdate);
	memcpy(pBleEvent->evtData, &commMem.dataLen, pBleEvent->evtDataSize);
}

static
void onExchangeMtuRequest(BLE_Evt *pBleEvent, ble_evt_t *pBleNrfEvt)
{
	int ret = 0;
	ble_gatts_evt_exchange_mtu_request_t *req = &pBleNrfEvt->evt.gatts_evt.params.exchange_mtu_request;
	BLE_PRT("onMtuReq: mtu=%d\n", req->client_rx_mtu);
	if (NRF_SDH_BLE_GATT_MAX_MTU_SIZE > req->client_rx_mtu) {
		commMem.client_rx_mtu = req->client_rx_mtu;
	} else {
		commMem.client_rx_mtu = NRF_SDH_BLE_GATT_MAX_MTU_SIZE;
	}
	ret = sd_ble_gatts_exchange_mtu_reply(pBleNrfEvt->evt.gatts_evt.conn_handle, commMem.client_rx_mtu);
	if (ret) {
		BLE_ERR("onLenUp: sd_ble_gatts_exchange_mtu_reply %d\n", ret);
	}

	pBleEvent->evtHeader = BLE_GATTS_EVENT_EXCHANGE_MTU;
	commMem.gattsExchangeMTU.connHandle = pBleNrfEvt->evt.gatts_evt.conn_handle;
	commMem.gattsExchangeMTU.client_rx_mtu = req->client_rx_mtu;
	pBleEvent->evtDataSize = sizeof(BLE_EvtGattsExchangeMTU);
	memcpy(pBleEvent->evtData, &commMem.gattsExchangeMTU, pBleEvent->evtDataSize);
}

static
void onTxComplete(BLE_Evt *pBleEvent, ble_evt_t *pBleNrfEvt)
{
	pBleEvent->evtHeader = BLE_EVENT_TX_COMPLETE;
	if (pBleNrfEvt->header.evt_id == BLE_GATTC_EVT_WRITE_CMD_TX_COMPLETE) {
		commMem.txCompleteData.connHandle = pBleNrfEvt->evt.gattc_evt.conn_handle;
		commMem.txCompleteData.count = pBleNrfEvt->evt.gattc_evt.params.write_cmd_tx_complete.count;
		commMem.txCompleteData.role = BLE_ROLE_CENTRAL;
	} else {
		commMem.txCompleteData.connHandle = pBleNrfEvt->evt.gattc_evt.conn_handle;
		commMem.txCompleteData.count = pBleNrfEvt->evt.gattc_evt.params.write_cmd_tx_complete.count;
		commMem.txCompleteData.role = BLE_ROLE_PERIPHERAL;
	}
	pBleEvent->evtDataSize = sizeof(BLE_EvtTxComplete);
	memcpy(pBleEvent->evtData, &commMem.txCompleteData, pBleEvent->evtDataSize);
}

static
void onConnect(BLE_Evt *pBleEvent, ble_evt_t *pBleNrfEvt)
{
	pBleEvent->evtHeader = BLE_GAP_EVENT_CONNECTED;
	ble_gap_evt_connected_t *connected = &pBleNrfEvt->evt.gap_evt.params.connected;
	commMem.connectData.handle = pBleNrfEvt->evt.gap_evt.conn_handle;
	commMem.connectData.addr.type = connected->peer_addr.addr_type;
	memcpy(commMem.connectData.addr.addr, connected->peer_addr.addr, BLE_GAP_ADDR_LENGTH);
	if (connected->role == BLE_GAP_ROLE_PERIPH ) {
		commMem.connectData.role   = BLE_ROLE_PERIPHERAL;
	} else if (connected->role == BLE_GAP_ROLE_CENTRAL) {
		commMem.connectData.role   = BLE_ROLE_CENTRAL;
	}
	commMem.gapMem->connParams = *(ble_gap_conn_params_t *)&connected->conn_params;
#ifdef BLE_DBGPRT_ENABLE
	BLE_PRT("onConnect: handle=%d\n", commMem.connectData.handle);
	BLE_PRT("onConnect: role=%d\n", commMem.connectData.role);
	BLE_PRT("onConnect: type=%d\n", commMem.connectData.addr.type);
//	BLE_PRT("onConnect: identity=%d\n", commMem.connectData.addr.identitisList);
	BLE_PRT("onConnect: addr=0x");
	for (int i=0; i<6; i++) {
		BLE_PRT("%02x", commMem.connectData.addr.addr[i]);
	}
	BLE_PRT("\n");
	BLE_PRT("onConnect: minConnInterval=%d\n", commMem.gapMem->connParams.min_conn_interval);
	BLE_PRT("onConnect: maxConnInterval=%d\n", commMem.gapMem->connParams.max_conn_interval);
	BLE_PRT("onConnect: slaveLatency=%d\n", commMem.gapMem->connParams.slave_latency);
	BLE_PRT("onConnect: connSupTimeout=%d\n", commMem.gapMem->connParams.conn_sup_timeout);
#endif
	commMem.gapMem->wrapperBondInfo.connHandle = pBleNrfEvt->evt.gap_evt.conn_handle;
	commMem.gapMem->wrapperBondInfo.bondInfo.addrType = connected->peer_addr.addr_type;
	memcpy(commMem.gapMem->wrapperBondInfo.bondInfo.addr, connected->peer_addr.addr, BLE_GAP_ADDR_LENGTH);
	pBleEvent->evtDataSize = sizeof(BLE_EvtConnected);
	memcpy(pBleEvent->evtData, &commMem.connectData, pBleEvent->evtDataSize);
}

static
void onConnParamUpdate(BLE_Evt *pBleEvent, ble_evt_t *pBleNrfEvt)
{
	pBleEvent->evtHeader = BLE_GAP_EVENT_CONN_PARAM_UPDATE;
	ble_gap_conn_params_t *params = &pBleNrfEvt->evt.gap_evt.params.conn_param_update.conn_params;
	commMem.connParams.connParams.minConnInterval = params->min_conn_interval;
	commMem.connParams.connParams.maxConnInterval = params->max_conn_interval;
	commMem.connParams.connParams.slaveLatency = params->slave_latency;
	commMem.connParams.connParams.connSupTimeout = params->conn_sup_timeout;
	commMem.gapMem->connParams = *(ble_gap_conn_params_t *)params;
	pBleEvent->evtDataSize = sizeof(BLE_EvtConnParamUpdate);
	memcpy(pBleEvent->evtData, &commMem.connParams, pBleEvent->evtDataSize);
}

static
void onConnParamUpdateRequest(BLE_Evt *pBleEvent, ble_evt_t *pBleNrfEvt)
{
	pBleEvent->evtHeader = BLE_GAP_EVENT_CONN_PARAM_UPDATE_REQUEST;
	ble_gap_conn_params_t *params = &pBleNrfEvt->evt.gap_evt.params.conn_param_update_request.conn_params;
	commMem.connParams.connParams.minConnInterval = params->min_conn_interval;
	commMem.connParams.connParams.maxConnInterval = params->max_conn_interval;
	commMem.connParams.connParams.slaveLatency = params->slave_latency;
	commMem.connParams.connParams.connSupTimeout = params->conn_sup_timeout;
	commMem.gapMem->connParams = *(ble_gap_conn_params_t *)params;
	pBleEvent->evtDataSize = sizeof(BLE_EvtConnParamUpdate);
	memcpy(pBleEvent->evtData, &commMem.connParams, pBleEvent->evtDataSize);
}

static
void onDisconnect(BLE_Evt *pBleEvent, ble_evt_t *pBleNrfEvt)
{
	pBleEvent->evtHeader = BLE_GAP_EVENT_DISCONNECTED;
	commMem.disconnectData.handle = pBleNrfEvt->evt.gap_evt.conn_handle;
	commMem.disconnectData.reason = pBleNrfEvt->evt.gap_evt.params.disconnected.reason;
	BLE_PRT("onDisconnect: reason=%d\n", pBleNrfEvt->evt.gap_evt.params.disconnected.reason);
	pBleEvent->evtDataSize = sizeof(BLE_EvtDisconnected);
	memcpy(pBleEvent->evtData, &commMem.disconnectData, pBleEvent->evtDataSize);
	commMem.gapMem->is_connected = false;
}

static
void onSecParamsRequest(BLE_Evt *pBleEvent, ble_evt_t *pBleNrfEvt)
{
	ble_gap_evt_sec_params_request_t *secParamReq = &pBleNrfEvt->evt.gap_evt.params.sec_params_request;
	pBleEvent->evtHeader = BLE_GAP_EVENT_EXCHANGE_FEATURE;
	commMem.exchangeFeatureData.handle              = pBleNrfEvt->evt.gap_evt.conn_handle;
	commMem.exchangeFeatureData.peerFeature.oob     = (BLE_GapOOB)secParamReq->peer_params.oob;
	commMem.exchangeFeatureData.peerFeature.ioCap   = (BLE_GapIoCap)secParamReq->peer_params.io_caps;
	commMem.exchangeFeatureData.peerFeature.authReq = (BLE_GapAuth)((secParamReq->peer_params.mitm)<<1 |(secParamReq->peer_params.bond));
	commMem.exchangeFeatureData.peerFeature.maxKeySize = secParamReq->peer_params.max_key_size;
	commMem.exchangeFeatureData.peerFeature.minKeySize = secParamReq->peer_params.min_key_size;
	pBleEvent->evtDataSize = sizeof(BLE_EvtExchangeFeature);
	memcpy(pBleEvent->evtData, &commMem.exchangeFeatureData, pBleEvent->evtDataSize);
}

static
void onAuthStatus(BLE_Evt *pBleEvent, ble_evt_t *pBleNrfEvt)
{
	pBleEvent->evtHeader = BLE_GAP_EVENT_AUTH_STATUS;
	ble_gap_evt_auth_status_t *authStatus = &pBleNrfEvt->evt.gap_evt.params.auth_status;
	commMem.authStatusData.handle = pBleNrfEvt->evt.gap_evt.conn_handle;
#ifdef BLE_DBGPRT_ENABLE
	BLE_PRT("onAuthStatus: handle=%d\n", pBleNrfEvt->evt.gap_evt.conn_handle);
	BLE_PRT("onAuthStatus: auth_status=%d\n", authStatus->auth_status);
	BLE_PRT("onAuthStatus: error_src=%d\n", authStatus->error_src);
	BLE_PRT("onAuthStatus: bonded=%d\n", authStatus->bonded);
	BLE_PRT("onAuthStatus: lesc=%d\n", authStatus->lesc);
	BLE_PRT("onAuthStatus: sm1_levels_lv1=%d\n", authStatus->sm1_levels.lv1);
	BLE_PRT("onAuthStatus: sm1_levels_lv2=%d\n", authStatus->sm1_levels.lv2);
	BLE_PRT("onAuthStatus: sm1_levels_lv3=%d\n", authStatus->sm1_levels.lv3);
	BLE_PRT("onAuthStatus: sm2_levels_lv1=%d\n", authStatus->sm2_levels.lv1);
	BLE_PRT("onAuthStatus: sm2_levels_lv2=%d\n", authStatus->sm2_levels.lv2);
	BLE_PRT("onAuthStatus: sm2_levels_lv3=%d\n", authStatus->sm2_levels.lv3);
	BLE_PRT("onAuthStatus: own.enc=%d\n", authStatus->kdist_own.enc);
	BLE_PRT("onAuthStatus: own.id=%d\n", authStatus->kdist_own.id);
	BLE_PRT("onAuthStatus: own.sign=%d\n", authStatus->kdist_own.sign);
	BLE_PRT("onAuthStatus: own.link=%d\n", authStatus->kdist_own.link);
	BLE_PRT("onAuthStatus: peer.enc=%d\n", authStatus->kdist_peer.enc);
	BLE_PRT("onAuthStatus: peer.id=%d\n", authStatus->kdist_peer.id);
	BLE_PRT("onAuthStatus: peer.sign=%d\n", authStatus->kdist_peer.sign);
	BLE_PRT("onAuthStatus: peer.link=%d\n", authStatus->kdist_peer.link);
#if 0
	// own
	BLE_PRT("onAuthStatus: own id_irk=0x");
	for (int i=0; i<16; i++) {
		BLE_PRT("%02x", commMem.gapMem->wrapperBondInfo.ownIdKey.id_info.irk[i]);
	}
	BLE_PRT("\n");
	BLE_PRT("onAuthStatus: own id_addr_type=%d\n", commMem.gapMem->wrapperBondInfo.ownIdKey.id_addr_info.addr_type);
	BLE_PRT("onAuthStatus: own id_addr=0x");
	for (int i=0; i<6; i++) {
		BLE_PRT("%02x", commMem.gapMem->wrapperBondInfo.ownIdKey.id_addr_info.addr[i]);
	}
	BLE_PRT("\n");
	BLE_PRT("onAuthStatus: own master_id_ediv=%d\n", commMem.gapMem->wrapperBondInfo.ownEncKey.master_id.ediv);
	BLE_PRT("onAuthStatus: own master_id_rand=0x");
	for (int i=0; i<8; i++) {
		BLE_PRT("%02x", commMem.gapMem->wrapperBondInfo.ownEncKey.master_id.rand[i]);
	}
	BLE_PRT("\n");
	BLE_PRT("onAuthStatus: own enc_info_auth=%d\n", commMem.gapMem->wrapperBondInfo.ownEncKey.enc_info.auth);
	if (commMem.gapMem->wrapperBondInfo.ownEncKey.enc_info.ltk_len) {
		BLE_PRT("onAuthStatus: own enc_info_ltk=0x");
		for (int i=0; i<commMem.gapMem->wrapperBondInfo.ownEncKey.enc_info.ltk_len; i++) {
			BLE_PRT("%02x", commMem.gapMem->wrapperBondInfo.ownEncKey.enc_info.ltk[i]);
		}
		BLE_PRT("\n");
	} else {
		BLE_PRT("onAuthStatus: own enc_info_ltk=%d\n", commMem.gapMem->wrapperBondInfo.ownEncKey.enc_info.ltk_len);
	}
#endif
	// peer
	BLE_PRT("onAuthStatus: peer id_irk=0x");
	for (int i=0; i<16; i++) {
		BLE_PRT("%02x", commMem.gapMem->wrapperBondInfo.peerIdKey.id_info.irk[i]);
	}
	BLE_PRT("\n");
	BLE_PRT("onAuthStatus: peer id_addr_type=%d\n", commMem.gapMem->wrapperBondInfo.peerIdKey.id_addr_info.addr_type);
	BLE_PRT("onAuthStatus: peer id_addr=0x");
	for (int i=0; i<6; i++) {
		BLE_PRT("%02x", commMem.gapMem->wrapperBondInfo.peerIdKey.id_addr_info.addr[i]);
	}
	BLE_PRT("\n");

	BLE_PRT("onAuthStatus: peer master_id_ediv=%d\n", commMem.gapMem->wrapperBondInfo.peerEncKey.master_id.ediv);
	BLE_PRT("onAuthStatus: peer master_id_rand=0x");
	for (int i=0; i<8; i++) {
		BLE_PRT("%02x", commMem.gapMem->wrapperBondInfo.peerEncKey.master_id.rand[i]);
	}
	BLE_PRT("\n");
	BLE_PRT("onAuthStatus: peer enc_info_auth=%d\n", commMem.gapMem->wrapperBondInfo.peerEncKey.enc_info.auth);
	if (commMem.gapMem->wrapperBondInfo.peerEncKey.enc_info.ltk_len) {
		BLE_PRT("onAuthStatus: peer enc_info_ltk=0x");
		for (int i=0; i<commMem.gapMem->wrapperBondInfo.peerEncKey.enc_info.ltk_len; i++) {
			BLE_PRT("%02x", commMem.gapMem->wrapperBondInfo.peerEncKey.enc_info.ltk[i]);
		}
		BLE_PRT("\n");
	} else {
		BLE_PRT("onAuthStatus: peer enc_info_ltk=%d\n", commMem.gapMem->wrapperBondInfo.peerEncKey.enc_info.ltk_len);
	}
#endif
	//Auth status code.
	if ( authStatus->auth_status < BLE_GAP_SEC_STATUS_PDU_INVALID )
	{
		commMem.authStatusData.status = authStatus->auth_status;
	}
	else if ((authStatus->auth_status > BLE_GAP_SEC_STATUS_RFU_RANGE1_END) && (authStatus->auth_status != BLE_GAP_SEC_STATUS_RFU_RANGE2_END))
	{
		commMem.authStatusData.status = authStatus->auth_status -(BLE_GAP_SEC_STATUS_RFU_RANGE1_END - BLE_GAP_SM_STATUS_RESERVED);
	}
	else {
		commMem.authStatusData.status = BLE_GAP_SM_STATUS_RESERVED;
	}
#if 1
	if (authStatus->kdist_peer.id) {
		BLE_PRT("update bondinfo\n");
		commMem.gapMem->wrapperBondInfo.bondInfo.addrType = commMem.gapMem->wrapperBondInfo.peerIdKey.id_addr_info.addr_type;
		memcpy(commMem.gapMem->wrapperBondInfo.bondInfo.addr, commMem.gapMem->wrapperBondInfo.peerIdKey.id_addr_info.addr, BLE_GAP_ADDR_LENGTH);
	}
#endif
	memcpy(&commMem.authStatusData.bondInfo, &commMem.gapMem->wrapperBondInfo.bondInfo, sizeof(BLE_GapBondInfo));
	pBleEvent->evtDataSize = sizeof(BLE_EvtAuthStatus);
	memcpy(pBleEvent->evtData, &commMem.authStatusData, pBleEvent->evtDataSize);
}

static
void onDispPasskey(BLE_Evt *pBleEvent, ble_evt_t *pBleNrfEvt)
{
	pBleEvent->evtHeader = BLE_GAP_EVENT_DISPLAY_PASSKEY;
	ble_gap_evt_passkey_display_t *dispPasskey = &pBleNrfEvt->evt.gap_evt.params.passkey_display;
	commMem.dispPasskeyData.handle = pBleNrfEvt->evt.gap_evt.conn_handle;
	memcpy(commMem.dispPasskeyData.passkey,dispPasskey->passkey,BLE_GAP_PASSKEY_LEN);
	commMem.dispPasskeyData.passkey[BLE_GAP_PASSKEY_LEN] = '\0';
	pBleEvent->evtDataSize = sizeof(BLE_EvtDisplayPasskey);
	memcpy(pBleEvent->evtData, &commMem.dispPasskeyData, pBleEvent->evtDataSize);
}

#if NRF_SD_BLE_API_VERSION > 5
extern uint8_t bleAdvReportBuffer[BLE_GAP_SCAN_BUFFER_EXTENDED_MIN];
#endif

static
void onAdvReport(BLE_Evt *pBleEvent, ble_evt_t *pBleNrfEvt)
{
	pBleEvent->evtHeader = BLE_GAP_EVENT_ADV_REPORT;
	ble_gap_evt_adv_report_t *advReport = &pBleNrfEvt->evt.gap_evt.params.adv_report;
#if NRF_SD_BLE_API_VERSION > 5
	BLE_PRT("onAdvReport: rssi=%d scan_rsp=%d type=%d dlen=%d\n",
		advReport->rssi, advReport->type.scan_response, advReport->type.status, advReport->data.len);
	BLE_PRT("             connectable=%d scannable=%d directed=%d ext_pdu=%d set=%d dat=%d\n",
		advReport->type.connectable, advReport->type.scannable, advReport->type.directed,
		advReport->type.extended_pdu, advReport->set_id, advReport->data_id);

	if (advReport->type.status == BLE_GAP_ADV_DATA_STATUS_COMPLETE) {
		commMem.advReportData.dlen = advReport->data.len;
		memcpy(commMem.advReportData.data, advReport->data.p_data, advReport->data.len);
	}

	commMem.advReportData.scan_rsp = advReport->type.scan_response;
	commMem.advReportData.dlen = advReport->data.len;
	memcpy(commMem.advReportData.data, advReport->data.p_data, advReport->data.len);
	commMem.advReportData.rssi = advReport->rssi;
	memcpy(commMem.advReportData.addr.addr, advReport->peer_addr.addr, BLE_GAP_ADDR_LENGTH);
	commMem.advReportData.addr.type = advReport->peer_addr.addr_type;
	pBleEvent->evtDataSize = sizeof(BLE_EvtAdvReportData);
	memcpy(pBleEvent->evtData, &commMem.advReportData, pBleEvent->evtDataSize);
	ble_data_t bleAdvReport = {bleAdvReportBuffer, BLE_GAP_SCAN_BUFFER_EXTENDED_MIN};
	if (sd_ble_gap_scan_start(NULL, &bleAdvReport)) {
		BLE_ERR("sd_ble_gap_scan_start failed.\n");
	}
#else
	BLE_PRT("onAdvReport: rssi=%d scan_rsp=%d type=%d dlen=%d\n",
		advReport->rssi, advReport->scan_rsp, advReport->type, advReport->dlen);
	commMem.advReportData.scan_rsp = advReport->scan_rsp;
	commMem.advReportData.dlen = advReport->dlen;
	memcpy(commMem.advReportData.data, advReport->data, BLE_GAP_ADV_MAX_SIZE);
	commMem.advReportData.rssi = advReport->rssi;
	memcpy(commMem.advReportData.addr.addr, advReport->peer_addr.addr, BLE_GAP_ADDR_LENGTH);
	commMem.advReportData.addr.type = advReport->peer_addr.addr_type;
	pBleEvent->evtDataSize = sizeof(BLE_EvtAdvReportData);
	memcpy(pBleEvent->evtData, &commMem.advReportData, pBleEvent->evtDataSize);
#endif
}

static
void onAuthKeyRequest(BLE_Evt *pBleEvent, ble_evt_t *pBleNrfEvt)
{
	pBleEvent->evtHeader = BLE_GAP_EVENT_AUTH_KEY_REQUEST;
	ble_gap_evt_auth_key_request_t *authKey = &pBleNrfEvt->evt.gap_evt.params.auth_key_request;
	commMem.authKeyData.handle = pBleNrfEvt->evt.gap_evt.conn_handle;
	commMem.authKeyData.keyType = authKey->key_type;
	pBleEvent->evtDataSize = sizeof(BLE_EvtAuthKey);
	memcpy(pBleEvent->evtData, &commMem.authKeyData, pBleEvent->evtDataSize);
}

static
void onConnSecUpdate(BLE_Evt *pBleEvent, ble_evt_t *pBleNrfEvt)
{
	pBleEvent->evtHeader = BLE_GAP_EVENT_CONN_SEC_UPDATE;
	pBleEvent->evtDataSize = 0;
#ifdef BLE_DBGPRT_ENABLE
	ble_gap_evt_conn_sec_update_t *sec = &pBleNrfEvt->evt.gap_evt.params.conn_sec_update;
	BLE_PRT("onConnSecUpdate: keysize=%d sm=%d lv=%d\n", sec->conn_sec.encr_key_size,
		sec->conn_sec.sec_mode.sm, sec->conn_sec.sec_mode.lv);
#endif
}

static
void onSecInfoRequest(BLE_Evt *pBleEvent, ble_evt_t *pBleNrfEvt)
{
	ble_gap_enc_info_t *enc_info = NULL;
	ble_gap_irk_t *id_info = NULL;
	ble_gap_sign_info_t *sign_info = NULL;
	ble_gap_evt_sec_info_request_t *secinfo = &pBleNrfEvt->evt.gap_evt.params.sec_info_request;
#ifdef BLE_DBGPRT_ENABLE
	BLE_PRT("onSecInfoRequest: handle=%d\n", pBleNrfEvt->evt.gap_evt.conn_handle);
	BLE_PRT("onSecInfoRequest: addr_type=%d\n", secinfo->peer_addr.addr_type);
	BLE_PRT("onSecInfoRequest: peer_addr=0x");
	for (int i=0; i<6; i++) {
		BLE_PRT("%02x", secinfo->peer_addr.addr[i]);
	}
	BLE_PRT("\n");
	BLE_PRT("onSecInfoRequest: master_id_ediv=%d\n", secinfo->master_id.ediv);
	BLE_PRT("onSecInfoRequest: master_id_rand=0x");
	for (int i=0; i<8; i++) {
		BLE_PRT("%02x", secinfo->master_id.rand[i]);
	}
	BLE_PRT("\n");
	BLE_PRT("onSecInfoRequest: enc_info=%d\n", secinfo->enc_info);
	BLE_PRT("onSecInfoRequest: id_info=%d\n", secinfo->id_info);
	BLE_PRT("onSecInfoRequest: sign_info=%d\n", secinfo->sign_info);
#if 0
	// own
	BLE_PRT("onSecInfoRequest: own id_irk=0x");
	for (int i=0; i<16; i++) {
		BLE_PRT("%02x", commMem.gapMem->wrapperBondInfo.ownIdKey.id_info.irk[i]);
	}
	BLE_PRT("\n");
	BLE_PRT("onSecInfoRequest: own id_addr_type=%d\n", commMem.gapMem->wrapperBondInfo.ownIdKey.id_addr_info.addr_type);
	BLE_PRT("onSecInfoRequest: own id_addr=0x");
	for (int i=0; i<6; i++) {
		BLE_PRT("%02x", commMem.gapMem->wrapperBondInfo.ownIdKey.id_addr_info.addr[i]);
	}
	BLE_PRT("\n");
	BLE_PRT("onSecInfoRequest: own master_id_ediv=%d\n", commMem.gapMem->wrapperBondInfo.ownEncKey.master_id.ediv);
	BLE_PRT("onSecInfoRequest: own master_id_rand=0x");
	for (int i=0; i<8; i++) {
		BLE_PRT("%02x", commMem.gapMem->wrapperBondInfo.ownEncKey.master_id.rand[i]);
	}
	BLE_PRT("\n");
	BLE_PRT("onSecInfoRequest: own enc_info_auth=%d\n", commMem.gapMem->wrapperBondInfo.ownEncKey.enc_info.auth);
	if (commMem.gapMem->wrapperBondInfo.ownEncKey.enc_info.ltk_len) {
		BLE_PRT("onSecInfoRequest: own enc_info_ltk=0x");
		for (int i=0; i<commMem.gapMem->wrapperBondInfo.ownEncKey.enc_info.ltk_len; i++) {
			BLE_PRT("%02x", commMem.gapMem->wrapperBondInfo.ownEncKey.enc_info.ltk[i]);
		}
		BLE_PRT("\n");
	} else {
		BLE_PRT("onSecInfoRequest: own enc_info_ltk=%d\n", commMem.gapMem->wrapperBondInfo.peerEncKey.enc_info.ltk_len);
	}
#endif
	// peer
	BLE_PRT("onSecInfoRequest: peer id_irk=0x");
	for (int i=0; i<16; i++) {
		BLE_PRT("%02x", commMem.gapMem->wrapperBondInfo.peerIdKey.id_info.irk[i]);
	}
	BLE_PRT("\n");
	BLE_PRT("onSecInfoRequest: peer id_addr_type=%d\n", commMem.gapMem->wrapperBondInfo.peerIdKey.id_addr_info.addr_type);
	BLE_PRT("onSecInfoRequest: peer id_addr=0x");
	for (int i=0; i<6; i++) {
		BLE_PRT("%02x", commMem.gapMem->wrapperBondInfo.peerIdKey.id_addr_info.addr[i]);
	}
	BLE_PRT("\n");
	BLE_PRT("onSecInfoRequest: peer master_id_ediv=%d\n", commMem.gapMem->wrapperBondInfo.peerEncKey.master_id.ediv);
	BLE_PRT("onSecInfoRequest: peer master_id_rand=0x");
	for (int i=0; i<8; i++) {
		BLE_PRT("%02x", commMem.gapMem->wrapperBondInfo.peerEncKey.master_id.rand[i]);
	}
	BLE_PRT("\n");
	BLE_PRT("onSecInfoRequest: peer enc_info_auth=%d\n", commMem.gapMem->wrapperBondInfo.peerEncKey.enc_info.auth);
	if (commMem.gapMem->wrapperBondInfo.peerEncKey.enc_info.ltk_len) {
		BLE_PRT("onSecInfoRequest: peer enc_info_ltk=0x");
		for (int i=0; i<commMem.gapMem->wrapperBondInfo.peerEncKey.enc_info.ltk_len; i++) {
			BLE_PRT("%02x", commMem.gapMem->wrapperBondInfo.peerEncKey.enc_info.ltk[i]);
		}
		BLE_PRT("\n");
	} else {
		BLE_PRT("onSecInfoRequest: peer enc_info_ltk=%d\n", commMem.gapMem->wrapperBondInfo.peerEncKey.enc_info.ltk_len);
	}
#endif
	uint32_t list = bleBondEnableList;
	for(int index = 0; index < BLE_SAVE_BOND_DEVICE_MAX_NUM; index++, list >>= 1) {
		if (!(list & 1)) {
			continue;
		}
		if(!memcmp(BondInfoInFlash[index].peerEncKey.master_id.rand, secinfo->master_id.rand, BLE_GAP_SEC_RAND_LEN)) {
			BLE_PRT("onSecInfoRequest: master_id exitsting index %d\n", index);
			enc_info = &BondInfoInFlash[index].peerEncKey.enc_info;
			id_info = &BondInfoInFlash[index].peerIdKey.id_info;
			break;
		}
	}
	sd_ble_gap_sec_info_reply(pBleNrfEvt->evt.gap_evt.conn_handle, enc_info, id_info, sign_info);
}

static
void onTimeout(BLE_Evt *pBleEvent, ble_evt_t *pBleNrfEvt)
{
	pBleEvent->evtHeader = BLE_GAP_EVENT_TIMEOUT;
	ble_gap_evt_timeout_t *timeout = &pBleNrfEvt->evt.gap_evt.params.timeout;
	commMem.timeoutData.handle = pBleNrfEvt->evt.gap_evt.conn_handle;
	commMem.timeoutData.timeoutSrc = timeout->src;
	pBleEvent->evtDataSize = sizeof(BLE_EvtTimeout);
	memcpy(pBleEvent->evtData, &commMem.timeoutData, pBleEvent->evtDataSize);
}

static
void onGattsTimeout(BLE_Evt *pBleEvent, ble_evt_t *pBleNrfEvt)
{
	pBleEvent->evtHeader = BLE_GATTS_EVENT_TIMEOUT;
	commMem.timeoutData.handle = pBleNrfEvt->evt.gatts_evt.conn_handle;
	commMem.timeoutData.timeoutSrc = BLE_GATT_TIMEOUT_PROTOCOL;
	pBleEvent->evtDataSize = sizeof(BLE_EvtTimeout);
	memcpy(pBleEvent->evtData, &commMem.timeoutData, pBleEvent->evtDataSize);
}

static
void onRssiChanged(BLE_Evt *pBleEvent, ble_evt_t *pBleNrfEvt)
{
	pBleEvent->evtHeader = BLE_GAP_EVENT_RSSI_CHANGED;
	ble_gap_evt_rssi_changed_t *rssiChanged = &pBleNrfEvt->evt.gap_evt.params.rssi_changed;
	commMem.rssiChangeData.handle = pBleNrfEvt->evt.gap_evt.conn_handle;
	commMem.rssiChangeData.rssi = rssiChanged->rssi;
	commMem.gapMem->peerRssi = rssiChanged->rssi;
	pBleEvent->evtDataSize = sizeof(BLE_EvtRssiChanged);
	memcpy(pBleEvent->evtData, &commMem.rssiChangeData, pBleEvent->evtDataSize);
}

static
void onGattsWrite(BLE_Evt *pBleEvent, ble_evt_t *pBleNrfEvt)
{
	pBleEvent->evtHeader = BLE_GATTS_EVENT_WRITE;
	ble_gatts_evt_write_t * gattsWrite = &pBleNrfEvt->evt.gatts_evt.params.write;
	commMem.gattsWriteData.connHandle = pBleNrfEvt->evt.gatts_evt.conn_handle;
	commMem.gattsWriteData.handle  = gattsWrite->handle;  //attribute handle
	commMem.gattsWriteData.dataLen = gattsWrite->len;
	commMem.gattsWriteData.offset  = gattsWrite->offset;
	memcpy(commMem.gattsWriteData.data, gattsWrite->data, MAX_RCV_DATA_LENGTH);
	pBleEvent->evtDataSize = sizeof(BLE_EvtGattsWrite);
	memcpy(pBleEvent->evtData, &commMem.gattsWriteData, pBleEvent->evtDataSize);
}

static
void onGattsIndConfirm(BLE_Evt *pBleEvent, ble_evt_t *pBleNrfEvt)
{
	pBleEvent->evtHeader = BLE_GATTS_EVENT_CFM;
	ble_gatts_evt_hvc_t * gattsConfirm = &pBleNrfEvt->evt.gatts_evt.params.hvc;
	commMem.gattsIndConfirmData.connHandle    = pBleNrfEvt->evt.gatts_evt.conn_handle;
	commMem.gattsIndConfirmData.handle        = gattsConfirm->handle;
	pBleEvent->evtDataSize = sizeof(BLE_EvtGattsIndConfirm);
	memcpy(pBleEvent->evtData, &commMem.gattsIndConfirmData, pBleEvent->evtDataSize);
}

static void onReadRsp(BLE_Evt *pBleEvent, ble_evt_t *pBleNrfEvt)
{
	pBleEvent->evtHeader = BLE_GATTC_EVENT_READ;
	ble_gattc_evt_t *bleGattcEvt                         = &(pBleNrfEvt->evt.gattc_evt);
	const ble_gattc_evt_read_rsp_t *bleGattcEvtReadRsp   = &(bleGattcEvt->params.read_rsp);
	commMem.gattcReadData.connHandle    = pBleNrfEvt->evt.gattc_evt.conn_handle;
	commMem.gattcReadData.charValHandle = bleGattcEvtReadRsp->handle;
	commMem.gattcReadData.charValLen    = bleGattcEvtReadRsp->len;
	memcpy(commMem.gattcReadData.charValData, bleGattcEvtReadRsp->data, MAX_VAL_DATA_LENGTH);
	pBleEvent->evtDataSize = sizeof(BLE_EvtGattcRead);
	memcpy(pBleEvent->evtData, &commMem.gattcReadData, pBleEvent->evtDataSize);
}

static void onWriteRsp(BLE_Evt *pBleEvent, ble_evt_t *pBleNrfEvt)
{
	pBleEvent->evtHeader = BLE_GATTC_EVENT_WRITE_RSP;
	pBleEvent->evtDataSize = sizeof(BLE_EvtGattcWriteRsp);

	BLE_EvtGattcWriteRsp* wrRsp = (BLE_EvtGattcWriteRsp*)pBleEvent->evtData;
	ble_gattc_evt_t *bleGattcEvt                         = &(pBleNrfEvt->evt.gattc_evt);
	const ble_gattc_evt_read_rsp_t *bleGattcEvtReadRsp   = &(bleGattcEvt->params.read_rsp);

	wrRsp->connHandle    = bleGattcEvt->conn_handle;
	wrRsp->charValHandle = bleGattcEvtReadRsp->handle;
	wrRsp->status        = bleGattcEvt->gatt_status;
}

static
void onHvx(BLE_Evt *pBleEvent, ble_evt_t *pBleNrfEvt)
{
	pBleEvent->evtHeader = BLE_GATTC_EVENT_NTFIND;
	ble_gattc_evt_t *bleGattcEvt                = &(pBleNrfEvt->evt.gattc_evt);
	const ble_gattc_evt_hvx_t *bleGattcEvtHvx   = &(bleGattcEvt->params.hvx);
	commMem.gattcNtfIndData.connHandle   = pBleNrfEvt->evt.gattc_evt.conn_handle;
	commMem.gattcNtfIndData.attrHandle   = bleGattcEvtHvx->handle;
	commMem.gattcNtfIndData.type         = (BLE_GattNtyIndType)bleGattcEvtHvx->type;
	commMem.gattcNtfIndData.attrValLen   = bleGattcEvtHvx->len;
	memcpy(commMem.gattcNtfIndData.attrValData, bleGattcEvtHvx->data, MAX_VAL_DATA_LENGTH);
	pBleEvent->evtDataSize = sizeof(BLE_EvtGattcNtfInd);
	memcpy(pBleEvent->evtData, &commMem.gattcNtfIndData, pBleEvent->evtDataSize);
}

static
void onSysAttrMissing(BLE_Evt *pBleEvent, ble_evt_t *pBleNrfEvt)
{
	int ret = BLE_SUCCESS;
	ret = sd_ble_gatts_sys_attr_set(commMem.connectData.handle, NULL, 0, 0);
	if (ret) {
		pBleEvent->evtHeader = BLE_GATTS_EVENT_SYS_ATTR_MISSING;
	}
}

static
void setDbDiscoveryEvent(BLE_Evt *pBleEvent, uint16_t connHandle, int result, int reason)
{
	commMem.gattcDbDiscoveryData.connHandle = connHandle;
	commMem.gattcDbDiscoveryData.result = result;
	commMem.gattcDbDiscoveryData.params.reason = reason;
	if (result == BLE_GATTC_RESULT_SUCCESS) {
		commMem.gattcDb.dbDiscovery.srvCount = commMem.gattcDb.currSrvInd;
		memcpy(&commMem.gattcDbDiscoveryData.params.dbDiscovery, &commMem.gattcDb.dbDiscovery, sizeof(BLE_GattcDbDiscovery));
	}
	pBleEvent->evtHeader = BLE_GATTC_EVENT_DBDISCOVERY;
	pBleEvent->evtDataSize = sizeof(BLE_EvtGattcDbDiscovery);
	memcpy(pBleEvent->evtData, &commMem.gattcDbDiscoveryData, pBleEvent->evtDataSize);
	commMem.gattcDb.currCharInd = 0;
	commMem.gattcDb.currSrvInd  = 0;
	return;
}

static
void onPrimarySrvDiscoveryRsp(bleGattcDb *gattcDbDiscovery, BLE_Evt *pBleEvent, ble_evt_t *pBleNrfEvt)
{
	uint16_t connHandle = 0;
	ble_gattc_evt_t *bleGattcEvt = NULL;
	BLE_GattcDbDiscSrv                   *srvBeingDiscovered = NULL;

	bleGattcEvt = &(pBleNrfEvt->evt.gattc_evt);
	connHandle  = bleGattcEvt->conn_handle;
	BLE_PRT2("onPrimary start ind %d\n", gattcDbDiscovery->currSrvInd);
	if (bleGattcEvt->gatt_status == BLE_GATT_STATUS_SUCCESS) {
		if (gattcDbDiscovery->currSrvInd >= BLE_DB_DISCOVERY_MAX_SRV) {
			gattcDbDiscovery->discoveryInProgress = false;
			BLE_ERR("onPrimary MAX_SRV\n");
			setDbDiscoveryEvent(pBleEvent, connHandle, BLE_GATTC_RESULT_FAILED, BLE_GATTC_REASON_SERVICE);
			return;
		}
		gattcDbDiscovery->dbDiscovery.connHandle = connHandle;
		srvBeingDiscovered = &(gattcDbDiscovery->dbDiscovery.services[gattcDbDiscovery->currSrvInd]);

		const ble_gattc_evt_prim_srvc_disc_rsp_t *primSrvcDiscRspEvt = &(bleGattcEvt->params.prim_srvc_disc_rsp);
		srvBeingDiscovered->srvUuid.value.baseAlias.uuidAlias = primSrvcDiscRspEvt->services[0].uuid.uuid; //BLE_ENABLE_NORDIC_ORIGINAL
		srvBeingDiscovered->srvUuid.type = (BLE_GattsUuidType)primSrvcDiscRspEvt->services[0].uuid.type; //BLE_ENABLE_NORDIC_ORIGINAL
		srvBeingDiscovered->srvHandleRange.startHandle =  primSrvcDiscRspEvt->services[0].handle_range.start_handle;
		srvBeingDiscovered->srvHandleRange.endHandle =  primSrvcDiscRspEvt->services[0].handle_range.end_handle;
		BLE_PRT2("onPrimary cnt %d type %d uuid 0x%04X sHdl %d eHdl %d\n",
			primSrvcDiscRspEvt->count,
			primSrvcDiscRspEvt->services[0].uuid.type,
			primSrvcDiscRspEvt->services[0].uuid.uuid,
			primSrvcDiscRspEvt->services[0].handle_range.start_handle,
			primSrvcDiscRspEvt->services[0].handle_range.end_handle);
		//get current service info finished. characteristic discovery which belongs to this service.
		(void)characteristicsDiscover(pBleEvent, gattcDbDiscovery);
	}
	else if (bleGattcEvt->gatt_status == BLE_GATT_STATUS_ATTERR_ATTRIBUTE_NOT_FOUND) {
		//db discovery complete.send success event to application
		gattcDbDiscovery->discoveryInProgress = true;
		setDbDiscoveryEvent(pBleEvent, connHandle, BLE_GATTC_RESULT_SUCCESS, 0);
	}
	else {
		//db discovery error.send event to application
		gattcDbDiscovery->discoveryInProgress = false;
		BLE_ERR("onPrimary Status error %d\n", bleGattcEvt->gatt_status);
		setDbDiscoveryEvent(pBleEvent, connHandle, BLE_GATTC_RESULT_FAILED, BLE_GATTC_REASON_SERVICE);
	}
}

static
void onCharacteristicDiscoveryRsp(bleGattcDb *const gattcDbDiscovery, BLE_Evt *pBleEvent, ble_evt_t *pBleNrfEvt)
{
	uint8_t numCharsPrevDisc = 0;
	uint8_t numCharsCurrDisc = 0;
	uint16_t connHandle = 0;
	uint32_t i               = 0;
	uint32_t j               = 0;
	bool   performDescDiscov = false;
	bool raiseDiscovComplete = false;
	ble_gattc_evt_t                 *bleGattcEvt = &(pBleNrfEvt->evt.gattc_evt);
	BLE_GattcDbDiscSrv       *srvBeingDiscovered = &(gattcDbDiscovery->dbDiscovery.services[gattcDbDiscovery->currSrvInd]);
	BLE_GattcChar            *lastKnownChar = NULL;

	connHandle  = bleGattcEvt->conn_handle;
	if (gattcDbDiscovery->currSrvInd >= BLE_DB_DISCOVERY_MAX_SRV) {
		BLE_ERR("onChar ind NG\n");
		gattcDbDiscovery->discoveryInProgress = false;
		setDbDiscoveryEvent(pBleEvent, connHandle, BLE_GATTC_RESULT_FAILED, BLE_GATTC_REASON_CHARACTERISTIC);
		return;
	}
	if (bleGattcEvt->gatt_status == BLE_GATT_STATUS_SUCCESS) {
		const ble_gattc_evt_char_disc_rsp_t * charDiscRspEvt;

		charDiscRspEvt = &(bleGattcEvt->params.char_disc_rsp);

		numCharsPrevDisc = srvBeingDiscovered->charCount;
		numCharsCurrDisc = charDiscRspEvt->count;
		BLE_PRT2("onChar preCnt %d curCnt %d\n", numCharsPrevDisc, numCharsCurrDisc);
		// Check if the total number of discovered characteristics are supported
		if ((numCharsPrevDisc + numCharsCurrDisc) <= BLE_DB_DISCOVERY_MAX_CHAR_PER_SRV) {
			// Update the characteristics count.
			srvBeingDiscovered->charCount += numCharsCurrDisc;
		}
		else {
			// The number of characteristics discovered at the peer is more than the supported maximum.
			srvBeingDiscovered->charCount = BLE_DB_DISCOVERY_MAX_CHAR_PER_SRV;
		}

		for (i = numCharsPrevDisc, j = 0; i < srvBeingDiscovered->charCount; i++, j++) {
			memcpy(&srvBeingDiscovered->characteristics[i].characteristic.charPrope, &charDiscRspEvt->chars[j].char_props, sizeof(BLE_CharPrope));
			srvBeingDiscovered->characteristics[i].characteristic.charDeclhandle = charDiscRspEvt->chars[j].handle_decl;
			srvBeingDiscovered->characteristics[i].characteristic.charValhandle  = charDiscRspEvt->chars[j].handle_value;
			srvBeingDiscovered->characteristics[i].characteristic.charValUuid.type = (BLE_GattsUuidType)charDiscRspEvt->chars[j].uuid.type; //BLE_ENABLE_NORDIC_ORIGINAL
			srvBeingDiscovered->characteristics[i].characteristic.charValUuid.value.baseAlias.uuidAlias = charDiscRspEvt->chars[j].uuid.uuid; //BLE_ENABLE_NORDIC_ORIGINAL
			srvBeingDiscovered->characteristics[i].cccdHandle = BLE_GATT_HANDLE_INVALID;

			BLE_PRT2("onChar type %d uuid 0x%04X ValHdl %d DecHdl %d\n",
				charDiscRspEvt->chars[j].uuid.type,
				charDiscRspEvt->chars[j].uuid.uuid,
				charDiscRspEvt->chars[j].handle_value,
				charDiscRspEvt->chars[j].handle_decl);
		}

		lastKnownChar = &(srvBeingDiscovered->characteristics[i - 1].characteristic);

		// If no more characteristic discovery is required, or if the maximum number of supported
		// characteristic per service has been reached, descriptor discovery will be performed.
		if (!isCharDiscoveryReqd(gattcDbDiscovery, lastKnownChar) ||
		(srvBeingDiscovered->charCount == BLE_DB_DISCOVERY_MAX_CHAR_PER_SRV)) {
			performDescDiscov = true;
		}
		else {
			// Update the current characteristic index.
			gattcDbDiscovery->currCharInd = srvBeingDiscovered->charCount;
			// Perform another round of characteristic discovery.
			BLE_PRT2("onChar charDiscover\n");
			(void)characteristicsDiscover(pBleEvent, gattcDbDiscovery);
		}
	}
	else {
		// The previous characteristic discovery resulted in no characteristics.
		// descriptor discovery should be performed.
		performDescDiscov = true;
	}

	if (performDescDiscov) {
		gattcDbDiscovery->currCharInd = 0;
		BLE_PRT2("onChar charDiscover\n");
		(void)descriptorsDiscover(pBleEvent, gattcDbDiscovery, &raiseDiscovComplete);
		if (raiseDiscovComplete) {
			BLE_PRT2("onChar onSrvDiscCompletion\n");
			onSrvDiscCompletion(pBleEvent, gattcDbDiscovery);
		}
	}
}

static
void onDescriptorDiscoveryRsp(bleGattcDb *const gattcDbDiscovery, BLE_Evt *pBleEvent, ble_evt_t *pBleNrfEvt)
{
	uint32_t i = 0;
	uint16_t connHandle = 0;
	bool raiseDiscovComplete = false;
	BLE_GattcDbDiscSrv *srvBeingDiscovered = NULL;
	ble_gattc_evt_t *bleGattcEvt = NULL;
	bleGattcEvt = &(pBleNrfEvt->evt.gattc_evt);
	BLE_PRT2("onDesc srvInd %d charInd %d\n",
		gattcDbDiscovery->currSrvInd, gattcDbDiscovery->currCharInd);
	srvBeingDiscovered = &(gattcDbDiscovery->dbDiscovery.services[gattcDbDiscovery->currSrvInd]);
	const ble_gattc_evt_desc_disc_rsp_t * descDiscRspEvt = &(bleGattcEvt->params.desc_disc_rsp);
	BLE_GattcDbDiscChar * charBeingDiscovered = &(srvBeingDiscovered->characteristics[gattcDbDiscovery->currCharInd]);

	connHandle  = bleGattcEvt->conn_handle;
	if ((gattcDbDiscovery->currSrvInd >= BLE_DB_DISCOVERY_MAX_SRV) ||
		(gattcDbDiscovery->currCharInd >= BLE_DB_DISCOVERY_MAX_CHAR_PER_SRV)) {
		BLE_ERR("onDesc ind NG\n");
		gattcDbDiscovery->discoveryInProgress = false;
		setDbDiscoveryEvent(pBleEvent, connHandle, BLE_GATTC_RESULT_FAILED, BLE_GATTC_REASON_DESCRIPTOR);
		return;
	}
	if (bleGattcEvt->gatt_status == BLE_GATT_STATUS_SUCCESS) {
		// The descriptor was found at the peer.
		// If the descriptor was a Client Characteristic Configuration Descriptor, then the cccdHandle needs to be populated.
		// Loop through all the descriptors to find the Client Characteristic Configuration Descriptor.
		for (i = 0; i < descDiscRspEvt->count; i++) {
			if (descDiscRspEvt->descs[i].uuid.uuid == BLE_UUID_DESCRIPTOR_CLIENT_CHAR_CONFIG) {
				charBeingDiscovered->cccdHandle = descDiscRspEvt->descs[i].handle;
				BLE_PRT2("onDesc ccc %d\n", descDiscRspEvt->descs[i].handle);
				break;
			}
		}
	}

	if ((gattcDbDiscovery->currCharInd + 1) == srvBeingDiscovered->charCount) {
		// No more characteristics and descriptors need to be discovered. Discovery is complete.
		raiseDiscovComplete = true;
	}
	else {
		BLE_PRT2("onDesc retry descriptorsDiscover\n");
		// Begin discovery of descriptors for the next characteristic.
		gattcDbDiscovery->currCharInd++;
		(void)descriptorsDiscover(pBleEvent, gattcDbDiscovery, &raiseDiscovComplete);
	}
	if (raiseDiscovComplete) {
		BLE_PRT2("onDesc onSrvDiscCompletion\n");
		onSrvDiscCompletion(pBleEvent, gattcDbDiscovery);
	}
}

static
void onSrvDiscCompletion(BLE_Evt *pBleEvent, bleGattcDb *gattcDbDiscovery)
{
	int ret = BLE_SUCCESS;
	uint16_t nextHandleStart = 0;
	BLE_GattcDbDiscSrv       *srvBeingDiscovered = NULL;
	BLE_GattcDbDiscSrv       *srvPrevDiscovered = NULL;

	// Reset the current characteristic index since a new service discovery is about to start.
	gattcDbDiscovery->currCharInd = 0;
	srvPrevDiscovered = &(gattcDbDiscovery->dbDiscovery.services[gattcDbDiscovery->currSrvInd]);

	gattcDbDiscovery->currSrvInd++;
	if (gattcDbDiscovery->currSrvInd > BLE_DB_DISCOVERY_MAX_SRV) {
		BLE_ERR("onCompletion ind NG\n");
		goto err;
	} else {
		// Initiate discovery of the next service.
		if (gattcDbDiscovery->currSrvInd < BLE_DB_DISCOVERY_MAX_SRV) {
			srvBeingDiscovered = &(gattcDbDiscovery->dbDiscovery.services[gattcDbDiscovery->currSrvInd]);
			// Reset the characteristic count in the current service to zero since a new service
			// discovery is about to start.
			srvBeingDiscovered->charCount = 0;
		}
	}
	if (srvPrevDiscovered->srvHandleRange.endHandle != BLE_GATTC_HANDLE_END) {
		nextHandleStart = srvPrevDiscovered->srvHandleRange.endHandle + 1;
	}
	else {
		nextHandleStart = BLE_GATTC_HANDLE_END;
	}
	BLE_PRT2("onCompletion nextHandle %d\n", nextHandleStart);
	ret = sd_ble_gattc_primary_services_discover(gattcDbDiscovery->dbDiscovery.connHandle, nextHandleStart, NULL);
	if (ret != NRF_SUCCESS) {
		BLE_ERR("sd_ble_gattc_primary_sd NG %d\n", ret);
		goto err;
	}
	return;
err:
	gattcDbDiscovery->discoveryInProgress = false;
	setDbDiscoveryEvent(pBleEvent, gattcDbDiscovery->dbDiscovery.connHandle,
		BLE_GATTC_RESULT_FAILED, BLE_GATTC_REASON_SERVICE);
	return;
}

static
int characteristicsDiscover(BLE_Evt *pBleEvent, bleGattcDb *const gattcDbDiscovery)
{
	int ret = 0;
	uint8_t              prevCharInd = 0;
	BLE_GattcChar        *prevChar = NULL;
	BLE_GattcDbDiscSrv   *srvBeingDiscovered = NULL;
	BLE_GattcHandleRange handleRange = {0};
	BLE_PRT2("charDiscover start srvInd %d charInd %d\n",
		gattcDbDiscovery->currSrvInd, gattcDbDiscovery->currCharInd);
	if (gattcDbDiscovery->currSrvInd >= BLE_DB_DISCOVERY_MAX_SRV) {
		BLE_ERR("charDiscover NG %d\n", NRF_ERROR_FORBIDDEN);
		goto err;
	}
	srvBeingDiscovered = &(gattcDbDiscovery->dbDiscovery.services[gattcDbDiscovery->currSrvInd]);
	if (gattcDbDiscovery->currCharInd != 0) {
		// This is not the first characteristic being discovered. Hence the 'start handle' to be
		// used must be computed using the charValhandle of the previous characteristic.

		prevCharInd = gattcDbDiscovery->currCharInd - 1;

		srvBeingDiscovered = &(gattcDbDiscovery->dbDiscovery.services[gattcDbDiscovery->currSrvInd]);
		prevChar = &(srvBeingDiscovered->characteristics[prevCharInd].characteristic);
		handleRange.startHandle = prevChar->charValhandle + 1;
	}
	else {
		// This is the first characteristic of this service being discovered.
		handleRange.startHandle = srvBeingDiscovered->srvHandleRange.startHandle;
	}

	handleRange.endHandle = srvBeingDiscovered->srvHandleRange.endHandle;
	BLE_PRT2("charDiscover sHdl %d eHdl %d\n",
		handleRange.startHandle, handleRange.endHandle);
	ret = sd_ble_gattc_characteristics_discover(gattcDbDiscovery->dbDiscovery.connHandle,
		(ble_gattc_handle_range_t *)&handleRange);
	if (ret) {
		BLE_ERR("sd_ble_gattc_cdr NG %d\n", ret);
		goto err;
	}
	return 0;
err:
	gattcDbDiscovery->discoveryInProgress = false;
	setDbDiscoveryEvent(pBleEvent, gattcDbDiscovery->dbDiscovery.connHandle,
		BLE_GATTC_RESULT_FAILED, BLE_GATTC_REASON_CHARACTERISTIC);
	return -1;
}

static
bool isCharDiscoveryReqd(bleGattcDb *const gattcDbDiscovery,
						   BLE_GattcChar         *afterChar)
{
	BLE_PRT2("isChar charEndHdl %d srvEndHdl %d\n", afterChar->charValhandle,
		gattcDbDiscovery->dbDiscovery.services[gattcDbDiscovery->currSrvInd].srvHandleRange.endHandle);
	if (gattcDbDiscovery->currSrvInd >= BLE_DB_DISCOVERY_MAX_SRV) {
		return false;
	}
	if (afterChar->charValhandle <
	gattcDbDiscovery->dbDiscovery.services[gattcDbDiscovery->currSrvInd].srvHandleRange.endHandle) {
		// Handle value of the characteristic being discovered is less than the end handle of
		// the service being discovered. There is a possibility of more characteristics being
		// present. Hence a characteristic discovery is required.
		return true;
	}
	return false;
}

static
bool isDescDiscoveryReqd(bleGattcDb       *gattcDbDiscovery,
								BLE_GattcDbDiscChar  *currChar,
								BLE_GattcDbDiscChar  *nextChar,
								BLE_GattcHandleRange *handleRange)
{
	BLE_PRT2("isDesc charValHdl %d srvEndHdl %d\n",
		currChar->characteristic.charValhandle,
		gattcDbDiscovery->dbDiscovery.services[gattcDbDiscovery->currSrvInd].srvHandleRange.endHandle);

	if (gattcDbDiscovery->currSrvInd >= BLE_DB_DISCOVERY_MAX_SRV) {
		return false;
	}
	if (nextChar == NULL) {
		// Current characteristic is the last characteristic in the service. Check if the value
		// handle of the current characteristic is equal to the service end handle.
		if (currChar->characteristic.charValhandle ==
			gattcDbDiscovery->dbDiscovery.services[gattcDbDiscovery->currSrvInd].srvHandleRange.endHandle
		) {
			// No descriptors can be present for the current characteristic. currChar is the last
			// characteristic with no descriptors.
			return false;
		}

		handleRange->startHandle = currChar->characteristic.charValhandle + 1;

		// Since the current characteristic is the last characteristic in the service, the end
		// handle should be the end handle of the service.
		handleRange->endHandle = gattcDbDiscovery->dbDiscovery.services[gattcDbDiscovery->currSrvInd].srvHandleRange.endHandle;
		BLE_PRT2("isDesc start %d end %d\n", handleRange->startHandle, handleRange->endHandle);
		return true;
	}

	// nextChar != NULL. Check for existence of descriptors between the current and the next
	// characteristic.
	if ((currChar->characteristic.charValhandle + 1) == nextChar->characteristic.charDeclhandle) {
		// No descriptors can exist between the two characteristic.
		return false;
	}

	handleRange->startHandle = currChar->characteristic.charValhandle + 1;
	handleRange->endHandle   = nextChar->characteristic.charDeclhandle - 1;
	BLE_PRT2("isDesc start %d end %d\n", handleRange->startHandle, handleRange->endHandle);
	return true;
}

static
int descriptorsDiscover(BLE_Evt *pBleEvent, bleGattcDb *const gattcDbDiscovery, bool *raiseDiscovComplete)
{
	uint8_t                    i = 0;
	int                        ret = BLE_SUCCESS;
	BLE_GattcHandleRange       handleRange = {0};
	BLE_GattcDbDiscChar        *currCharBeingDiscovered = NULL;
	BLE_GattcDbDiscSrv         *srvBeingDiscovered = NULL;
	BLE_GattcDbDiscChar        *nextChar = NULL;
	bool                       isDiscoveryReqd = false;

	*raiseDiscovComplete = false;
	BLE_PRT2("descDiscover SrvInd %d CharInd %d\n",
		gattcDbDiscovery->currSrvInd, gattcDbDiscovery->currCharInd);
	if ((gattcDbDiscovery->currSrvInd >= BLE_DB_DISCOVERY_MAX_SRV) || (gattcDbDiscovery->currCharInd >= BLE_DB_DISCOVERY_MAX_CHAR_PER_SRV)) {
		BLE_ERR("descDiscover NG %d\n", NRF_ERROR_FORBIDDEN);
		goto err;
	}

	srvBeingDiscovered = &(gattcDbDiscovery->dbDiscovery.services[gattcDbDiscovery->currSrvInd]);
	currCharBeingDiscovered = &(srvBeingDiscovered->characteristics[gattcDbDiscovery->currCharInd]);

	if ((gattcDbDiscovery->currCharInd + 1) == srvBeingDiscovered->charCount) {
		// This is the last characteristic of this service.
		isDiscoveryReqd = isDescDiscoveryReqd(gattcDbDiscovery, currCharBeingDiscovered, NULL, &handleRange);
	}
	else {
		for (i = gattcDbDiscovery->currCharInd; i < srvBeingDiscovered->charCount; i++) {
			if (i == (srvBeingDiscovered->charCount - 1)) {
				// The current characteristic is the last characteristic in the service.
				nextChar = NULL;
			}
			else {
				nextChar = &(srvBeingDiscovered->characteristics[i + 1]);
			}

			// Check if it is possible for the current characteristic to have a descriptor.
			if (isDescDiscoveryReqd(gattcDbDiscovery, currCharBeingDiscovered, nextChar, &handleRange)) {
				isDiscoveryReqd = true;
				break;
			}
			else {
				// No descriptors can exist.
				currCharBeingDiscovered = nextChar;
				gattcDbDiscovery->currCharInd++;
			}
		}
	}

	if (!isDiscoveryReqd) {
		// No more descriptor discovery required. Discovery is complete.
		*raiseDiscovComplete = true;
		return NRF_SUCCESS;
	}

	BLE_PRT2("descDiscover sHdl %d eHdl %d\n", handleRange.startHandle, handleRange.endHandle);
	ret = sd_ble_gattc_descriptors_discover(gattcDbDiscovery->dbDiscovery.connHandle, (ble_gattc_handle_range_t *)&handleRange);
	if (ret) {
		BLE_ERR("sd_ble_gattc_dd NG %d\n", ret);
		goto err;
	}
	return 0;
err:
	gattcDbDiscovery->discoveryInProgress = false;
	setDbDiscoveryEvent(pBleEvent, gattcDbDiscovery->dbDiscovery.connHandle,
		BLE_GATTC_RESULT_FAILED, BLE_GATTC_REASON_DESCRIPTOR);
	return -1;
}
