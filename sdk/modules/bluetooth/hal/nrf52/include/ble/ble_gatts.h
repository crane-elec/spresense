/****************************************************************************
 * apps/include/ble/ble_gatts.h
 *
 *   Copyright (C) 2016 Sony Corporation. All rights reserved.
 *   Author: Wen, Yuchi <Yuchi.Wen@sony.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor Sony nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
/**
 * @file       ble_gatts.h
 */

#ifndef BLE_GATTS_H
#define BLE_GATTS_H

/**
 * @defgroup BLE Bluetooth LE GATT server
 *
 * #include <ble/ble_gatts.h>
 *
 * @{
 */

/*-----------------------------------------------------------------------------
 * include files
 *---------------------------------------------------------------------------*/

#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif  /* __cplusplus */

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/**
 * @defgroup ble_defs Defines
 * @{
 */

#define UUID_128BIT_LENGTH  16 /**< 16 bytes of 128 bit UUID length */
#if defined(CONFIG_BCM20707)
#define MAX_RCV_DATA_LENGTH 512 /**< Max receive data length. */
#elif defined(CONFIG_BLUETOOTH_NRF52)
#define MAX_RCV_DATA_LENGTH 247 /**< Max receive data length. */
#else
#define MAX_RCV_DATA_LENGTH 20  /**< Max receive data length. */
#endif

/** @} ble_defs */

/****************************************************************************
 * Public Types
 ****************************************************************************/
/**
 * @defgroup ble_datatypes Data types
 * @{
 */

/**@brief GATTS service type*/
typedef enum
{
	BLE_GATTS_SRVTYP_PRIMARY = 1, /**< GATTS service type primary */
	BLE_GATTS_SRVTYP_SECONDARY,   /**< GATTS service type secondary */
}BLE_GattsSrvType;

/**@brief GATTS UUID type
 */
typedef enum
{
	BLE_UUID_TYPE_UUID128 = 0,         /**< UUID type 128-bit UUID */
	BLE_UUID_TYPE_BASEALIAS_BTSIG,     /**< UUID type base alias bluetooth SIG qualified */
	BLE_UUID_TYPE_BASEALIAS_VENDOR,    /**< UUID type base alias vendor */
}BLE_GattsUuidType;

/**@brief Gatts write event structure
 * @details When the event coming from the stack to the application, the @ref BLE_Evt structure member evtHeader is set to @ref BLE_GATTS_EVENT_WRITE, and the member evtData is point to @ref BLE_EvtGattsWrite structure.
 */
typedef struct
{
	uint16_t   connHandle;                     /**< Connection handle */
	uint16_t   handle;                         /**< Attribute handle */
	uint16_t   offset;                         /**< Offset for the write operation */
	uint16_t   dataLen;                        /**< Length of the received data */
	uint8_t    reserve[2];                     /**< Reserve */
	uint8_t    data[MAX_RCV_DATA_LENGTH];      /**< Received data */
}BLE_EvtGattsWrite;

/**@brief Indication confirm event
 * @details When the event coming from the stack to the application, the @ref BLE_Evt structure member evtHeader is set to @ref BLE_GATTS_EVENT_CFM, and the member evtData is point to @ref BLE_EvtGattsIndConfirm structure.
 */
typedef struct
{
	uint16_t    connHandle; /**< Connection handle */
	uint16_t    handle;     /**< Attribute handle */
}BLE_EvtGattsIndConfirm;

/**@brief Exchange MTU event
 * @details When the event coming from the stack to the application, the @ref BLE_Evt structure member evtHeader is set to @ref BLE_GATTS_EVENT_EXCHANGE_MTU, and the member evtData is point to @ref BLE_EvtGattsExchangeMTU structure.
 */
typedef struct
{
	uint16_t    connHandle; /**< Connection handle */
	uint16_t    client_rx_mtu;
}BLE_EvtGattsExchangeMTU;

/**@brief 128-bit UUID structure
 */
typedef struct
{
	uint8_t uuid128[UUID_128BIT_LENGTH]; /**< 128-bit UUID */
}BLE_Uuid128;

/**@brief UUID base alias structure
 */
typedef struct
{
	BLE_Uuid128 uuidBase;  /**< 128-bit UUID */
	uint16_t    uuidAlias; /**< UUID alias */
}BLE_uuidBaseAlias;

/**@brief BLE UUID structure
 */
typedef struct
{
	BLE_GattsUuidType type;      /**< UUID type */
	union
	{
		BLE_Uuid128   uuid128;   /**< 128-bit UUID */
		BLE_uuidBaseAlias baseAlias; /**< UUID base alias */
	}value;   /**< value union */
}BLE_Uuid;

/**@brief Characteristic standard properties
 * @details According to ble core spec 4.1 vol3;Part G 3.3.1.1, Current version support read, write, notify properties.
 */
typedef struct
{
	uint8_t broadcast       :1; /**< Broadcasting of the value permitted. */
	uint8_t read            :1; /**< Reading the value permitted. */
	uint8_t writeWoResp   :1; /**< Writing the value with Write Command permitted. */
	uint8_t write           :1; /**< Writing the value with Write Request permitted. */
	uint8_t notify          :1; /**< Notification of the value permitted. */
	uint8_t indicate        :1; /**< Indication of the value permitted. */
	uint8_t authSignedWr  :1; /**< Writing the value with Signed Write Command permitted. */
	uint8_t reserve         :1; /**< Reserve */
}BLE_CharPrope;

/**@brief BLE security mode security level.
 * @details According to ble core spec v4.1 Vol 3;Part C;10.2 LE security modes and Vol 3;Part F;3.2.5 attribute permissions.
 */
typedef enum{
	BLE_SEC_MODE_NO_ACCESS  = 0x00,    /**< No access rigths */
	BLE_SEC_MODE1LV1_NO_SEC,           /**< Security mode 1 level 1, no security open link */
	BLE_SEC_MODE1LV2_NO_MITM_ENC,      /**< Security mode 1 level 2, unauthenticated pairing with encryption */
	BLE_SEC_MODE1LV3_MITM_ENC,         /**< Security mode 1 level 3, authenticated pairing with encryption */
	BLE_SEC_MODE2LV1_NO_MITM_DATA_SGN, /**< Security mode 2 level 1, unauthenticated pairing with data signing, not supported now */
	BLE_SEC_MODE2LV2__MITM_DATA_SGN,   /**< Security mode 2 level 2, authenticated pairing with data signing, not supported now */
}BLE_SecMode;

/**@brief Attribute permission structure
 */
typedef struct
{
	BLE_SecMode  readPerm; /**< Read permissions. */
	BLE_SecMode writePerm; /**< Write permissions. */
}BLE_AttrPerm;

/**@brief Characteristic meta datastructure
 */
typedef struct
{
	BLE_CharPrope                 charPrope; /**< Characteristic Properties */
	uint8_t                         reserve; /**< Reserve */
	uint16_t               charUserDescSize; /**< The size of the user description */
	uint8_t                   *charUserDesc; /**< Pointer to a UTF-8 encoded string */
	BLE_AttrPerm                userDescDpr; /**< User Description descriptor, not supported now */
	BLE_AttrPerm           clientCharCfgDpr; /**< Client Characteristic Configuration Descriptor */
	BLE_AttrPerm           serverCharCfgDpr; /**< Server Characteristic Configuration Descriptor, not supported now */
}BLE_CharMeta;

/**@brief the GATTS descriptor structure
 */
typedef struct
{
	uint16_t                userDescHandle; /**< Handle to the User Description descriptor */
	uint16_t                    cccdHandle; /**< Handle to the Client Characteristic Configuration Descriptor */
	uint16_t                    sccdHandle; /**< Handle to the Server Characteristic Configuration Descriptor */
}BLE_GattsDprHandles;

/**@brief GATT Characteristic Definition Handles
 */
typedef struct
{
	uint16_t                  charHandle; /**< Handle to the characteristic value */
	BLE_GattsDprHandles       dprHandle; /**< Handle to the descriptor structure belongs to characteristic */
}BLE_GattsCharHandles;

/**@brief characteristic value attribute structure
 */
typedef struct BLE_GattsAttr_t
{
	BLE_Uuid                 valueUuid; /**< Attribute value UUID */
	BLE_AttrPerm              attrPerm; /**< Attribute permission structure */
	uint8_t                 *attrValue; /**< Pointer to attribute value */
	uint16_t                  valueLen; /**< Attribute value length */
}BLE_GattsAttr;

/**@brief Notification or Indication Type
 * @sa @ref BLE_EvtGattcNtfInd
 */
typedef enum
{
	BLE_GATT_NOTIFICATION = 0x01, /**< GATT type notification */
	BLE_GATT_INDICATION,          /**< GATT type indication */
}BLE_GattNtyIndType;

/**@brief GATTS handle value notification or indication parameters structure
 */
typedef struct
{
	BLE_GattNtyIndType      type;         /**< Indication or notification */
	uint8_t                 reserve;      /**< Reserve */
	uint8_t                 *attrValData; /**< Attribute value data content */
	uint16_t                attrValLen;   /**< Attribute value length in bytes */
	uint16_t                attrHandle;   /**< Attribute handle */
}BLE_GattsHandleValueNfyIndParams;


/**@brief GATTS Attribute value parameters structure
 */
typedef struct
{
	uint16_t  connHandle;   /**< Connection handle */
	uint16_t  attrHandle;   /**< Attribute handle */
	uint16_t  attrValLen;   /**< Attribute value length */
	uint16_t  reserve;      /**< Reserve */
	uint8_t   *attrValData; /**< Attribute value data */
}BLE_GattsAttrValueParam;

/** @} ble_datatypes */

/**
 * @defgroup ble_funcs Functions
 * @{
 */

/**@brief GATT server add service
 * @details Add a service declaration to the attribute table. There is not a maximum number of services. The number is varied depending on number of attributes in each service and its size. So user could add services until space runs out and no more can be added.
 *
 * @param[in]  type: Primary or secondary service
 * @param[in]  uuid: Pointer to service UUID
 * @param[out]  serviceHandle: Pointer to a 16-bit word where the assigned handle will be stored
 * @return     0: success
 *
 * @par Blocking
 *     Yes
 * @par Context
 *     Task
 * @par Reentrant
 *     No
 *
 */
int BLE_GattsAddService(BLE_GattsSrvType type, BLE_Uuid const* uuid, uint16_t* serviceHandle);

/**@brief GATT server add characteristic
 * @details Add a characteristic declaration. This call allows the application to add a characteristic declaration, a characteristic value declaration and option characteristic descriptor declarations to the attribute table. There is not a maximum number of characteristic, it depends on the available resources.
 *
 * @param[in]  serviceHandle: Service handle that the characteristic to be added belongs to
 * @param[in]  charMeta: Characteristic meta data
 * @param[in]  charValue: Characteristic value structure
 * @param[out]  handles: Handles of added characteristic
 * @return     0: success
 *
 * @par Blocking
 *     Yes
 * @par Context
 *     Task
 * @par Reentrant
 *     No
 *
 */
int BLE_GattsAddCharacteristic(uint16_t serviceHandle, BLE_CharMeta const* charMeta, BLE_GattsAttr const* charValue, BLE_GattsCharHandles* handles);

/**@brief GATT server handle value notification/indication
 * @details Handle an attribute value notification or indication.
 *
 * @param[in]  connHandle: Connection handle
 * @param[in]  handleValueNfyInd: Characteristic meta data pointer to an handle value notify or indicate parameters structure
 *
 * @par Blocking
 *     Yes
 * @par Context
 *     Task
 * @par Reentrant
 *     No
 *
 */
int BLE_GattsHandleValueNfyInd(uint16_t connHandle, BLE_GattsHandleValueNfyIndParams const* handleValueNfyInd);

/**@brief GATT server update attribute value
 * @details This call allows the application to update an attribute value. for example, if you want to update your characteristic value(an attribute), you can use this call with attribute handle set to characteristic value handle.
 *
 * @param[in]  attrValueParam: Attribute value, must be existing connHandle and attrHandle.
 * @return     0: success
 *
 * @par Blocking
 *     Yes
 * @par Context
 *     Task
 * @par Reentrant
 *     No
 *
 * @note Wait a bit and retry this if the return value is -EBUSY.
 */
int BLE_GattsUpdateAttrValue(BLE_GattsAttrValueParam *attrValueParam);

/** @} ble_funcs */

#ifdef __cplusplus
}
#endif  /* __cplusplus */
/** @} ble_gatts */
#endif  /* BLE_GATTS_H */
