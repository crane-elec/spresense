/*
 * COPYRIGHT (C) 2016 Sony Corporation.
 *
 * ble_storage_operations.h
 */

#ifndef __BLE_STORAGE_OPERATIONS_H__
#define __BLE_STORAGE_OPERATIONS_H__

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

#include <stdint.h>

typedef struct BSO_KeyPair_Tag{
    uint32_t key;
    const void* value;
    uint32_t size;
} BSO_KeyPair;

// APIs should be called in a single thread, not thread safe
int BSO_Init(void* data);
void BSO_Sync(void);
int BSO_Finalize(void* data);
int BSO_GetRegistryValue(uint32_t key, void* value, uint32_t size);
int BSO_SetRegistryValueList(BSO_KeyPair* list, uint32_t pairsNum);
int BSO_SetRegistryValue(uint32_t key, const void* value, uint32_t size);
int BSO_DeleteRegistryKey(uint32_t key);
int BSO_CleanRegistry(void);
uint32_t BSO_GenerateRegistryKey(const char* keyName, uint32_t attr);

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // __BLE_STORAGE_OPERATIONS_H__
