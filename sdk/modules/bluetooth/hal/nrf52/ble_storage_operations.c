/*
 * COPYRIGHT (C) 2016 Sony Corporation.
 *
 * ble_storage_operations.c
 */

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <errno.h>

#include "queue.h" /* TODO: replace to nuttx/include/queue.h */
#include "ble_storage_operations.h"

/*
#define BSO_FILE_PATH      "/var/hostif"
#define BSO_FILE_PATH_NAME BSO_FILE_PATH"/%s"
#define BSO_REGISTRY_DB    "bso_registry_db"
*/
#define REGDB_VALUE_MAX    60
#define REGDB_KEY_NAME_MAX 4
/*
#define BSO_FILE_NAME_MAX  64
#define BSO_FILE_PATH_MAX  128
*/
#define BITS_PER_BYTE      8

#define MIN(a, b) ((a) < (b) ? (a) : (b))

#define FLAG_INITIALIZED (1ul << 0)

#define BSO_REG_KEY_READONLY (1u << 15)
#define BSO_REG_KEY_ATTR_MASK (BSO_REG_KEY_READONLY)

typedef struct BSO_REGDB_RECORD_Tag {
    uint32_t key;
    uint8_t value[REGDB_VALUE_MAX];
} BSO_REGDB_RECORD;

typedef struct BSO_REGDB_ENTRY {
    TAILQ_ENTRY(BSO_REGDB_ENTRY) entry;
    BSO_REGDB_RECORD record;
} BSO_REGDB_ENTRY;

typedef struct BSO_CONTEXT_Tag {
    uint32_t flags;
    TAILQ_HEAD(, BSO_REGDB_ENTRY) regDbHead;
} BSO_CONTEXT;

static BSO_CONTEXT gBsoContext;

static int initRegistryDatabase(BSO_CONTEXT* ctx)
{
    TAILQ_INIT(&ctx->regDbHead);

    return 0;
}

int BSO_Init(void* data)
{
    (void)data;

    int ret          = 0;
    BSO_CONTEXT* ctx = &gBsoContext;

    if (ctx->flags & FLAG_INITIALIZED) {
        return 0;
    }

    ret = initRegistryDatabase(ctx);

    if (!ret) {
        ctx->flags |= FLAG_INITIALIZED;
    }

    return ret;
}

void BSO_Sync(void)
{
    /* TODO: impl after persistent storage is ready */
}

int BSO_Finalize(void* data)
{
    (void)data;

    BSO_CONTEXT* ctx       = &gBsoContext;
    int ret                = 0;

    if (!(ctx->flags & FLAG_INITIALIZED)) {
        return 0;
    }

    ret = BSO_CleanRegistry();

    ctx->flags = 0;

    return ret;
}

int BSO_GetRegistryValue(uint32_t key, void* value, uint32_t size)
{
    BSO_CONTEXT* ctx       = &gBsoContext;
    BSO_REGDB_ENTRY* entry = NULL;

    if (!(ctx->flags & FLAG_INITIALIZED)) {
        return -ENXIO;
    }

    if (size > REGDB_VALUE_MAX) {
        return -EINVAL;
    }

    TAILQ_FOREACH(entry, &ctx->regDbHead, entry) {
        if (key == entry->record.key) {
            memcpy(value, entry->record.value, MIN(size, REGDB_VALUE_MAX));
            return 0;
        }
    }

    return -ENOENT;
}

int BSO_SetRegistryValueList(BSO_KeyPair* list, uint32_t pairsNum)
{
    BSO_CONTEXT* ctx       = &gBsoContext;
    BSO_REGDB_ENTRY* entry = NULL;
    uint32_t i             = 0;

    if (!(ctx->flags & FLAG_INITIALIZED)) {
        return -ENXIO;
    }

    if (!list) {
        return -EINVAL;
    }

    for (i = 0; i < pairsNum; ++i, ++list) {
        if (list->size > REGDB_VALUE_MAX) {
            return -EINVAL;
        }

        TAILQ_FOREACH(entry, &ctx->regDbHead, entry) {
            if (list->key == entry->record.key) {
                if (list->key & BSO_REG_KEY_READONLY) {
                    return -EACCES;
                }

                memcpy(entry->record.value, list->value, MIN(list->size, REGDB_VALUE_MAX));

                break;
            }
        }

        if (!entry) {
            entry = (BSO_REGDB_ENTRY*)malloc(sizeof(BSO_REGDB_ENTRY));

            if (!entry) {
                return -ENOSPC;
            }

            memset(entry, 0, sizeof(BSO_REGDB_ENTRY));
            entry->record.key = list->key;
            memcpy(entry->record.value, list->value, MIN(list->size, REGDB_VALUE_MAX));
            TAILQ_INSERT_TAIL(&ctx->regDbHead, entry, entry);
        }
    }

    return 0;
}

int BSO_SetRegistryValue(uint32_t key, const void* value, uint32_t size)
{
    BSO_KeyPair kp = {key, value, size};
    return BSO_SetRegistryValueList(&kp, 1);
}

int BSO_DeleteRegistryKey(uint32_t key)
{
    BSO_CONTEXT* ctx       = &gBsoContext;
    BSO_REGDB_ENTRY* entry = NULL;

    if (!(ctx->flags & FLAG_INITIALIZED)) {
        return -ENXIO;
    }

    TAILQ_FOREACH(entry, &ctx->regDbHead, entry) {
        if (key == entry->record.key) {
            break;
        }
    }

    if (entry) {
        TAILQ_REMOVE(&ctx->regDbHead, entry, entry);
        free(entry);
        entry = NULL;
    }
    else {
        return -ENOENT;
    }

    return 0;
}

int BSO_CleanRegistry(void)
{
    BSO_CONTEXT* ctx       = &gBsoContext;
    BSO_REGDB_ENTRY* entry = NULL;

    if (!(ctx->flags & FLAG_INITIALIZED)) {
        return -ENXIO;
    }

    while (!TAILQ_EMPTY(&ctx->regDbHead)) {
        entry = TAILQ_FIRST(&ctx->regDbHead);
        TAILQ_REMOVE(&ctx->regDbHead, entry, entry);

        free(entry);
        entry = NULL;
    }

    return 0;
}

uint32_t BSO_GenerateRegistryKey(const char* keyName, uint32_t attr)
{
    uint32_t key = 0;
    uint32_t len = strnlen(keyName, REGDB_KEY_NAME_MAX);
    uint32_t i   = 0;

    for (i = 0; i < len; ++i) {
        key |= ((uint32_t)keyName[i] << (i * BITS_PER_BYTE));
    }

    key |= (attr & BSO_REG_KEY_ATTR_MASK);

    return key;
}
