#pragma once

#include "common.h"

#define STORAGE_ID_BIND_NUM     0x01

#pragma pack(1)
struct app_storage_fmt {
    uint8_t bind_num[4];
};
#pragma pack()

int storage_get(uint32_t id, void *data, uint32_t len);

