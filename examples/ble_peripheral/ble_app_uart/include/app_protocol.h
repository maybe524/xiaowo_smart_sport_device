#pragma once

struct app_gen_command {
    uint8_t     id;
    uint16_t    len;
    uint8_t     buff[20];
};

struct app_d2h_accelerator_data {
    uint16_t    x, y, z;
};

struct app_h2d_set_accel_task {
    uint8_t     is_need_upload;
};

#define CMD_H2D_ID_SET_BIND         0x01
#define CMD_H2D_ID_SET_ACCEL_TASK   0x02


#define CMD_D2H_ID_ACCEL_DATA       0x11