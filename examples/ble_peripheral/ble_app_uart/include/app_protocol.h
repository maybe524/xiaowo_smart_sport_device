#pragma once

#pragma pack(1)
struct app_gen_command {
    uint8_t     id;
    uint16_t    comm_id;
    uint8_t     flags;
    uint16_t    len;
    uint8_t     buff[20];
};

struct app_d2h_accelerator_data {
    uint16_t    opt_task;
    uint8_t     opt_id;
    uint16_t    x, y, z;
};

struct app_d2h_red_ir_data {
    uint16_t    opt_task;
    uint8_t     opt_id;
    uint32_t    red, ir;
};

struct app_h2d_set_accel_task {
    uint8_t     is_need_upload;
};

struct app_h2d_set_task_infos {
    uint16_t    task_mask;
    uint8_t     task_need_open;
    uint8_t     task_param;
};

struct app_d2h_power_percent {
    uint8_t     percent;
};

#pragma pack()

#define CMD_H2D_ID  0x01
#define CMD_D2H_ID  0x02

#define CMD_H2D_ID_SET_BIND         0x01
#define CMD_H2D_ID_SET_TASK         0x03
#define CMD_H2D_ID_GET_BAT_PERCENT  0x04


#define CMD_D2H_ID_ACCEL_DATA       0x11
#define CMD_D2H_ID_GET_BAT_PERCENT          CMD_H2D_ID_GET_BAT_PERCENT
#define CMD_D2H_ID_GET_TASK         CMD_H2D_ID_SET_TASK

#define CHANG_TO_BIGENDING(v)       ((v & 0xFF) << 8) | ((v >> 8) & 0xFF)
#define CHANG_TO_BIGENDING_32(v)    (((v & 0xFF) << 24) | (((v >> 8) & 0xFF) << 16) | \
                                    (((v >> 16) & 0xFF) << 8) | (v >> 24) & 0xFF)
