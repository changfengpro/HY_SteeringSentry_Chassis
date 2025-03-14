/*
 * @Description: 
 * @Author: changfeng
 * @brief: 
 * @version: 
 * @Date: 2025-02-01 20:35:00
 * @LastEditors:  
 * @LastEditTime: 2025-03-13 03:54:09
 */
#ifndef CMD_VEL_H
#define CMD_VEL_H

#include "stdint.h"
#include "bsp_usart.h"
#include "daemon.h"
#include "bsp_log.h"
#include "usart.h"
#include "rm_referee.h"

#pragma pack(1)
typedef struct 
{
    float x;
    float y;
    float z;
} Vector3;  //通用三维向量结构体

typedef enum
{
    CHASSIS_ROTATE_S = 0,
    CHASSIS_NO_FOLLOW_S = 1,
    CHASSIS_FOLLOW_GIMBAL_YAW_S = 2,
} Chassis_mode_t;

typedef enum
{
    REMOTE_MODE_S = 0,
    RADA_MODE_S = 1,
} Chassis_State_t;

typedef struct 
{
    Vector3 linear;     //线速度
    Vector3 angular;    //角速度
} Radar_Data;
#pragma pack()

/* ------------------------- Internal Data ----------------------------------- */

/**
 * @brief 初始化导航接收
 * @param {UART_HandleTypeDef} *cmd_vel_uasrt_handle
 * @return 
 */
Radar_Data *CmdVelControlInit(UART_HandleTypeDef *cmd_vel_uasrt_handle);

/**
 * @brief 检查遥控器是否在线
 * @return 
 */
uint8_t CmdVelControlIsOnline();

attitude_t *gimbal_IMU_data_ptr(void);

#endif  