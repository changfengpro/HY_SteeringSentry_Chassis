/*
 * @Description: 
 * @Author: changfeng
 * @brief: 
 * @version: 
 * @Date: 2025-02-01 20:35:00
 * @LastEditors:  
 * @LastEditTime: 2025-02-04 19:24:51
 */
#ifndef CMD_VEL_H
#define CMD_VEL_H

#include "stdint.h"
#include "bsp_usart.h"
#include "daemon.h"
#include "bsp_log.h"
#include "usart.h"

#pragma pack(1)
typedef struct 
{
    float x;
    float y;
    float z;
} Vector3;  //通用三维向量结构体


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

#endif  