/*
 * @Description: 
 * @Author: changfeng
 * @brief: 
 * @version: 
 * @Date: 2025-02-01 20:35:00
 * @LastEditors:  
 * @LastEditTime: 2025-02-01 23:08:23
 */
#ifndef CMD_VEL_H
#define CMD_VEL_H

#pragma pack(1)
typedef struct 
{
    float linear_x;
    float angular_z;   
} Radar_Data;
#pragma pack()

/* ------------------------- Internal Data ----------------------------------- */

/**
 * @brief 初始化导航接收
 * @param {UART_HandleTypeDef} *cmd_vel_uasrt_handle
 * @return 
 */
Radar_Data CmdVelInit(UART_HandleTypeDef *cmd_vel_uasrt_handle);

/**
 * @brief 检查遥控器是否在线
 * @return 
 */
uint8_t CmdVelControlIsOnline();

#endif  