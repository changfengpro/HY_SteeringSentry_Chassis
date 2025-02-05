/*
 * @Description: 
 * @Author: changfeng
 * @brief: 
 * @version: 
 * @Date: 2025-02-01 20:34:56
 * @LastEditors:  
 * @LastEditTime: 2025-02-05 17:15:06
 */

#include "cmd_vel.h"


#define START_BYTE 0xAA
#define END_BYTE 0x55
#define CMD_VEL_CONTROL_FRAME_SIZE 27u  //导航接的buffer大小

static Radar_Data radar_ctrl;
static uint8_t cmd_vel_init_flag;

static USARTInstance *cmd_vel_usart_instance;   //导航串口实例
static DaemonInstance *cmd_vel_daemo_instance;  //导航守护进程实例

/**
 * @brief cmd_vel数据包解析
 * @param cmd_vel_buf 数据包缓冲区
 * @return 
 */
static void Cmd_vel_Parse(const uint8_t *cmd_vel_buf)
{
    // 检查起始字节和结束字节
    if(cmd_vel_buf[0] != START_BYTE || cmd_vel_buf[26] != END_BYTE)
    {
        LOGWARNING("[cmd_vel] Packet format error");
        return;
    }

    // 计算校验和（前 25 字节）
    uint8_t checksum = 0;
    for(int i = 0; i < 25; i++)
    {
        checksum ^= cmd_vel_buf[i];
    }
    
    if(checksum == cmd_vel_buf[25]) // 校验正确
    {
        // 解析数据
        memcpy(&radar_ctrl.linear.x, &cmd_vel_buf[1], sizeof(float));
        memcpy(&radar_ctrl.linear.y, &cmd_vel_buf[5], sizeof(float));
        memcpy(&radar_ctrl.linear.z, &cmd_vel_buf[9], sizeof(float));
        memcpy(&radar_ctrl.angular.x, &cmd_vel_buf[13], sizeof(float));
        memcpy(&radar_ctrl.angular.y, &cmd_vel_buf[17], sizeof(float));
        memcpy(&radar_ctrl.angular.z, &cmd_vel_buf[21], sizeof(float));

        LOGINFO("[cmd_vel] Parsed data: Linear x: %.6f, Linear y: %.6f, Linear z: %.6f, "
                "Angular x: %.6f, Angular y: %.6f, Angular z: %.6f",
                radar_ctrl.linear.x, radar_ctrl.linear.y, radar_ctrl.linear.z,
                radar_ctrl.angular.x, radar_ctrl.angular.y, radar_ctrl.angular.z);
    }
    else
    {
        LOGWARNING("[cmd_vel] Checksum error");
    }
}

/**
 * @brief 导航数据解析回调函数
 * @return 
 */
static void CmdVelControlRxCallback()
{
    DaemonReload(cmd_vel_daemo_instance);   //先喂狗
    Cmd_vel_Parse(cmd_vel_usart_instance->recv_buff);   //进行协议解析

}

/**
 * @brief 导航离线回调函数
 * @return 
 */
static void CmdVelLostCallback()
{
    memset(&radar_ctrl, 0, sizeof(radar_ctrl)); //清空cmd_vel数据
    USARTServiceInit(cmd_vel_usart_instance);   //尝试重新启动

    LOGWARNING("[Cmd_Vel] radar control lost");
    
}

/**
 * @brief 初始化cmd_vel串口通信
 * @param cmd_vel_uasrt_handle 串口句柄
 * @return 初始化后的雷达数据
 */
Radar_Data *CmdVelControlInit(UART_HandleTypeDef *cmd_vel_usart_handle)
{
    USART_Init_Config_s config;
    config.module_callback = CmdVelControlRxCallback;
    config.usart_handle = cmd_vel_usart_handle;
    config.recv_buff_size = CMD_VEL_CONTROL_FRAME_SIZE;
    cmd_vel_usart_instance = USARTRegister(&config);

    // 进行进程守护的注册，用于定时检查串口是否正常工作
    Daemon_Init_Config_s daemo_conf = {
        .reload_count = 10, //100ms未接收到数据视为离线
        .callback = CmdVelLostCallback,
        .owner_id = NULL,   //只有一个cmd_vel不需要id
    };
    cmd_vel_daemo_instance = DaemonRegister(&daemo_conf);
    cmd_vel_init_flag = 1;

    return &radar_ctrl;
}

/**
 * @brief 检查导航控制是否在线
 * @return 1 表示在线, 0 表示离线
 */
uint8_t CmdVelControlIsOnline()
{
    if(cmd_vel_init_flag == 1)
        return DaemonIsOnline(cmd_vel_daemo_instance);
    return 0;
}




