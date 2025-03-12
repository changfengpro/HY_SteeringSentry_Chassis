#include "referee_transport.h"

#define START_BYTE 0xAA
#define END_BYTE 0x55
// #define REFEREE_DATA_CONTROL_FRAME_SIZE sizeof(referee_info_t)

referee_info_t referee_data;
static uint8_t referee_init_flag;

static USARTInstance *referee_data_usart_instance;  // 裁判系统数据转发串口实例
static DaemonInstance *referee_data_daemo_instance; // 裁判系统数据转发进程守护实例


static void RefereeDataRxCallback()
{
    
}

/**
 * @brief 初始化裁判系统数据转发串口通信 
 * @return
 */
referee_info_t *RefereeDataTransportInit(UART_HandleTypeDef *referee_data_usart_handle)
{
    USART_Init_Config_s config;
    config.module_callback = RefereeDataRxCallback;
    config.usart_handle = referee_data_usart_handle;
    // config.recv_buff_size = REFEREE_DATA_CONTROL_FRAME_SIZE;
    referee_data_usart_instance = USARTRegister(&config);
}


