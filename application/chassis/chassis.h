/*
 * @Description: 
 * @Author: changfeng
 * @brief: 
 * @version: 
 * @Date: 2025-01-23 18:43:05
 * @LastEditors:  
 * @LastEditTime: 2025-01-24 19:06:41
 */
#ifndef CHASSIS_H
#define CHASSIS_H

/**
 * @brief 底盘应用初始化,请在开启rtos之前调用(目前会被RobotInit()调用)
 * 
 */
void ChassisInit();

/**
 * @brief 底盘应用任务,放入实时系统以一定频率运行
 * 
 */
void ChassisTask();

enum Direction
{
    forward_rotation = 1,
    back_rotation = -1
};

#pragma pack(1)
typedef struct
{
    float chassis_motor_speed[4];
    float chassis_steer_motor_angle[4];
    float vx,vy,wz;
    float last_steer_target_angle[4];
    float motor_set_speed[4];
    float motor_set_steer[4];
    float max_speed;
    int TurnFlag[4];
    enum  Direction direction[4];
} ChassisHandle_t;



#pragma pack()

#endif // CHASSIS_H