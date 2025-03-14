/**
 * @file chassis.c
 * @author NeoZeng neozng1@hnu.edu.cn
 * @brief 底盘应用,负责接收robot_cmd的控制命令并根据命令进行运动学解算,得到输出
 *        注意底盘采取右手系,对于平面视图,底盘纵向运动的正前方为x正方向;横向运动的右侧为y正方向
 *
 * @version 0.1
 * @date 2022-12-04
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "chassis.h"
#include "robot_def.h"
#include "power_control.h"
#include "super_cap.h"
#include "message_center.h"
#include "referee_task.h"
#include "dmmotor.h"
#include "general_def.h"
#include "bsp_dwt.h"
#include "referee_UI.h"
#include "arm_math.h"
#include "cmd_vel.h"

/* 根据robot_def.h中的macro自动计算的参数 */
#define WHEEL_LINE_RATION (1 / (180.0f * REDUCTION_RATIO_WHEEL)) * PI * RADIUS_WHEEL   //将舵轮电机转速转换为底盘线速度的比例
#define WHEEL_RPM_RATION  (1 / (RADIUS_WHEEL * REDUCTION_RATIO_WHEEL)) *(180.0f / PI)  //将底盘线速度转换为舵轮电机转速的比例
#define RADIAN_TO_ANGLE 180 / PI;   //将弧度制转为角度制
#define YAW_REMOTE_COEFF 0.034090909   //遥控器映射到母云台电机速度系数
/* 底盘应用包含的模块和信息存储,底盘是单例模式,因此不需要为底盘建立单独的结构体 */
#ifdef CHASSIS_BOARD // 如果是底盘板,使用板载IMU获取底盘转动角速度
#include "can_comm.h"
#include "ins_task.h"
static CANCommInstance *chasiss_can_comm; // 双板通信CAN comm
attitude_t *Chassis_IMU_data;
#endif // CHASSIS_BOARD
#ifdef ONE_BOARD
static Publisher_t *chassis_pub;                    // 用于发布底盘的数据
static Subscriber_t *chassis_sub;                   // 用于订阅底盘的控制命令
static Publisher_t *GimbalBase_Pub;                 //用于订阅母云台的数据
attitude_t *Chassis_IMU_data;
attitude_t *gimbal_imu_recv_data;
#endif                                              // !ONE_BOARD

static Chassis_Ctrl_Cmd_s chassis_cmd_recv;         // 底盘接收到的控制命令
static Chassis_Upload_Data_s chassis_feedback_data; // 底盘回传的反馈数据

static PIDInstance buffer_PID;             // 用于底盘的缓冲能量PID
static referee_info_t *referee_data;       // 用于获取裁判系统的数据
static Referee_Interactive_info_t ui_data; // UI数据，将底盘中的数据传入此结构体的对应变量中，UI会自动检测是否变化，对应显示UI

static SuperCapInstance *cap;                                       // 超级电容
static DJIMotorInstance *motor_lf, *motor_rf, *motor_lb, *motor_rb; // left right forward back

/* 用于自旋变速策略的时间变量 */
// static float t;

/* 私有函数计算的中介变量,设为静态避免参数传递的开销 */
static float chassis_vx, chassis_vy;                      // 将云台系的速度投影到底盘
static float vt_lf, vt_rf, vt_lb, vt_rb;                  // 底盘速度解算后的临时输出,待进行限幅
static float CHASSIS_6020_1_Y_ANGLE, CHASSIS_6020_2_Y_ANGLE, CHASSIS_6020_3_Y_ANGLE, CHASSIS_6020_4_Y_ANGLE;
// static attitude_t *chassis_IMU_data; // 底盘IMU数据
float Init_angle[4] = { 1.0f , 110.0f , 13.0f , 0.0f };
static float Yaw_single_angle, Yaw_angle_sum, YawTotalAngle;


/* 私有函数 */
static void ChassisHandle_Deliver_Config();
static void Steer_Speed_Calcu(ChassisHandle_t *chassis_handle, float chassis_vx, float chassis_vy, float chassis_wz);
static void Steer_angle_change(ChassisHandle_t *chassis_handle, float chassis_vx, float chassis_vy, float chassis_wz);
static void Steer_Calculate(ChassisHandle_t *chassis_handle, float chassis_vx, float chassis_vy, float chassis_wz);
static void Steer_Chassis_Control(ChassisHandle_t *Chassis_hanlde);
static void YawAngleCalculate();

// first表示第一象限， second表示第二象限，以此类推
static DJIMotorInstance *First_GM6020_motor, *Second_GM6020_motor, *Third_GM6020_motor, *Fourth_GM6020_motor, \
                        *First_M3508_motor,  *Second_M3508_motor,  *Third_M3508_motor,  *Fourth_M3508_motor;
static DMMotorInstance  *Gimbal_Base;
// extern Chassis_Ctrl_Cmd_s chassis_cmd_send;
ChassisHandle_t chassis_handle;

void ChassisInit()
{
    Chassis_IMU_data = INS_Init(); // 底盘IMU初始化
    float gimbal_base_angle_feed_ptr = Chassis_IMU_data->YawTotalAngle;
    gimbal_imu_recv_data = gimbal_IMU_data_ptr();

    Motor_Init_Config_s chassis_first_GM6020_motor_config =       //first表示第一象限， second表示第二象限，以此类推
    {
        .motor_type = GM6020,
        .can_init_config = 
        {.can_handle = &hcan1,
            .tx_id = 1
        },
        .controller_setting_init_config = 
        {.angle_feedback_source = MOTOR_FEED,
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
            .speed_feedback_source = MOTOR_FEED,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL
        },
        .controller_param_init_config = {.angle_PID = {.Improve = PID_Integral_Limit | PID_DerivativeFilter,
                                                        .Kp = 25,
                                                        .Ki = 1,
                                                        .Kd = 0,
                                                        .DeadBand = 0,
                                                        .MaxOut = 2000,
                                                        .IntegralLimit = 3000},
        .speed_PID = {.Improve = PID_Integral_Limit | PID_DerivativeFilter,
                        .Kp = 20,
                        .Ki = 1,
                        .Kd = 0,
                        .DeadBand = 0,
                        .MaxOut = 20000,
                        .IntegralLimit = 3000
        }

        }
    };

    Motor_Init_Config_s chassis_second_GM6020_motor_config = 
    {
        .motor_type = GM6020,
        .can_init_config = 
        {.can_handle = &hcan2,
            .tx_id = 1
        },
        .controller_setting_init_config = 
        {.angle_feedback_source = MOTOR_FEED,
            .outer_loop_type = ANGLE_LOOP ,
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
            .speed_feedback_source = MOTOR_FEED,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL
        },
        .controller_param_init_config = {.angle_PID = {.Improve = PID_Integral_Limit | PID_DerivativeFilter,
                                                        .Kp = 25,
                                                        .Ki = 1,
                                                        .Kd = 0,
                                                        .DeadBand = 0,
                                                        .MaxOut = 2000,
                                                        .IntegralLimit = 3000},
        .speed_PID = {.Improve = PID_Integral_Limit | PID_DerivativeFilter,
                        .Kp = 20,
                        .Ki = 1,
                        .Kd = 0,
                        .DeadBand = 0,
                        .MaxOut = 20000,
                        .IntegralLimit = 3000
        }

        }
    };

    Motor_Init_Config_s chassis_third_GM6020_motor_config = 
    {
        .motor_type = GM6020,
        .can_init_config = 
        {.can_handle = &hcan2,
            .tx_id = 2
        },
        .controller_setting_init_config = 
        {.angle_feedback_source = MOTOR_FEED,
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
            .speed_feedback_source = MOTOR_FEED,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL
        },
        .controller_param_init_config = {.angle_PID = {.Improve = PID_Integral_Limit | PID_DerivativeFilter,
                                                        .Kp = 25,
                                                        .Ki = 1,
                                                        .Kd = 0,
                                                        .DeadBand = 0,
                                                        .MaxOut = 2000,
                                                        .IntegralLimit = 3000},
        .speed_PID = {.Improve = PID_Integral_Limit | PID_DerivativeFilter,
                        .Kp = 20,
                        .Ki = 1,
                        .Kd = 0,
                        .DeadBand = 0,
                        .MaxOut = 20000,
                        .IntegralLimit = 3000
        }

        }
    };

    Motor_Init_Config_s chassis_fourth_GM6020_motor_config = 
    {
        .motor_type = GM6020,
        .can_init_config = 
        {.can_handle = &hcan1,
            .tx_id = 2
        },
        .controller_setting_init_config = 
        {.angle_feedback_source = MOTOR_FEED,
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
            .speed_feedback_source = MOTOR_FEED,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL
        },
        .controller_param_init_config = {.angle_PID = {.Improve = PID_Integral_Limit | PID_DerivativeFilter,
                                                        .Kp = 25,
                                                        .Ki = 1,
                                                        .Kd = 0,
                                                        .DeadBand = 0,
                                                        .MaxOut = 2000,
                                                        .IntegralLimit = 3000},
        .speed_PID = {.Improve = PID_Integral_Limit | PID_DerivativeFilter,
                        .Kp = 20,
                        .Ki = 1,
                        .Kd = 0,
                        .DeadBand = 0,
                        .MaxOut = 20000,
                        .IntegralLimit = 3000,

        }

        }
    };
    
/************************************************************************************************** */

    Motor_Init_Config_s chassis_first_M3508_motor_config = 
    {
        .motor_type = M3508,
        .can_init_config = 
        {
            .can_handle = &hcan1,
            .tx_id = 3
        },
        .controller_setting_init_config = 
        {
            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = SPEED_LOOP,
            .speed_feedback_source = MOTOR_FEED,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL
        },
        .controller_param_init_config = {.speed_PID = {.Improve = PID_Integral_Limit | PID_DerivativeFilter | PID_SlopeAccelerationDeceleration,
                                                        .Kp = 2.5,
                                                        .Ki = 1,
                                                        .Kd = 0,
                                                        .DeadBand = 0,
                                                        .MaxOut = 20000,
                                                        .IntegralLimit = 3000,

                                                        .slope = {.decrease_value = 50,
                                                                  .increase_value = 50,
                                                                  .slope_first = SLOPE_FIRST_REAL
                                                                 }
                                                        }              
        }
    };

     Motor_Init_Config_s chassis_second_M3508_motor_config = 
    {
        .motor_type = M3508,
        .can_init_config = 
        {
            .can_handle = &hcan2,
            .tx_id = 3
        },
        .controller_setting_init_config = 
        {
            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = SPEED_LOOP,
            .speed_feedback_source = MOTOR_FEED,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL
        },
        .controller_param_init_config = {.speed_PID = {.Improve = PID_Integral_Limit | PID_DerivativeFilter | PID_SlopeAccelerationDeceleration,
                                                        .Kp = 2.5,
                                                        .Ki = 1,
                                                        .Kd = 0,
                                                        .DeadBand = 0,
                                                        .MaxOut = 20000,
                                                        .IntegralLimit = 3000,
                                                        .slope = {.decrease_value = 50,
                                                                  .increase_value = 50,
                                                                  .slope_first = SLOPE_FIRST_REAL
                                                                 }
                                                        },
        }
    };

    Motor_Init_Config_s chassis_third_M3508_motor_config = 
    {
        .motor_type = M3508,
        .can_init_config = 
        {
            .can_handle = &hcan2,
            .tx_id = 4
        },
        .controller_setting_init_config = 
        {
            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = SPEED_LOOP,
            .speed_feedback_source = MOTOR_FEED,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL
        },
        .controller_param_init_config = {.speed_PID = {.Improve = PID_Integral_Limit | PID_DerivativeFilter | PID_SlopeAccelerationDeceleration,
                                                        .Kp = 2.5,
                                                        .Ki = 1,
                                                        .Kd = 0,
                                                        .DeadBand = 0,
                                                        .MaxOut = 20000,
                                                        .IntegralLimit = 3000,
                                                        .slope = {.decrease_value = 50,
                                                                  .increase_value = 50,
                                                                  .slope_first = SLOPE_FIRST_REAL
                                                                 }
                                                        },
        }
    };

    Motor_Init_Config_s chassis_fourth_M3508_motor_config = 
    {
        .motor_type = M3508,
        .can_init_config = 
        {
            .can_handle = &hcan1,
            .tx_id = 4
        },
        .controller_setting_init_config = 
        {
            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = SPEED_LOOP,
            .speed_feedback_source = MOTOR_FEED,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL
        },
        .controller_param_init_config = {.speed_PID = {.Improve = PID_Integral_Limit | PID_DerivativeFilter | PID_SlopeAccelerationDeceleration, 
                                                        .Kp = 2.5,
                                                        .Ki = 1,
                                                        .Kd = 0,
                                                        .DeadBand = 0,
                                                        .MaxOut = 20000,
                                                        .IntegralLimit = 3000,
                                                        .slope = {.decrease_value = 50,
                                                                  .increase_value = 50,
                                                                  .slope_first = SLOPE_FIRST_REAL
                                                                 }
                                                        },
        }
    };

Motor_Init_Config_s DMmotor_Motor_Config = {
    .controller_setting_init_config.angle_feedback_source = MOTOR_FEED,
    .controller_setting_init_config.close_loop_type = ANGLE_LOOP,
    .controller_setting_init_config.feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL,
    .controller_setting_init_config.feedforward_flag = SPEED_FEEDFORWARD,
    .controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
    .controller_setting_init_config.outer_loop_type = ANGLE_LOOP,
    .controller_setting_init_config.speed_feedback_source = MOTOR_FEED,
    .controller_param_init_config.other_angle_feedback_ptr = &gimbal_base_angle_feed_ptr,
    .can_init_config.can_handle = &hcan1,
    // .can_init_config.can_module_callback = &DMMotorLostCallback,
    // .can_init_config.id = (void *)0x00,
    .can_init_config.rx_id = 0x10,
    .can_init_config.tx_id = 0x20F,


    .controller_param_init_config = {

        .angle_PID = {
            .Kp = 0.3,
            .Ki = 0,
            .Kd = 0,
            .MaxOut = 30,
            .IntegralLimit = 15,
            .Improve = PID_Integral_Limit | PID_DerivativeFilter
        },

        .speed_PID = {
            .Kp = 5,
            .Ki = 0.1,
            .Kd = 0,
            .MaxOut = 50,
            .IntegralLimit = 15,
            .IntegralLimit = PID_Integral_Limit | PID_DerivativeFilter
        }
    }
    };

    First_GM6020_motor  = DJIMotorInit(&chassis_first_GM6020_motor_config);
    Second_GM6020_motor = DJIMotorInit(&chassis_second_GM6020_motor_config);
    Third_GM6020_motor  = DJIMotorInit(&chassis_third_GM6020_motor_config);
    Fourth_GM6020_motor = DJIMotorInit(&chassis_fourth_GM6020_motor_config);
    First_M3508_motor   = DJIMotorInit(&chassis_first_M3508_motor_config);
    Second_M3508_motor  = DJIMotorInit(&chassis_second_M3508_motor_config);
    Third_M3508_motor   = DJIMotorInit(&chassis_third_M3508_motor_config);
    Fourth_M3508_motor  = DJIMotorInit(&chassis_fourth_M3508_motor_config);
    Gimbal_Base         = DMMotorInit(&DMmotor_Motor_Config);



    ChassisHandle_Deliver_Config();

    // 发布订阅初始化,如果为双板,则需要can comm来传递消息
#ifdef CHASSIS_BOARD
    Chassis_IMU_data = INS_Init(); // 底盘IMU初始化

    CANComm_Init_Config_s comm_conf = {
        .can_config = {
            .can_handle = &hcan2,
            .tx_id = 0x311,
            .rx_id = 0x312,
        },
        .recv_data_len = sizeof(Chassis_Ctrl_Cmd_s),
        .send_data_len = sizeof(Chassis_Upload_Data_s),
    };
    chasiss_can_comm = CANCommInit(&comm_conf); // can comm初始化
#endif                                          // CHASSIS_BOARD

#ifdef ONE_BOARD // 单板控制整车,则通过pubsub来传递消息
    chassis_sub = SubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_pub = PubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
    GimbalBase_Pub = PubRegister("GimbalBase_feed", sizeof(float));
#endif // ONE_BOARD
}


/**
 * @brief 根据裁判系统和电容剩余容量对输出进行限制并设置电机参考值
 *
 */
static void LimitChassisOutput()
{
    // 功率限制待添加
    // referee_data->PowerHeatData.chassis_power;
    // referee_data->PowerHeatData.chassis_power_buffer;

    // 完成功率限制后进行电机参考输入设定
    DJIMotorSetRef(First_GM6020_motor, (float)(chassis_handle.motor_set_steer[0])  + Init_angle[0] );
    DJIMotorSetRef(Second_GM6020_motor, (float)(chassis_handle.motor_set_steer[1]) + Init_angle[1]);
    DJIMotorSetRef(Third_GM6020_motor, (float)(chassis_handle.motor_set_steer[2])  + Init_angle[2]);
    DJIMotorSetRef(Fourth_GM6020_motor, (float)(chassis_handle.motor_set_steer[3]) + Init_angle[3]);

    DJIMotorSetRef(First_M3508_motor, chassis_handle.motor_set_speed[0]);
    DJIMotorSetRef(Second_M3508_motor, chassis_handle.motor_set_speed[1]);
    DJIMotorSetRef(Third_M3508_motor, chassis_handle.motor_set_speed[2]);
    DJIMotorSetRef(Fourth_M3508_motor, -chassis_handle.motor_set_speed[3]);
    DMMotorSetRef(Gimbal_Base, chassis_handle.gimbal_angle);
    
}

/**
 * @brief 根据每个轮子的速度反馈,计算底盘的实际运动速度,逆运动解算
 *        对于双板的情况,考虑增加来自底盘板IMU的数据
 *
 */
static void EstimateSpeed()
{
    // 根据电机速度和陀螺仪的角速度进行解算,还可以利用加速度计判断是否打滑(如果有)
    // chassis_feedback_data.vx vy wz =
    //  ...

}
/**
 * @brief 传递遥控器参数
 * @return {*}
 */
static void ChassisHandle_Deliver_Config()
{
    chassis_handle.vx = chassis_cmd_recv.vx;
    chassis_handle.vy = chassis_cmd_recv.vy;
    chassis_handle.wz = chassis_cmd_recv.wz;
}

/**
 * @brief 舵轮速度解算
 * @param {ChassisHandle_t} *chassis_handle
 * @param {float} chassis_vx
 * @param {float} chassis_vy
 * @param {float} chassis_wz
 * @return None
 */
static void Steer_Speed_Calcu(ChassisHandle_t *chassis_handle, float chassis_vx, float chassis_vy, float chassis_wz)
{
    float theta = atan(1.0 / 1.0);  //返回45度
    float steer_wz = chassis_wz * PI /180.0f;  //chassis_wz传入的是度/秒，转化为弧度制


    //根据公式计算电机转速
    chassis_handle->chassis_motor_speed[0] = sqrt( pow(chassis_vx - steer_wz * RADIUS * cos(theta), 2) + pow(chassis_vy + steer_wz * RADIUS * sin(theta), 2)) * WHEEL_RPM_RATION;
    chassis_handle->chassis_motor_speed[1] = sqrt( pow(chassis_vx - steer_wz * RADIUS * cos(theta), 2) + pow(chassis_vy - steer_wz * RADIUS * sin(theta), 2)) * WHEEL_RPM_RATION; 
    chassis_handle->chassis_motor_speed[2] = sqrt( pow(chassis_vx + steer_wz * RADIUS * cos(theta), 2) + pow(chassis_vy - steer_wz * RADIUS * sin(theta), 2)) * WHEEL_RPM_RATION;
    chassis_handle->chassis_motor_speed[3] = sqrt( pow(chassis_vx + steer_wz * RADIUS * cos(theta), 2) + pow(chassis_vy + steer_wz * RADIUS * sin(theta), 2)) * WHEEL_RPM_RATION;

    //寻找转速最大值，以及实现正反转
    for(int i = 0; i < 4; i++)
    {
        if(chassis_handle->TurnFlag[i] == 1)
            chassis_handle->chassis_motor_speed[i] = -chassis_handle->chassis_motor_speed[i];

        else
            chassis_handle->chassis_motor_speed[i] = chassis_handle->chassis_motor_speed[i];

        if(fabs(chassis_handle->chassis_motor_speed[i]) > chassis_handle->max_speed)
            chassis_handle->max_speed = fabs(chassis_handle->chassis_motor_speed[i]);
    }

    for(int i = 0; i < 4; i++)
    {
        chassis_handle->motor_set_speed[i] = chassis_handle->chassis_motor_speed[i];
    }

}

/**
 * @brief 舵轮角度解算
 * @param {ChassisHandle_t} *chassis_handle
 * @param {float} chassis_vx
 * @param {float} chassis_vy
 * @param {float} chassis_wz
 * @return None
 */
static void Steer_angle_change(ChassisHandle_t *chassis_handle, float chassis_vx, float chassis_vy, float chassis_wz)
{
    float theta = atan(1.0 / 1.0);  //返回45度
    float steer_wz = 15 * chassis_wz * PI /180.0f;  //chassis_wz传入的是度/秒，转化为弧度制(单位已经乱了，无力修改)
    float atan_angle[4];

    if((chassis_vx == 0) && (chassis_vy == 0) && (chassis_wz == 0))
    {
        for(int i = 0; i < 4; i++)
        {
            chassis_handle->motor_set_steer[i] = chassis_handle->last_steer_target_angle[i];
        }
    }
    else
    {
        atan_angle[0] = atan2((chassis_vx - steer_wz * RADIUS * cos(theta)), (chassis_vy + steer_wz * RADIUS * sin(theta)));
        atan_angle[1] = atan2((chassis_vx - steer_wz * RADIUS * cos(theta)),(chassis_vy - steer_wz * RADIUS * sin(theta)));
        atan_angle[2] = atan2((chassis_vx + steer_wz * RADIUS * cos(theta)),(chassis_vy - steer_wz * RADIUS * sin(theta)));
        atan_angle[3] = atan2((chassis_vx + steer_wz * RADIUS * cos(theta)),(chassis_vy + steer_wz * RADIUS * sin(theta)));

        chassis_handle->motor_set_steer[0] = CHASSIS_6020_1_Y_ANGLE + atan_angle[0];
        chassis_handle->motor_set_steer[1] = CHASSIS_6020_2_Y_ANGLE + atan_angle[1];
        chassis_handle->motor_set_steer[2] = CHASSIS_6020_3_Y_ANGLE + atan_angle[2];
        chassis_handle->motor_set_steer[3] = CHASSIS_6020_4_Y_ANGLE + atan_angle[3];

        for(int i = 0; i < 4; i++)
        {
            if((chassis_handle->motor_set_steer[i]  - chassis_handle->last_steer_target_angle[i]) > (PI / 2))
            {
                chassis_handle->motor_set_steer[i] = fmodf(chassis_handle->motor_set_steer[i] - PI, 2 * PI);
                chassis_handle->TurnFlag[i] = 1;
            }
            else if((chassis_handle->motor_set_steer[i]  - chassis_handle->last_steer_target_angle[i]) < (-PI / 2))
            {
                chassis_handle->motor_set_steer[i] = fmodf(chassis_handle->motor_set_steer[i] + PI, 2 * PI);
                chassis_handle->TurnFlag[i] = 1;
            }
            else
            {
                chassis_handle->TurnFlag[i] = 0;
            }
        }

        for(int i = 0; i < 4; i++)
        {
            chassis_handle->motor_set_steer[i] = chassis_handle->motor_set_steer[i] * RADIAN_TO_ANGLE;

            chassis_handle->last_steer_target_angle[i] = chassis_handle->motor_set_steer[i];    //记录上一次的角度数据
        }
    }

}

static void Steer_Calculate(ChassisHandle_t *chassis_handle, float chassis_vx, float chassis_vy, float chassis_wz)
{
    Steer_angle_change(chassis_handle, chassis_vx, chassis_vy, chassis_wz);
    Steer_Speed_Calcu(chassis_handle, chassis_vx, chassis_vy, chassis_wz);
}

static void Steer_Chassis_Control(ChassisHandle_t *Chassis_hanlde)
{   
    Steer_Calculate(Chassis_hanlde, Chassis_hanlde->vy, Chassis_hanlde->vx, Chassis_hanlde->wz);    //修改为机器人标准坐标系，不想修改解算函数，直接将Vx,Vy调换位置传入
}

/**
 * @brief 母云台达妙电机总角度计算
 * @return 
 */
static void YawAngleCalculate()
{
    static float angle, last_angle, temp, rad_sum;
    angle = Gimbal_Base->measure.position; // 从云台获取的当前yaw电机角度
    
    if(fabs(angle - last_angle) > 0.0001)
    {
        if((angle - last_angle) < -12.5)
        {
            rad_sum += 25 + angle - last_angle;
        }
        else if((angle - last_angle) >  12.5)
        {
            rad_sum += 25 + last_angle - angle;
        }
        else
        {
            rad_sum += angle - last_angle;
        }
    }
    Yaw_angle_sum = rad_sum * RAD_2_DEGREE;
    temp = fmodf(Yaw_angle_sum, 360.0);
    Yaw_single_angle = fabs(temp);
    Gimbal_Base->measure.single_angle = Yaw_single_angle;
    Gimbal_Base->measure.total_angle = Yaw_angle_sum;
    last_angle = angle;
}


/* 机器人底盘控制核心任务 */
void ChassisTask()
{   
    chassis_feedback_data.chassis_imu_data = Chassis_IMU_data;
    YawTotalAngle = gimbal_imu_recv_data->YawTotalAngle;
    // YawAngleCalculate();    //yaw电机总角度计算

    // 后续增加没收到消息的处理(双板的情况)
    // 获取新的控制信息
#ifdef ONE_BOARD
    SubGetMessage(chassis_sub, &chassis_cmd_recv);
#endif
#ifdef CHASSIS_BOARD
    chassis_cmd_recv = *(Chassis_Ctrl_Cmd_s *)CANCommGet(chasiss_can_comm);
#endif // CHASSIS_BOARD

    // SetPowerLimit(referee_data->GameRobotState.chassis_power_limit);//设置功率限制
    if (chassis_cmd_recv.chassis_mode == CHASSIS_ZERO_FORCE)
    { 
    // 如果出现重要模块离线或遥控器设置为急停,让电机停止
    DJIMotorStop(First_GM6020_motor);
    DJIMotorStop(Second_GM6020_motor);
    DJIMotorStop(Third_GM6020_motor);
    DJIMotorStop(Fourth_GM6020_motor);
    DJIMotorStop(First_M3508_motor);
    DJIMotorStop(Second_M3508_motor);
    DJIMotorStop(Third_M3508_motor);
    DJIMotorStop(Fourth_M3508_motor);
    DMMotorStop(Gimbal_Base);
    }
    else
    { // 正常工作
    DJIMotorEnable(First_GM6020_motor);
    DJIMotorEnable(Second_GM6020_motor);
    DJIMotorEnable(Third_GM6020_motor);
    DJIMotorEnable(Fourth_GM6020_motor);
    DJIMotorEnable(First_M3508_motor);
    DJIMotorEnable(Second_M3508_motor);
    DJIMotorEnable(Third_M3508_motor);
    DJIMotorEnable(Fourth_M3508_motor);
    DMMotorEnable(Gimbal_Base);
    }

    // 根据控制模式设定旋转速度
    switch (chassis_cmd_recv.chassis_mode)
    {
    case CHASSIS_NO_FOLLOW: // 底盘不旋转,但维持全向机动,一般用于调整云台姿态
        chassis_cmd_recv.wz = 0;
        break;
    case CHASSIS_FOLLOW_GIMBAL_YAW: // 跟随云台,不单独设置pid,以误差角度平方为速度输出
        // DMMotorChangeFeed(Gimbal_Base, ANGLE_LOOP, OTHER_FEED);
        // chassis_cmd_recv.wz = -1.5f * chassis_cmd_recv.offset_angle * abs(chassis_cmd_recv.offset_angle);
        break;
    case CHASSIS_ROTATE: // 自旋,同时保持全向机动;当前wz维持定值,后续增加不规则的变速策略
        chassis_cmd_recv.wz = 4000;
        break;
    case CHASSIS_RADAR: //  导航模式
        break;
    default:
        break;
    }


    // 根据云台和底盘的角度offset将控制量映射到底盘坐标系上
    // 底盘逆时针旋转为角度正方向;云台命令的方向以云台指向的方向为x,采用右手系(x指向正北时y在正东)
    static float sin_theta, cos_theta;
    cos_theta = arm_cos_f32(chassis_cmd_recv.offset_angle * DEGREE_2_RAD);
    sin_theta = arm_sin_f32(chassis_cmd_recv.offset_angle * DEGREE_2_RAD);
    chassis_handle.vx = chassis_cmd_recv.vx * cos_theta - chassis_cmd_recv.vy * sin_theta;
    chassis_handle.vy = chassis_cmd_recv.vx * sin_theta + chassis_cmd_recv.vy * cos_theta;
    chassis_handle.wz = chassis_cmd_recv.wz;
    chassis_handle.gimbal_angle = chassis_cmd_recv.gimbal_angle;
    // ChassisHandle_Deliver_Config();

    // 根据控制模式进行运动学解算,计算底盘输出
    Steer_Chassis_Control(&chassis_handle);

    

    // 根据裁判系统的反馈数据和电容数据对输出限幅并设定闭环参考值
    LimitChassisOutput();

    // 根据电机的反馈速度和IMU(如果有)计算真实速度
    EstimateSpeed();

    // // 获取裁判系统数据   建议将裁判系统与底盘分离，所以此处数据应使用消息中心发送
    // // 我方颜色id小于7是红色,大于7是蓝色,注意这里发送的是对方的颜色, 0:blue , 1:red
    // chassis_feedback_data.enemy_color = referee_data->GameRobotState.robot_id > 7 ? 1 : 0;
    // // 当前只做了17mm热量的数据获取,后续根据robot_def中的宏切换双枪管和英雄42mm的情况
    // chassis_feedback_data.bullet_speed = referee_data->GameRobotState.shooter_id1_17mm_speed_limit;
    // chassis_feedback_data.rest_heat = referee_data->PowerHeatData.shooter_heat0;

    // 推送反馈消息
#ifdef ONE_BOARD
    PubPushMessage(chassis_pub, (void *)&chassis_feedback_data);
    PubPushMessage(GimbalBase_Pub, (void *)&YawTotalAngle);
#endif
#ifdef CHASSIS_BOARD
    CANCommSend(chasiss_can_comm, (void *)&chassis_feedback_data);
#endif // CHASSIS_BOARD
}