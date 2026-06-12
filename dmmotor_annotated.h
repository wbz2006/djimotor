/**
 * ====================================================================================
 * 
 * 文件名: dmmotor.h
 * 
 * 功能: DM系列电机（Dynamixel协议电机）驱动头文件
 * 
 * 描述: 
 *   本文件定义了DM系列电机的数据类型、结构体、枚举和驱动接口
 *   适用于使用Dynamixel协议的CAN总线通信电机
 * 
 * 适用电机: DM系列电机（如DM4346、DM6215等）
 * 
 * 通信协议: CAN 2.0A, 标准帧, Dynamixel Protocol
 * 
 * 作者: 基于DJI Motor SDK架构修改
 * 
 * ====================================================================================
 */

#ifndef DMMOTOR_H
#define DMMOTOR_H

/* ================================ 头文件引用 ================================ */

#include <stdint.h>          // 标准整数类型定义（uint8_t, uint16_t, int32_t等）
#include "bsp_can.h"         // CAN总线驱动头文件（CANInstance, CANTransmit等）
#include "controller.h"      // PID控制器头文件（PIDInstance等）
#include "motor_def.h"       // 通用电机定义（Motor_Init_Config_s等）

/* ================================ 宏定义 ================================ */

/**
 * @brief 最大支持的DM电机数量
 * 
 * 功能说明:
 *   定义系统中最多可以注册的DM电机数量
 *   用于静态数组大小和边界检查
 * 
 * 当前值: 4
 * 
 * @note 如果需要支持更多电机，可以修改此值
 * @note 同时需要考虑CAN总线的带宽限制
 */
#define DM_MOTOR_CNT 4

/*-------------------------- 电机参数范围定义 --------------------------*/

/**
 * @brief DM电机位置参数范围
 * 
 * 单位: 弧度 (rad)
 * 说明: 电机单圈位置的范围
 * 
 * @note 负值表示反向，零位由编码器校准决定
 */
#define DM_P_MIN  (-12.5f)   /**< 位置最小值: -12.5 rad */
#define DM_P_MAX  12.5f      /**< 位置最大值: +12.5 rad */

/**
 * @brief DM电机速度参数范围
 * 
 * 单位: 弧度/秒 (rad/s)
 * 说明: 电机允许的最大转速范围
 * 
 * @note 负值表示反向转动
 */
#define DM_V_MIN  (-45.0f)   /**< 速度最小值: -45 rad/s */
#define DM_V_MAX  45.0f      /**< 速度最大值: +45 rad/s */

/**
 * @brief DM电机力矩参数范围
 * 
 * 单位: 牛米 (N·m)
 * 说明: 电机允许输出的力矩范围
 * 
 * @note 负值表示反向输出力矩
 * @note 这是电机控制的主要参数
 */
#define DM_T_MIN  (-18.0f)   /**< 力矩最小值: -18 N·m */
#define DM_T_MAX   18.0f     /**< 力矩最大值: +18 N·m */

/* ================================ 数据结构体定义 ================================ */

/**
 * @brief DM电机反馈数据结构体
 * 
 * 功能说明:
 *   存储电机运行时的实时反馈数据
 *   包括位置、速度、力矩和温度信息
 * 
 * 数据更新:
 *   由DMMotorDecode()函数在CAN接收中断中更新
 *   用户可以直接读取这些值获取电机状态
 * 
 * 成员说明:
 *   - id: 电机ID（在CAN报文中的标识）
 *   - state: 电机状态（预留）
 *   - velocity: 当前速度（rad/s）
 *   - last_position: 上一次的位置（用于计算位置变化）
 *   - position: 当前单圈位置（rad）
 *   - torque: 当前输出力矩（N·m）
 *   - T_Mos: MOS管温度（℃）
 *   - T_Rotor: 转子温度（℃）
 *   - total_round: 累计转过的圈数（用于多圈位置）
 * 
 * 使用示例:
 *   @code
 *   // 读取电机当前速度
 *   float speed = motor->measure.velocity;
 *   
 *   // 读取电机当前位置
 *   float pos = motor->measure.position;
 *   
 *   // 读取电机温度
 *   float mos_temp = motor->measure.T_Mos;
 *   @endcode
 * 
 * @see DMMotorDecode() - 反馈数据解析函数
 */
typedef struct 
{
    uint8_t id;              /**< 电机ID (1-255) */
    uint8_t state;           /**< 电机状态（预留） */
    float velocity;          /**< 当前速度，单位: rad/s */
    float last_position;     /**< 上一次位置，用于计算位置变化，单位: rad */
    float position;          /**< 当前单圈位置，单位: rad */
    float torque;            /**< 当前力矩，单位: N·m */
    float T_Mos;             /**< MOS管温度，单位: ℃ */
    float T_Rotor;           /**< 转子温度，单位: ℃ */
    int32_t total_round;     /**< 累计圈数（多圈位置计算用） */
} DM_Motor_Measure_s;

/**
 * @brief DM电机CAN发送数据结构体
 * 
 * 功能说明:
 *   封装电机控制指令的数据结构
 *   在发送前需要将数据编码为CAN报文格式
 * 
 * 数据说明:
 *   - position_des: 目标位置（当前版本设为0）
 *   - velocity_des: 目标速度（当前版本设为0）
 *   - torque_des: 目标力矩（主要控制量）
 *   - Kp/Kd: PID参数（当前版本保留）
 * 
 * 编码说明:
 *   所有参数在发送前需要通过float_to_uint()编码
 *   不同参数使用不同的编码位数以平衡精度
 * 
 * @see float_to_uint() - 数据编码函数
 * @see DMMotorControl() - 控制函数
 */
typedef struct
{
    uint16_t position_des;   /**< 目标位置编码值 (16位) */
    uint16_t velocity_des;   /**< 目标速度编码值 (12位) */
    uint16_t torque_des;     /**< 目标力矩编码值 (12位) */
    uint16_t Kp;             /**< PID比例参数编码值 */
    uint16_t Kd;             /**< PID微分参数编码值 */
} DMMotor_Send_s;

/**
 * @brief DM电机完整实例结构体
 * 
 * 功能说明:
 *   包含电机控制所需的所有信息
 *   是电机驱动的核心数据结构
 * 
 * 组成部分:
 *   - measure: 电机反馈数据（位置、速度、力矩、温度）
 *   - motor_settings: 电机控制设置（闭环类型、反转标志等）
 *   - PID控制器: 三环PID实例（电流环、速度环、位置环）
 *   - 反馈数据指针: 用于外部数据反馈（如IMU）
 *   - pid_ref: 控制参考值（目标值）
 *   - stop_flag: 电机使能/停止标志
 *   - motor_can_instace: CAN通信实例
 *   - lost_cnt: 通信丢失计数（预留）
 * 
 * 内存管理:
 *   - 由DMMotorInit()动态分配
 *   - 内存大小约几百字节
 * 
 * 使用注意:
 *   - 不要直接修改结构体成员，应使用驱动提供的接口函数
 *   - 通过motor->measure可以读取电机状态
 *   - 通过motor->motor_can_instace可以访问CAN配置
 * 
 * @see DMMotorInit() - 初始化函数
 * @see DMMotorSetRef() - 设置控制量
 * @see DMMotorControl() - 控制主函数
 */
typedef struct 
{
    /*-------------------------- 反馈数据 --------------------------*/
    DM_Motor_Measure_s measure;               /**< 电机反馈数据 */
    
    /*-------------------------- 控制设置 --------------------------*/
    Motor_Control_Setting_s motor_settings;   /**< 电机控制设置 */
    
    /*-------------------------- PID控制器 --------------------------*/
    PIDInstance current_PID;                  /**< 电流环PID控制器 */
    PIDInstance speed_PID;                    /**< 速度环PID控制器 */
    PIDInstance angle_PID;                    /**< 角度环PID控制器 */
    
    /*-------------------------- 反馈数据指针 --------------------------*/
    float *other_angle_feedback_ptr;          /**< 外部角度反馈指针（如IMU） */
    float *other_speed_feedback_ptr;          /**< 外部速度反馈指针 */
    float *speed_feedforward_ptr;             /**< 速度前馈指针 */
    float *current_feedforward_ptr;           /**< 电流前馈指针 */
    
    /*-------------------------- 控制量 --------------------------*/
    float pid_ref;                            /**< PID参考值（设定值） */
    
    /*-------------------------- 状态标志 --------------------------*/
    Motor_Working_Type_e stop_flag;           /**< 电机使能/停止标志 */
    
    /*-------------------------- CAN通信 --------------------------*/
    CANInstance *motor_can_instace;           /**< CAN通信实例 */
    
    /*-------------------------- 诊断数据 --------------------------*/
    uint32_t lost_cnt;                        /**< 通信丢失计数（预留） */
} DMMotorInstance;

/* ================================ 枚举定义 ================================ */

/**
 * @brief DM电机模式命令枚举
 * 
 * 功能说明:
 *   定义DM电机的各种控制命令
 *   通过DMMotorSetMode()函数发送给电机
 * 
 * 命令说明:
 *   - DM_CMD_MOTOR_MODE: 使能电机，切换到力控模式
 *   - DM_CMD_RESET_MODE: 重置电机，停止输出
 *   - DM_CMD_ZERO_POSITION: 设置当前位置为零位
 *   - DM_CMD_CLEAR_ERROR: 清除错误状态
 * 
 * 使用示例:
 *   @code
 *   // 使能电机
 *   DMMotorSetMode(DM_CMD_MOTOR_MODE, motor);
 *   
 *   // 停止电机
 *   DMMotorSetMode(DM_CMD_RESET_MODE, motor);
 *   
 *   // 校准零位
 *   DMMotorSetMode(DM_CMD_ZERO_POSITION, motor);
 *   @endcode
 * 
 * @see DMMotorSetMode() - 发送模式命令
 */
typedef enum
{
    DM_CMD_MOTOR_MODE = 0xfc,       /**< 使能电机，进入力控模式 */
    DM_CMD_RESET_MODE = 0xfd,       /**< 重置电机，停止输出 */
    DM_CMD_ZERO_POSITION = 0xfe,    /**< 设置当前位置为零位 */
    DM_CMD_CLEAR_ERROR = 0xfb       /**< 清除电机错误状态 */
} DMMotor_Mode_e;

/* ================================ 函数接口声明 ================================ */

/**
 * @brief DM电机初始化函数
 * 
 * 功能:
 *   初始化一个DM电机实例
 * 
 * 参数:
 *   @param config 电机初始化配置结构体指针
 *                  包含:
 *                  - motor_type: 电机类型
 *                  - can_init_config: CAN配置（CAN句柄、TX ID）
 *                  - controller_setting_init_config: 控制设置
 *                  - controller_param_init_config: PID参数
 * 
 * 返回值:
 *   @return DMMotorInstance* 电机实例指针
 * 
 * 使用示例:
 *   @code
 *   Motor_Init_Config_s config = {
 *       .motor_type = M3508,
 *       .can_init_config = {
 *           .can_handle = &hcan1,
 *           .tx_id = 1
 *       },
 *       .controller_setting_init_config = {
 *           .outer_loop_type = CURRENT_LOOP,
 *           .close_loop_type = CURRENT_LOOP,
 *           .motor_reverse_flag = MOTOR_DIRECTION_NORMAL
 *       },
 *       .controller_param_init_config = {
 *           .current_PID = {
 *               .Kp = 10,
 *               .Ki = 0.5,
 *               .Kd = 0,
 *               .MaxOut = 16384
 *           }
 *       }
 *   };
 *   DMMotorInstance *motor = DMMotorInit(&config);
 *   @endcode
 * 
 * @see DMMotorSetRef() - 设置控制量
 * @see DMMotorControl() - 控制主函数
 */
DMMotorInstance *DMMotorInit(Motor_Init_Config_s *config);

/**
 * @brief 设置电机参考值
 * 
 * 功能:
 *   设置电机的目标控制量（力矩）
 * 
 * 参数:
 *   @param motor 电机实例指针
 *   @param ref 控制参考值（N·m）
 * 
 * @see DMMotorControl() - 发送控制指令
 */
void DMMotorSetRef(DMMotorInstance *motor, float ref);

/**
 * @brief 设置电机外环闭环类型
 * 
 * 功能:
 *   切换电机的主要控制闭环（预留，当前版本未使用）
 * 
 * 参数:
 *   @param motor 电机实例指针
 *   @param closeloop_type 闭环类型
 * 
 * @note 当前版本仅支持力控模式
 */
void DMMotorOuterLoop(DMMotorInstance *motor, Closeloop_Type_e closeloop_type);

/**
 * @brief 使能电机
 * 
 * 功能:
 *   将电机设置为使能状态，允许响应控制指令
 * 
 * 参数:
 *   @param motor 电机实例指针
 * 
 * @see DMMotorStop() - 停止电机
 */
void DMMotorEnable(DMMotorInstance *motor);

/**
 * @brief 停止电机
 * 
 * 功能:
 *   将电机设置为停止状态，不响应控制指令
 * 
 * 参数:
 *   @param motor 电机实例指针
 * 
 * @see DMMotorEnable() - 使能电机
 */
void DMMotorStop(DMMotorInstance *motor);

/**
 * @brief 校准电机编码器
 * 
 * 功能:
 *   将电机当前位置设置为机械零位
 * 
 * 参数:
 *   @param motor 电机实例指针
 * 
 * @note 校准后，当前位置将变为0
 */
void DMMotorCaliEncoder(DMMotorInstance *motor);

/**
 * @brief DM电机控制主函数
 * 
 * 功能:
 *   电机控制主循环，需要周期性调用
 *   遍历所有注册的电机，发送控制指令
 * 
 * 调用周期:
 *   推荐1-5ms（100-500Hz）
 * 
 * @note 需要在主循环或定时器中周期性调用
 */
void DMMotorControl(void);

#endif /* DMMOTOR_H */

/* ================================ 文件结束 ================================ */
