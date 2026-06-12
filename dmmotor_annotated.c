/**
 * ====================================================================================
 * 
 * 文件名: dmmotor.c
 * 
 * 功能: DM系列电机（Dynamixel协议电机）驱动实现
 * 
 * 描述: 
 *   本文件实现了对DM系列力控电机的完整驱动，包括：
 *   - 电机初始化与配置
 *   - CAN通信协议封装
 *   - 电机反馈数据解析
 *   - 力矩控制模式实现
 *   - 编码器校准功能
 * 
 * 适用电机: DM系列电机（如DM4346、DM6215等使用Dynamixel协议的电机）
 * 
 * 通信协议: CAN 2.0A, 标准帧, Dynamixel Protocol
 * 
 * 作者: 基于DJI Motor SDK架构修改
 * 
 * ====================================================================================
 */

#include "dmmotor.h"      // DM电机驱动头文件，定义电机类型、结构体和函数接口
#include "memory.h"        // 内存管理相关函数（malloc等）
#include "general_def.h"   // 通用定义（可能是通用工具函数）
#include "user_lib.h"      // 用户自定义库函数
#include "string.h"        // 字符串处理函数（memset等）
#include "stdlib.h"        // 标准库函数（malloc等）
#include "bsp_log.h"      // 日志系统头文件
#include "bsp_dwt.h"       // DWT定时器头文件，用于精确延时

/* ================================ 全局变量定义 ================================ */

/**
 * @brief 电机实例索引计数器
 * 
 * 功能说明:
 *   - 用于追踪已注册的电机数量
 *   - 每当成功注册一个新电机时自动递增
 *   - 初始化为0，表示还没有电机注册
 * 
 * 注意事项:
 *   - 最大支持数量由 DM_MOTOR_CNT 定义（当前为4）
 *   - 不可超过最大数量，否则会导致数组越界
 */
static uint8_t idx;

/**
 * @brief DM电机实例指针数组
 * 
 * 功能说明:
 *   - 存储所有已注册的DM电机实例指针
 *   - 通过索引idx管理电机实例
 *   - 每个电机独立发送CAN报文（与DJI电机的分组发送不同）
 * 
 * 内存管理:
 *   - 内存由malloc在DMMotorInit中动态分配
 *   - 每个电机实例占用一个DMMotorInstance结构体大小
 * 
 * @see DMMotorInit() - 电机初始化函数
 */
static DMMotorInstance *dm_motor_instance[DM_MOTOR_CNT];

/* ================================ 数据转换函数 ================================ */

/**
 * @brief 将浮点数转换为无符号整数值
 * 
 * 函数功能:
 *   将给定范围的浮点数值映射到指定位数的无符号整数值
 *   这是CAN通信协议中发送数据的编码方式
 * 
 * 算法说明:
 *   公式: result = (x - offset) * (2^bits - 1) / span
 *   其中:
 *   - span = x_max - x_min (数值范围)
 *   - offset = x_min (最小值偏移)
 *   - 2^bits - 1 = 最大编码值（如16位为65535）
 * 
 * 参数说明:
 *   @param x       待转换的浮点数值
 *   @param x_min   浮点数值的最小值（下限）
 *   @param x_max   浮点数值的最大值（上限）
 *   @param bits    编码位数（决定精度）
 * 
 * 返回值:
 *   @return uint16_t 转换后的无符号整数值
 * 
 * 使用场景:
 *   - 位置指令编码 (16位)
 *   - 速度指令编码 (12位)
 *   - 力矩指令编码 (12位)
 * 
 * @note 不同数据使用不同位数是为了在有限的8字节CAN数据中
 *       平衡各参数的精度需求
 */
static uint16_t float_to_uint(float x, float x_min, float x_max, uint8_t bits)
{
    float span = x_max - x_min;       // 计算数值范围
    float offset = x_min;              // 保存最小值作为偏移量
    // 应用映射公式：(x - offset) * max_code / span
    return (uint16_t)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

/**
 * @brief 将无符号整数值转换为浮点数
 * 
 * 函数功能:
 *   将无符号整数值反向映射回原始物理量
 *   这是解析CAN反馈数据时的解码方式
 * 
 * 算法说明:
 *   公式: result = x_int * span / max_code + offset
 *   这是float_to_uint的逆运算
 * 
 * 参数说明:
 *   @param x_int   待转换的无符号整数值
 *   @param x_min   浮点数值的最小值（下限）
 *   @param x_max   浮点数值的最大值（上限）
 *   @param bits    编码位数（必须与编码时一致）
 * 
 * 返回值:
 *   @return float 转换后的浮点数值（物理量）
 * 
 * 使用场景:
 *   - 解析电机反馈的位置值
 *   - 解析电机反馈的速度值
 *   - 解析电机反馈的力矩值
 * 
 * @warning 必须确保bits参数与编码时一致，否则结果错误
 */
static float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;       // 计算数值范围
    float offset = x_min;              // 保存最小值作为偏移量
    // 应用反向映射公式：x_int * span / max_code + offset
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

/* ================================ 电机模式控制 ================================ */

/**
 * @brief 设置DM电机的工作模式
 * 
 * 函数功能:
 *   向电机发送模式设置命令
 *   DM电机支持多种工作模式，通过不同的命令码切换
 * 
 * CAN报文格式说明:
 *   - 前7字节 (0-6): 填充0xFF（DM电机协议规定）
 *   - 第8字节 (7): 命令码
 * 
 * 报文发送:
 *   - 使用电机实例绑定的CAN通道发送
 *   - 超时时间1ms
 * 
 * 参数说明:
 *   @param cmd   电机模式命令（见DMMotor_Mode_e枚举）
 *   @param motor 电机实例指针
 * 
 * 常用命令:
 *   - DM_CMD_MOTOR_MODE (0xFC): 使能电机，进入力控模式
 *   - DM_CMD_RESET_MODE (0xFD): 停止电机
 *   - DM_CMD_ZERO_POSITION (0xFE): 设置当前位置为零位
 *   - DM_CMD_CLEAR_ERROR (0xFB): 清除错误状态
 * 
 * @see DMMotor_Mode_e - 模式命令枚举定义
 * @see CANTransmit() - CAN发送函数
 */
static void DMMotorSetMode(DMMotor_Mode_e cmd, DMMotorInstance *motor)
{
    // 将发送缓冲区前7字节填充为0xFF（DM协议规定）
    memset(motor->motor_can_instace->tx_buff, 0xff, 7);
    // 第8字节写入命令码
    motor->motor_can_instace->tx_buff[7] = (uint8_t)cmd;
    // 通过CAN发送报文
    CANTransmit(motor->motor_can_instace, 1);
}

/* ================================ 数据解析函数 ================================ */

/**
 * @brief 解析DM电机的CAN反馈数据
 * 
 * 函数功能:
 *   解析电机返回的CAN报文，提取电机状态信息
 *   包括位置、速度、力矩和温度数据
 * 
 * CAN反馈报文格式（8字节）:
 *   Byte 0: 电机ID
 *   Byte 1-2: 位置数据（16位，Big Endian）
 *   Byte 3-4: 速度数据（12位，跨字节存储）
 *   Byte 5: 力矩数据低8位
 *   Byte 6: MOS温度
 *   Byte 7: 转子温度
 * 
 * 数据解码算法:
 *   - 位置: 16位无符号，直接组合
 *   - 速度: 12位，低4位在Byte3，高8位在Byte4
 *   - 力矩: 12位，低4位在Byte4，高8位在Byte5
 * 
 * 参数说明:
 *   @param motor_can CAN实例指针（包含接收到的数据）
 * 
 * 内部处理:
 *   1. 从CAN实例获取电机实例指针
 *   2. 解析各字节数据
 *   3. 通过uint_to_float转换为物理量
 *   4. 更新电机状态结构体
 * 
 * @attention 此函数作为回调函数被CAN接收中断调用，
 *           需要尽可能高效，避免耗时操作
 * 
 * @see uint_to_float() - 数据解码函数
 * @see CANFIFOxCallback() - CAN接收中断回调
 */
static void DMMotorDecode(CANInstance *motor_can)
{
    uint16_t tmp;  // 临时变量，用于暂存解析的原始值
    
    // 获取接收缓冲区和电机实例指针
    uint8_t *rxbuff = motor_can->rx_buff;
    DMMotorInstance *motor = (DMMotorInstance *)motor_can->id;
    DM_Motor_Measure_s *measure = &(motor->measure);

    // 保存上一次的位置值（用于计算位置变化）
    measure->last_position = measure->position;
    
    /*-------------------------- 位置数据解析 --------------------------*/
    // 位置数据: Byte1(高8位) << 8 | Byte2(低8位)
    // 使用16位无符号整数接收，然后通过映射转换为物理量
    tmp = (uint16_t)((rxbuff[1] << 8) | rxbuff[2]);
    // 将原始编码值转换为实际位置（单位：弧度或角度，视电机而定）
    measure->position = uint_to_float(tmp, DM_P_MIN, DM_P_MAX, 16);

    /*-------------------------- 速度数据解析 --------------------------*/
    // 速度数据: 12位跨字节存储
    // Byte3的bit7-bit4 (4位) 作为高位
    // Byte4的全部8位作为低位
    tmp = (uint16_t)((rxbuff[3] << 4) | rxbuff[4] >> 4);
    // 将原始编码值转换为实际速度（单位：rad/s或rpm）
    measure->velocity = uint_to_float(tmp, DM_V_MIN, DM_V_MAX, 12);

    /*-------------------------- 力矩数据解析 --------------------------*/
    // 力矩数据: 12位跨字节存储
    // Byte4的bit3-bit0 (4位) 作为高位
    // Byte5的全部8位作为低位
    tmp = (uint16_t)(((rxbuff[4] & 0x0f) << 8) | rxbuff[5]);
    // 将原始编码值转换为实际力矩（单位：N·m）
    measure->torque = uint_to_float(tmp, DM_T_MIN, DM_T_MAX, 12);

    /*-------------------------- 温度数据解析 --------------------------*/
    // MOS管温度: 直接取Byte6
    measure->T_Mos = (float)rxbuff[6];
    // 转子温度: 直接取Byte7
    measure->T_Rotor = (float)rxbuff[7];
}

/* ================================ 初始化与配置 ================================ */

/**
 * @brief 校准电机编码器（设置零位）
 * 
 * 函数功能:
 *   将电机当前位置设置为机械零位
 *   执行后，电机的当前位置将变为0
 * 
 * 使用场景:
 *   - 初始化时校准零位
 *   - 机械装配后需要重新校准
 *   - 更换编码器后需要校准
 * 
 * 实现说明:
 *   - 发送DM_CMD_ZERO_POSITION命令
 *   - 等待100ms让命令生效
 * 
 * 参数说明:
 *   @param motor 电机实例指针
 * 
 * @see DMMotorSetMode() - 模式设置函数
 * @see DWT_Delay() - DWT延时函数
 */
void DMMotorCaliEncoder(DMMotorInstance *motor)
{
    // 发送零位校准命令
    DMMotorSetMode(DM_CMD_ZERO_POSITION, motor);
    // 等待100ms让命令生效
    DWT_Delay(0.1);
}

/**
 * @brief DM电机初始化函数
 * 
 * 函数功能:
 *   初始化一个DM电机实例，完成以下工作：
 *   1. 分配电机实例内存
 *   2. 初始化PID控制器
 *   3. 注册到CAN总线
 *   4. 使能电机
 *   5. 设置力控模式
 *   6. 校准编码器零位
 * 
 * 初始化流程:
 *   1. malloc分配DMMotorInstance结构体内存
 *   2. 初始化各成员变量为0
 *   3. 配置电机控制设置（闭环类型、反转标志等）
 *   4. 初始化三环PID控制器
 *   5. 设置反馈数据来源指针
 *   6. 注册CAN回调函数和解码器
 *   7. 将电机实例注册到CAN总线
 *   8. 使能电机
 *   9. 发送力控模式命令
 *   10. 延时等待模式切换
 *   11. 校准编码器零位
 *   12. 延时等待校准完成
 *   13. 保存实例到数组
 *   14. 返回实例指针
 * 
 * 参数说明:
 *   @param config 电机初始化配置结构体指针
 *                  包含:
 *                  - motor_type: 电机类型
 *                  - can_init_config: CAN配置（CAN句柄、TX ID）
 *                  - controller_setting_init_config: 控制设置
 *                  - controller_param_init_config: PID参数
 * 
 * 返回值:
 *   @return DMMotorInstance* 电机实例指针
 *          需要保存此指针以便后续控制电机
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
 *           .current_PID = {.Kp = 10, .Ki = 0.5, .Kd = 0, .MaxOut = 16384}
 *       }
 *   };
 *   DMMotorInstance *motor = DMMotorInit(&config);
 *   @endcode
 * 
 * @attention 初始化后电机默认处于力控模式
 * @attention 确保CAN总线和电机供电正常
 * 
 * @see DMMotorSetRef() - 设置电机控制量
 * @see DMMotorControl() - 电机控制主函数
 */
DMMotorInstance *DMMotorInit(Motor_Init_Config_s *config)
{
    /*-------------------------- 内存分配与初始化 --------------------------*/
    // 使用malloc分配电机实例内存
    DMMotorInstance *motor = (DMMotorInstance *)malloc(sizeof(DMMotorInstance));
    // 将内存清零，确保所有成员初始化为0
    memset(motor, 0, sizeof(DMMotorInstance));
    
    /*-------------------------- 电机控制设置 --------------------------*/
    // 配置电机控制设置（闭环类型、反转标志等）
    motor->motor_settings = config->controller_setting_init_config;
    
    /*-------------------------- PID控制器初始化 --------------------------*/
    // 初始化电流环PID
    PIDInit(&motor->current_PID, &config->controller_param_init_config.current_PID);
    // 初始化速度环PID
    PIDInit(&motor->speed_PID, &config->controller_param_init_config.speed_PID);
    // 初始化角度环PID
    PIDInit(&motor->angle_PID, &config->controller_param_init_config.angle_PID);
    
    /*-------------------------- 反馈数据指针设置 --------------------------*/
    // 设置外部角度反馈来源（如果有）
    motor->other_angle_feedback_ptr = config->controller_param_init_config.other_angle_feedback_ptr;
    // 设置外部速度反馈来源（如果有）
    motor->other_speed_feedback_ptr = config->controller_param_init_config.other_speed_feedback_ptr;
    
    /*-------------------------- CAN注册 --------------------------*/
    // 设置CAN接收回调函数（数据解析）
    config->can_init_config.can_module_callback = DMMotorDecode;
    // 设置CAN ID（指向电机实例，用于在回调中识别电机）
    config->can_init_config.id = motor;
    // 注册电机到CAN总线，获取CAN实例指针
    motor->motor_can_instace = CANRegister(&config->can_init_config);

    /*-------------------------- 电机使能与模式设置 --------------------------*/
    // 使能电机（设置工作标志）
    DMMotorEnable(motor);
    // 发送力控模式命令
    DMMotorSetMode(DM_CMD_MOTOR_MODE, motor);
    // 等待100ms让模式切换完成
    DWT_Delay(0.1);
    
    /*-------------------------- 编码器校准 --------------------------*/
    // 校准编码器零位
    DMMotorCaliEncoder(motor);
    // 等待100ms让校准完成
    DWT_Delay(0.1);
    
    /*-------------------------- 保存实例 --------------------------*/
    // 将电机实例添加到数组（idx先作为数组索引，然后自增）
    dm_motor_instance[idx++] = motor;
    // 返回电机实例指针
    return motor;
}

/* ================================ 控制接口函数 ================================ */

/**
 * @brief 设置电机参考值（控制量）
 * 
 * 函数功能:
 *   设置电机的目标控制量
 *   对于DM电机，当前实现为力矩控制
 * 
 * 控制量说明:
 *   - 对于力控模式: 设置目标力矩（N·m）
 *   - 范围: DM_T_MIN ~ DM_T_MAX（-18 ~ 18 N·m）
 * 
 * 参数说明:
 *   @param motor 电机实例指针
 *   @param ref   控制参考值（目标力矩）
 * 
 * 使用示例:
 *   @code
 *   // 设置电机输出5N·m力矩
 *   DMMotorSetRef(motor, 5.0f);
 *   
 *   // 设置电机输出反向10N·m力矩
 *   DMMotorSetRef(motor, -10.0f);
 *   @endcode
 * 
 * @note 此函数仅设置参考值，实际发送需要调用DMMotorControl()
 * 
 * @see DMMotorControl() - 执行控制循环，发送CAN报文
 */
void DMMotorSetRef(DMMotorInstance *motor, float ref)
{
    // 直接将参考值存入电机实例的pid_ref成员
    motor->pid_ref = ref;
}

/**
 * @brief 使能电机
 * 
 * 函数功能:
 *   将电机设置为使能状态，允许响应控制指令
 * 
 * 使能后的效果:
 *   - 电机将响应设定的控制量
 *   - 可以正常发送力矩指令
 * 
 * 参数说明:
 *   @param motor 电机实例指针
 * 
 * @see DMMotorStop() - 停止电机
 * @see DMMotorControl() - 电机控制主函数
 */
void DMMotorEnable(DMMotorInstance *motor)
{
    // 设置电机工作状态为使能
    motor->stop_flag = MOTOR_ENALBED;
}

/**
 * @brief 停止电机
 * 
 * 函数功能:
 *   将电机设置为停止状态，不响应控制指令
 * 
 * 停止后的效果:
 *   - 电机不再响应任何控制量
 *   - 电机处于自由状态（可手动转动）
 *   - 电机仍会接收反馈数据
 * 
 * 与失能的区别:
 *   - 停止: 软停止，控制量置零，电机可以自由转动
 *   - 失能: 通常指硬件使能关闭，电机处于高阻态
 * 
 * 参数说明:
 *   @param motor 电机实例指针
 * 
 * @see DMMotorEnable() - 使能电机
 * @see DMMotorControl() - 电机控制主函数
 */
void DMMotorStop(DMMotorInstance *motor)
{
    // 设置电机工作状态为停止
    motor->stop_flag = MOTOR_STOP;
}

/**
 * @brief 设置电机外环闭环类型
 * 
 * 函数功能:
 *   切换电机的主要控制闭环类型
 *   用于实现不同控制模式（位置环、速度环等）
 * 
 * 当前DM电机实现说明:
 *   - 当前版本仅实现力控模式
 *   - 外环类型参数暂时未使用
 *   - PID控制器已保留，可扩展多环控制
 * 
 * 参数说明:
 *   @param motor 电机实例指针
 *   @param type  闭环类型（见Closeloop_Type_e枚举）
 *                - ANGLE_LOOP: 位置环（外环）
 *                - SPEED_LOOP: 速度环（外环）
 *                - CURRENT_LOOP: 电流环（外环）
 * 
 * 使用示例:
 *   @code
 *   // 切换到位置环控制
 *   DMMotorOuterLoop(motor, ANGLE_LOOP);
 *   
 *   // 切换到速度环控制
 *   DMMotorOuterLoop(motor, SPEED_LOOP);
 *   @endcode
 * 
 * @note 此函数可用于未来扩展多闭环控制
 * @see Closeloop_Type_e - 闭环类型枚举
 */
void DMMotorOuterLoop(DMMotorInstance *motor, Closeloop_Type_e type)
{
    // 设置电机外环闭环类型
    motor->motor_settings.outer_loop_type = type;
}

/* ================================ 电机控制主函数 ================================ */

/**
 * @brief DM电机控制主函数
 * 
 * 函数功能:
 *   电机控制的主循环函数，需要周期性调用
 *   完成以下工作：
 *   1. 遍历所有注册的电机
 *   2. 读取设定值并进行处理
 *   3. 编码CAN发送数据
 *   4. 通过CAN发送控制指令
 * 
 * 调用周期:
 *   - 推荐调用周期: 1-5ms（100-500Hz）
 *   - 具体周期取决于应用需求和CAN总线负载
 * 
 * 控制模式说明:
 *   当前版本采用开环力矩控制:
 *   - 直接将设定值作为力矩指令发送
 *   - 未使用PID闭环控制
 *   - 位置和速度设定值均设为0（保持当前状态）
 * 
 * CAN发送报文格式（8字节）:
 *   Byte 0-1: 目标位置 (16位, 设为0)
 *   Byte 2-3: 目标速度 (12位, 设为0)
 *   Byte 4-5: 目标力矩 (12位, 主控制量)
 *   Byte 6-7: Kp/Kd参数 (保留)
 * 
 * @attention 此函数需要周期性调用，建议在定时器中断或RTOS任务中调用
 * @attention 多电机时注意CAN发送频率，避免邮箱溢出
 * 
 * 调用示例:
 *   @code
 *   // 在main函数的循环中调用
 *   while (1) {
 *       DMMotorControl();
 *       HAL_Delay(2);  // 2ms周期
 *   }
 *   @endcode
 * 
 * @see DMMotorSetRef() - 设置电机控制量
 * @see CANTransmit() - CAN发送函数
 */
void DMMotorControl(void)
{
    /*-------------------------- 变量定义 --------------------------*/
    float pid_ref, set;                    // pid_ref: 设定值, set: 处理后的设定值
    DMMotorInstance *motor;                // 电机实例指针
    Motor_Control_Setting_s *setting;      // 电机控制设置指针
    DMMotor_Send_s motor_send_mailbox;     // CAN发送数据结构体

    /*-------------------------- 遍历所有电机 --------------------------*/
    // 遍历dm_motor_instance数组中的所有已注册电机
    for (size_t i = 0; i < idx; i++)
    {
        /*-------------------------- 获取电机实例 --------------------------*/
        motor = dm_motor_instance[i];           // 获取电机实例指针
        setting = &motor->motor_settings;       // 获取电机控制设置指针

        /*-------------------------- 读取设定值 --------------------------*/
        pid_ref = motor->pid_ref;               // 从电机实例读取设定值
        set = pid_ref;                          // 保存到本地变量

        /*-------------------------- 方向处理 --------------------------*/
        // 检查是否需要反转电机方向
        if (setting->motor_reverse_flag == MOTOR_DIRECTION_REVERSE)
            set *= -1;                          // 反转设定值

        /*-------------------------- 限幅处理 --------------------------*/
        // 将设定值限制在电机允许的范围内
        LIMIT_MIN_MAX(set, DM_T_MIN, DM_T_MAX);

        /*-------------------------- 构造发送数据 --------------------------*/
        // 目标位置: 设为0，保持当前位置
        motor_send_mailbox.position_des = float_to_uint(0, DM_P_MIN, DM_P_MAX, 16);
        
        // 目标速度: 设为0，保持当前速度
        motor_send_mailbox.velocity_des = float_to_uint(0, DM_V_MIN, DM_V_MAX, 12);
        
        // 目标力矩: 使用设定值（主要控制量）
        motor_send_mailbox.torque_des = float_to_uint(set, DM_T_MIN, DM_T_MAX, 12);
        
        // Kp/Kd参数: 设为0（当前版本未使用）
        motor_send_mailbox.Kp = 0;
        motor_send_mailbox.Kd = 0;

        /*-------------------------- 停止状态处理 --------------------------*/
        // 如果电机处于停止状态，将力矩设为0
        if (motor->stop_flag == MOTOR_STOP)
            motor_send_mailbox.torque_des = float_to_uint(0, DM_T_MIN, DM_T_MAX, 12);

        /*-------------------------- 填充CAN发送缓冲区 --------------------------*/
        // Byte 0: 位置高8位
        motor->motor_can_instace->tx_buff[0] = (uint8_t)(motor_send_mailbox.position_des >> 8);
        // Byte 1: 位置低8位
        motor->motor_can_instace->tx_buff[1] = (uint8_t)(motor_send_mailbox.position_des);
        
        // Byte 2: 速度高8位
        motor->motor_can_instace->tx_buff[2] = (uint8_t)(motor_send_mailbox.velocity_des >> 4);
        // Byte 3: 速度低4位(bit7-4) + Kp高4位(bit3-0)
        motor->motor_can_instace->tx_buff[3] = (uint8_t)(((motor_send_mailbox.velocity_des & 0xF) << 4) | (motor_send_mailbox.Kp >> 8));
        
        // Byte 4: Kp低8位
        motor->motor_can_instace->tx_buff[4] = (uint8_t)(motor_send_mailbox.Kp);
        
        // Byte 5: Kd高8位
        motor->motor_can_instace->tx_buff[5] = (uint8_t)(motor_send_mailbox.Kd >> 4);
        // Byte 6: Kd低4位(bit7-4) + 力矩高4位(bit11-8)
        motor->motor_can_instace->tx_buff[6] = (uint8_t)(((motor_send_mailbox.Kd & 0xF) << 4) | (motor_send_mailbox.torque_des >> 8));
        
        // Byte 7: 力矩低8位
        motor->motor_can_instace->tx_buff[7] = (uint8_t)(motor_send_mailbox.torque_des);

        /*-------------------------- 发送CAN报文 --------------------------*/
        // 通过CAN发送控制指令
        CANTransmit(motor->motor_can_instace, 10);  // 超时时间10ms
        
        // 多电机时添加短暂延时，避免CAN邮箱溢出
        if (idx > 1 && i < idx - 1) {
            DWT_Delay(0.0005);  // 0.5ms延时
        }
    }
}

/* ================================ 文件结束 ================================ */
