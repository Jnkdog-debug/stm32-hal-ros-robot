#include "motor_task.h"
#include "robot_sys.h"
#include "encodermotor.h"
#include "queue.h"
#include "cmsis_os.h" 
#include "adc.h"

// 引入外部定义的电机实例
extern MotorControl_t* Mecanum_Motors[4]; 
extern MotorControl_t Motor_A; // 左前轮 (FL)
extern MotorControl_t Motor_B; // 右前轮 (FR)
extern MotorControl_t Motor_C; // 左后轮 (RL)
extern MotorControl_t Motor_D; // 右后轮 (RR)

// ==========================================
// 🤖 麦克纳姆轮底盘物理参数
// ==========================================
// 根据你的 Python 脚本：直径 0.047m
#define WHEEL_RADIUS    0.0235f  
// 假设轮距与轴距均为 0.15m，因此一半为 0.075m
#define HALF_TRACK      0.075f   
#define HALF_WHEELBASE  0.075f   
// 麦轮几何常数 K = Lx + Ly
#define K_FACTOR        (HALF_TRACK + HALF_WHEELBASE)
// 将线速度 (m/s) 转换为 RPM 的比例常数
// RPM = V / (2 * PI * R) * 60
#define VEL_TO_RPM      (60.0f / (2.0f * 3.14159265f * WHEEL_RADIUS))

// 10ms 控制周期 (100Hz)
#define MOTOR_CTRL_PERIOD_MS 10

// ADC 电池电压采样相关定义
#define ADC_REF_VOLTAGE 3.3f        
#define ADC_RESOLUTION 4095.0f      
#define VOLTAGE_DIVIDER_RATIO 11.0f  
#define ADC_SAMPLE_COUNT 10         

/**
 * @brief ADC 采样电池电压（带滤波）
 * @return 电池电压 (V)
 */
static float ADC_GetBatteryVoltage(void)
{
    uint32_t adc_sum = 0;
    
    for (int i = 0; i < ADC_SAMPLE_COUNT; i++) {
        if (HAL_ADC_Start(&hadc1) != HAL_OK) return 0.0f;  
        if (HAL_ADC_PollForConversion(&hadc1, 1000) != HAL_OK) return 0.0f;  
        adc_sum += HAL_ADC_GetValue(&hadc1);
        HAL_ADC_Stop(&hadc1);
    }
    
    uint32_t adc_avg = adc_sum / ADC_SAMPLE_COUNT;
    float measured_voltage = (adc_avg / ADC_RESOLUTION) * ADC_REF_VOLTAGE;
    return measured_voltage * VOLTAGE_DIVIDER_RATIO;
}

void StartmotorTask(void *argument)
{
    // --- 1. 初始化 4 个电机 ---
    for (int i = 0; i < 4; i++) {
        Motor_Init(Mecanum_Motors[i]);
        Motor_Encoder_Init(Mecanum_Motors[i]);
    }

    uint8_t debug_counter = 0;
    uint8_t adc_sample_counter = 0;
    TickType_t xLastWakeTime = xTaskGetTickCount();

    // 局部变量：全局空间速度和电池电压
    float vx = 0.0f, vy = 0.0f, wz = 0.0f;
    uint32_t last_time = 0;
    float battery_voltage = 0.0f;

    for(;;)
    {
        // --- 2. 线程安全地读取上位机指令 ---
        osMutexAcquire(robotMutexHandle, osWaitForever);
        vx = g_robot.target_Vx;
        vy = g_robot.target_Vy;
        wz = g_robot.target_Wz;
        last_time = g_robot.last_cmd_time;
        osMutexRelease(robotMutexHandle);

        // --- 致命级安全锁：断联看门狗 (Timeout Failsafe) ---
        // 如果超过 500ms 没收到香橙派的新指令，强制刹车
        if ((xTaskGetTickCount() - last_time) > pdMS_TO_TICKS(500)) {
            vx = 0.0f; vy = 0.0f; wz = 0.0f;
        }

        // --- 3. 麦克纳姆轮逆运动学解算 ---
        // 计算 4 个轮子的目标线速度 (m/s)
        float v_fl = vx - vy - wz * K_FACTOR; // Motor_A
        float v_fr = vx + vy + wz * K_FACTOR; // Motor_B
        float v_rl = vx + vy - wz * K_FACTOR; // Motor_C
        float v_rr = vx - vy + wz * K_FACTOR; // Motor_D

        // 转换为目标 RPM
        float target_rpm[4];
        target_rpm[0] = v_fl * VEL_TO_RPM;
        target_rpm[1] = v_fr * VEL_TO_RPM;
        target_rpm[2] = v_rl * VEL_TO_RPM;
        target_rpm[3] = v_rr * VEL_TO_RPM;


        // --- 4. 刷新编码器数据 (计算位移 + 当前转速) ---
        for (int i = 0; i < 4; i++) {
            Motor_Encoder_Update(Mecanum_Motors[i]); 
        }    

        // --- 5. 每隔5个周期（50ms）采样一次电池电压 ---
        if (++adc_sample_counter >= 5) {
            adc_sample_counter = 0;
            battery_voltage = ADC_GetBatteryVoltage();
            
            osMutexAcquire(robotMutexHandle, osWaitForever);
            g_robot.battery_voltage = battery_voltage;
            osMutexRelease(robotMutexHandle);
        }

        // --- 6. 优雅的 4 路 PID 计算与电机执行 ---
        for (int i = 0; i < 4; i++) {
            int16_t out_pwm = 0;
            
            // 停机逻辑：目标与当前均为 0 时关断输出
            if (target_rpm[i] == 0 && Mecanum_Motors[i]->duty_rpm == 0) {
                out_pwm = 0;
                Mecanum_Motors[i]->pid_out = 0; 
                Mecanum_Motors[i]->pid_last_error = 0; 
                Mecanum_Motors[i]->pid_prev_error = 0;
            } else {
                out_pwm = Incremental_PID(Mecanum_Motors[i], target_rpm[i]);
            }
            
            Motor_SetPWM(Mecanum_Motors[i], out_pwm);
        }

        // --- 7. 发送调试数据到串口 ---
        // 10ms (100Hz) 发送一次
        if (++debug_counter >= 1) 
        {
            debug_counter = 0;
            MotorData_t* p_data = pvPortMalloc(sizeof(MotorData_t));
            if (p_data != NULL)
            {
                for (int i = 0; i < 4; i++) {
                    p_data->rpm[i]   = Mecanum_Motors[i]->duty_rpm;
                    p_data->count[i] = Mecanum_Motors[i]->total_count;
                }
                p_data->battery_voltage = battery_voltage;

                TxMessage_t msg;
                msg.type = MSG_TYPE_MOTOR;
                msg.p_data = p_data;
                
                if (xQueueSend(dataQueueHandle, &msg, 0) != pdPASS) {
                    vPortFree(p_data);
                }
            }
        }

        // --- 8. 等待下一个 10ms 周期 ---
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(MOTOR_CTRL_PERIOD_MS));
    }
}




// void StartmotorTask(void *argument)
// {
//     // --- 0. 极其关键：显式绑定指针数组！(防止编译器优化导致野指针) ---
//     Mecanum_Motors[0] = &Motor_A;
//     Mecanum_Motors[1] = &Motor_B;
//     Mecanum_Motors[2] = &Motor_C;
//     Mecanum_Motors[3] = &Motor_D;

//     // --- 1. 初始化 4 个电机 ---
//     for (int i = 0; i < 4; i++) {
//         Motor_Init(Mecanum_Motors[i]);
//         Motor_Encoder_Init(Mecanum_Motors[i]);
//     }

//     uint8_t debug_counter = 0;
//     uint8_t adc_sample_counter = 0;
//     TickType_t xLastWakeTime = xTaskGetTickCount();

//     // 局部变量：全局空间速度和电池电压
//     float vx = 0.0f, vy = 0.0f, wz = 0.0f;
//     uint32_t last_time = 0;
//     float battery_voltage = 0.0f;

//     for(;;)
//     {
//         // --- 2. 线程安全地读取上位机指令 ---
//         osMutexAcquire(robotMutexHandle, osWaitForever);
//         vx = g_robot.target_Vx;
//         vy = g_robot.target_Vy;
//         wz = g_robot.target_Wz;
//         last_time = g_robot.last_cmd_time;
//         osMutexRelease(robotMutexHandle);

//         // --- 致命级安全锁：断联看门狗 (Timeout Failsafe) ---
//         if ((xTaskGetTickCount() - last_time) > pdMS_TO_TICKS(500)) {
//             vx = 0.0f; vy = 0.0f; wz = 0.0f;
//         }

//         // --- 3. 刷新编码器数据 (计算位移 + 当前转速) ---
//         for (int i = 0; i < 4; i++) {
//             Motor_Encoder_Update(Mecanum_Motors[i]); 
//         }    

//         // --- 4. 每隔5个周期（50ms）采样一次电池电压 ---
//         if (++adc_sample_counter >= 5) {
//             adc_sample_counter = 0;
//             battery_voltage = ADC_GetBatteryVoltage();
            
//             osMutexAcquire(robotMutexHandle, osWaitForever);
//             g_robot.battery_voltage = battery_voltage;
//             osMutexRelease(robotMutexHandle);
//         }

//         // ==========================================================
//         // --- 5. 【纯硬件开环测试模式】剥离 PID，直接下发 PWM ---
//         // ==========================================================
        
        

//         Motor_SetPWM(Mecanum_Motors[0], 700);
//         Motor_SetPWM(Mecanum_Motors[1], 700);
//         Motor_SetPWM(Mecanum_Motors[2], 700);
//         Motor_SetPWM(Mecanum_Motors[3], 700);




//         // ==========================================================

//         // --- 6. 发送调试数据到串口 ---
//         if (++debug_counter >= 1) 
//         {
//             debug_counter = 0;
//             MotorData_t* p_data = pvPortMalloc(sizeof(MotorData_t));
//             if (p_data != NULL)
//             {
//                 for (int i = 0; i < 4; i++) {
//                     p_data->rpm[i]   = Mecanum_Motors[i]->duty_rpm;
//                     p_data->count[i] = Mecanum_Motors[i]->total_count;
//                 }
//                 p_data->battery_voltage = battery_voltage;

//                 TxMessage_t msg;
//                 msg.type = MSG_TYPE_MOTOR;
//                 msg.p_data = p_data;
                
//                 if (xQueueSend(dataQueueHandle, &msg, 0) != pdPASS) {
//                     vPortFree(p_data);
//                 }
//             }
//         }

//         // --- 7. 等待下一个 10ms 周期 ---
//         vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(MOTOR_CTRL_PERIOD_MS));
//     }
// }