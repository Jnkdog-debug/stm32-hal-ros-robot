
#include "motor_task.h"
#include "robot_sys.h"
#include "encodermotor.h"
#include "queue.h"
#include "cmsis_os.h" // 确保包含 RTOS 头文件
#include "adc.h"

// 引入外部定义的电机实例
extern MotorControl_t Motor_A;

extern MotorControl_t Motor_D;

// 10ms 控制周期 (100Hz)
#define MOTOR_CTRL_PERIOD_MS 10

// ADC 电池电压采样相关定义
#define ADC_REF_VOLTAGE 3.3f        // ADC 参考电压 (V)
#define ADC_RESOLUTION 4095.0f      // 12位ADC分辨率 (2^12 - 1)
#define VOLTAGE_DIVIDER_RATIO 11.0f  // 分压比 (如果用分压电路: R1+R2/R2，例如10K+20K/10K=3)
#define ADC_SAMPLE_COUNT 10         // 采样次数（滤波用）


/**
 * @brief ADC 采样电池电压（带滤波）
 * @return 电池电压 (V)
 */
static float ADC_GetBatteryVoltage(void)
{
    uint32_t adc_sum = 0;
    uint32_t adc_value = 0;
    
    // 采集多次样本进行平均滤波
    for (int i = 0; i < ADC_SAMPLE_COUNT; i++) {
        // 启动 ADC 转换
        if (HAL_ADC_Start(&hadc1) != HAL_OK) {
            return 0.0f;  // 启动失败
        }
        
        // 等待转换完成（超时时间1000ms）
        if (HAL_ADC_PollForConversion(&hadc1, 1000) != HAL_OK) {
            return 0.0f;  // 转换超时
        }
        
        // 获取转换结果
        adc_value = HAL_ADC_GetValue(&hadc1);
        adc_sum += adc_value;
        
        // 停止 ADC
        HAL_ADC_Stop(&hadc1);
    }
    
    // 求平均值
    uint32_t adc_avg = adc_sum / ADC_SAMPLE_COUNT;
    
    // ADC值转换为电压
    // 公式：V_measured = (ADC_AVG / ADC_RESOLUTION) * ADC_REF_VOLTAGE
    //      V_battery = V_measured * VOLTAGE_DIVIDER_RATIO (如有分压)
    float measured_voltage = (adc_avg / ADC_RESOLUTION) * ADC_REF_VOLTAGE;
    float battery_voltage = measured_voltage * VOLTAGE_DIVIDER_RATIO;
    
    return battery_voltage;
}

void StartmotorTask(void *argument)
{
    // --- 1. 初始化 ---
    Motor_Init(&Motor_A);
    Motor_Init(&Motor_D); 
    Motor_Encoder_Init(&Motor_A);
    Motor_Encoder_Init(&Motor_D);

    uint8_t debug_counter = 0;
    uint8_t adc_sample_counter = 0;
    TickType_t xLastWakeTime = xTaskGetTickCount();

    // 局部变量：目标速度和电池电压
    float target_L = 0.0f;
    float target_R = 0.0f;
    float battery_voltage = 0.0f;

    for(;;)
    {
        // --- 2. 获取上位机下方的目标速度 (RPM) ---
        osMutexAcquire(robotMutexHandle, osWaitForever);
        {
            target_L = g_robot.speed_L; // 假设上位机发来的是目标 RPM
            target_R = g_robot.speed_R;
        }
        osMutexRelease(robotMutexHandle);

        // --- 3. 刷新编码器数据 (计算位移 + 当前转速) ---
        // 注意：这里调用你重构后的函数，它会更新 total_count 和 duty_rpm
        Motor_Encoder_Update(&Motor_A); 
        Motor_Encoder_Update(&Motor_D);
    
        // --- 3.5 每隔5个周期（50ms）采样一次电池电压 ---
        if (++adc_sample_counter >= 5) {
            adc_sample_counter = 0;
            battery_voltage = ADC_GetBatteryVoltage();
            
            // 更新全局变量中的电池电压
            osMutexAcquire(robotMutexHandle, osWaitForever);
            {
                g_robot.battery_voltage = battery_voltage;
            }
            osMutexRelease(robotMutexHandle);
        }

        // --- 4. PID 闭环计算 ---
        int16_t out_pwm_L = 0;
        int16_t out_pwm_R = 0;

        if (target_L == 0 && target_R == 0) {
            // 停机逻辑：如果目标是0，直接关断输出并清空PID，防止电机震荡
            out_pwm_L = 0;
            out_pwm_R = 0;
            Motor_A.pid_out = 0; Motor_A.pid_last_error = 0; Motor_A.pid_prev_error = 0;
            Motor_D.pid_out = 0; Motor_D.pid_last_error = 0; Motor_D.pid_prev_error = 0;
        } else {
            // 运行 PID
            out_pwm_L = Incremental_PID(&Motor_A, target_L);
            out_pwm_R = Incremental_PID(&Motor_D, target_R);
        }

         // --- 5. 执行硬件控制 ---
        Motor_SetPWM(&Motor_A, out_pwm_L);
        Motor_SetPWM(&Motor_D, out_pwm_R);

              // --- 5. 执行硬件控制 ---
        //Motor_SetPWM(&Motor_A, 1000);
        //Motor_SetPWM(&Motor_D, 1000);



        // --- 6. 更新全局状态 (给串口发送任务使用) ---
        osMutexAcquire(robotMutexHandle, osWaitForever);
        {
            g_robot.encoder_L = Motor_A.total_count; // 发送总脉冲，上位机算里程
            g_robot.encoder_R = Motor_D.total_count;
            // 顺便记录一下当前实际速度，方便在香橙派上 debug PID 曲线
            g_robot.current_speed_L = Motor_A.duty_rpm; 
            g_robot.current_speed_R = Motor_D.duty_rpm;
        }
        osMutexRelease(robotMutexHandle);
        
        // --- 7. 发送调试数据到串口 ---
        if (++debug_counter >= 1) // 100ms 发送一次
        {
            debug_counter = 0;
            MotorData_t* p_data = pvPortMalloc(sizeof(MotorData_t));
            if (p_data != NULL)
            {
                p_data->speed_L = Motor_A. duty_rpm;// 给上位机发脉冲总数
                p_data->speed_R = Motor_D.duty_rpm;
                
                p_data->count_L   = Motor_A.total_count;
                p_data->count_R   = Motor_D.total_count;
                p_data->battery_voltage = battery_voltage;  // 上报电池电压


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