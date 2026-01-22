#include "main.h"
#include "math.h"

PID Force_Pid = {
    .Kp = 0.1,
    .Ki = 0.02,
    .Kd = 0.005,
    .error = 0.0,
    .prev_error = 0.0,
    .prev_prev_error = 0.0,
    .output = 0.0,
    .target = 0.0
};

void pid(float CH1, float CH2){
    float Force_data = (CH1 + CH2) / 2.0f;
    Force_Pid.error = Force_Pid.target - Force_data;
    Force_Pid.output = Force_Pid.output + Force_Pid.Kp * Force_Pid.error + Force_Pid.Ki * Force_Pid.prev_error + Force_Pid.Kd * (Force_Pid.error - 2 * Force_Pid.prev_error + Force_Pid.prev_prev_error);
    Force_Pid.prev_prev_error = Force_Pid.prev_error;
    Force_Pid.prev_error = Force_Pid.error;

    if (Force_Pid.output > 15.0f){
        Force_Pid.output = 15.0f;
    } else if (Force_Pid.output < -15.0f){
        Force_Pid.output = -15.0f;
    }
    
    float rpm = Force_Pid.output * 50.0f;
    // 控制电机转速
    if (rpm > 400.0f){
        rpm = 400.0f;
    } else if (rpm < -0.0f){
        rpm = 0.0f;
    }

    if (fabsf(Force_Pid.target) < 0.01f){
        rpm = 0.0f;
    }

    HAL_GPIO_TogglePin(PID_LED_GPIO_Port, PID_LED_Pin);

    // 控制电机
    silentcontrol(rpm);
}