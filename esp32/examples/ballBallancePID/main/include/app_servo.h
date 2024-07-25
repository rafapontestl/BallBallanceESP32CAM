#ifndef _APP_SERVO_H_
#define _APP_SERVO_H_
#include <cstdint>
#include <stdio.h>
#include <math.h>

#include "const_def.h"
#include "InverseKinematics.h"
//#include "app_pwm.h"

typedef struct Serv {
    uint8_t ang1; 
    uint8_t ang2; 
    uint8_t ang3; 
} Serv;

extern "C"{
    void initiateServos();
    void setServoAngles(uint8_t servo_1_angle, uint8_t servo_2_angle, uint8_t servo_3_angle);
    void setServosOffset(uint8_t servo1_offset_input, uint8_t servo2_offset_input, uint8_t servo3_offset_input);
    void setServoAnglesForPID(float pidX_output, float pidY_output);
}

#endif