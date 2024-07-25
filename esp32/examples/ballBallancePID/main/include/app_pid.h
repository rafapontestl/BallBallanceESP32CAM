#ifndef _PID_H_
#define _PID_H_

#include <stdint.h>
#include "const_def.h"

typedef struct PID_t{
    float Kp, Ki, Kd;
    uint16_t setpoint;
    float error[2];
    float dt;
    float output[2];
    float P, I, D;
    float integral;
    uint16_t min, max;
    bool inverted_mode;
}PID_t;


typedef struct Ponto{
    uint16_t x;
    uint16_t y;
} Ponto_t;

typedef struct Pair {
    float R;
    float H;
} Pair;

typedef struct Verts {
    Pair P1; 
    Pair P2;
    Pair P3;
} Verts;


#if __cplusplus
extern "C" {
#endif
    PID_t createPID(float Kp, float Ki, float Kd, uint16_t setpoint, bool mode,
                uint16_t min_angle, uint16_t max_angle);

    void PIDCompute(PID_t* pidX, PID_t* pidY, Ball_t ball,float setpointX, float setpointY, float limite);

    void updatePID(PID_t* pidx, PID_t* pidy, float Kp, float Ki, float Kd);

    float saturationFilter(float value , float T_MIN, float T_MAX);

    bool changeSetpoint(PID_t* XPID, PID_t* YPID,  Ball_t* ball, Ponto_t sp);
#if __cplusplus
}
#endif



#endif
