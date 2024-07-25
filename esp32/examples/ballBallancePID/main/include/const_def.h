#pragma once
#include <stdio.h>      // Para operações de entrada e saída
#include <stdint.h>     // Para definições de tipos uint16_t, etc.
#include <stdbool.h>    // Para definições de tipos bool, true, false
#include <math.h> 

#include "esp_log.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_camera.h"
#include "driver/ledc.h"
#include "driver/gpio.h"

#define HSV_DISCOUVER 0

typedef struct Ball {
    bool        detected;
    uint16_t    x[8];
    uint16_t    y[8];
    short       dx[8];
    short       dy[8];
    short       smooth_dx;
    short       smooth_dy;
} Ball_t;

/*===============================================*/
/*ABOUT camera*/

//camera pins
#define PWDN_GPIO_NUM    32
#define RESET_GPIO_NUM   -1
#define XCLK_GPIO_NUM     0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27

#define Y9_GPIO_NUM    35
#define Y8_GPIO_NUM    34
#define Y7_GPIO_NUM    39
#define Y6_GPIO_NUM    36
#define Y5_GPIO_NUM    21
#define Y4_GPIO_NUM    19
#define Y3_GPIO_NUM    18
#define Y2_GPIO_NUM    5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22
#define XCLK_FREQ       20000000

#define LED_GPIO_NUM   33

/*===============================================*/

//_ Global Variables _______________________

/*===============================================*/
/*ABOUT SERVOS*/

#define SERVO_1_PIN 12
#define SERVO_2_PIN 13
#define SERVO_3_PIN 14

#define LEDC_PWM_SPEED_MODE LEDC_HIGH_SPEED_MODE
#define LEDC_CHANNEL_SERVO_1 LEDC_CHANNEL_1
#define LEDC_CHANNEL_SERVO_2 LEDC_CHANNEL_2
#define LEDC_CHANNEL_SERVO_3 LEDC_CHANNEL_3
#define LEDC_TIMER_SERVOS LEDC_TIMER_1 

#define   ANGLE_OFFSET      20

#define   ANGLE_ORIGIN      135

#define   X_HALF_ANGLE      60
#define	  X_MAX_ANGLE       X_HALF_ANGLE+ANGLE_OFFSET
#define   X_MIN_ANGLE       X_HALF_ANGLE-ANGLE_OFFSET

#define   Y_HALF_ANGLE      60
#define	  Y_MAX_ANGLE       Y_HALF_ANGLE+ANGLE_OFFSET
#define   Y_MIN_ANGLE       Y_HALF_ANGLE-ANGLE_OFFSET
#define   SERVO_MAX_ANGLE   ANGLE_ORIGIN + 35
#define   SERVO_MIN_ANGLE   ANGLE_ORIGIN - 35

#define BALL_DETECTION 2
#define UPDATE_BALL 1
#define CREATE_BALL 2
#define X_POSITION 0
#define Y_POSITION 1

#define     PI                  3.1415

/*===============================================*/
/*ABOUT IMAGE*/

#define     CONTROL_AREA        240

#define     FRAME_WIDTH         240
#define     FRAME_HEIGHT        240
#define     MIN_OBJECT_AREA 25*25
#define     MAX_OBJECT_AREA FRAME_HEIGHT*FRAME_WIDTH/1.5
#define     MAX_NUM_OBJECTS 50
#define     N_CONST 3

#define     SETPOINT_X          FRAME_WIDTH/2
#define     SETPOINT_Y          FRAME_HEIGHT/2

#define     LIMIT_WITHOUT_BALL 10

/*===============================================*/
/*ABOUT MECANIC*/

#define L1 4.0
#define L2 5.5
#define HEIGHT 7.54 
#define CORNER_BASE 8
#define CORNER_PLATFORM 8

/*===============================================*/
/*ABOUT PID*/

#define KPx 0.00624
#define KDx 0.117
#define KIx 0.0000832

#define KPy KPx
#define KDy 0.133
#define KIy 0.000073
