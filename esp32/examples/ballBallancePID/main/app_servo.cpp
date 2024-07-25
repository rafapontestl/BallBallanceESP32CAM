#include "app_servo.h"

static const char *TAG = "app_servo";

uint8_t angles[3];
int ticks1,ticks2,ticks3;
uint8_t servo_1_offset_value = 2;
uint8_t servo_2_offset_value = 0;
uint8_t servo_3_offset_value = 0;
Machine machine(10.0, 9.75, 4.0, 5.5);

void configureLEDCChannelsServos() {
    ledc_channel_config_t ledc_channel1 = {
        .gpio_num =SERVO_1_PIN,
        .speed_mode = LEDC_PWM_SPEED_MODE,
        .channel = LEDC_CHANNEL_SERVO_1,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_SERVOS,
        .duty = 0, 
        .hpoint = 0 
    };
    ledc_channel_config_t ledc_channel2 = {
        .gpio_num =SERVO_2_PIN,
        .speed_mode = LEDC_PWM_SPEED_MODE,
        .channel = LEDC_CHANNEL_SERVO_2,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_SERVOS,
        .duty = 0, 
        .hpoint = 0 
    };
    ledc_channel_config_t ledc_channel3 = {
        .gpio_num = SERVO_3_PIN,
        .speed_mode = LEDC_PWM_SPEED_MODE,
        .channel = LEDC_CHANNEL_SERVO_3,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_SERVOS,
        .duty = 0, 
        .hpoint = 0 
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel1));
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel2));
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel3));
}

void setLEDCDutyServos(int dutyServo_1, int dutyServo_2, int dutyServo_3){
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_PWM_SPEED_MODE, LEDC_CHANNEL_SERVO_1, dutyServo_1));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_PWM_SPEED_MODE, LEDC_CHANNEL_SERVO_1));
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_PWM_SPEED_MODE, LEDC_CHANNEL_SERVO_2, dutyServo_2));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_PWM_SPEED_MODE, LEDC_CHANNEL_SERVO_2));
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_PWM_SPEED_MODE, LEDC_CHANNEL_SERVO_3, dutyServo_3));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_PWM_SPEED_MODE, LEDC_CHANNEL_SERVO_3));
    //vTaskDelay(500 / portTICK_PERIOD_MS);
    //ledc_stop(speedMode,channel,0);
}

void configureLEDCTimer(){
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_PWM_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_10_BIT, 
        .timer_num = LEDC_TIMER_SERVOS, 
        .freq_hz = 50,
        .clk_cfg = LEDC_AUTO_CLK
    };
    
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));
}

int getTicksFromAngle(uint8_t i){
  uint16_t value = 544 + (2400-544)*i/180;
  return (int)((double)value / ((double)20000 / (double)1024)*(((double)50)/50.0));
}

void setServosAnglesLEDC(uint8_t servo_1_angle, uint8_t servo_2_angle, uint8_t servo_3_angle) {
    ticks1 = getTicksFromAngle(servo_1_angle);
    ticks2 = getTicksFromAngle(servo_2_angle);
    ticks3 = getTicksFromAngle(servo_3_angle);
    setLEDCDutyServos(ticks1, ticks2, ticks3);
}

void setServoAngles(uint8_t servo_1_angle, uint8_t servo_2_angle, uint8_t servo_3_angle){
    
	if(servo_1_angle > SERVO_MAX_ANGLE) servo_1_angle = SERVO_MAX_ANGLE;
    if(servo_1_angle < SERVO_MIN_ANGLE) servo_1_angle = SERVO_MIN_ANGLE;
	if(servo_2_angle > SERVO_MAX_ANGLE) servo_2_angle = SERVO_MAX_ANGLE;
    if(servo_2_angle < SERVO_MIN_ANGLE) servo_2_angle = SERVO_MIN_ANGLE;
	if(servo_3_angle > SERVO_MAX_ANGLE) servo_3_angle = SERVO_MAX_ANGLE;
    if(servo_3_angle < SERVO_MIN_ANGLE) servo_3_angle = SERVO_MIN_ANGLE;

	ESP_LOGI(TAG, "Os angulos depois: (%d, %d, %d)\n", servo_1_angle, servo_2_angle, servo_3_angle);
	
    setServosAnglesLEDC(servo_1_angle, servo_2_angle, servo_3_angle);	
}

void setServosOffset(uint8_t servo_1_offset_input, uint8_t servo_2_offset_input, uint8_t servo_3_offset_input){
    servo_1_offset_value = servo_1_offset_input;
    servo_2_offset_value = servo_2_offset_input;
    servo_3_offset_value = servo_3_offset_input;
}

void setServoAnglesForPID(float pidX_output, float pidY_output){
    for (int i = 0; i < 3; i++) {
      angles[i] = round((machine.theta(i, 7.54, -pidX_output, pidY_output)) );
    }
    setServoAngles(angles[0] + servo_1_offset_value, angles[1] + servo_2_offset_value, angles[2] + servo_3_offset_value);
}

void initiateServos() {
    configureLEDCTimer();
	configureLEDCChannelsServos();
    setServosAnglesLEDC(ANGLE_ORIGIN + servo_1_offset_value, ANGLE_ORIGIN + servo_2_offset_value, ANGLE_ORIGIN + servo_3_offset_value);
}