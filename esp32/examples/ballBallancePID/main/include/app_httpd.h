#ifndef _APP_HTTPD_H_
#define _APP_HTTPD_H_

#include "esp_http_server.h"

#include "app_process_image.h"
#include "const_def.h"

#define ARDUHAL_LOG_LEVEL 1
#define ARDUHAL_LOG_LEVEL_INFO    0
#define CONFIG_LED_ILLUMINATOR_ENABLED 1

#ifdef __cplusplus
extern "C"{
#endif
void startCameraServer(void *arg);
void setupLedFlash(int pin);
#if __cplusplus
}
#endif
#endif
