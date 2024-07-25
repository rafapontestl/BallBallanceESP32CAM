#ifndef _APP_PID_SYSTEM_H_
#define _APP_PID_SYSTEM_H_
#include "const_def.h"

extern "C"{
    void runSystemPID(void *arg);
    void getImageFromCameraTask(void *args);
}

#endif