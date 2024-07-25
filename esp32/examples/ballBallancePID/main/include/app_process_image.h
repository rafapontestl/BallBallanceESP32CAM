#pragma once
#undef EPS      // specreg.h defines EPS which interfere with opencv
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#define EPS 192

#include "const_def.h"

typedef struct HSV {
    uint8_t HMIN;
    uint8_t HMAX;  
    uint8_t SMIN;
    uint8_t SMAX;
    uint8_t VMIN; 
    uint8_t VMAX; 
} HSV_t;

extern "C"{
    void convertFrameToRGB888(camera_fb_t *fb);
    uint16_t* searchForBallOnImage(bool isBallDetected, uint16_t response[3]);
    #if HSV_DISCOUVER
    camera_fb_t* getFilteredJPG(camera_fb_t *fbIn, HSV_t &hsvValues);
    #endif
}