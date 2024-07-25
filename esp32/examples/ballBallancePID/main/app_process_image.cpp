#include "app_process_image.h"

using namespace cv;
using namespace std;

Mat frameRGB888, imageThreshold, HSV;

const Scalar GREEN = Scalar(0, 255, 0);
const Scalar ORANGE = Scalar(0, 165, 255);

HSV_t hsvFilter = {83,255,48,255,109,255}; 

void morphOpsOverThreshold(){

	//the kernel chosen here is a 5px by 5px square
	Mat erodeElement = getStructuringElement(MORPH_RECT, Size(5,5));

    //dilate with larger element so make sure object is nicely visible
	Mat dilateElement = getStructuringElement(MORPH_RECT, Size(5,5));

	erode(imageThreshold,imageThreshold,erodeElement);
	//erode(thresh,thresh,erodeElement);
	erodeElement.release();
	//dilate(thresh,thresh,dilateElement);
	dilate(imageThreshold,imageThreshold,dilateElement);
	dilateElement.release();
}

void updateThresholdFromFrameRGB888(){
    cvtColor(frameRGB888, HSV, COLOR_RGB2HSV);
    Scalar lower_bound(hsvFilter.HMIN, hsvFilter.SMIN, hsvFilter.VMIN);
    Scalar upper_bound(hsvFilter.HMAX, hsvFilter.SMAX, hsvFilter.VMAX);
    inRange(HSV, lower_bound, upper_bound, imageThreshold);
}

void convertFrameToRGB888(camera_fb_t *fb){
    frameRGB888.create(fb->height, fb->width, CV_8UC3);
	uint8_t *dst = frameRGB888.data;
    uint16_t *src = (uint16_t *)fb->buf;
    for (int i = 0; i < fb->width * fb->height; i++) {
        uint16_t pixel = *src++;
        *dst++ = (pixel >> 8) & 0xF8;  // R
        *dst++ = (pixel >> 3) & 0xFC;  // G
        *dst++ = (pixel << 3) & 0xF8;  // B
    }
}

uint16_t* searchForBallOnImage(bool isBallDetected, uint16_t response[3]){
    updateThresholdFromFrameRGB888();
    morphOpsOverThreshold();

    vector< vector<Point> > contours;
	vector<Vec4i> hierarchy;
	float refArea = 0;
    response[X_POSITION] = 0;
    response[Y_POSITION] = 0;
    response[BALL_DETECTION] = 0;
	//find contours of filtered image using openCV findContours function
	findContours(imageThreshold,contours,hierarchy, RETR_CCOMP, CHAIN_APPROX_SIMPLE);

	if (hierarchy.size() > 0) {
		int numObjects = hierarchy.size();
        //if number of objects greater than MAX_NUM_OBJECTS we have a noise error
        if(numObjects<MAX_NUM_OBJECTS){
			for (int index = 0; index >= 0; index = hierarchy[index][0]) {
				Moments moment = moments((Mat)contours[index]);
				float area = moment.m00;

                if(area>MIN_OBJECT_AREA && area<MAX_OBJECT_AREA && area>refArea){
					if(isBallDetected){
						response[BALL_DETECTION] = 1;
                        response[X_POSITION] = (moment.m10/area);
                        response[Y_POSITION] = (moment.m01/area);
						refArea = area;
					}
					else {
                        response[BALL_DETECTION] = 2;
                        response[X_POSITION] = (moment.m10/area)+(FRAME_WIDTH-CONTROL_AREA)/2;
                        response[Y_POSITION] = (moment.m01/area)+(FRAME_HEIGHT-CONTROL_AREA)/2;
					}
				}
			}
		}
		
	}
    return response;
}

#if HSV_DISCOUVER
camera_fb_t* getFilteredJPG(camera_fb_t *fbIn, HSV_t &hsvValues) {
    hsvFilter = hsvValues;
    convertFrameToRGB888(fbIn);
    esp_camera_fb_return(fbIn);
    
    updateThresholdFromFrameRGB888();
    morphOpsOverThreshold();
    
    // Verifique se a imagem é contínua na memória
    if (!imageThreshold.isContinuous()) {
        ESP_LOGE(TAG,"Image data is not continuous.");
        return nullptr;
    }

    // Crie um framebuffer
    camera_fb_t *fb = (camera_fb_t*)malloc(sizeof(camera_fb_t));
    if (!fb) {
        ESP_LOGE(TAG,"Failed to allocate memory for framebuffer.");
        return nullptr;
    }

    // Defina as propriedades do framebuffer
    fb->width = imageThreshold.cols;
    fb->height = imageThreshold.rows;
    fb->format = PIXFORMAT_GRAYSCALE; // Como é uma imagem binária, use o formato de escala de cinza
    fb->len = imageThreshold.total() * imageThreshold.elemSize(); // Total de elementos vezes o tamanho de cada elemento

    // Alocar memória para os dados da imagem
    fb->buf = (uint8_t*)malloc(fb->len);
    if (!fb->buf) {
        ESP_LOGE(TAG,"Failed to allocate memory for image data.");
        free(fb); // Liberar o framebuffer se a alocação falhar
        return nullptr;
    }

    // Copie os dados da thresholdm para o buffer do framebuffer
    memcpy(fb->buf, imageThreshold.data, fb->len);

    return fb;
}
#endif