#include "app_pid_system.h"

#if !HSV_DISCOUVER
#include <map>
#include <utility>
#include <iostream>
#include <cstdint>
#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include "lwip/dns.h"
#include "lwip/err.h"
#include "lwip/sys.h"

#include "app_pid.h"
//#include "app_servo.h"
#include "app_process_image.h"


#define PORT 12345

using namespace std;

static const char *TAG = "app_pid_system";

string pattern = "center";
float *pos = new float[2];

int posIndex;
int cyclesWithoutBall;
int formatTemp;
int sock;
int limites = 1;
Ball_t ball;
PID_t pidX;
PID_t pidY;

map<string, vector<pair<float, float>>> mapPatterns;
map<string,int> mapTemp;

camera_fb_t *fb = NULL;
TaskHandle_t getImageFromCameraTaskHandle = NULL;

void getImageFromCameraTask(void *args){
	while (true) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        fb = esp_camera_fb_get();
        if (!fb) {
            ESP_LOGE(TAG,"Camera capture failed");
        }
    }
}

Ball_t createBall(uint16_t _x, uint16_t _y){
	Ball_t b = {
		.detected = false,
		.x 		= { _x,_x,_x,_x,_x,_x,_x,_x },
		.y 		= { _y,_y,_y,_y,_y,_y,_y,_y },
		.dx 	= { 0,0,0,0,0,0,0,0 },
		.dy 	= { 0,0,0,0,0,0,0,0 },
		.smooth_dx 	= 0,
		.smooth_dy 	= 0
	};
	return b;
}

void updateBall(Ball_t *b, uint16_t _x, uint16_t _y){
	//update X position
	b->x[7] = b->x[6];
    b->x[6] = b->x[5];
    b->x[5] = b->x[4];
	b->x[4] = b->x[3];
	b->x[3] = b->x[2];
	b->x[2] = b->x[1];
	b->x[1] = b->x[0];
	b->x[0] = _x;

	//update X speed
	b->dx[7] = b->dx[6];
	b->dx[6] = b->dx[5];
	b->dx[5] = b->dx[4];
	b->dx[4] = b->dx[3];
	b->dx[3] = b->dx[2];
	b->dx[2] = b->dx[1];
	b->dx[1] = b->dx[0];
	b->dx[0] = b->x[0] - b->x[1];

	//recursively update moving average for X
	b->smooth_dx =
		(N_CONST * b->smooth_dx - b->dx[N_CONST] + b->dx[0]) / N_CONST;

	
	//update Y position
	b->y[7] = b->y[6];
	b->y[6] = b->y[5];
	b->y[5] = b->y[4];
	b->y[4] = b->y[3];
	b->y[3] = b->y[2];
	b->y[2] = b->y[1];
	b->y[1] = b->y[0];
	b->y[0] = _y;
	
	//update Y speed
	b->dy[7] = b->dy[6];
	b->dy[6] = b->dy[5];
	b->dy[5] = b->dy[4];
	b->dy[4] = b->dy[3];
	b->dy[3] = b->dy[2];
	b->dy[2] = b->dy[1];
	b->dy[1] = b->dy[0];
	b->dy[0] = b->y[0] - b->y[1];
	
	//recursively update moving average
	b->smooth_dy =
		(N_CONST * (b->smooth_dy) - b->dy[N_CONST] + b->dy[0]) / N_CONST;
}

float* getNextPosition(int& pos, vector<pair<float,float>> positions){
	float* newPair = new float[2];
	
	pos = (pos+1)%positions.size();
	newPair[0] = positions[pos].first;
	newPair[1] = positions[pos].second;
		
	return newPair;
}

vector<pair<float, float>> generateSquarePatter(int size){
	vector<pair<float, float>> vec;
	vec.push_back(make_pair(-size,size));
	vec.push_back(make_pair(size,size));
	vec.push_back(make_pair(size,-size));
	vec.push_back(make_pair(-size,-size));
	return vec;
}

vector<pair<float, float>> generateTrianglePattern(int G){
	vector<pair<float, float>> vec;
  	vec.push_back(make_pair(0,G));
	vec.push_back(make_pair(-sqrt(3)*G/2,-G/2));
	vec.push_back(make_pair(sqrt(3)*G/2,-G/2));
	return vec;
}

vector<pair<float, float>> generateLimniscatePattern(float r){
	vector<pair<float, float>> vec;
  	float start = 0;
 	double theta;
  	double scale;
  
	theta = start;
	for (double j = 0; j < 2 * PI; j += 0.05) {
	  	scale = r * (2 / (3 - cos(2 * theta))); 
		vec.push_back(make_pair(scale * cos(theta), scale * sin(2 * theta) / 1.5));  
	  	theta += start == 0 ? -0.05 : (start == -2 * PI ? 0.05 : 0);
	}
  
	return vec;
}

void generatePatterns(map<string, vector<pair<float, float>>>& mapPatterns, map<string,int>& mapTemp){
	vector<pair<float,float>> vecCenter = {make_pair(0.0,0.0)};
	mapPatterns["center"] = vecCenter;
	mapPatterns["square"] = generateSquarePatter(3);
	mapPatterns["triangle"] = generateTrianglePattern(4);
	mapPatterns["limniscate"] = generateLimniscatePattern(5);
	mapTemp["center"] = 60;
	mapTemp["square"] = 60;
	mapTemp["triangle"] = 60;
	mapTemp["limniscate"] = 1;
}
/*
void reciverAndUpdateConstants(){
	/// recebe a mensagem UDP
	char rx_buffer[128];
    struct sockaddr_in source_addr;
    socklen_t socklen = sizeof(source_addr);
    int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, MSG_DONTWAIT, (struct sockaddr *)&source_addr, &socklen);

    if (len < 0) {
        if (errno != EWOULDBLOCK) {
            ESP_LOGE(TAG, "recvfrom failed: errno %d", errno);
        }
    } else if (len == 0) {
        ESP_LOGI(TAG, "Connection closed");

    } else {
        rx_buffer[len] = '\0';
        ESP_LOGI(TAG, "Received %d bytes: %s", len, rx_buffer);
	
	/// desmembra mensagem
		uint8_t count = 0;
		float constPID[3];
		uint8_t angulos[3];
		string aux = "";

		for (int i = 0; i < len; i++) {
			if (i == len -1) {
				aux += rx_buffer[i]; 
				angulos[count] = stoi(aux) ;
			} else if (rx_buffer[i] == '/') {
				if(count < 3) constPID[count] = stof(aux);
				else angulos[count] = stoi(aux);
				aux = "";
				count += 1;
			} else {
				aux += rx_buffer[i];
			}
		}
		/// atualiza constantes
		updatePID(&pidX,&pidY,constPID[0], constPID[1], constPID[2]);
		setServosOffset(angulos[0], angulos[1], angulos[2]);
    }
}

void initVariables(){
	posIndex = 0;
	cyclesWithoutBall = 0;
	formatTemp = 0;
	pos[0] = 0;
	pos[1] = 0; 

	generatePatterns(mapPatterns, mapTemp);
	pidX = createPID(KPx,KIx,KDx,SETPOINT_X, false, X_MIN_ANGLE, X_MAX_ANGLE);
    pidY = createPID(KPy,KIy,KDy,SETPOINT_Y, false, Y_MIN_ANGLE, Y_MAX_ANGLE);

	///////////////////////////// init socket /////////////////////////////////
	struct sockaddr_in6 dest_addr;
    int addr_family = AF_INET6;
    int ip_protocol = IPPROTO_IPV6;

    // Configuração do endereço do socket
    memset(&dest_addr, 0, sizeof(dest_addr));
    dest_addr.sin6_family = AF_INET6;
    dest_addr.sin6_port = htons(PORT);
    dest_addr.sin6_addr = in6addr_any;

    // Criar socket UDP
    sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
    if (sock < 0) {
        ESP_LOGE("udpServer", "Unable to create socket: errno %d", errno);
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI("udpServer", "Socket created");

    // Vincular o socket ao endereço e porta especificados
    int err = bind(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    if (err < 0) {
        ESP_LOGE("udpServer", "Socket unable to bind: errno %d", errno);
        close(sock);
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI("udpServer", "Socket bound, port %d", PORT);
	
	fb = esp_camera_fb_get();
    if (!fb) {
        ESP_LOGE(TAG,"Camera capture failed");
        return;
    }
}

void responseForBallPosition(){
	if(ball.detected){
		cyclesWithoutBall = 0;
		formatTemp++;
		if (formatTemp == mapTemp[pattern]){
			pos = getNextPosition(ref(posIndex), mapPatterns[pattern]);
			formatTemp = 0;
		}
		reciverAndUpdateConstants();
		ESP_LOGI(TAG, "Ball position: (x: %hu, y: %hu)", ball.x[0], ball.y[0]);
        PIDCompute(&pidX, &pidY, ball, pos[X_POSITION], pos[Y_POSITION], limites);
		setServoAnglesForPID(pidX.output[0],pidY.output[0]);
    }
	else{
		formatTemp = 0;
		pos[0] = 0;
		pos[1] = 0; 
		if (cyclesWithoutBall < LIMIT_WITHOUT_BALL){
			cyclesWithoutBall++;
		}
		if (cyclesWithoutBall == LIMIT_WITHOUT_BALL) {
			cyclesWithoutBall++;
			pidX.integral = 0;
			pidY.integral = 0;
			setServoAnglesForPID(0,0);
		}
	}	
}
*/
uint16_t ticks;
int value;

uint16_t getTicksFromAngle(uint8_t i){
  uint16_t value = 544 + (2400-544)*i/180;
  return (int)((double)value / ((double)20000 / (double)1024)*(((double)50)/50.0));
}

typedef struct Serv{
  uint8_t ang1;
  uint8_t ang2;
  uint8_t ang3;
} Serv;


void setServoAngleLEDC(ledc_channel_t channel, uint8_t angle) {
    uint16_t ticks = getTicksFromAngle(angle);
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_HIGH_SPEED_MODE, channel, ticks));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_HIGH_SPEED_MODE, channel));
}

void setServoAngles(Serv *ang){
    
	if(ang->ang1 > SERVO_MAX_ANGLE) ang->ang1 = SERVO_MAX_ANGLE;
    if(ang->ang1 < SERVO_MIN_ANGLE) ang->ang1 = SERVO_MIN_ANGLE;
	if(ang->ang2 > SERVO_MAX_ANGLE) ang->ang2 = SERVO_MAX_ANGLE;
    if(ang->ang2 < SERVO_MIN_ANGLE) ang->ang2 = SERVO_MIN_ANGLE;
	if(ang->ang3 > SERVO_MAX_ANGLE) ang->ang3 = SERVO_MAX_ANGLE;
    if(ang->ang3 < SERVO_MIN_ANGLE) ang->ang3 = SERVO_MIN_ANGLE;

	ESP_LOGI(TAG,"Os angulos depois: (%u, %u, %u)\n", ang->ang1, ang->ang2, ang->ang3);
	
    setServoAngleLEDC(LEDC_CHANNEL_1, ang->ang1);
    setServoAngleLEDC(LEDC_CHANNEL_2, ang->ang2);
	setServoAngleLEDC(LEDC_CHANNEL_4, ang->ang3);
    
	
}

void configureLEDCChannel(int gpio_num, ledc_channel_t channel) {
    ledc_channel_config_t ledc_channel = {
        .gpio_num = gpio_num, // Pino GPIO conectado ao servo
        .speed_mode = LEDC_HIGH_SPEED_MODE, // Modo de alta velocidade
        .channel = channel, // Canal do LEDC
        .intr_type = LEDC_INTR_DISABLE, // Desabilitar interrupção
        .timer_sel = LEDC_TIMER_1,
        .duty = 0, 
        .hpoint = 0 
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
}

void configureLEDCServos() {
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_HIGH_SPEED_MODE, // Modo de alta velocidade
        .duty_resolution = LEDC_TIMER_10_BIT, 
        .timer_num = LEDC_TIMER_1, 
        .freq_hz = 50, // Frequência de 50 Hz para servos
        .clk_cfg = LEDC_AUTO_CLK // Seleção automática de clock
    };
    
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

	  configureLEDCChannel(SERVO_1_PIN, LEDC_CHANNEL_1); 
    configureLEDCChannel(SERVO_2_PIN, LEDC_CHANNEL_2); 
    configureLEDCChannel(SERVO_3_PIN, LEDC_CHANNEL_4); 
}

void runSystemPID(void *args){
	configureLEDCServos();
	uint8_t i = 0;
	Serv anglesPosition[3];
	anglesPosition[0].ang1 = 100;
	anglesPosition[0].ang2 = 130;
	anglesPosition[0].ang3 = 170;
	anglesPosition[1].ang1 = 130;
	anglesPosition[1].ang2 = 170;
	anglesPosition[1].ang3 = 100;
	anglesPosition[2].ang1 = 170;
	anglesPosition[2].ang2 = 100;
	anglesPosition[2].ang3 = 130;
	
	int64_t fr_end, frame_time;
	static int64_t last_frame = 0;
	ball.detected = false;

	uint16_t ballInformationResponse[3] = {0,0,0}; 
	//teste servos
	

	//initiateServos();
	initVariables();
	xTaskCreatePinnedToCore(getImageFromCameraTask, "getImageFromCameraTask", 1024*9, NULL, 5, &getImageFromCameraTaskHandle, 1);
    
	last_frame = esp_timer_get_time();
    while (true) {
		convertFrameToRGB888(fb);
		esp_camera_fb_return(fb);

		xTaskNotifyGive(getImageFromCameraTaskHandle);
		
		searchForBallOnImage(ball.detected,ballInformationResponse);

		if(ballInformationResponse[BALL_DETECTION] == UPDATE_BALL){ // update case
			updateBall(&ball, ballInformationResponse[X_POSITION], ballInformationResponse[Y_POSITION]);
		} 
		else if(ballInformationResponse[BALL_DETECTION] == CREATE_BALL){ //create case
			ball = createBall(ballInformationResponse[X_POSITION], ballInformationResponse[Y_POSITION]);
			ball.detected = true;
		}
		else ball.detected = false; // no ball case
		setServoAngles(&anglesPosition[i]);
		i = (i+1)%3;
		//responseForBallPosition(); 

		/// calculate fps rate
		
		fr_end = esp_timer_get_time();
		
		frame_time = fr_end - last_frame;
		frame_time /= 1000;
		last_frame = fr_end;

		ESP_LOGI(TAG,"MJPG: (%.1ffps)", 1000.0 / (uint32_t)frame_time);  
    	vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

#endif