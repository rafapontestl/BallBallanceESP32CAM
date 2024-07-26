#include "nvs_flash.h"
#include "esp_wifi.h"
#include <cstring>

#include "app_camera.h"
#include "const_def.h"


#if HSV_DISCOUVER
#include "app_httpd.h"
#else
#include "app_pid_system.h"
#endif

static const char *TAG = "app_main";

extern "C"{
  void app_main(void);
}

void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        esp_wifi_connect();
        ESP_LOGI(TAG, "Retrying to connect to the AP");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
    }
}

void initializeNVS(){
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);
}

void initializeWiFi(){
   ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // Register event handler
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL));

    wifi_config_t wifi_config = {};
    strcpy(reinterpret_cast<char *>(wifi_config.sta.ssid), "Rptl");
    strcpy(reinterpret_cast<char *>(wifi_config.sta.password), "11111111");

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    // Wait for Wi-Fi connection
    ESP_LOGI(TAG, "Connecting to WiFi...");
};


void app_main() {

    initializeNVS();

    app_camera_init();

    ESP_ERROR_CHECK(esp_netif_init());

    initializeWiFi();

    #if HSV_DISCOUVER
    xTaskCreatePinnedToCore(startCameraServer, "start camera server", 1024 * 9, nullptr, 5, nullptr, 0);
    #else
    xTaskCreatePinnedToCore(runSystemPID, "run system pid", 1024 * 9, nullptr, 5, nullptr, 0);
    #endif
}
