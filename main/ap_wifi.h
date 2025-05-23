#ifndef AP_WIFI_H
#define AP_WIFI_H

#include "esp_err.h"
#include "esp_wifi.h"

/* ESP will send data to this URL after STA connection */
#define SERVER_URL "http://cazangiilor.go.ro:57348/data"
/* Acces Point name */
#define AP_SSID             "ESP_Setup"
/* Acces Point password */
#define AP_PASS             "12345678"

/**
 * @brief Initialize WiFi and NVS Flash.
 * 
 * @return:
 *  - true if OK
 *  - false if not OK
 */
esp_err_t wifi_init();

/**
 * @brief Connect ESP to STA or AP.
 * 
 * @returns STA_MODE if connected as STA, AP_MODE if started as AP
 */
wifi_mode_t wifi_connect();

/* END AP_WIFI_H */
#endif