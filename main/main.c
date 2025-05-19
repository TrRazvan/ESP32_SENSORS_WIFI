#include <stdio.h>
#include "esp_err.h"
#include "ap_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_http_client.h"
#include "esp_random.h"
#include <string.h>
#include "ssd1306.h"
#include "gy291_acc.h"
#include "dht11.h"
#define DEBUG
#include "util.h"

/* I2C SCL pin num */
#define I2C_MASTER_SCL_IO   (22u)
/* I2C SDA pin num */
#define I2C_MASTER_SDA_IO   (21u)
/* I2C Frequency */
#define I2C_MASTER_FREQ_HZ  (100000u)

/* SPI MOSI pin num */
#define SPI_PIN_NUM_MOSI (23u)
/* SPI CLK pin num */
#define SPI_PIN_NUM_CLK  (18u)

/* SSD1306 pin nums */
#define SSD1306_PIN_NUM_CS   (5u)
#define SSD1306_PIN_NUM_DC   (16u)
#define SSD1306_PIN_NUM_RST  (17u)

/* Terminal TAG */
#define TAG "MAIN"
#define WIFI_MAX_PAYLOAD_LEN    (64u)

/* GY291 accelerometer variables */
static int16_t x, y, z;
static float gx, gy, gz;

/* TODO: Task get data + task send data (or just one task for both) */
// void post_data_task(void *pvParameters)
// {
//     while (1)
//     {
//         char payload[WIFI_MAX_PAYLOAD_LEN];
//         int value = esp_random() % 100;
//         snprintf(payload, sizeof(payload), "{\"value\": %d}", value);

//         esp_http_client_config_t config = {
//             .url = SERVER_URL,  // IP-ul serverului Flask
//             .method = HTTP_METHOD_POST,
//         };

//         esp_http_client_handle_t client = esp_http_client_init(&config);
//         if (client == NULL) {
//             ESP_LOGE(TAG, "Failed to init client");
//             vTaskDelay(pdMS_TO_TICKS(5000)); // așteaptă 5 secunde și încearcă din nou
//             continue;
//         }

//         esp_http_client_set_header(client, "Content-Type", "application/json");
//         esp_http_client_set_post_field(client, payload, strlen(payload));

//         esp_err_t err = esp_http_client_perform(client);

//         if (err == ESP_OK)
//         {
//             int status_code = esp_http_client_get_status_code(client);
//             ESP_LOGI(TAG, "Trimis! Status: %d", status_code);
//         } else {
//             ESP_LOGE(TAG, "Eroare trimitere: %s", esp_err_to_name(err));
//         }

//         esp_http_client_cleanup(client);

//         vTaskDelay(pdMS_TO_TICKS(1000)); // Așteaptă 5 secunde între POST-uri
//     }
// }

void app_main(void)
{
    esp_err_t ret = ESP_OK;

    /* Payload to send via HTTP */
    char payload[WIFI_MAX_PAYLOAD_LEN];

    /* HTTP status code response */
    int status_code;

    /* SSD1306 config structure */
    ssd1306_t ssd = {
        .pin_dc = SSD1306_PIN_NUM_DC,
        .pin_rst = SSD1306_PIN_NUM_RST,
        .pin_clk = SPI_PIN_NUM_CLK,
        .pin_cs = SSD1306_PIN_NUM_CS,
        .pin_mosi = SPI_PIN_NUM_MOSI,
        .spi_host = SPI2_HOST
    };

    /* GY291 accelerometer sensor config structure*/
    gy291_acc_t gy291_conf = {
        .conf = {
            .mode = I2C_MODE_MASTER,
            .sda_io_num = I2C_MASTER_SDA_IO,
            .sda_pullup_en = GPIO_PULLUP_ENABLE,
            .scl_io_num = I2C_MASTER_SCL_IO,
            .scl_pullup_en = GPIO_PULLUP_ENABLE,
            .master.clk_speed = I2C_MASTER_FREQ_HZ
        },
        .i2c_num = I2C_NUM_0
    };
    
    /* DHT11 temperature & humidity config structure */
    dht11_t dht11_sensor = {
        .dht11_pin = CONFIG_DHT11_PIN
    };

    /* WiFi config structure */
    esp_http_client_config_t config = {
            .url = SERVER_URL,  /* Flusk server URL */
            .method = HTTP_METHOD_POST,
    };

    /* Init ESP as a http client */
    esp_http_client_handle_t client;

    /* Initialize SSD1306 display */
    ssd1306_init(&ssd);
    ssd1306_clear();
    ssd1306_draw_string(20, 5, "Initializing...", true);
    ssd1306_refresh();

    /* Initialize wifi */
    ret = wifi_init();
    if (ret == ESP_OK)
    {
        wifi_connect();
    }

    /* Initialize GY291 sensor */
    ret = gy291_acc_init(&gy291_conf);
    if (ret == ESP_OK)
    {
        SHOW_INF(TAG, "GY291 accelerometer sensor calibrating...");
        calibrate_gy291();
        SHOW_INF(TAG, "GY291 accelerometer sensor calibration done!");
    }

    while (1)
    {
        /* TODO: Get all sensors data */
        /* Read DHT11 temperature & humidity data */
        dht11_read(&dht11_sensor, CONFIG_CONNECTION_TIMEOUT);
        SHOW_INF(TAG, "TEMP: %.2f, HUMI: %.2f", dht11_sensor.temperature, dht11_sensor.humidity);

        /* Read GY291 accelerometer data */
        gy291_read_xyz(&x, &y, &z, &gx, &gy, &gz);
        SHOW_INF(TAG, "X: %.2f, Y: %.2f, Z: %.2f", gx, gy, gz);

        /* Init ESP as a http client */
        client = esp_http_client_init(&config);

        /* Concatenate sensors data */
        sprintf(payload, "T=%.2f,H=%.2f,X=%.2f,Y=%.2f,Z=%.2f", dht11_sensor.temperature, dht11_sensor.humidity, gx, gy, gz);

        /* Set client header */
        esp_http_client_set_header(client, "Content-Type", "application/json");

        /* Set post field with data */
        esp_http_client_set_post_field(client, payload, strlen(payload));

        /* Send data */
        ret = esp_http_client_perform(client);
        if (ret == ESP_OK)
        {
            /* Get status code after sending data */
            status_code = esp_http_client_get_status_code(client);
            SHOW_INF(TAG, "Sent! Status: %d", status_code);
        }
        else
        {
            SHOW_INF(TAG, "Sending error: %s", esp_err_to_name(ret));
        }

        /* Cleanup client data */
        esp_http_client_cleanup(client);

        vTaskDelay(pdMS_TO_TICKS(100));
    }

    // xTaskCreate(&post_data_task, "post_data_task", 4096, NULL, 5, NULL);
}