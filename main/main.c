#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "esp_http_client.h"
#include <string.h>
#include "driver/i2c.h"
#include "ap_wifi.h"
#include "ssd1306.h"
#include "gy291_acc.h"
#include "sht21.h"
#include "acs712.h"
#define DEBUG
#include "util.h"

#define WIFI_MAX_PAYLOAD_LEN    (64u)
#define TAG "MAIN"

/* SPI MOSI pin num */
#define SPI_PIN_NUM_MOSI (23u)
/* SPI CLK pin num */
#define SPI_PIN_NUM_CLK  (18u)

/* SSD1306 pin nums */
#define SSD1306_PIN_NUM_CS   (5u)
#define SSD1306_PIN_NUM_DC   (16u)
#define SSD1306_PIN_NUM_RST  (17u)

/* I2C SCL pin num */
#define I2C_MASTER_SCL_IO   (22u)
/* I2C SDA pin num */
#define I2C_MASTER_SDA_IO   (21u)
/* I2C Frequency */
#define I2C_MASTER_FREQ_HZ  (100000u)

/* Receiving buffer size. Only slave mode will use this value, it is ignored in master mode */
#define I2C_RX_BUFF_SIZE    (0u)
/* Sending buffer size. Only slave mode will use this value, it is ignored in master mode */
#define I2C_TX_BUFF_SIZE    (0u)
/* Flags used to allocate the interrupt */
#define I2C_INTR_FLAGS      (0u)

void app_main(void)
{
    esp_err_t ret = ESP_OK;
    
    /* HTTP status code response */
    int status_code;
    /* Payload to send via HTTP */
    char payload[WIFI_MAX_PAYLOAD_LEN];
    char acc[WIFI_MAX_PAYLOAD_LEN];
    char temp_humi[WIFI_MAX_PAYLOAD_LEN];
    char temp_humi_dht[WIFI_MAX_PAYLOAD_LEN];
    char current_acs[WIFI_MAX_PAYLOAD_LEN];

    /* GY291 accelerometer variables */
    static int16_t x, y, z;
    static float gx, gy, gz;
    float sht21_temp = INIT_USGN_VAL;
    float sht21_humi = INIT_USGN_VAL;
    float current = INIT_USGN_VAL;
    
     /* WiFi config structure */
    esp_http_client_config_t config = {
            .url = SERVER_URL,  /* Flusk server URL */
            .method = HTTP_METHOD_POST,
    };

    /* Init ESP as a http client */
    esp_http_client_handle_t client;

    /* SSD1306 config structure */
    ssd1306_t ssd = {
        .pin_dc = SSD1306_PIN_NUM_DC,
        .pin_rst = SSD1306_PIN_NUM_RST,
        .pin_clk = SPI_PIN_NUM_CLK,
        .pin_cs = SSD1306_PIN_NUM_CS,
        .pin_mosi = SPI_PIN_NUM_MOSI,
        .spi_host = SPI2_HOST
    };

    i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ
    };

    acs712_t acs712_config = {
        .adc_atten = ADC_ATTEN_DB_11,       /* Read permissions over ~3.3V */
        .adc_width_bit = ADC_WIDTH_BIT_12,  /* ADC resolution */
        .adc_ch = ADC1_CHANNEL_0,           /* ADC Channel - GPIO 36*/
    };

    /* Initialize SSD1306 display */
    ssd1306_init(&ssd);
    ssd1306_clear();
    ssd1306_draw_string(10, 5, "INITIALIZING...", true);
    ssd1306_refresh();

    vTaskDelay(pdMS_TO_TICKS(100));

    /* Initialize wifi */
    ret = wifi_init();
    if (ret == ESP_OK)
    {
        wifi_connect();
    }

     /* Config I2C controller */
    ret = i2c_param_config(I2C_NUM_0, &i2c_conf);
    if (ret == ESP_OK)
    {
        /* Install I2C driver */
        ret = i2c_driver_install(I2C_NUM_0, i2c_conf.mode, I2C_RX_BUFF_SIZE, I2C_TX_BUFF_SIZE, I2C_INTR_FLAGS);
        if (ret != ESP_OK)
        {
            SHOW_ERR(TAG, "I2C driver install error");
        }
    }
    else
    {
        SHOW_ERR(TAG, "I2C param config error");
    }

    /* Initialize GY291 sensor */
    ret = gy291_acc_init(I2C_NUM_0);
    if (ret == ESP_OK)
    {
        SHOW_INF(TAG, "GY291 accelerometer sensor calibrating...");
        calibrate_gy291();
        SHOW_INF(TAG, "GY291 accelerometer sensor calibration done!");
    }
    else
    {
        SHOW_ERR(TAG, "GY291 init error");
    }

    /* Initialize SHT21 sensor */
    ret = sht21_init(I2C_NUM_0);
    if (ret != ESP_OK)
    {
        SHOW_ERR(TAG, "SHT21 sensor init error");
    }

    /* Initialize ACS712 */
    ret = acs712_init(&acs712_config);
    if (ret != ESP_OK)
    {
        SHOW_ERR(TAG, "ACS712 init error");
    }

     while (1)
    {
        /* Read GY291 accelerometer data */
        gy291_read_xyz(&x, &y, &z, &gx, &gy, &gz);
        SHOW_INF(TAG, "X: %.2f, Y: %.2f, Z: %.2f", gx, gy, gz);
        sprintf(acc, "G: %.2f, %.2f, %.2f", gx, gy, gz);

        sht21_read(&sht21_temp, &sht21_humi);
        SHOW_INF(TAG, "TEMP: %2.fC, HUMI: %.2f%%", sht21_temp, sht21_humi);
        sprintf(temp_humi, "T: %.2fC, H: %.2f%%", sht21_temp, sht21_humi);

        /* Read ACS712 current */
        current = acs712_read_current();
        SHOW_INF(TAG, "Current: %.2f", current);
        sprintf(current_acs, "I: %.2f A", current);

        /* Print data on display */
        ssd1306_clear();
        /* TODO: Add battery level and WiFi connection */
        ssd1306_draw_string(0, 5, "DIGI", true);
        ssd1306_draw_string(0, 25, temp_humi, true);
        ssd1306_draw_string(0, 35, acc, true);
        ssd1306_draw_string(0, 45, temp_humi_dht, true);
        ssd1306_draw_string(0, 45, current_acs, true);
        ssd1306_refresh();

        /* Init ESP as a http client */
        client = esp_http_client_init(&config);

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
}
