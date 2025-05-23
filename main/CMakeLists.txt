idf_component_register(SRCS "ap_wifi.c" "main.c" "ssd1306.c" "gy291_acc.c" "sht21.c" "acs712.c"
                    PRIV_REQUIRES spi_flash driver
                    INCLUDE_DIRS ""
                    REQUIRES esp_wifi esp_netif esp_http_server esp_http_client nvs_flash)
