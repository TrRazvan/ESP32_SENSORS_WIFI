idf_component_register(SRCS "dht11.c" "ssd1306.c" "gy291_acc.c" "main.c" "ap_wifi.c"
                    PRIV_REQUIRES spi_flash driver
                    INCLUDE_DIRS ""
                    REQUIRES esp_wifi esp_netif esp_http_server esp_http_client nvs_flash esp_system)
