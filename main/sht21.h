#ifndef SHT21_H
#define SHT21_H 

#include "driver/i2c.h"

/**
 * @brief Initialize I2C driver for SHT21 sensor.
 *
 * @param[in] i2c_num I2C num.
 * @returns ESP_OK in success, error code otherwise
 */
esp_err_t sht21_init(i2c_port_t i2c_num);

/**
 * @brief Initialize I2C driver for SHT21 sensor.
 *
 * @param[out] temperature  Variable to save temperature data.
 * @param[out] humidity     Variable to save humidity data.
 * @returns ESP_OK in success, error code otherwise
 */
esp_err_t sht21_read(float *temperature, float *humidity);

#endif