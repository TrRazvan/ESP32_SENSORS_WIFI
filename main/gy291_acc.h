#ifndef GY291_ACC_H
#define GY291_ACC_H

#include "driver/i2c.h"

/**
 * @brief Initialize I2C and GY291 Accelerometer senzor.
 *
 * @param[in] i2c_num I2C num.
 * @returns ESP_OK in success, error code otherwise
 */
esp_err_t gy291_acc_init(i2c_port_t i2c_num);

/**
 * @brief Calibrate GY291 Accelerometer senzor.
 */
void calibrate_gy291();

/**
 * @brief Read GY291 Accelerometer senzor axis data.
 *
 * @param[out] x    Raw X axis
 * @param[out] y    Raw Y axis
 * @param[out] z    Raw Z axis
 * @param[out] gx   g units for x axis
 * @param[out] gy   g units for y axis
 * @param[out] gz   g units for z axis 
 * @returns ESP_OK in success, error code otherwise
 */
esp_err_t gy291_read_xyz(int16_t *x, int16_t *y, int16_t *z, float *gx, float *gy, float *gz);

#endif
