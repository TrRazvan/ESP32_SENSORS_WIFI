#ifndef ACS712_H
#define ACS712_H

#include "driver/adc.h"

typedef struct {
    adc1_channel_t adc_ch;
    adc_bits_width_t adc_width_bit;
    adc_atten_t adc_atten;
} acs712_t;

/**
 * @brief Initialize ADC1 for ACS217 sensor.
 *
 * @param[in] conf  Pointer to the config structure
 * @returns ESP_OK in success, error code otherwise
 */
esp_err_t acs712_init(acs712_t *conf);

/**
 * @brief Read ACS712 current.
 * @returns Current value.
 */
float acs712_read_current();

#endif