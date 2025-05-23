#include "acs712.h"
#define DEBUG
#include "util.h"

/* Samples for average calculation */
#define ADC_AVG_SAMPLES     (100u)
/* ADC resolutin of 12 bits -> 4095 samples */
#define ADC_MAX_RESOLUTION  (4095.0)
/* Current offset after applying voltage divider */
#define ZERO_CURRENT_VOLT   (1.65)
/* Sensor sensivity after applying voltage divider */        
#define SENSITIVITY         (0.1221)
/* ADC maximum voltage */
#define ADC_MAX_VOLTAGE     (3.3)

/* Terminal tag */
#define TAG "ACS721"

/* Local config structure */
static acs712_t *global_conf;

/**
 * @brief Initialize ADC1 for ACS217 sensor.
 *
 * @param[in] conf  Pointer to the config structure
 * @returns ESP_OK in success, error code otherwise
 */
esp_err_t acs712_init(acs712_t *conf)
{
    esp_err_t ret = ESP_OK;

    /* Copy configuration pointer to a global pointer */
    global_conf = conf;
    
    /* Config ADC bits witdh */
    ret = adc1_config_width(conf->adc_width_bit);
    if (ret == ESP_OK)
    {
        /* Config read permissions */
        ret = adc1_config_channel_atten(conf->adc_ch, conf->adc_atten);
        if (ret != ESP_OK)
        {
            SHOW_ERR(TAG, "ADC1 config ch atten error");
        }
    }
    else
    {
        SHOW_ERR(TAG, "ADC1 config width error");
    }

    return ret;
}

/**
 * @brief Read average ADC voltage value.
 * @returns Average ADC voltage value.
 */
static float acs712_read_average_voltage()
{
    uint32_t sum = INIT_USGN_VAL;
    int val = INIT_SGN_VAL;
    float avg = INIT_SGN_VAL;

    /* Read multiple values and sum them */
    for (uint8_t i = INIT_USGN_VAL; i < ADC_AVG_SAMPLES; i++)
    {
        /* Read ADC value */
        val = adc1_get_raw(global_conf->adc_ch);

        /* Sum values */
        sum += val;
    }

    /* Calculate the average of values */
    avg = sum / (float)ADC_AVG_SAMPLES;

    /* Calculate and return adc voltage */
    return (avg / ADC_MAX_RESOLUTION) * ADC_MAX_VOLTAGE;
}

/**
 * @brief Read ACS712 current.
 * @returns Current value.
 */
float acs712_read_current()
{
    float voltage = INIT_SGN_VAL;
    float current = INIT_SGN_VAL;

    /* Read average voltage */
    voltage = acs712_read_average_voltage();

    /* Calculate the current */
    current = (voltage - ZERO_CURRENT_VOLT) / SENSITIVITY;

    return current;
}