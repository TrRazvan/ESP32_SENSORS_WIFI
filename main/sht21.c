#include "sht21.h"
#define DEBUG
#include "util.h"

/* Terminal TAG*/
#define TAG "SHT21"
/* Disable I2C TX buffer */
#define I2C_MASTER_TX_BUF_DISABLE   (0u)
/* Disable I2C RX buffer */
#define I2C_MASTER_RX_BUF_DISABLE   (0u)
/* No interrupt I2C flags */
#define I2C_NO_INTR_FLAGS           (0u)
/* SHT21 sensor address */
#define SHT21_SENSOR_ADDR           (0x40)
/* Trigger temperature measurement command */
#define SHT21_TRIG_TEMP_MEASUREMENT (0xF3)
/* Trigger humidity measurement command */
#define SHT21_TRIG_HUM_MEASUREMENT  (0xF5)
/* Eliminate status bits mask */
#define ELIMINATE_STATUS_BITS_MASK  (0x0003)

/* 100 ms delay */
#define DELAY_100_MS    (100u)
/* 50 ms delay */
#define DELAY_50_MS     (50u)

/* The length of the data buffer */
#define SHT21_BUF_DATA_LEN  (3u)

static i2c_port_t global_i2c_num = INIT_USGN_VAL;

/**
 * @brief Read raw data from SHT21 via I2C.
 *
 * @param[in]  cmd          Command to trigger temperature/humidity measurement.
 * @param[out] raw_value    Variable to save raw data.
 * @returns ESP_OK in success, error code otherwise
 */
static esp_err_t sht21_read_raw_data(const uint8_t cmd, uint16_t *raw_value)
{
    uint8_t data[SHT21_BUF_DATA_LEN] = {INIT_USGN_VAL};
    esp_err_t ret = ESP_OK;

    /* Create and initialize an I2C commands list with a given buffer */
    i2c_cmd_handle_t handle = i2c_cmd_link_create();

    /* Trigger measurement */
    /* Queue a "START signal" to the given commands list */
    i2c_master_start(handle);
    /* Queue a "write byte" command to the commands list */
    i2c_master_write_byte(handle, (SHT21_SENSOR_ADDR << SHIFT_1_BIT) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(handle, cmd, true);
    /* Queue a "STOP signal" to the given commands list */
    i2c_master_stop(handle);
    /* Send all the queued commands on the I2C bus, in master mode */
    i2c_master_cmd_begin(global_i2c_num, handle, pdMS_TO_TICKS(DELAY_50_MS));
    /* Free the I2C commands list */
    i2c_cmd_link_delete(handle);

    /* Conversion delay */
    vTaskDelay(pdMS_TO_TICKS(DELAY_100_MS));

    /* Read measurement data */
    handle = i2c_cmd_link_create();
    i2c_master_start(handle);
    i2c_master_write_byte(handle, (SHT21_SENSOR_ADDR << SHIFT_1_BIT) | I2C_MASTER_READ, true);
    i2c_master_read(handle, data, 3, I2C_MASTER_LAST_NACK);
    i2c_master_stop(handle);
    ret = i2c_master_cmd_begin(global_i2c_num, handle, pdMS_TO_TICKS(DELAY_50_MS));
    i2c_cmd_link_delete(handle);

    if (ret == ESP_OK)
    {
        /* Concatenate 2 x 8 bytes data in a uint16_t data pointer */
        *raw_value = (data[IDX_0] << BYTE_LEN) | data[IDX_1];
        /* Eliminate status bits */
        *raw_value &= ~ELIMINATE_STATUS_BITS_MASK;
    }
    else
    {
        SHOW_ERR(TAG, "SHT21 read data error");
    }

    return ret;
}

/**
 * @brief Initialize I2C driver for SHT21 sensor.
 *
 * @param[in] i2c_num I2C num.
 * @returns ESP_OK in success, error code otherwise
 */
esp_err_t sht21_init(i2c_port_t i2c_num)
{
    esp_err_t ret = ESP_OK;

    global_i2c_num = i2c_num;

    return ret;
}

/**
 * @brief Initialize I2C driver for SHT21 sensor.
 *
 * @param[out] temperature  Variable to save temperature data.
 * @param[out] humidity     Variable to save humidity data.
 * @returns ESP_OK in success, error code otherwise
 */
esp_err_t sht21_read(float *temperature, float *humidity)
{
    esp_err_t ret = ESP_OK;

    uint16_t raw_temp_data = INIT_USGN_VAL;
    uint16_t raw_humi_data = INIT_USGN_VAL;

    /* Trigger temperature measurement and read raw data */
    if ((sht21_read_raw_data(SHT21_TRIG_TEMP_MEASUREMENT, &raw_temp_data) == ESP_OK) && (sht21_read_raw_data(SHT21_TRIG_HUM_MEASUREMENT, &raw_humi_data) == ESP_OK))
    {
        /* Calculate temeperature in Celsius */
        *temperature = -46.85 + 175.72 * (float)raw_temp_data / 65536.0;
        /* Calculate humidity in procents */
        *humidity = -6.0 + 125.0 * (float)raw_humi_data / 65536.0;
    }
    else
    {
        ret = ESP_FAIL;
        SHOW_ERR(TAG, "SHT21 read raw data error");
    }    

    return ret;
}
