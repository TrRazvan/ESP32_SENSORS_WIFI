#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "gy291_acc.h"
#define DEBUG
#include "util.h"

/* Accelerometer I2C address */
#define GY291_ADDR              (0x53)
/* Accelerometer data format register */
#define GY291_REG_DATA_FORMAT   (0x31)
/* Accelerometer power control register */
#define GY291_REG_POWER_CTL     (0x2D)
/* Accelerometer start data register */
#define GY291_REG_DATAX0        (0x32)
/* Axis data size */
#define GY291_AXIS_DATA_SIZE    (6u)

/* Used samples for calibration */
#define CALIBRATION_SAMPLES     (100)
/* Buffer dimension for average filter */
#define FILTER_DEPTH            (10)
/* Scaling factor for conversion to gravity */
#define SCALE_MULTIPLIER        (0.0078f)

/* Receiving buffer size. Only slave mode will use this value, it is ignored in master mode */
#define I2C_RX_BUFF_SIZE    (0u)
/* Sending buffer size. Only slave mode will use this value, it is ignored in master mode */
#define I2C_TX_BUFF_SIZE    (0u)
/* Flags used to allocate the interrupt */
#define I2C_INTR_FLAGS      (0u)

/* 100 ms timeout */
#define TIMEOUT_100_MS      (100u)
/* 10 ms delay */
#define DELAY_10_MS         (10u)
/* Size of the data to be written in register */
#define WRITE_REG_DATA_SIZE (2u)
/* Size of the data to be read from register */
#define READ_REG_DATA_SIZE  (1u)

/* Tag for terminal */
#define TAG "GY291"

/* Local config variable */
static gy291_acc_t *global_conf = NULL;

/* Global variables for calibration offset */
int16_t offset_x = INIT_SGN_VAL;
int16_t offset_y = INIT_SGN_VAL;
int16_t offset_z = INIT_SGN_VAL;

/* Filter buffers for every axis */
int32_t filter_x[FILTER_DEPTH] = {INIT_SGN_VAL};
int32_t filter_y[FILTER_DEPTH] = {INIT_SGN_VAL};
int32_t filter_z[FILTER_DEPTH] = {INIT_SGN_VAL};

/**
 * @brief Write in GY291 Accelerometer register.
 *
 * @param[in] reg   Register to be written.
 * @param[in] data  Data to be written in register.
 * @returns ESP_OK in success, error code otherwise
 */
static esp_err_t gy291_write(const uint8_t reg, const uint8_t data)
{   
    esp_err_t ret = ESP_OK;

    /* Write GY291 register via I2C */
    ret = i2c_master_write_to_device(global_conf->i2c_num, GY291_ADDR, (uint8_t[]){reg, data}, WRITE_REG_DATA_SIZE, pdMS_TO_TICKS(TIMEOUT_100_MS));
    if (ret != ESP_OK)
    {
        SHOW_ERR(TAG, "GY291 register writting error");
    }

    return ret;
}

/**
 * @brief Read from GY291 Accelerometer register.
 *
 * @param[in] reg   Register to be read.
 * @param[in] buf   Buffer which saves read data.
 * @param[in] len   Data size to be read.
 * @returns ESP_OK in success, error code otherwise
 */
static esp_err_t gy291_read(const uint8_t reg, uint8_t *buf, const size_t len)
{
    esp_err_t ret = ESP_OK;
    
    /* Check if buf is null */
    if (buf == NULL)
    {
        SHOW_ERR(TAG, "GY291 read register null buffer");
        ret = ESP_FAIL;
    }
    else
    {   /* Read 1 byte from GY291 register */
        ret = i2c_master_write_read_device(global_conf->i2c_num, GY291_ADDR, &reg, READ_REG_DATA_SIZE, buf, len, pdMS_TO_TICKS(TIMEOUT_100_MS));
        if (ret != ESP_OK)
        {
            SHOW_ERR(TAG, "GY291 read register error");
        }
    }

    return ret;
}

/**
 * @brief Apply a moving average filter to a buffer.
 *
 * @param[out] buffer   Buffer with filtered data.
 * @param[in]  value    Value to be filtered.    
 */
static int16_t apply_filter(int32_t *buffer, const int16_t value)
{
    int32_t sum = INIT_SGN_VAL;

    for (int i = FILTER_DEPTH - IDX_1; i > INIT_SGN_VAL; i--)
    {
        /* Move values to the final */
        buffer[i] = buffer[i - IDX_1];
    }

    /* Add new value in front */
    buffer[IDX_0] = value;

    for (int i = INIT_SGN_VAL; i < FILTER_DEPTH; i++) 
    {
        /* Sum */
        sum += buffer[i];
    }

    /* Return average */
    return (int16_t)(sum / FILTER_DEPTH);
}

/**
 * @brief Initialize I2C and GY291 Accelerometer senzor.
 *
 * @param[in] conf  Pointer to the configuration structure.
 * @returns ESP_OK in success, error code otherwise
 */
esp_err_t gy291_acc_init(gy291_acc_t *conf)
{
    esp_err_t ret = ESP_OK;

    /* Check if config arg is null */
    if (conf == NULL)
    {
        SHOW_ERR(TAG, "GY291 init error");
    }
    else
    {
        /* Config I2C controller */
        ret = i2c_param_config(conf->i2c_num, &conf->conf);
        if (ret == ESP_OK)
        {
            /* Install I2C driver */
            ret = i2c_driver_install(conf->i2c_num, conf->conf.mode, I2C_RX_BUFF_SIZE, I2C_TX_BUFF_SIZE, I2C_INTR_FLAGS);
            if (ret == ESP_OK)
            {
                /* Copy config data to local variable */
                global_conf = conf;

                /* Activate senzor */
                ret = gy291_write(GY291_REG_POWER_CTL, 0x08);
                if (ret == ESP_OK)
                {
                    /* Set ±4g interval */
                    ret = gy291_write(GY291_REG_DATA_FORMAT, 0x01);
                    if (ret != ESP_OK)
                    {
                        SHOW_ERR(TAG, "GY291 set interval error");
                    }                    
                }
                else
                {
                    SHOW_ERR(TAG, "GY291 activate error");
                }
            }
            else
            {
                SHOW_ERR(TAG, "GY291 I2C driver install error");
            }
        }
        else
        {
            SHOW_ERR(TAG, "GY291 I2C config controller error");
        }
    }

    return ret;
}

/**
 * @brief Calibrate GY291 Accelerometer senzor.
 */
void calibrate_gy291()
{
    int32_t sum_x = INIT_SGN_VAL;
    int32_t sum_y = INIT_SGN_VAL;
    int32_t sum_z = INIT_SGN_VAL;
    int16_t x = INIT_SGN_VAL;
    int16_t y = INIT_SGN_VAL;
    int16_t z = INIT_SGN_VAL;

    /* Read CALIBRATION_SAMPLES times */
    for (uint16_t i = INIT_USGN_VAL; i < CALIBRATION_SAMPLES; i++)
    {
        gy291_read_xyz(&x, &y, &z, NULL, NULL, NULL);

        /* Add values to sum */
        sum_x += x;
        sum_y += y;
        sum_z += z;
        vTaskDelay(pdMS_TO_TICKS(DELAY_10_MS));
    }

    offset_x = sum_x / CALIBRATION_SAMPLES;
    offset_y = sum_y / CALIBRATION_SAMPLES;
    offset_z = (sum_z / CALIBRATION_SAMPLES);
}

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
esp_err_t gy291_read_xyz(int16_t *x, int16_t *y, int16_t *z, float *gx, float *gy, float *gz)
{
    esp_err_t ret = ESP_OK;

    uint8_t data[GY291_AXIS_DATA_SIZE];

    /* Read 6 bytes (DATAX0 -> DATAZ1) */
    ret = gy291_read(GY291_REG_DATAX0, data, GY291_AXIS_DATA_SIZE);
    if (ret == ESP_OK)
    {
        /* Parse read axis data */
        *x = ((int16_t)(data[1] << 8) | data[0]) - offset_x;
        *y = ((int16_t)(data[3] << 8) | data[2]) - offset_y;
        *z = ((int16_t)(data[5] << 8) | data[4]) - offset_z;

        /* Filter data */
        *x = apply_filter(filter_x, *x);
        *y = apply_filter(filter_y, *y);
        *z = apply_filter(filter_z, *z);

        /* If these are null, function called for calibration */
        if ((gx != NULL) && (gy != NULL) && (gz != NULL))
        {
            /* Convert in g units */
            *gx = *x * SCALE_MULTIPLIER;
            *gy = *y * SCALE_MULTIPLIER;
            *gz = *z * SCALE_MULTIPLIER;
        }
    }
    else
    {
        SHOW_ERR(TAG, "GY291 read axis error");
    }

    return ret;
}
