#include "dht11.h"
#include <rom/ets_sys.h>
/* Debug enabled - show errors in monitoring window */
#define DEBUG
#include "util.h"

/* Time delay of 1 microsecond */
#define TIME_DELAY_1_US     (1u)
/* Time delay of 40 microseconds */
#define TIME_DELAY_40_US    (40u)
/* Time delay of 55 microseconds */
#define TIME_DELAY_55_US    (55u)
/* Time delay of 75 microseconds */
#define TIME_DELAY_75_US    (75u)
/* Time delay of 80 microseconds */
#define TIME_DELAY_80_US    (80u)
/* Time delay of 18 miliseconds */
#define TIME_DELAY_18_MS    (18000u)

/* Maximum length of data DHT11 can send */
#define MAX_DHT11_DATA_LEN  (5u)

/* Pin low state */
#define PIN_LOW     (0u)
/* Pin high state */
#define PIN_HIGH    (1u)

/* DHT11 debug tag */
#define DHT11_TAG   "DHT11"

/**
 * @brief   Wait on pin until it reaches the specified level
 *
 * @param[in]    level      Level to wait for
 * @param[in]    timeout_us If counter reaches timeout the function returns -1
 *
 * @returns  
 * - Returns either the time waited or -1 in the case of a timeout
*/
static uint64_t wait_pin_level(dht11_t dht11, uint16_t level, uint64_t timeout_us)
{
    uint64_t time = INIT_USGN_VAL;

    /* Set DHT pin direction to input mode */
    gpio_set_direction(dht11.dht11_pin, GPIO_MODE_INPUT);
    
    /* Wait for specified pin level */
    while((gpio_get_level(dht11.dht11_pin) != level) && (time < timeout_us))
    {
        time++;
        ets_delay_us(TIME_DELAY_1_US);
    }

    /* Check if while condition terminated by timeout and set error return if true */
    if (time >= timeout_us)
    {
        time = TIMEOUT_ERR;
    }

    return time;
}

/**
 * @brief   Holds the pin low for the specified duration
 *
 * @param[in]    hold_time_us   Time to hold the pin low for in microseconds
*/
static void hold_low(dht11_t dht11, int64_t hold_time_us)
{
    /* Set DHT pin direction to output mode */
    gpio_set_direction(dht11.dht11_pin, GPIO_MODE_OUTPUT);
    /* Set DHT pin level to low voltage */
    gpio_set_level(dht11.dht11_pin, PIN_LOW);
    /* Hold DHT pin to low voltage */
    ets_delay_us(hold_time_us);
    /* Set DHT pin level back to high voltage */
    gpio_set_level(dht11.dht11_pin, PIN_HIGH);
}

/**
 * @brief   The function for reading temperature and humidity values from the DHT11
 * @note    This function is blocking, ie: it forces the cpu to busy wait for the duration necessary to finish comms with the sensor.                             
 * @note    Wait for atleast 1 second between reads 
 * @param[in]    dht11              Pointer to DHT11 structure
 * @param[in]    connection_timeout The number of connection attempts before declaring a timeout
 *
 * @returns  
 * - Returns true if success, false if failure
*/
bool dht11_read(dht11_t *dht11, uint64_t connection_timeout)
{
    /* Save duration of pin high */
    uint64_t one_duration = INIT_USGN_VAL;
    /* Save duration of pin low */
    uint64_t zero_duration = INIT_USGN_VAL;
    /* Save the number of connections tries */
    uint64_t timeout_counter = INIT_USGN_VAL;
    /* Save checksum */
    uint8_t crc = INIT_USGN_VAL;
    /* Initialize data buffer with 0 */
    uint8_t received_data[MAX_DHT11_DATA_LEN] = {INIT_USGN_VAL};
    /* Synchronization flag */
    bool sync_flag = false;
    /* Return true if success or false if failed */
    bool ret = true;

    /* Try establish a connection for a connection_timeout number of time */
    while((sync_flag == false) && (timeout_counter < connection_timeout))
    {
        /* Increase tries of connections */
        timeout_counter++;

        /* Set DHT pin to input mode and hold it low for 18ms for start signal */
        gpio_set_direction(dht11->dht11_pin,GPIO_MODE_INPUT);
        hold_low(*dht11, TIME_DELAY_18_MS);
        
        /* Wait for low pin voltage response from DHT11 */
        if (wait_pin_level(*dht11, PIN_LOW, TIME_DELAY_40_US))
        {
            /* Wait for high pin voltage response from DHT11 */
            if (wait_pin_level(*dht11, PIN_HIGH, TIME_DELAY_80_US))
            {
                /* Wait for low pin voltage response from DHT11 */
                if (wait_pin_level(*dht11, PIN_LOW, TIME_DELAY_80_US))
                {
                    /* DHT synchronized, set flag */
                    sync_flag = true;
                    SHOW_INF(DHT11_TAG, "DHT11 synchronized");
                }
                else
                {
                    SHOW_ERR(DHT11_TAG, "Failed sync at phase 3");
                    ret = false;
                }
            }
            else
            {
                SHOW_ERR(DHT11_TAG, "Failed sync at phase 2");
                ret = false;
            }
        }
        else
        {
            SHOW_ERR(DHT11_TAG, "Failed sync at phase 1");
            ret = false;
        }
    }

    /* Check if DHT sensor synchronized */
    if (sync_flag)
    {   
        /* Read 5 bytes of data sent by DHT sensor */
        for(uint8_t byte_idx = INIT_USGN_VAL; byte_idx < MAX_DHT11_DATA_LEN; byte_idx++)
        {   
            /* Read bit by bit from every byte */
            for(uint8_t bit_idx = INIT_USGN_VAL; bit_idx < BYTE_LEN; bit_idx++)
            {
                /* Read duration starting data bit */
                zero_duration = wait_pin_level(*dht11, PIN_HIGH, TIME_DELAY_55_US);
                /* Read duration of high data bit */
                one_duration = wait_pin_level(*dht11, PIN_LOW, TIME_DELAY_75_US);
                /* Save bit in buffer */
                received_data[byte_idx] |= (one_duration > zero_duration) << (7 - bit_idx);
            }
        }

        /* CRC (checksum) is the sum of the data */
        crc = received_data[IDX_0] + received_data[IDX_1] + received_data[IDX_2] + received_data[IDX_3];

        /* Check the integrity of data with CRC from DHT */
        if (crc == received_data[IDX_4])
        {
            dht11->humidity = received_data[IDX_0] + received_data[IDX_1] / 10.0;
            dht11->temperature = received_data[IDX_2] + received_data[IDX_3] / 10.0;
        }
        else
        {
            SHOW_ERR(DHT11_TAG, "Wrong checksum");
            ret = false;
        }
    }

    return ret;   
}