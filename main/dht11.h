#ifndef DHT11_H
#define DHT11_H

#include <stdio.h>
#include <driver/gpio.h>

/* Set pin num for DHT11 communication */
#define CONFIG_DHT11_PIN            GPIO_NUM_4
/* Set the number of sync tries with DHT11 */
#define CONFIG_CONNECTION_TIMEOUT   (5u)

/**
 * @brief   Containing readings and info about the DHT11
 * 
 * @var dht11_pin   The pin associated with the DHT11
 * @var temperature Last temperature reading
 * @var humidity    Last humidity reading 
*/
typedef struct
{
    gpio_num_t dht11_pin;
    float temperature;
    float humidity;
} dht11_t;

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
bool dht11_read(dht11_t *dht11, uint64_t connection_timeout);

#endif