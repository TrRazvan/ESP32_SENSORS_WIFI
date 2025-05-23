#ifndef UTIL_H
#define UTIL_H

#include "esp_log.h"

/* If debug mode, enable printing functions */
#ifdef DEBUG
#define SHOW_ERR(tag, format, ...)   ESP_LOGE(tag, format, ##__VA_ARGS__)
#define SHOW_INF(tag, format, ...)   ESP_LOGI(tag, format, ##__VA_ARGS__)
#define SHOW_WAR(tag, format, ...)   ESP_LOGW(tag, format, ##__VA_ARGS__)
#else
#define SHOW_ERR(...)
#define SHOW_INF(...)
#define SHOW_WAR(...)
#endif

/* Initialize variables with signed 0 */
#define INIT_SGN_VAL    (0)
/* Initialize variables with unsigned 0 */
#define INIT_USGN_VAL   (0u)

/* Low level pin */
#define PIN_LOW_LEVEL   (0u)
/* High level pin */
#define PIN_HIGH_LEVEL  (1u)

/* Timeout error code */
#define TIMEOUT_ERR (-1)

/* Byte length in bits */
#define BYTE_LEN    (8u)

/* Shift 1 bit */
#define SHIFT_1_BIT  (1u)

/* Null parameter */
#define NULL_PARAMETER  (-1)

/**
 * @brief Contains indexes
 */
typedef enum {
    IDX_0 = 0u, /* Index 0 */
    IDX_1,
    IDX_2,
    IDX_3,
    IDX_4
} index_t;

#endif