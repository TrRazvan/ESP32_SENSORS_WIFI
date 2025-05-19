#ifndef SSD1306_H
#define SSD1306_H

#include "driver/spi_master.h"
#include "esp_err.h"

/* The width of the display */
#define SSD1306_WIDTH   (128u)
/* The height of the display */
#define SSD1306_HEIGHT  (64u)
/* The pages display have */
#define DISPLAY_PAGES   (8u)

typedef struct {
    spi_device_handle_t spi;    /* SPI handler for OLED display*/
    spi_host_device_t spi_host;  /* SPI Host ID */
    uint8_t pin_dc;             /* Pin num for DC pin */
    uint8_t pin_rst;            /* Pin num for RST pin */
    uint8_t pin_mosi;           /* Pin num for MOSI pin */
    uint8_t pin_clk;            /* Pin num for CLK pin */
    uint8_t pin_cs;             /* Pin num for CS pin */
    uint8_t buffer[SSD1306_WIDTH * SSD1306_HEIGHT / DISPLAY_PAGES];  /* 128x64 pixels */
} ssd1306_t;

/**
 * @brief Initialize SSD1306 OLED display.
 *
 * @param[in/out] ssd SSD1306 structure handler.
 */
esp_err_t ssd1306_init(ssd1306_t *ssd);

/**
 * @brief Clears the display.
 */
void ssd1306_clear(void);

/**
 * @brief Draw an individual pixel on display.
 *
 * @param[in] x     x axis position for pixel.
 * @param[in] y     y axis position for pixel.
 * @param[in] color True for iluminated pixel, false for not iluminated pixel.
 * @returns
 *  - true if pixel was drawn 
 *  - false otherwise
 */
bool ssd1306_draw_pixel(uint8_t x, uint8_t y, bool color);

/**
 * @brief Draw a character on display.
 *
 * @param[in] x     x axis position for pixel.
 * @param[in] y     y axis position for pixel.
 * @param[in] c     Character to be drawn.
 * @param[in] color True for iluminated pixel, false for not iluminated pixel.
 */
void ssd1306_draw_char(uint8_t x, uint8_t y, char c, bool color);

/**
 * @brief Draw a string on display.
 *
 * @param[in] x     x axis position for pixel.
 * @param[in] y     y axis position for pixel.
 * @param[in] str   String to be drawn.
 * @param[in] color True for iluminated pixel, false for not iluminated pixel.
 */
void ssd1306_draw_string(uint8_t x, uint8_t y, const char* str, bool color);

/**
 * @brief Refresh display with all the buffer content.
 */
void ssd1306_refresh(void);

#endif