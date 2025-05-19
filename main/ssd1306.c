#include "ssd1306.h"
#include "driver/gpio.h"
#include "freertos/task.h"
#include <string.h>
#define DEBUG
#include "util.h"

/* Status pixel to ON */
#define PIXEL_ON        (1u)
/* The integer number for first ASCII character */
#define MIN_ASCII_CODE  (32u)
/* The integer number for last ASCII character */
#define MAX_ASCII_CODE  (127u)
/* Delay of 10 ms */
#define DELAY_10_MS     (10u)
/* Font Height*/
#define FONT_HEIGHT     (7u)
/* Font width */
#define FONT_WIDTH      (5u)
/* Mask for bit extraction */
#define EXTRACT_BIT_MASK    (0x01)
/* Null parameter */
#define NULL_PARAMETER  (-1)
/* SPI Clock Speed in Hertz */
#define SPI_CLK_SPEED_HZ (10*1000*1000u)
/* SPI queue size */
#define SPI_QUEUE_SIZE  (1u)
/* Terminal TAG */
#define TAG "SSD1306"

/* Local ssd config data */
static ssd1306_t *global_ssd;

/* ASCII characters in 5x7 pixels*/
static const uint8_t font5x7[][5] = {
    /* Figures '0' - '9' */
    ['0' - 32] = {0x3E,0x51,0x49,0x45,0x3E},
    ['1' - 32] = {0x00,0x42,0x7F,0x40,0x00},
    ['2' - 32] = {0x42,0x61,0x51,0x49,0x46},
    ['3' - 32] = {0x21,0x41,0x45,0x4B,0x31},
    ['4' - 32] = {0x18,0x14,0x12,0x7F,0x10},
    ['5' - 32] = {0x27,0x45,0x45,0x45,0x39},
    ['6' - 32] = {0x3C,0x4A,0x49,0x49,0x30},
    ['7' - 32] = {0x01,0x71,0x09,0x05,0x03},
    ['8' - 32] = {0x36,0x49,0x49,0x49,0x36},
    ['9' - 32] = {0x06,0x49,0x49,0x29,0x1E},

    /* Uppercase letters 'A' - 'Z' */
    ['A' - 32] = {0x7E,0x11,0x11,0x11,0x7E},
    ['B' - 32] = {0x7F,0x49,0x49,0x49,0x36},
    ['C' - 32] = {0x3E,0x41,0x41,0x41,0x22},
    ['D' - 32] = {0x7F,0x41,0x41,0x22,0x1C},
    ['E' - 32] = {0x7F,0x49,0x49,0x49,0x41},
    ['F' - 32] = {0x7F,0x09,0x09,0x09,0x01},
    ['G' - 32] = {0x3E,0x41,0x49,0x49,0x7A},
    ['H' - 32] = {0x7F,0x08,0x08,0x08,0x7F},
    ['I' - 32] = {0x00,0x41,0x7F,0x41,0x00},
    ['J' - 32] = {0x20,0x40,0x41,0x3F,0x01},
    ['K' - 32] = {0x7F,0x08,0x14,0x22,0x41},
    ['L' - 32] = {0x7F,0x40,0x40,0x40,0x40},
    ['M' - 32] = {0x7F,0x02,0x0C,0x02,0x7F},
    ['N' - 32] = {0x7F,0x04,0x08,0x10,0x7F},
    ['O' - 32] = {0x3E,0x41,0x41,0x41,0x3E},
    ['P' - 32] = {0x7F,0x09,0x09,0x09,0x06},
    ['Q' - 32] = {0x3E,0x41,0x51,0x21,0x5E},
    ['R' - 32] = {0x7F,0x09,0x19,0x29,0x46},
    ['S' - 32] = {0x46,0x49,0x49,0x49,0x31},
    ['T' - 32] = {0x01,0x01,0x7F,0x01,0x01},
    ['U' - 32] = {0x3F,0x40,0x40,0x40,0x3F},
    ['V' - 32] = {0x1F,0x20,0x40,0x20,0x1F},
    ['W' - 32] = {0x3F,0x40,0x38,0x40,0x3F},
    ['X' - 32] = {0x63,0x14,0x08,0x14,0x63},
    ['Y' - 32] = {0x07,0x08,0x70,0x08,0x07},
    ['Z' - 32] = {0x61,0x51,0x49,0x45,0x43},

    /* Special ASCII characters */
    [' ' - 32] = {0x00,0x00,0x00,0x00,0x00},
    ['%' - 32] = {0x23,0x13,0x08,0x64,0x62},
    ['(' - 32] = {0x00,0x1C,0x22,0x41,0x00},
    [')' - 32] = {0x00,0x41,0x22,0x1C,0x00},
    ['*' - 32] = {0x14,0x08,0x3E,0x08,0x14},
    ['+' - 32] = {0x08,0x08,0x3E,0x08,0x08},
    [',' - 32] = {0x00,0x50,0x30,0x00,0x00},
    ['-' - 32] = {0x08,0x08,0x08,0x08,0x08},
    ['.' - 32] = {0x00,0x60,0x60,0x00,0x00},
    [':' - 32] = {0x00,0x36,0x36,0x00,0x00},
    [';' - 32] = {0x00,0x56,0x36,0x00,0x00},
    ['<' - 32] = {0x08,0x14,0x22,0x41,0x00},
    ['=' - 32] = {0x14,0x14,0x14,0x14,0x14},
    ['>' - 32] = {0x00,0x41,0x22,0x14,0x08},
    ['?' - 32] = {0x02,0x01,0x51,0x09,0x06},
    ['[' - 32] = {0x00,0x7F,0x41,0x41,0x00},
    [']' - 32] = {0x00,0x41,0x41,0x7F,0x00},
    ['^' - 32] = {0x04,0x02,0x01,0x02,0x04},
    ['{' - 32] = {0x08,0x36,0x41,0x41,0x00},
    ['|' - 32] = {0x00,0x00,0x7F,0x00,0x00},
    ['}' - 32] = {0x00,0x41,0x41,0x36,0x08},
    ['~' - 32] = {0x08,0x04,0x08,0x10,0x08},
};

/**
 * @brief Resets the display.
 */
static void ssd1306_reset(void)
{
    gpio_set_level(global_ssd->pin_rst, PIN_LOW_LEVEL);
    vTaskDelay(pdMS_TO_TICKS(DELAY_10_MS));
    gpio_set_level(global_ssd->pin_rst, PIN_HIGH_LEVEL);
    vTaskDelay(pdMS_TO_TICKS(DELAY_10_MS));
}

/**
 * @brief Writes command to display.
 *
 * @param[in] cmd   Command to be written.
 */
static void ssd1306_write_cmd(uint8_t cmd)
{
    spi_transaction_t t = {
        .length = BYTE_LEN,
        .tx_buffer = &cmd,
    };

    /* Set DC pin to low for command transmitting */
    gpio_set_level(global_ssd->pin_dc, PIN_LOW_LEVEL);

    /* Transmit the command via SPI */
    spi_device_transmit(global_ssd->spi, &t);
}

/**
 * @brief Writes data to display.
 *
 * @param[in] data  Command to be written.
 * @param[in] len   The length of the data.
 */
static void ssd1306_write_data(const uint8_t* data, uint16_t len)
{
    spi_transaction_t t = {
        .length = len * BYTE_LEN,
        .tx_buffer = data,
    };

    /* Set DC pin to low for data transmitting */
    gpio_set_level(global_ssd->pin_dc, PIN_HIGH_LEVEL);
    
    /* Transmit the data via SPI */
    spi_device_transmit(global_ssd->spi, &t);
}

/**
 * @brief Initialize SSD1306 OLED display.
 *
 * @param[in/out] ssd SSD1306 structure handler.
 */
esp_err_t ssd1306_init(ssd1306_t *ssd)
{
    spi_bus_config_t buscfg = {
        .miso_io_num = NULL_PARAMETER,
        .mosi_io_num = ssd->pin_mosi,
        .sclk_io_num = ssd->pin_clk,
        .quadwp_io_num = NULL_PARAMETER,
        .quadhd_io_num = NULL_PARAMETER,
    };

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = SPI_CLK_SPEED_HZ,
        .mode = INIT_USGN_VAL,
        .spics_io_num = ssd->pin_cs,
        .queue_size = SPI_QUEUE_SIZE,
    };

    esp_err_t ret = spi_bus_initialize(ssd->spi_host, &buscfg, SPI_DMA_CH_AUTO);
    if (ret == ESP_OK)
    {
        ret = spi_bus_add_device(ssd->spi_host, &devcfg, &ssd->spi);
        if (ret == ESP_OK)
        {
            /* Copy initialization arguments to local data */
            global_ssd = ssd;

            /* Set DC and RST pin to output */
            gpio_set_direction(ssd->pin_dc, GPIO_MODE_OUTPUT);
            gpio_set_direction(ssd->pin_rst, GPIO_MODE_OUTPUT);

            /* Reset the display for initialization */
            ssd1306_reset();

            /* Official sequence for SSD1306 initialization */
            ssd1306_write_cmd(0xAE); /* Display OFF */
            ssd1306_write_cmd(0xD5); /* Set Display Clock Divide Ratio/Oscillator Frequency */
            ssd1306_write_cmd(0x80); /* 1:1 ratio */
            ssd1306_write_cmd(0xA8); /* Set Multiplex Ratio */
            ssd1306_write_cmd(0x3F); /* Set 64 pixels */
            ssd1306_write_cmd(0xD3); /* Set Display Offset */
            ssd1306_write_cmd(0x00); /* Offset to 0 */
            ssd1306_write_cmd(0x40); /* Set Start Line */
            ssd1306_write_cmd(0x8D); /* Charge Pump Setting */
            ssd1306_write_cmd(0x14); /* Set Pump ON */
            ssd1306_write_cmd(0x20); /* Memory Addressing Mode */
            ssd1306_write_cmd(0x00); /* Horizontal Addressing Mode */
            ssd1306_write_cmd(0xA1); /* Segment Re-map */
            ssd1306_write_cmd(0xC8); /* COM Output Scan Direction */
            ssd1306_write_cmd(0xDA); /* COM Pins Hardware Configuration */
            ssd1306_write_cmd(0x12); /* 128x64 layout standard */
            ssd1306_write_cmd(0x81); /* Contrast Control */
            ssd1306_write_cmd(0xCF); /* Set powerfull contrast */
            ssd1306_write_cmd(0xD9); /* Pre-charge Period */
            ssd1306_write_cmd(0xF1); /* Value for pre-char Period */
            ssd1306_write_cmd(0xDB); /* VCOMH Deselect Level */
            ssd1306_write_cmd(0x40); /* Set VCOMH Level */
            ssd1306_write_cmd(0xA4); /* Entire Display ON from RAM */
            ssd1306_write_cmd(0xA6); /* Normal Display */
            ssd1306_write_cmd(0xAF); /* Display ON */

            /* Clear and refresh display */
            ssd1306_clear();
            ssd1306_refresh();
        }
        else
        {
            SHOW_ERR(TAG, "SPI device add failed");
        }
    }
    else
    {
        SHOW_ERR(TAG, "SPI bus init failed");
    }

    return ret;
}

/**
 * @brief Clears the display.
 */
void ssd1306_clear(void)
{
    /* Fill image buffer with 0 -> clears the display */
    memset(global_ssd->buffer, INIT_USGN_VAL, sizeof(global_ssd->buffer));
}

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
bool ssd1306_draw_pixel(uint8_t x, uint8_t y, bool color)
{
    bool ret = true;

    /* Find the right pixel in the buffer */
    uint16_t byte_index = x + (y / BYTE_LEN) * SSD1306_WIDTH;

    /* Check if given coordinates are in the display dimensions */
    if (x >= SSD1306_WIDTH || y >= SSD1306_HEIGHT)
    {
        /* Pixel was not drawn */
        ret = false;
    }
    else if (color)
    {
        /* Get pixel from the byte and turn on the pixel */
        global_ssd->buffer[byte_index] |= (PIXEL_ON << (y % BYTE_LEN));
    }
    else
    {
        /* Get pixel from the byte and turn off the pixel */
        global_ssd->buffer[byte_index] &= ~(PIXEL_ON << (y % BYTE_LEN));
    }
    
    return ret;
}

/**
 * @brief Draw a character on display.
 *
 * @param[in] x     x axis position for pixel.
 * @param[in] y     y axis position for pixel.
 * @param[in] c     Character to be drawn.
 * @param[in] color True for iluminated pixel, false for not iluminated pixel.
 */
void ssd1306_draw_char(uint8_t x, uint8_t y, char c, bool color)
{
    uint8_t line = INIT_USGN_VAL;
    bool pixel_on = false;

    /* Check if the given character is valid */
    if ((c >= MIN_ASCII_CODE) && (c <= MAX_ASCII_CODE))
    {   
        /* Select character font from bitmap */
        const uint8_t *glyph = font5x7[c - MIN_ASCII_CODE];
        /* Cover every pixel from character */
        for (uint8_t i = INIT_USGN_VAL; i < FONT_WIDTH; i++)
        {
            line = glyph[i];
            for (uint8_t j = INIT_USGN_VAL; j < FONT_HEIGHT; j++)
            {   
                /* Extract every bit from column line */
                pixel_on = (line >> j) & EXTRACT_BIT_MASK;
                /* Turn on the pixel if it is true */
                ssd1306_draw_pixel(x + i, y + j, pixel_on ? color : !color);
            }
        }
    }
}

/**
 * @brief Draw a string on display.
 *
 * @param[in] x     x axis position for pixel.
 * @param[in] y     y axis position for pixel.
 * @param[in] str   String to be drawn.
 * @param[in] color True for iluminated pixel, false for not iluminated pixel.
 */
void ssd1306_draw_string(uint8_t x, uint8_t y, const char* str, bool color)
{
    /* Draw character by character */
    while (*str)
    {
        ssd1306_draw_char(x, y, *str++, color);
        /* 5 pixels + 1 space */
        x += FONT_WIDTH + 1;
        if (x + FONT_WIDTH >= SSD1306_WIDTH)
        {
            break;
        }
    }
}

/**
 * @brief Refresh display with all the buffer content.
 */
void ssd1306_refresh(void)
{
    /* Cover every page of display (128x64 pixels = 8 pages × 8 lines/page × 128 columns) */
    for (uint8_t page = INIT_USGN_VAL; page < DISPLAY_PAGES; page++)
    {
        /* Selecte page for writing data */
        ssd1306_write_cmd(0xB0 + page);
        /* Set cursor to column 0 on current page */
        ssd1306_write_cmd(0x00);
        ssd1306_write_cmd(0x10);

        /* Send data to page */
        ssd1306_write_data(&global_ssd->buffer[page * SSD1306_WIDTH], SSD1306_WIDTH);
    }
}
