/*
 * SPDX-FileCopyrightText: 2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_rgb.h"
#include "esp_lcd_touch_cytma568.h"
#include "esp_log.h"
#include "esp_rom_sys.h"

#include "sdkconfig.h"
#include "bsp_err_check.h"
#include "bsp_sub_board.h"
#include "bsp/esp32_s3_lcd_ev_board.h"

#define Delay(t)        vTaskDelay(pdMS_TO_TICKS(t))
#define udelay(t)       esp_rom_delay_us(t)
#define CS(n)           BSP_ERROR_CHECK_RETURN_ERR(gpio_set_level(BSP_LCD_SPI_CS, n))
#define SCK(n)          BSP_ERROR_CHECK_RETURN_ERR(gpio_set_level(BSP_LCD_SPI_SCK, n))
#define SDO(n)          BSP_ERROR_CHECK_RETURN_ERR(gpio_set_level(BSP_LCD_SPI_SDO, n))

static const char *TAG = "SUB-BOARD_480x480";

static bsp_lcd_trans_done_cb_t trans_done = NULL;
#if CONFIG_BSP_LCD_RGB_REFRESH_MANUALLY
static TaskHandle_t lcd_task_handle = NULL;
#endif

static esp_err_t lcd_config();

/**************************************************************************************************
 *
 * LCD Panel Function
 *
 **************************************************************************************************/
IRAM_ATTR static bool on_vsync_event(esp_lcd_panel_handle_t panel, const esp_lcd_rgb_panel_event_data_t *edata, void *user_ctx)
{
    BaseType_t need_yield = pdFALSE;
#if CONFIG_BSP_LCD_RGB_REFRESH_MANUALLY
    xTaskNotifyFromISR(lcd_task_handle, ULONG_MAX, eNoAction, &need_yield);
#endif
    if (trans_done) {
        if (trans_done(panel)) {
            need_yield = pdTRUE;
        }
    }

    return (need_yield == pdTRUE);
}

#if CONFIG_BSP_LCD_RGB_REFRESH_MANUALLY
static void lcd_task(void *arg)
{
    ESP_LOGI(TAG, "Starting LCD refresh task");

    TickType_t tick;
    for (;;) {
        esp_lcd_rgb_panel_refresh((esp_lcd_panel_handle_t)arg);
        tick = xTaskGetTickCount();
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        vTaskDelayUntil(&tick, pdMS_TO_TICKS(CONFIG_BSP_LCD_RGB_REFRESH_TASK_PERIOD));
    }
}
#endif

esp_lcd_panel_handle_t bsp_lcd_init()
{
    BSP_ERROR_CHECK_RETURN_ERR(lcd_config());

    ESP_LOGI(TAG, "Initialize RGB panel");
    esp_lcd_panel_handle_t panel_handle = NULL;
    esp_lcd_rgb_panel_config_t panel_conf = {
        .clk_src = LCD_CLK_SRC_PLL160M,
        .psram_trans_align = 64,
        .data_width = 16,
        .de_gpio_num = BSP_LCD_DE,
        .pclk_gpio_num = BSP_LCD_PCLK,
        .vsync_gpio_num = BSP_LCD_VSYNC,
        .hsync_gpio_num = BSP_LCD_HSYNC,
        .data_gpio_nums = {
            BSP_LCD_DATA0,
            BSP_LCD_DATA1,
            BSP_LCD_DATA2,
            BSP_LCD_DATA3,
            BSP_LCD_DATA4,
            BSP_LCD_DATA5,
            BSP_LCD_DATA6,
            BSP_LCD_DATA7,
            BSP_LCD_DATA8,
            BSP_LCD_DATA9,
            BSP_LCD_DATA10,
            BSP_LCD_DATA11,
            BSP_LCD_DATA12,
            BSP_LCD_DATA13,
            BSP_LCD_DATA14,
            BSP_LCD_DATA15,
        },
        .timings = {
            .pclk_hz = BSP_LCD_PIXEL_CLOCK_HZ,
            .h_res = BSP_LCD_H_RES,
            .v_res = BSP_LCD_V_RES,
            .hsync_back_porch = BSP_LCD_HSYNC_BACK_PORCH,
            .hsync_front_porch = BSP_LCD_HSYNC_FRONT_PORCH,
            .hsync_pulse_width = BSP_LCD_HSYNC_PULSE_WIDTH,
            .vsync_back_porch = BSP_LCD_VSYNC_BACK_PORCH,
            .vsync_front_porch = BSP_LCD_VSYNC_FRONT_PORCH,
            .vsync_pulse_width = BSP_LCD_VSYNC_PULSE_WIDTH,
            .flags.pclk_active_neg = BSP_LCD_PCLK_ACTIVE_NEG,
        },
        .flags.fb_in_psram = 1,
#if CONFIG_BSP_LCD_RGB_REFRESH_MANUALLY
        .flags.refresh_on_demand = 1,
#endif
#if CONFIG_BSP_LCD_RGB_BUFFER_NUMS == 2
        .flags.double_fb = 1,
#elif CONFIG_BSP_LCD_RGB_BUFFER_NUMS == 3
        .num_fbs = 3,
#endif
#if CONFIG_BSP_LCD_RGB_BOUNCE_BUFFER_MODE
        .bounce_buffer_size_px = BSP_LCD_H_RES * CONFIG_BSP_LCD_RGB_BOUNCE_BUFFER_HEIGHT,
#endif
    };
    BSP_ERROR_CHECK_RETURN_NULL(esp_lcd_new_rgb_panel(&panel_conf, &panel_handle));
    esp_lcd_rgb_panel_event_callbacks_t cbs = {
        .on_vsync = on_vsync_event,
    };
    esp_lcd_rgb_panel_register_event_callbacks(panel_handle, &cbs, NULL);
    esp_lcd_panel_reset(panel_handle);
    esp_lcd_panel_init(panel_handle);
    esp_lcd_panel_disp_on_off(panel_handle, true);

#if CONFIG_BSP_LCD_RGB_REFRESH_MANUALLY
    xTaskCreate(lcd_task, "LCD task", 2048, panel_handle, CONFIG_BSP_LCD_RGB_REFRESH_TASK_PRIORITY, &lcd_task_handle);
#endif

    return panel_handle;
}

esp_err_t bsp_lcd_register_trans_done_callback(bsp_lcd_trans_done_cb_t callback)
{
#if CONFIG_LCD_RGB_ISR_IRAM_SAFE
    if (callback) {
        ESP_RETURN_ON_FALSE(esp_ptr_in_iram(callback), ESP_ERR_INVALID_ARG, TAG, "Callback not in IRAM");
    }
#endif
    trans_done = callback;

    return ESP_OK;
}

/**************************************************************************************************
 *
 * Touch Panel Function
 *
 **************************************************************************************************/
esp_lcd_touch_handle_t bsp_touch_panel_init(void)
{
    esp_lcd_panel_io_handle_t tp_io_handle = NULL;
    esp_lcd_touch_handle_t tp_handle = NULL;
    const esp_lcd_panel_io_i2c_config_t tp_io_config = ESP_LCD_TOUCH_IO_I2C_CYTMA568_CONFIG();
    const esp_lcd_touch_config_t tp_cfg = {
        .x_max = BSP_LCD_H_RES,
        .y_max = BSP_LCD_V_RES,
        .rst_gpio_num = GPIO_NUM_NC,
        .int_gpio_num = GPIO_NUM_NC,
        .levels = {
            .reset = 0,
            .interrupt = 0,
        },
        .flags = {
            .swap_xy = 0,
            .mirror_x = 0,
            .mirror_y = 0,
        },
    };

    BSP_ERROR_CHECK_RETURN_ERR(esp_lcd_new_panel_io_i2c((esp_lcd_i2c_bus_handle_t)BSP_I2C_NUM, &tp_io_config, &tp_io_handle));
    BSP_ERROR_CHECK_RETURN_ERR(esp_lcd_touch_new_i2c_cytma568(tp_io_handle, &tp_cfg, &tp_handle));

    return tp_handle;
}

/**************************************************************************************************
 *
 * LCD Configuration Function
 *
 **************************************************************************************************/
/**
 * @brief Simulate SPI to write data using io expander
 *
 * @param data: Data to write
 *
 * @return
 *      - ESP_OK: Success, otherwise returns ESP_ERR_xxx
 */
static esp_err_t spi_write(uint16_t data)
{
    for (uint8_t n = 0; n < 9; n++) {
        if (data & 0x0100) {
            SDO(1);
        } else {
            SDO(0);
        }
        data = data << 1;

        SCK(0);
        udelay(10);
        SCK(1);
        udelay(10);
    }

    return ESP_OK;
}

/**
 * @brief Simulate SPI to write LCD command using io expander
 *
 * @param data: LCD command to write
 *
 * @return
 *      - ESP_OK: Success, otherwise returns ESP_ERR_xxx
 */
static esp_err_t spi_write_cmd(uint16_t data)
{
    CS(0);
    udelay(10);

    spi_write((data & 0x00FF));

    udelay(10);
    CS(1);
    SCK(1);
    SDO(1);
    udelay(10);

    return ESP_OK;
}

static esp_err_t spi_write_read_cmd(uint16_t data)
{
    udelay(10);

    for (uint8_t n = 0; n < 9; n++) {
        if (data & 0x0100) {
            SDO(1);
        } else {
            SDO(0);
        }
        data = data << 1;

        SCK(0);
        udelay(10);
        SCK(1);
        udelay(10);
    }
    // SCK(1);
    // SDO(1);

    return ESP_OK;
}

/**
 * @brief Simulate SPI to write LCD data using io expander
 *
 * @param data: LCD data to write
 *
 * @return
 *      - ESP_OK: Success, otherwise returns ESP_ERR_xxx
 */
static esp_err_t spi_write_data(uint16_t data)
{
    CS(0);
    udelay(10);

    data &= 0x00FF;
    data |= 0x0100;
    spi_write(data);

    udelay(10);
    CS(1);
    SCK(1);
    SDO(1);
    udelay(10);

    return ESP_OK;
}

/**
 * @brief LCD configuration data structure type
 *
 */
typedef struct {
    uint8_t cmd;            // LCD command
    uint8_t data[52];       // LCD data
    uint8_t data_bytes;     // Length of data in above data array; 0xFF = end of cmds.
} lcd_config_data_t;

// *INDENT-OFF*
#if 0
const static lcd_config_data_t LCD_CONFIG_CMD_1[] = {
    {0xFF, {0x77, 0x01, 0x00, 0x00, 0x10}, 5},
    {0xC0, {0x63, 0x00}, 2},
    {0xC1, {0x11, 0x02}, 2},
    {0xC2, {0x01, 0x08}, 2},
    {0xC3, {0x02}, 1},
    {0xCD, {0x01}, 1},
    {0xC7, {0x00}, 1},
    {0xCC, {0x18}, 1},
    {0xB0, {0x00, 0x88, 0x0f, 0x0e, 0x51, 0x07, 0x02, 0x09, 0x08, 0x1f, 0x06, 0x95, 0x12, 0x24, 0xA6, 0x88}, 16},
    {0xB1, {0x80, 0x8B, 0x51, 0x0B, 0x4f, 0x05, 0x00, 0x07, 0x07, 0x1e, 0x05, 0x94, 0x14, 0x23, 0x28, 0x48}, 16},
    {0xFF, {0x77, 0x01, 0x00, 0x00, 0x11}, 5},//0xFF 20200413 James
    {0xB0, {0x5D}, 1},
    {0xB1, {0x60}, 1},
    {0xB2, {0x07}, 1},//0X71 20200413 James
    {0xB3, {0x80}, 1},
    {0xB5, {0x47}, 1},
    {0xB7, {0x85}, 1},
    {0xB8, {0x20}, 1},
    {0xB9, {0x10}, 1},  //08-08
    {0xC1, {0x78}, 1},
    {0xC2, {0x78}, 1},
    {0xD0, {0x88}, 1},

    {0xE0, {0x00, 0x00, 0x02}, 3},
    {0xE1, {0x08, 0x00, 0x0A, 0x00, 0x07, 0x00, 0x09, 0x00, 0x00, 0x33, 0x33}, 11},
    {0xE2, {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, 13},
    {0xE3, {0x00, 0x00, 0x33, 0x33}, 4},
    {0xE4, {0x44, 0x44}, 2},
    {0xE5, {0x0E, 0x60, 0xA0, 0xA0, 0x10, 0x60, 0xA0, 0xA0, 0x0A, 0x60, 0xA0, 0xA0, 0x0C, 0x60, 0xA0, 0xA0}, 16},
    {0xE6, {0x00, 0x00, 0x33, 0x33}, 4},
    {0xE7, {0x44, 0x44}, 2},
    {0xE8, {0x0D, 0x60, 0xA0, 0xA0, 0x0F, 0x60, 0xA0, 0xA0, 0x09, 0x60, 0xA0, 0xA0, 0x0B, 0x60, 0xA0, 0xA0}, 16},
    {0xEB, {0x02, 0x01, 0xE4, 0xE4, 0x44, 0x00, 0x40}, 7},//0xFF 20200413 James
    {0xEC, {0x02, 0x01}, 2},
    {0xED, {0xAB, 0x89, 0x76, 0x54, 0x01, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x10, 0x45, 0x67, 0x98, 0xBA}, 16},
    {0xFF, {0x77, 0x01, 0x00, 0x00, 0x00}, 5},//0X71 20200413 James
    // {0x3A, {0x77}, 1},
    // {0x3a, {0x66}, 1},   /* RGB666 */
    {0x3a, {0x55}, 1},      /* RGB565 */
    {0x36, {0x00}, 1},

    {0x20, {0x00}, 0},
    {0x11, {0x00}, 0},
    {0x29, {0x00}, 0},
    {0x00, {0x00}, 0xff},
};
#else
const static lcd_config_data_t LCD_CONFIG_CMD_1[] = {
    {0xFF, {0x77, 0x01, 0x00, 0x00, 0x10}, 5},
    {0xC0, {0x3B, 0x00}, 2},
    {0xC1, {0x0D, 0x02}, 2},
    {0xC2, {0x21, 0x08}, 2},

    {0xB0, {0x00,0x11,0x18,0x0E,0x11,0x06,0x07,0x08,0x07,0x22,0x04,0x12,0x0F,0xAA,0x31,0x18}, 16},
    {0xB1, {0x00,0x11,0x19,0x0E,0x12,0x07,0x08,0x08,0x08,0x22,0x04,0x11,0x11,0xA9,0x32,0x18}, 16},
    {0xFF, {0x77, 0x01, 0x00, 0x00, 0x11}, 5},//0xFF 20200413 James
    {0xB0, {0x60}, 1},
    {0xB1, {0x30}, 1},
    {0xB2, {0x87}, 1},//0X71 20200413 James
    {0xB3, {0x80}, 1},
    {0xB5, {0x49}, 1},
    {0xB7, {0x85}, 1},
    {0xC1, {0x78}, 1},
    {0xC2, {0x78}, 1},

    {0xE0, {0x00, 0x1B, 0x02}, 3},
    {0xE1, {0x08,0xA0,0x00,0x00,0x07,0xA0,0x00,0x00,0x00,0x44,0x44}, 11},
    {0xE2, {0x11,0x11,0x44,0x44,0xED,0xA0,0x00,0x00,0xEC,0xA0,0x00,0x00}, 12},
    {0xE3, {0x00, 0x00, 0x11, 0x11}, 4},
    {0xE4, {0x44, 0x44}, 2},
    {0xE5, {0x0A,0xE9,0xD8,0xA0,0x0C,0xEB,0xD8,0xA0,0x0E,0xED,0xD8,0xA0,0x10,0xEF,0xD8,0xA0}, 16},
    {0xE6, {0x00, 0x00, 0x11, 0x11}, 4},
    {0xE7, {0x44, 0x44}, 2},
    {0xE8, {0x09,0xE8,0xD8,0xA0,0x0B,0xEA,0xD8,0xA0,0x0D,0xEC,0xD8,0xA0,0x0F,0xEE,0xD8,0xA0}, 16},
    {0xEB, {0x02,0x00,0xE4,0xE4,0x88,0x00,0x40}, 7},//0xFF 20200413 James
    {0xEC, {0x3C, 0x00}, 2},
    {0xED, {0xAB,0x89,0x76,0x54,0x02,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0x20,0x45,0x67,0x98,0xBA}, 16},
    {0xFF, {0x77, 0x01, 0x00, 0x00, 0x00}, 5},//0X71 20200413 James
    // {0x3A, {0x77}, 1},
    // {0x3a, {0x66}, 1},   /* RGB666 */
    {0x3a, {0x55}, 1},      /* RGB565 */
    {0x36, {0x00}, 1},

    {0x20, {0x00}, 0},
    {0x11, {0x00}, 0},
    {0x29, {0x00}, 0},
    {0x00, {0x00}, 0xff},
};
#endif
// *INDENT-OFF*

/**
 * @brief Configure LCD with specific commands and data
 *
 * @return
 *      - ESP_OK: Success, otherwise returns ESP_ERR_xxx
 *
 */
static esp_err_t lcd_config(void)
{
    uint64_t bit_msk = BIT64(BSP_LCD_SPI_CS) | BIT64(BSP_LCD_SPI_SCK) | BIT64(BSP_LCD_SPI_SDO);
    if (BSP_LCD_RST != -1) {
        bit_msk |= BIT64(BSP_LCD_RST);
    }
    gpio_config_t config = {
        .mode = GPIO_MODE_OUTPUT,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pin_bit_mask = bit_msk,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&config), TAG, "SPI GPIO config failed");


    CS(1);
    SCK(1);
    SDO(1);
    vTaskDelay(pdMS_TO_TICKS(120));

    // BSP_ERROR_CHECK_RETURN_ERR(spi_write_cmd(0x11));
    vTaskDelay(pdMS_TO_TICKS(150));
    for (uint8_t i = 0; LCD_CONFIG_CMD_1[i].data_bytes != 0xff; i++) {
        BSP_ERROR_CHECK_RETURN_ERR(spi_write_cmd(LCD_CONFIG_CMD_1[i].cmd));
        for (uint8_t j = 0; j < LCD_CONFIG_CMD_1[i].data_bytes; j++) {
            BSP_ERROR_CHECK_RETURN_ERR(spi_write_data(LCD_CONFIG_CMD_1[i].data[j]));
        }
        // vTaskDelay(pdMS_TO_TICKS(120));
    }

    // gpio_reset_pin(BSP_LCD_SPI_CS);
    gpio_reset_pin(BSP_LCD_SPI_SCK);
    gpio_reset_pin(BSP_LCD_SPI_SDO);

    return ESP_OK;
}
