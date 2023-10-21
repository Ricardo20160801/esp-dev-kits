/*
 * SPDX-FileCopyrightText: 2015-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_check.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_touch.h"

static const char *TAG = "cytma568";

/* Registers */
#define cytma568_DEVICE_MODE      (0x00)
#define cytma568_GESTURE_ID       (0x01)
#define cytma568_TOUCH_POINTS     (0xFE)

#define cytma568_TOUCH1_EV_FLAG   (0x03)
#define cytma568_TOUCH1_XH        (0x03)
#define cytma568_TOUCH1_XL        (0x04)
#define cytma568_TOUCH1_YH        (0x05)
#define cytma568_TOUCH1_YL        (0x06)

#define cytma568_TOUCH2_EV_FLAG   (0x09)
#define cytma568_TOUCH2_XH        (0x09)
#define cytma568_TOUCH2_XL        (0x0A)
#define cytma568_TOUCH2_YH        (0x0B)
#define cytma568_TOUCH2_YL        (0x0C)

#define cytma568_TOUCH3_EV_FLAG   (0x0F)
#define cytma568_TOUCH3_XH        (0x0F)
#define cytma568_TOUCH3_XL        (0x10)
#define cytma568_TOUCH3_YH        (0x11)
#define cytma568_TOUCH3_YL        (0x12)

#define cytma568_TOUCH4_EV_FLAG   (0x15)
#define cytma568_TOUCH4_XH        (0x15)
#define cytma568_TOUCH4_XL        (0x16)
#define cytma568_TOUCH4_YH        (0x17)
#define cytma568_TOUCH4_YL        (0x18)

#define cytma568_TOUCH5_EV_FLAG   (0x1B)
#define cytma568_TOUCH5_XH        (0x1B)
#define cytma568_TOUCH5_XL        (0x1C)
#define cytma568_TOUCH5_YH        (0x1D)
#define cytma568_TOUCH5_YL        (0x1E)

#define cytma568_ID_G_THGROUP             (0x80)
#define cytma568_ID_G_THPEAK              (0x81)
#define cytma568_ID_G_THCAL               (0x82)
#define cytma568_ID_G_THWATER             (0x83)
#define cytma568_ID_G_THTEMP              (0x84)
#define cytma568_ID_G_THDIFF              (0x85)
#define cytma568_ID_G_CTRL                (0x86)
#define cytma568_ID_G_TIME_ENTER_MONITOR  (0x87)
#define cytma568_ID_G_PERIODACTIVE        (0x88)
#define cytma568_ID_G_PERIODMONITOR       (0x89)
#define cytma568_ID_G_AUTO_CLB_MODE       (0xA0)
#define cytma568_ID_G_LIB_VERSION_H       (0xA1)
#define cytma568_ID_G_LIB_VERSION_L       (0xA2)
#define cytma568_ID_G_CIPHER              (0xA3)
#define cytma568_ID_G_MODE                (0xA4)
#define cytma568_ID_G_PMODE               (0xA5)
#define cytma568_ID_G_FIRMID              (0xA6)
#define cytma568_ID_G_STATE               (0xA7)
#define cytma568_ID_G_FT5201ID            (0xA8)
#define cytma568_ID_G_ERR                 (0xA9)

/*******************************************************************************
* Function definitions
*******************************************************************************/
static esp_err_t esp_lcd_touch_cytma568_read_data(esp_lcd_touch_handle_t tp);
static bool esp_lcd_touch_cytma568_get_xy(esp_lcd_touch_handle_t tp, uint16_t *x, uint16_t *y, uint16_t *strength, uint8_t *point_num, uint8_t max_point_num);
static esp_err_t esp_lcd_touch_cytma568_del(esp_lcd_touch_handle_t tp);

/* I2C read */
static esp_err_t touch_cytma568_i2c_write(esp_lcd_touch_handle_t tp, uint8_t reg, uint8_t data);
static esp_err_t touch_cytma568_i2c_read(esp_lcd_touch_handle_t tp, uint8_t reg, uint8_t *data, uint8_t len);

/* cytma568 init */
static esp_err_t touch_cytma568_init(esp_lcd_touch_handle_t tp);

/*******************************************************************************
* Public API functions
*******************************************************************************/

esp_err_t esp_lcd_touch_new_i2c_cytma568(const esp_lcd_panel_io_handle_t io, const esp_lcd_touch_config_t *config, esp_lcd_touch_handle_t *out_touch)
{
    esp_err_t ret = ESP_OK;

    assert(config != NULL);
    assert(out_touch != NULL);

    /* Prepare main structure */
    esp_lcd_touch_handle_t esp_lcd_touch_cytma568 = heap_caps_calloc(1, sizeof(esp_lcd_touch_t), MALLOC_CAP_DEFAULT);
    ESP_GOTO_ON_FALSE(esp_lcd_touch_cytma568, ESP_ERR_NO_MEM, err, TAG, "no mem for cytma568 controller");

    /* Communication interface */
    esp_lcd_touch_cytma568->io = io;

    /* Only supported callbacks are set */
    esp_lcd_touch_cytma568->read_data = esp_lcd_touch_cytma568_read_data;
    esp_lcd_touch_cytma568->get_xy = esp_lcd_touch_cytma568_get_xy;
    esp_lcd_touch_cytma568->del = esp_lcd_touch_cytma568_del;

    /* Mutex */
    esp_lcd_touch_cytma568->data.lock.owner = portMUX_FREE_VAL;

    /* Save config */
    memcpy(&esp_lcd_touch_cytma568->config, config, sizeof(esp_lcd_touch_config_t));

    /* Prepare pin for touch interrupt */
    if (esp_lcd_touch_cytma568->config.int_gpio_num != GPIO_NUM_NC) {
        const gpio_config_t int_gpio_config = {
            .mode = GPIO_MODE_INPUT,
            .pin_bit_mask = BIT64(esp_lcd_touch_cytma568->config.int_gpio_num)
        };
        ret = gpio_config(&int_gpio_config);
        ESP_GOTO_ON_ERROR(ret, err, TAG, "GPIO config failed");
    }

    /* Prepare pin for touch controller reset */
    if (esp_lcd_touch_cytma568->config.rst_gpio_num != GPIO_NUM_NC) {
        const gpio_config_t rst_gpio_config = {
            .mode = GPIO_MODE_OUTPUT,
            .intr_type = GPIO_INTR_NEGEDGE,
            .pin_bit_mask = BIT64(esp_lcd_touch_cytma568->config.rst_gpio_num)
        };
        ret = gpio_config(&rst_gpio_config);
        ESP_GOTO_ON_ERROR(ret, err, TAG, "GPIO config failed");

        /* Register interrupt callback */
        if (esp_lcd_touch_cytma568->config.interrupt_callback) {
            esp_lcd_touch_register_interrupt_callback(esp_lcd_touch_cytma568, esp_lcd_touch_cytma568->config.interrupt_callback);
        }
    }

    /* Init controller */
    ret = touch_cytma568_init(esp_lcd_touch_cytma568);
    ESP_GOTO_ON_ERROR(ret, err, TAG, "cytma568 init failed");

err:
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error (0x%x)! Touch controller cytma568 initialization failed!", ret);
        if (esp_lcd_touch_cytma568) {
            esp_lcd_touch_cytma568_del(esp_lcd_touch_cytma568);
        }
    }

    *out_touch = esp_lcd_touch_cytma568;

    return ret;
}

#if 0
static esp_err_t esp_lcd_touch_cytma568_read_data(esp_lcd_touch_handle_t tp)
{
    static unsigned int gLastNumFinger = 0;

    esp_err_t err;
    uint8_t data[50];
    uint8_t points;
    size_t i = 0;

    assert(tp != NULL);

    err = touch_cytma568_i2c_read(tp, cytma568_TOUCH_POINTS, data, 7);
    ESP_RETURN_ON_ERROR(err, TAG, "I2C read error!");

    if (data[0] && (data[0] != 0x02)) {
    } else {
        return ESP_OK;
    }

    size_t datalen = data[0];

    err = touch_cytma568_i2c_read(tp, cytma568_TOUCH_POINTS, data, datalen);
    ESP_RETURN_ON_ERROR(err, TAG, "I2C read error!");

    portENTER_CRITICAL(&tp->data.lock);

    char loopNum = 0;
    unsigned int lfnum = gLastNumFinger;

    char device_mode = data[0];
    /* Device Mode[2:0] == 0 :Normal operating Mode */
    if (device_mode <= 2) {
        portEXIT_CRITICAL(&tp->data.lock);
        return ESP_FAIL;
    }

    /* buf[2]!=0x01: NOT a touch report*/
    if (data[2] != 0x01) {
        portEXIT_CRITICAL(&tp->data.lock);
        return ESP_FAIL;
    }

    /* Number of touched points */
    points = data[5] & 0x1F;
    tp->data.points = points;
    loopNum = points;

    if (lfnum > points) {
        for (i = points; i < lfnum; i++) {
            if ( !(data[8 + 10 * i] & 0x80) && ((data[8 + 10 * i] & 0x60) == 0x60) ) {
                loopNum++;
            }
        }
    }

    if (loopNum) {
        for (int i = 0; i < loopNum; i++) {
            bool pressure;
            if ( (data[8 + 10 * i] & 0x80) && ((data[8 + 10 * i] & 0x60) != 0x60) ) {
                pressure = true;
            } else {
                pressure = false;
            }

            int id = (unsigned int)(data[8 + 10 * i] & 0x1F); //touch id
            int finger = loopNum;
            int x = (int)((((unsigned int)data[10 + 10 * i] << 8) & 0x0F00) | ((unsigned int)data[9 + 10 * i]));
            int y = (int)((((unsigned int)data[12 + 10 * i] << 8) & 0x0F00) | ((unsigned int)data[11 + 10 * i]));
            // printf("RAW->[%d][%d, %d]--> %d %d %d\n", i, id, finger, pressure, x, y);

            tp->data.coords[i].x = pressure ? x : 0;
            tp->data.coords[i].y = pressure ? y : 0;
        }
    }

    gLastNumFinger = points;
    portEXIT_CRITICAL(&tp->data.lock);

    return ESP_OK;
}
#else
static esp_err_t esp_lcd_touch_cytma568_read_data(esp_lcd_touch_handle_t tp)
{
    esp_err_t err;
    uint8_t data[50];
    uint8_t points;
    size_t i = 0;
    static uint8_t lastPoints = 0;

    assert(tp != NULL);

    err = touch_cytma568_i2c_read(tp, cytma568_TOUCH_POINTS, data, 7);
    ESP_RETURN_ON_ERROR(err, TAG, "I2C read error!");

    // for(int i = 0; i < 7; i++){
    //     printf("%02x ", data[i]);   
    // }
    // printf("\r\n");

    if ((data[0] == 0x00) || ((data[0] == 0x02) && (0x00 == (data[5] & 0x1F)))) {
        return ESP_OK;
    } else if ((data[0] == 0x07)) { //lift off
        printf("lift off\r\n");
        return ESP_OK;
    } else if((data[0] == 0x02) && (data[5] & 0x1F)){//hold on
        tp->data.points = lastPoints;
        // for(int i = 0; i < 7; i++){
        //     printf("%02x ", data[i]);   
        // }
        // printf("\r\n");
        printf("hold on\r\n");
        return ESP_OK;
    }

    size_t datalen = data[0];
    err = touch_cytma568_i2c_read(tp, cytma568_TOUCH_POINTS, data, datalen);
    ESP_RETURN_ON_ERROR(err, TAG, "I2C read error!");

    // for(int i = 0; i < datalen; i++){
    //     printf("%02x ", data[i]);
    // }
    // printf("\r\n");

    portENTER_CRITICAL(&tp->data.lock);

    /* Device Mode[2:0] == 0 :Normal operating Mode */
    if (data[0] <= 2) {
        tp->data.points = lastPoints;
        portEXIT_CRITICAL(&tp->data.lock);
        return ESP_FAIL;
    }

    /* buf[2]!=0x01: NOT a touch report*/
    if (data[2] != 0x01) {
        tp->data.points = lastPoints;
        portEXIT_CRITICAL(&tp->data.lock);
        return ESP_FAIL;
    }

    /* Number of touched points */
    points = data[5] & 0x1F;
    if (points > 5 || points == 0) {
        tp->data.points = 0;
        lastPoints = 0;
        return ESP_OK;
    }

    /* Number of touched points */
    tp->data.points = points;
    lastPoints = points;
    
    for (int i = 0; i < points; i++) {
        tp->data.coords[i].x = (int)((((unsigned int)data[10 + 10 * i] << 8) & 0x0F00) | ((unsigned int)data[9 + 10 * i]));
        tp->data.coords[i].y = (int)((((unsigned int)data[12 + 10 * i] << 8) & 0x0F00) | ((unsigned int)data[11 + 10 * i]));
        // printf("RAW->[%d][%d, %d]--> %d %d %d\n", i, id, finger, pressure, x, y);
    }

    portEXIT_CRITICAL(&tp->data.lock);

    return ESP_OK;
}
#endif

static bool esp_lcd_touch_cytma568_get_xy(esp_lcd_touch_handle_t tp, uint16_t *x, uint16_t *y, uint16_t *strength, uint8_t *point_num, uint8_t max_point_num)
{
    assert(tp != NULL);
    assert(x != NULL);
    assert(y != NULL);
    assert(point_num != NULL);
    assert(max_point_num > 0);

    portENTER_CRITICAL(&tp->data.lock);

    /* Count of points */
    *point_num = (tp->data.points > max_point_num ? max_point_num : tp->data.points);

    for (size_t i = 0; i < *point_num; i++) {
        x[i] = tp->data.coords[i].x;
        y[i] = tp->data.coords[i].y;

        if (strength) {
            strength[i] = tp->data.coords[i].strength;
        }
    }

    /* Invalidate */
    tp->data.points = 0;

    portEXIT_CRITICAL(&tp->data.lock);

    return (*point_num > 0);
}

static esp_err_t esp_lcd_touch_cytma568_del(esp_lcd_touch_handle_t tp)
{
    assert(tp != NULL);

    /* Reset GPIO pin settings */
    if (tp->config.int_gpio_num != GPIO_NUM_NC) {
        gpio_reset_pin(tp->config.int_gpio_num);
    }

    /* Reset GPIO pin settings */
    if (tp->config.rst_gpio_num != GPIO_NUM_NC) {
        gpio_reset_pin(tp->config.rst_gpio_num);
    }

    free(tp);

    return ESP_OK;
}

/*******************************************************************************
* Private API function
*******************************************************************************/

static esp_err_t touch_cytma568_init(esp_lcd_touch_handle_t tp)
{
    esp_err_t ret = ESP_OK;

    // Valid touching detect threshold
    // ret |= touch_cytma568_i2c_write(tp, cytma568_ID_G_THGROUP, 70);

    // // valid touching peak detect threshold
    // ret |= touch_cytma568_i2c_write(tp, cytma568_ID_G_THPEAK, 60);

    // // Touch focus threshold
    // ret |= touch_cytma568_i2c_write(tp, cytma568_ID_G_THCAL, 16);

    // // threshold when there is surface water
    // ret |= touch_cytma568_i2c_write(tp, cytma568_ID_G_THWATER, 60);

    // // threshold of temperature compensation
    // ret |= touch_cytma568_i2c_write(tp, cytma568_ID_G_THTEMP, 10);

    // // Touch difference threshold
    // ret |= touch_cytma568_i2c_write(tp, cytma568_ID_G_THDIFF, 20);

    // // Delay to enter 'Monitor' status (s)
    // ret |= touch_cytma568_i2c_write(tp, cytma568_ID_G_TIME_ENTER_MONITOR, 2);

    // // Period of 'Active' status (ms)
    // ret |= touch_cytma568_i2c_write(tp, cytma568_ID_G_PERIODACTIVE, 12);

    // // Timer to enter 'idle' when in 'Monitor' (ms)
    // ret |= touch_cytma568_i2c_write(tp, cytma568_ID_G_PERIODMONITOR, 40);

    return ret;
}

static esp_err_t touch_cytma568_i2c_write(esp_lcd_touch_handle_t tp, uint8_t reg, uint8_t data)
{
    assert(tp != NULL);

    // *INDENT-OFF*
    /* Write data */
    return esp_lcd_panel_io_tx_param(tp->io, reg, (uint8_t[]){data}, 1);
    // *INDENT-ON*
}

static esp_err_t touch_cytma568_i2c_read(esp_lcd_touch_handle_t tp, uint8_t reg, uint8_t *data, uint8_t len)
{
    assert(tp != NULL);
    assert(data != NULL);

    /* Read data */
    return esp_lcd_panel_io_rx_param(tp->io, reg, data, len);
}
