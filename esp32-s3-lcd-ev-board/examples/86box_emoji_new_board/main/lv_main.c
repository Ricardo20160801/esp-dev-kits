/*
 * SPDX-FileCopyrightText: 2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_spiffs.h"

#include "bsp_board.h"
#include "bsp/esp-bsp.h"
#include "audio_player.h"
#include "file_iterator.h"
#include "es8311.h"

#include "nvs_flash.h"
#include "settings.h"
#include "app_wifi.h"
#include "app_weather.h"
#include "app_sr.h"
#include "lv_example_pub.h"

static char *TAG = "app_main";

#define LOG_MEM_INFO    (1)

void memory_monitor()
{
    static char buffer[128];    /* Make sure buffer is enough for `sprintf` */
    if (1) {
        sprintf(buffer, "   Biggest /     Free /    Total\n"
                "\t  SRAM : [%8d / %8d / %8d]\n"
                "\t PSRAM : [%8d / %8d / %8d]",
                heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL),
                heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
                heap_caps_get_total_size(MALLOC_CAP_INTERNAL),
                heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM),
                heap_caps_get_free_size(MALLOC_CAP_SPIRAM),
                heap_caps_get_total_size(MALLOC_CAP_SPIRAM));
        ESP_LOGI("MEM", "%s", buffer);
    }
}

static file_iterator_instance_t *file_iterator;

esp_err_t bsp_spiffs_mount(void)
{
    esp_vfs_spiffs_conf_t conf = {
        .base_path = CONFIG_BSP_SPIFFS_MOUNT_POINT,
        .partition_label = CONFIG_BSP_SPIFFS_PARTITION_LABEL,
        .max_files = CONFIG_BSP_SPIFFS_MAX_FILES,
#ifdef CONFIG_BSP_SPIFFS_FORMAT_ON_MOUNT_FAIL
        .format_if_mount_failed = true,
#else
        .format_if_mount_failed = false,
#endif
    };

    esp_err_t ret_val = esp_vfs_spiffs_register(&conf);

    ESP_ERROR_CHECK(ret_val);

    size_t total = 0, used = 0;
    ret_val = esp_spiffs_info(conf.partition_label, &total, &used);
    if (ret_val != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get SPIFFS partition information (%s)", esp_err_to_name(ret_val));
    } else {
        ESP_LOGI(TAG, "Partition size: total: %d, used: %d", total, used);
    }

    return ret_val;
}

esp_err_t bsp_spiffs_unmount(void)
{
    return esp_vfs_spiffs_unregister(CONFIG_BSP_SPIFFS_PARTITION_LABEL);
}

file_iterator_instance_t *get_file_iterator_instance(void)
{
    return file_iterator;
}

static esp_err_t audio_mute_function(AUDIO_PLAYER_MUTE_SETTING setting)
{
    // Volume saved when muting and restored when unmuting. Restoring volume is necessary
    // as es8311_set_voice_mute(true) results in voice volume (REG32) being set to zero.
    static int last_volume;
    bsp_codec_config_t *codec_handle = bsp_board_get_codec_handle();

    codec_handle->mute_set_fn(setting == AUDIO_PLAYER_MUTE ? true : false);

    // restore the voice volume upon unmuting
    if (setting == AUDIO_PLAYER_UNMUTE) {
        codec_handle->volume_set_fn(80, NULL);
    }

    ESP_LOGI(TAG, "mute setting %d, volume:%d", setting, last_volume);

    return ESP_OK;
}

void app_main(void)
{
    /* Initialize NVS. */
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
    ESP_ERROR_CHECK(settings_read_parameter_from_nvs());
    sys_param_t *sys_set = settings_get_parameter();

    sys_set->need_hint = false;
    sys_set->demo_gui = true;
    sys_set->sr_enable = false;
    sys_set->sr_lang = SR_LANG_CN;

    bsp_i2c_init();
    bsp_display_start();
    bsp_board_init();

#if CONFIG_BSP_DISPLAY_LVGL_AVOID_TEAR
    ESP_LOGI(TAG, "Avoid lcd tearing effect");
#if CONFIG_BSP_DISPLAY_LVGL_FULL_REFRESH
    ESP_LOGI(TAG, "LVGL full-refresh");
#elif CONFIG_BSP_DISPLAY_LVGL_DIRECT_MODE
    ESP_LOGI(TAG, "LVGL direct-mode");
#endif
#endif

    ESP_LOGI(TAG, "Display LVGL demo");

    bsp_display_lock(0);
    lv_style_pre_init();
    lv_create_home(&boot_Layer);
    lv_create_clock(&clock_screen_layer);
    bsp_display_unlock();

    bsp_spiffs_mount();

    bsp_codec_config_t *codec_handle = bsp_board_get_codec_handle();
    file_iterator = file_iterator_new("/spiffs/mp3");
    assert(file_iterator != NULL);
    audio_player_config_t config = { .mute_fn = audio_mute_function,
                                     .write_fn = codec_handle->i2s_write_fn,
                                     .clk_set_fn = codec_handle->i2s_reconfig_clk_fn,
                                     .priority = 5
                                   };
    ESP_ERROR_CHECK(audio_player_new(config));

    app_weather_start();
    app_network_start();

    while (true == sys_set->need_hint) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    if (sys_set->sr_enable) {
        ESP_LOGI(TAG, "### speech recognition start");
        app_sr_start(false);
    } else {
        ESP_LOGI(TAG, "### speech recognition Disable");
    }


#if LOG_MEM_INFO
    static char buffer[128];    /* Make sure buffer is enough for `sprintf` */
    while (1) {
        sprintf(buffer, "   Biggest /     Free /    Total\n"
                "\t  SRAM : [%8d / %8d / %8d]\n"
                "\t PSRAM : [%8d / %8d / %8d]",
                heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL),
                heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
                heap_caps_get_total_size(MALLOC_CAP_INTERNAL),
                heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM),
                heap_caps_get_free_size(MALLOC_CAP_SPIRAM),
                heap_caps_get_total_size(MALLOC_CAP_SPIRAM));
        ESP_LOGI("MEM", "%s", buffer);

        vTaskDelay(pdMS_TO_TICKS(10000));
    }
#endif
}
