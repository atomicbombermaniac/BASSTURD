/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "esp_system.h"
#include "esp_log.h"

#include "esp_bt.h"
#include "bt_app_core.h"
#include "bt_app_av.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
#include "esp_a2dp_api.h"
#include "esp_avrc_api.h"
//#include "sdkconfig.h"
#include "driver/i2c.h"
#include "OLEDDisplay.h"
#include "rotary_encoder.h"
#include "driver/ledc.h"

#define _I2C_NUMBER(num) I2C_NUM_##num
#define I2C_NUMBER(num) _I2C_NUMBER(num)
#define I2C_MASTER_SCL_IO 4               /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO 5               /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM I2C_NUMBER(1) /*!< I2C port number for master dev */

#define ROT_ENC_A_GPIO (18)
#define ROT_ENC_B_GPIO (19)

#define ENABLE_HALF_STEPS false  // Set to true to enable tracking of rotary encoder at half step resolution
#define RESET_AT          0      // Set to a positive non-zero number to reset the position if this value is exceeded
#define FLIP_DIRECTION    false  // Set to true to reverse the clockwise/counterclockwise sense

/* device name */
#define LOCAL_DEVICE_NAME    "BASSTURD"

volatile  uint8_t level_r, level_l;
volatile  uint8_t rot_vol = 30;
static inline int16_t _max(int16_t x,int16_t y) { if (x>y) return (x); return(y);}
static inline int16_t _min(int16_t x,int16_t y) { if (x<y) return (x); return(y);}
static inline int16_t _abs(int16_t x) { if (x>=0) return (x); return(-x);}

/* event for stack up */
enum {
    BT_APP_EVT_STACK_UP = 0,
};

/********************************
 * STATIC FUNCTION DECLARATIONS
 *******************************/

static void set_pwm(uint8_t chan, uint32_t val);

/* GAP callback function */
static void bt_app_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param);
/* handler for bluetooth stack enabled events */
static void bt_av_hdl_stack_evt(uint16_t event, void *p_param);

/*******************************
 * STATIC FUNCTION DEFINITIONS
 ******************************/

static void bt_app_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
    uint8_t *bda = NULL;

    switch (event) {
    /* when authentication completed, this event comes */
    case ESP_BT_GAP_AUTH_CMPL_EVT: {
        if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS) {
            ESP_LOGI(BT_AV_TAG, "authentication success: %s", param->auth_cmpl.device_name);
            esp_log_buffer_hex(BT_AV_TAG, param->auth_cmpl.bda, ESP_BD_ADDR_LEN);
        } else {
            ESP_LOGE(BT_AV_TAG, "authentication failed, status: %d", param->auth_cmpl.stat);
        }
        break;
    }

#if (CONFIG_BT_SSP_ENABLED == true)
    /* when Security Simple Pairing user confirmation requested, this event comes */
    case ESP_BT_GAP_CFM_REQ_EVT:
        ESP_LOGI(BT_AV_TAG, "ESP_BT_GAP_CFM_REQ_EVT Please compare the numeric value: %"PRIu32, param->cfm_req.num_val);
        esp_bt_gap_ssp_confirm_reply(param->cfm_req.bda, true);
        break;
    /* when Security Simple Pairing passkey notified, this event comes */
    case ESP_BT_GAP_KEY_NOTIF_EVT:
        ESP_LOGI(BT_AV_TAG, "ESP_BT_GAP_KEY_NOTIF_EVT passkey: %"PRIu32, param->key_notif.passkey);
        break;
    /* when Security Simple Pairing passkey requested, this event comes */
    case ESP_BT_GAP_KEY_REQ_EVT:
        ESP_LOGI(BT_AV_TAG, "ESP_BT_GAP_KEY_REQ_EVT Please enter passkey!");
        break;
#endif

    /* when GAP mode changed, this event comes */
    case ESP_BT_GAP_MODE_CHG_EVT:
        ESP_LOGI(BT_AV_TAG, "ESP_BT_GAP_MODE_CHG_EVT mode: %d", param->mode_chg.mode);
        break;
    /* when ACL connection completed, this event comes */
    case ESP_BT_GAP_ACL_CONN_CMPL_STAT_EVT:
        bda = (uint8_t *)param->acl_conn_cmpl_stat.bda;
        ESP_LOGI(BT_AV_TAG, "ESP_BT_GAP_ACL_CONN_CMPL_STAT_EVT Connected to [%02x:%02x:%02x:%02x:%02x:%02x], status: 0x%x",
                 bda[0], bda[1], bda[2], bda[3], bda[4], bda[5], param->acl_conn_cmpl_stat.stat);
        break;
    /* when ACL disconnection completed, this event comes */
    case ESP_BT_GAP_ACL_DISCONN_CMPL_STAT_EVT:
        bda = (uint8_t *)param->acl_disconn_cmpl_stat.bda;
        ESP_LOGI(BT_AV_TAG, "ESP_BT_GAP_ACL_DISC_CMPL_STAT_EVT Disconnected from [%02x:%02x:%02x:%02x:%02x:%02x], reason: 0x%x",
                 bda[0], bda[1], bda[2], bda[3], bda[4], bda[5], param->acl_disconn_cmpl_stat.reason);
        break;
    /* others */
    default: {
        ESP_LOGI(BT_AV_TAG, "event: %d", event);
        break;
    }
    }
}

static void bt_av_hdl_stack_evt(uint16_t event, void *p_param)
{
    ESP_LOGD(BT_AV_TAG, "%s event: %d", __func__, event);

    switch (event) {
    /* when do the stack up, this event comes */
    case BT_APP_EVT_STACK_UP: {
        esp_bt_dev_set_device_name(LOCAL_DEVICE_NAME);
        esp_bt_gap_register_callback(bt_app_gap_cb);

        assert(esp_avrc_ct_init() == ESP_OK);
        esp_avrc_ct_register_callback(bt_app_rc_ct_cb);
        assert(esp_avrc_tg_init() == ESP_OK);
        esp_avrc_tg_register_callback(bt_app_rc_tg_cb);

        esp_avrc_rn_evt_cap_mask_t evt_set = {0};
        esp_avrc_rn_evt_bit_mask_operation(ESP_AVRC_BIT_MASK_OP_SET, &evt_set, ESP_AVRC_RN_VOLUME_CHANGE);
        assert(esp_avrc_tg_set_rn_evt_cap(&evt_set) == ESP_OK);

        assert(esp_a2d_sink_init() == ESP_OK);
        esp_a2d_register_callback(&bt_app_a2d_cb);
        esp_a2d_sink_register_data_callback(bt_app_a2d_data_cb);

        /* Get the default value of the delay value */
        esp_a2d_sink_get_delay_value();

        /* set discoverable and connectable mode, wait to be connected */
        esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
        break;
    }
    /* others */
    default:
        ESP_LOGE(BT_AV_TAG, "%s unhandled event: %d", __func__, event);
        break;
    }
}

void set_rot_vol(uint8_t vol){
    rot_vol = vol;
}

static void i2c_test_task(void *arg) {
    OLEDDisplay_t *oled = OLEDDisplay_init(
        I2C_MASTER_NUM, 0x78, I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO);
    OLEDDisplay_clear(oled);
    OLEDDisplay_display(oled);
    OLEDDisplay_mirrorScreen(oled);
    OLEDDisplay_flipScreenVertically(oled);
    vTaskDelay(250 / portTICK_PERIOD_MS);

    // esp32-rotary-encoder requires that the GPIO ISR service is
    // installed before calling rotary_encoder_register()
    ESP_ERROR_CHECK(gpio_install_isr_service(0));

    // Initialise the rotary encoder device with the GPIOs for A and
    // B signals
    rotary_encoder_info_t info = {0};
    ESP_ERROR_CHECK(rotary_encoder_init(&info, ROT_ENC_A_GPIO, ROT_ENC_B_GPIO));
    ESP_ERROR_CHECK(rotary_encoder_enable_half_steps(&info, ENABLE_HALF_STEPS));
#ifdef FLIP_DIRECTION
    ESP_ERROR_CHECK(rotary_encoder_flip_direction(&info));
#endif

    // Create a queue for events from the rotary encoder driver.
    // Tasks can read from this queue to receive up to date position
    // information.
    QueueHandle_t event_queue = rotary_encoder_create_queue();
    ESP_ERROR_CHECK(rotary_encoder_set_queue(&info, event_queue));

    
    volatile uint8_t old_vol = 30;
    volatile char vol_string[10];
    volatile uint8_t current_vol_frame = 0;
    uint8_t update_vol_to_source = 0;
    uint8_t time_since_last_update = 0;

    volatile uint8_t local_level_l = 0, local_level_r = 0;
    static volatile uint16_t peak_level_l = 0, peak_level_r = 0;
    static uint8_t timer_delay = 2;

    while (1) {
        local_level_l = level_l;
        local_level_r = level_r;

        set_pwm(0, local_level_l*10);
        set_pwm(1, local_level_r*10);

        // OLEDDisplay_setColor(oled,INVERSE);
        // OLEDDisplay_fillRect(oled,6,6,20,20);
        // OLEDDisplay_fillRect(oled,6,32,20,20);
        // if(old_level_l != level_l || old_level_r != level_r){
        peak_level_l = _max(local_level_l * 1.2, peak_level_l);
        peak_level_r = _max(local_level_r * 1.2, peak_level_r);
        OLEDDisplay_clear(oled);
        OLEDDisplay_drawProgressBar(oled, 0, 0, 120, 28, local_level_l,
                                    peak_level_l + 2);
        OLEDDisplay_drawProgressBar(oled, 0, 32, 120, 28, local_level_r,
                                    peak_level_r + 2);

        oled->color = INVERSE;
        OLEDDisplay_drawHorizontalLine(oled, 1, 31, 120);
        OLEDDisplay_drawVerticalLine(oled, 0, 24, 16);
        OLEDDisplay_drawVerticalLine(oled, 25, 24, 16);
        OLEDDisplay_drawVerticalLine(oled, 50, 24, 16);
        OLEDDisplay_drawVerticalLine(oled, 75, 24, 16);
        OLEDDisplay_drawVerticalLine(oled, 100, 24, 16);
        OLEDDisplay_drawVerticalLine(oled, 126, 24, 16);

        if(current_vol_frame)
        {
            OLEDDisplay_drawString(oled, 10, 0, vol_string);
        }
        oled->color = WHITE;

        OLEDDisplay_display(oled);
        //}

        if (timer_delay == 0) {
            if (peak_level_l) {
              peak_level_l--;
            }
            if (peak_level_r) {
              peak_level_r--;
            }
            timer_delay = 1;
        } else {
            timer_delay--;
        }
        vTaskDelay(30 / portTICK_PERIOD_MS);

        // Wait for incoming events on the event queue.
        rotary_encoder_event_t event = { 0 };
        if (xQueueReceive(event_queue, &event, 1 / portTICK_PERIOD_MS) == pdTRUE)
        {
            if(event.state.direction == ROTARY_ENCODER_DIRECTION_CLOCKWISE){
                if(rot_vol < 100){
                    rot_vol++;
                    update_vol_to_source = 1;
                }
            }else if(event.state.direction == ROTARY_ENCODER_DIRECTION_COUNTER_CLOCKWISE){
                if(rot_vol > 0){
                    rot_vol--;
                    update_vol_to_source = 1;
                }
            }
            
            volume_set_by_controller(rot_vol);

            //ESP_LOGI(BT_AV_TAG, "Event: position %d, direction %s, VOL:%d%%", (int) event.state.position,
            //         event.state.direction ? (event.state.direction == ROTARY_ENCODER_DIRECTION_CLOCKWISE ? "CW" : "CCW") : "NOT_SET", rot_vol);
        }



        if(time_since_last_update < 250){
            time_since_last_update++;
        }

        if(update_vol_to_source == 1){
            if(time_since_last_update > 15){
                update_vol_to_source = 0;
                time_since_last_update = 0;
                volume_set_by_local_host(rot_vol);
            }
        }
        

        if(old_vol != rot_vol) //volume changed, show it
        {
            old_vol = rot_vol;
            sprintf(vol_string, "Vol=%d%%", old_vol);
            current_vol_frame = 50;
        }

        if(current_vol_frame)//elapse nr of frames to show vol for
        {
            current_vol_frame--;
        }
    }

    vTaskDelete(NULL);
}

static void pwm_init(void)
{
    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer1 = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .timer_num        = LEDC_TIMER_0,
        .duty_resolution  = LEDC_TIMER_13_BIT,
        .freq_hz          = (5000),  // Set output frequency at 5 kHz
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer1));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel1 = {
        .speed_mode     = LEDC_LOW_SPEED_MODE,
        .channel        = LEDC_CHANNEL_0,
        .timer_sel      = LEDC_TIMER_0,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = (27), //14
        .duty           = 0, // Set duty to 0%
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel1));

        // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer2 = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .timer_num        = LEDC_TIMER_1,
        .duty_resolution  = LEDC_TIMER_13_BIT,
        .freq_hz          = (5000),  // Set output frequency at 5 kHz
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer2));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel2 = {
        .speed_mode     = LEDC_LOW_SPEED_MODE,
        .channel        = LEDC_CHANNEL_1,
        .timer_sel      = LEDC_TIMER_1,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = (14), 
        .duty           = 0, // Set duty to 0%
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel2));
}

//(4095) // Set duty to 50%. ((2 ** 13) - 1) * 50% = 4095
static void set_pwm(uint8_t chan, uint32_t val){
    if(chan == 0){
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, val));
        // Update duty to apply the new value
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0));
    }else if (chan == 1){
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, val));
        // Update duty to apply the new value
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1));
    }
}

/*******************************
 * MAIN ENTRY POINT
 ******************************/

void app_main(void)
{
    /* initialize NVS — it is used to store PHY calibration data */
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    pwm_init();

    /*
     * This example only uses the functions of Classical Bluetooth.
     * So release the controller memory for Bluetooth Low Energy.
     */
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    if ((err = esp_bt_controller_init(&bt_cfg)) != ESP_OK) {
        ESP_LOGE(BT_AV_TAG, "%s initialize controller failed: %s\n", __func__, esp_err_to_name(err));
        return;
    }
    if ((err = esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT)) != ESP_OK) {
        ESP_LOGE(BT_AV_TAG, "%s enable controller failed: %s\n", __func__, esp_err_to_name(err));
        return;
    }
    if ((err = esp_bluedroid_init()) != ESP_OK) {
        ESP_LOGE(BT_AV_TAG, "%s initialize bluedroid failed: %s\n", __func__, esp_err_to_name(err));
        return;
    }
    if ((err = esp_bluedroid_enable()) != ESP_OK) {
        ESP_LOGE(BT_AV_TAG, "%s enable bluedroid failed: %s\n", __func__, esp_err_to_name(err));
        return;
    }

#if (CONFIG_BT_SSP_ENABLED == true)
    /* set default parameters for Secure Simple Pairing */
    esp_bt_sp_param_t param_type = ESP_BT_SP_IOCAP_MODE;
    esp_bt_io_cap_t iocap = ESP_BT_IO_CAP_IO;
    esp_bt_gap_set_security_param(param_type, &iocap, sizeof(uint8_t));
#endif

    /* set default parameters for Legacy Pairing (use fixed pin code 1234) */
    esp_bt_pin_type_t pin_type = ESP_BT_PIN_TYPE_FIXED;
    esp_bt_pin_code_t pin_code;
    pin_code[0] = '1';
    pin_code[1] = '2';
    pin_code[2] = '3';
    pin_code[3] = '4';
    esp_bt_gap_set_pin(pin_type, 4, pin_code);

    bt_app_task_start_up();
    xTaskCreate(i2c_test_task, "i2c_test_task_0", 1024 * 4, (void *)0, 10, NULL);
    /* bluetooth device name, connection mode and profile set up */
    bt_app_work_dispatch(bt_av_hdl_stack_evt, BT_APP_EVT_STACK_UP, NULL, 0, NULL);
}
