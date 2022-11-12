/* This example code is in the Public Domain (or CC0 licensed, at your option.)
   Unless required by applicable law or agreed to in writing, this software is
   distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"

#include "esp_hidd_prf_api.h"
#include "esp_bt_defs.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_gatt_defs.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "driver/gpio.h"
#include "hid_dev.h"

/**
 * Brief:
 * This example Implemented BLE HID device profile related functions, in which the HID device
 * has 4 Reports (1 is mouse, 2 is keyboard and LED, 3 is Consumer Devices, 4 is Vendor devices).
 * Users can choose different reports according to their own application scenarios.
 * BLE HID profile inheritance and USB HID class.
 */

/**
 * Note:
 * 1. Win10 does not support vendor report , So SUPPORT_REPORT_VENDOR is always set to FALSE, it defines in hidd_le_prf_int.h
 * 2. Update connection parameters are not allowed during iPhone HID encryption, slave turns
 * off the ability to automatically update connection parameters during encryption.
 * 3. After our HID device is connected, the iPhones write 1 to the Report Characteristic Configuration Descriptor,
 * even if the HID encryption is not completed. This should actually be written 1 after the HID encryption is completed.
 * we modify the permissions of the Report Characteristic Configuration Descriptor to `ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE_ENCRYPTED`.
 * if you got `GATT_INSUF_ENCRYPTION` error, please ignore.
 */

#define HID_DEMO_TAG "HID_DEMO"

// #define KEYBOARD_ROWS 4
// #define KEYBOARD_COLS 2
// int keyboard_matrix[KEYBOARD_ROWS][KEYBOARD_COLS] = {
//     {}
// }


static uint16_t hid_conn_id = 0;
static bool sec_conn = false;
#define CHAR_DECLARATION_SIZE   (sizeof(uint8_t))

static void hidd_event_callback(esp_hidd_cb_event_t event, esp_hidd_cb_param_t *param);

#define HIDD_DEVICE_NAME            "a1exkbd"
static uint8_t hidd_service_uuid128[] = {
    /* LSB <--------------------------------------------------------------------------------> MSB */
    //first uuid, 16bit, [12],[13] is the value
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0x12, 0x18, 0x00, 0x00,
};

static esp_ble_adv_data_t hidd_adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = true,
    .min_interval = 0x0006, //slave connection min interval, Time = min_interval * 1.25 msec
    .max_interval = 0x0010, //slave connection max interval, Time = max_interval * 1.25 msec
    .appearance = 0x03c0,       //HID Generic,
    .manufacturer_len = 0,
    .p_manufacturer_data =  NULL,
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = sizeof(hidd_service_uuid128),
    .p_service_uuid = hidd_service_uuid128,
    .flag = 0x6,
};

static esp_ble_adv_params_t hidd_adv_params = {
    .adv_int_min        = 0x20,
    .adv_int_max        = 0x30,
    .adv_type           = ADV_TYPE_IND,
    .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
    //.peer_addr            =
    //.peer_addr_type       =
    .channel_map        = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};




static void hidd_event_callback(esp_hidd_cb_event_t event, esp_hidd_cb_param_t *param)
{
    switch(event) {
        case ESP_HIDD_EVENT_REG_FINISH: {
            if (param->init_finish.state == ESP_HIDD_INIT_OK) {
                //esp_bd_addr_t rand_addr = {0x04,0x11,0x11,0x11,0x11,0x05};
                esp_ble_gap_set_device_name(HIDD_DEVICE_NAME);
                esp_ble_gap_config_adv_data(&hidd_adv_data);

            }
            break;
        }
        case ESP_BAT_EVENT_REG: {
            break;
        }
        case ESP_HIDD_EVENT_DEINIT_FINISH:
	     break;
		case ESP_HIDD_EVENT_BLE_CONNECT: {
            ESP_LOGI(HID_DEMO_TAG, "ESP_HIDD_EVENT_BLE_CONNECT");
            hid_conn_id = param->connect.conn_id;
            break;
        }
        case ESP_HIDD_EVENT_BLE_DISCONNECT: {
            sec_conn = false;
            ESP_LOGI(HID_DEMO_TAG, "ESP_HIDD_EVENT_BLE_DISCONNECT");
            esp_ble_gap_start_advertising(&hidd_adv_params);
            break;
        }
        case ESP_HIDD_EVENT_BLE_VENDOR_REPORT_WRITE_EVT: {
            ESP_LOGI(HID_DEMO_TAG, "%s, ESP_HIDD_EVENT_BLE_VENDOR_REPORT_WRITE_EVT", __func__);
            ESP_LOG_BUFFER_HEX(HID_DEMO_TAG, param->vendor_write.data, param->vendor_write.length);
        }
        default:
            break;
    }
    return;
}

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        esp_ble_gap_start_advertising(&hidd_adv_params);
        break;
     case ESP_GAP_BLE_SEC_REQ_EVT:
        for(int i = 0; i < ESP_BD_ADDR_LEN; i++) {
             ESP_LOGD(HID_DEMO_TAG, "%x:",param->ble_security.ble_req.bd_addr[i]);
        }
        esp_ble_gap_security_rsp(param->ble_security.ble_req.bd_addr, true);
	 break;
     case ESP_GAP_BLE_AUTH_CMPL_EVT:
        sec_conn = true;
        esp_bd_addr_t bd_addr;
        memcpy(bd_addr, param->ble_security.auth_cmpl.bd_addr, sizeof(esp_bd_addr_t));
        ESP_LOGI(HID_DEMO_TAG, "remote BD_ADDR: %08x%04x",\
                (bd_addr[0] << 24) + (bd_addr[1] << 16) + (bd_addr[2] << 8) + bd_addr[3],
                (bd_addr[4] << 8) + bd_addr[5]);
        ESP_LOGI(HID_DEMO_TAG, "address type = %d", param->ble_security.auth_cmpl.addr_type);
        ESP_LOGI(HID_DEMO_TAG, "pair status = %s",param->ble_security.auth_cmpl.success ? "success" : "fail");
        if(!param->ble_security.auth_cmpl.success) {
            ESP_LOGE(HID_DEMO_TAG, "fail reason = 0x%x",param->ble_security.auth_cmpl.fail_reason);
        }
        break;
    default:
        break;
    }
}

int cmp_uint8(const void *a, const void *b) {
    const uint8_t *lhs = a;
    const uint8_t *rhs = b;
    if (*lhs < *rhs) {
        return -1;
    } else if (*lhs == *rhs) {
        return 0;
    } else {
        return 1;
    }
}


#define KEY_QUEUE_SIZE 6
uint8_t last_key_queue[KEY_QUEUE_SIZE] = {0};
uint8_t key_queue[KEY_QUEUE_SIZE] = {0};
int key_queue_pos = 0;

uint64_t output_pins[] = {GPIO_NUM_4, GPIO_NUM_5};
uint64_t input_pins[] = {GPIO_NUM_36, GPIO_NUM_35, GPIO_NUM_48, GPIO_NUM_47};
uint8_t key_codes[sizeof(output_pins) / sizeof(output_pins[0])]
                 [sizeof(input_pins) / sizeof(input_pins[0])] = {
    /**
     * phisical keyboard map
     * |---------------------|
     * | Ctrl-Left|Ctrl-Right|
     * |      Left|     Right|
     * |      Down|        Up|
     * |Shift-Down|    Ctrl-T|
     * |---------------------|
     */
    // from top to bottom
    {HID_KEY_LEFT_ARROW, HID_KEY_LEFT_ARROW, HID_KEY_DOWN_ARROW, HID_KEY_DOWN_ARROW}, // left col
    {HID_KEY_RIGHT_ARROW, HID_KEY_RIGHT_ARROW, HID_KEY_UP_ARROW, HID_KEY_T}, // right col
};

void enque_key_pressed(int output_pin, int input_pin) {
}

void hid_demo_task(void *pvParameters)
{
    vTaskDelay(500 / portTICK_PERIOD_MS);

    for (int i = 0; i < sizeof(output_pins) / sizeof(output_pins[0]); i++) {
        gpio_set_level(output_pins[i], 0);
    }

    while(1) {
        bool changed = false;// workaround: always send
        uint8_t modifier = 0;
        memcpy(last_key_queue, key_queue, sizeof(key_queue));
        memset(key_queue, 0, sizeof(key_queue));
        key_queue_pos = 0;
        for (int i = 0; i < sizeof(output_pins) / sizeof(output_pins[0]); i++) {
            gpio_set_level(output_pins[i], 1);
            for (int j = 0; j < sizeof(input_pins) / sizeof(input_pins[0]); j++) {
                int value = gpio_get_level(input_pins[j]);
                if (value == 1) {
                    int output_pin = i;
                    int input_pin = j;
                    uint8_t code = key_codes[output_pin][input_pin];
                    key_queue[key_queue_pos++] = code;
                    key_queue_pos %= sizeof(key_queue) / sizeof(key_queue[0]);
                    if (input_pin == 3 && output_pin == 0) {
                        // shift down
                        key_queue[key_queue_pos++] = HID_KEY_LEFT_SHIFT;
                        key_queue_pos %= sizeof(key_queue) / sizeof(key_queue[0]);
                        modifier = modifier | 2; // left shift
                    } else if (input_pin == 3 && output_pin == 1) {
                        // ctrl T
                        key_queue[key_queue_pos++] = HID_KEY_LEFT_CTRL;
                        key_queue_pos %= sizeof(key_queue) / sizeof(key_queue[0]);
                        modifier = modifier | 1; // left ctrl
                    } else if (input_pin == 0) {
                        // ctrl left/right
                        key_queue[key_queue_pos++] = HID_KEY_LEFT_CTRL;
                        key_queue_pos %= sizeof(key_queue) / sizeof(key_queue[0]);
                        modifier = modifier | 1; // left ctrl
                    }
                }
            }
            gpio_set_level(output_pins[i], 0);
        }

        qsort(key_queue, 6, 1, cmp_uint8);
        qsort(last_key_queue, 6, 1, cmp_uint8);
        if (memcmp(last_key_queue, key_queue, 6) != 0) {
            changed = true;
        }

        // only send when bluetooth is connected
        if (sec_conn && changed) {
          esp_hidd_send_keyboard_value(hid_conn_id, modifier, key_queue, 6);
        }

        // 1ms
        vTaskDelay(1);
    }
}

void app_main(void)
{
    esp_err_t ret;

    // Initialize NVS.
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    // init gpio
    for (int i = 0; i < sizeof(output_pins) / sizeof(output_pins[0]); i++) {
      gpio_config_t pin = {.intr_type = GPIO_INTR_DISABLE,
                                   .mode = GPIO_MODE_OUTPUT,
                                   .pin_bit_mask = (1ull << output_pins[i]),
                                   .pull_down_en = 1,
                                   .pull_up_en = 0};
      esp_err_t err = gpio_config(&pin);
      ESP_LOGI(HID_DEMO_TAG, "Set output pin %llu %d\n", output_pins[i], err);
    }
    for (int i = 0; i < sizeof(input_pins) / sizeof(input_pins[0]); i++) {
      gpio_config_t pin = {.intr_type = GPIO_INTR_DISABLE,
                                  .mode = GPIO_MODE_INPUT,
                                  .pin_bit_mask = (1ull << input_pins[i]),
                                  .pull_down_en = 1,
                                  .pull_up_en = 0};
      gpio_config(&pin);
    }

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(HID_DEMO_TAG, "%s initialize controller failed\n", __func__);
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(HID_DEMO_TAG, "%s enable controller failed\n", __func__);
        return;
    }

    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(HID_DEMO_TAG, "%s init bluedroid failed\n", __func__);
        return;
    }

    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(HID_DEMO_TAG, "%s init bluedroid failed\n", __func__);
        return;
    }

    if((ret = esp_hidd_profile_init()) != ESP_OK) {
        ESP_LOGE(HID_DEMO_TAG, "%s init bluedroid failed\n", __func__);
    }

    ///register the callback function to the gap module
    esp_ble_gap_register_callback(gap_event_handler);
    esp_hidd_register_callbacks(hidd_event_callback);

    /* set the security iocap & auth_req & key size & init key response key parameters to the stack*/
    esp_ble_auth_req_t auth_req = ESP_LE_AUTH_BOND;     //bonding with peer device after authentication
    esp_ble_io_cap_t iocap = ESP_IO_CAP_NONE;           //set the IO capability to No output No input
    uint8_t key_size = 16;      //the key size should be 7~16 bytes
    uint8_t init_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    uint8_t rsp_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    esp_ble_gap_set_security_param(ESP_BLE_SM_AUTHEN_REQ_MODE, &auth_req, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_IOCAP_MODE, &iocap, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_MAX_KEY_SIZE, &key_size, sizeof(uint8_t));
    /* If your BLE device act as a Slave, the init_key means you hope which types of key of the master should distribute to you,
    and the response key means which key you can distribute to the Master;
    If your BLE device act as a master, the response key means you hope which types of key of the slave should distribute to you,
    and the init key means which key you can distribute to the slave. */
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_INIT_KEY, &init_key, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_RSP_KEY, &rsp_key, sizeof(uint8_t));

    xTaskCreate(&hid_demo_task, "hid_task", 2048, NULL, 5, NULL);
}
