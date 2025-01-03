#include <stdlib.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "tinyusb.h"
#include "class/hid/hid_device.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include <ctype.h>

#define APP_BUTTON (GPIO_NUM_0)
#define PWM_PIN GPIO_NUM_7
#define PWM_FREQ_HZ 1000
#define PWM_RESOLUTION LEDC_TIMER_8_BIT
#define PWM_CHANNEL LEDC_CHANNEL_0
#define PWM_TIMER LEDC_TIMER_0

static const char *TAG = "example";

/************* TinyUSB descriptors ****************/
#define TUSB_DESC_TOTAL_LEN (TUD_CONFIG_DESC_LEN + TUD_HID_DESC_LEN)
#define VENDOR_ID  0x0b49
#define PRODUCT_ID 0x064f
#ifdef CFG_TUSB_DEBUG
#undef CFG_TUSB_DEBUG
#endif

#define CFG_TUSB_DEBUG 3  // Poziom debugowania, gdzie 3 to najbardziej szczegółow

const uint8_t hid_report_descriptor[] = {
    TUD_HID_REPORT_DESC_GENERIC_INOUT(8)
};

void vibration_strange(uint8_t strength);

const char* hid_string_descriptor[5] = {
    (char[]){0x09, 0x04},  // Język angielski
    "ASCII CORPORATION",    // Producent
    "Drmn4ea Tech",         // Produkt
    "123456",               // Numer seryjny
    "ASCII Vib",            // Nazwa interfejsu
};

static const tusb_desc_device_t hid_device_descriptor = {
    .bLength = sizeof(tusb_desc_device_t),
    .bDescriptorType = TUSB_DESC_DEVICE,
    .bcdUSB = 0x0200,      // USB 1.1
    .bDeviceClass = 0x00,
    .bDeviceSubClass = 0,
    .bDeviceProtocol = 0,
    .bMaxPacketSize0 = 0x40,
    .idVendor = VENDOR_ID,
    .idProduct = PRODUCT_ID,
    .bcdDevice = 0x0100,
    .iManufacturer = 1,
    .iProduct = 2,
    .iSerialNumber = 3,
    .bNumConfigurations = 1
};

bool tud_control_request_cb(uint8_t rhport, tusb_control_request_t const *request) {
    ESP_LOGI(TAG, "Obsługa Setup request: bmRequestType=0x%02X, bRequest=0x%02X, wValue=0x%04X, wIndex=0x%04X",
             request->bmRequestType, request->bRequest, request->wValue, request->wIndex);

    if (request->bmRequestType == 0x41 || request->bmRequestType == 0x40 ) {
        ESP_LOGI(TAG, "Vendor-specific request otrzymano!");

        if (request->bRequest == 0x00) {
            const char* response = "Vendor-specific response";
            tud_control_xfer(rhport, request, (void*)response, strlen(response));
            return true;
        }
    }

    return false;
}

uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t* buffer, uint16_t reqlen) {
    ESP_LOGI(TAG, "GET_REPORT request: instance=%d, report_id=%d, report_type=%d", instance, report_id, report_type);
    memset(buffer, 0, reqlen);
    return reqlen;
}

static const uint8_t hid_configuration_descriptor[] = {
    TUD_CONFIG_DESCRIPTOR(1, 1, 0, TUSB_DESC_TOTAL_LEN, 0x80, 98),
    TUD_HID_DESCRIPTOR(0, 4, false, sizeof(hid_report_descriptor), 0x81, 8, 10),
    TUD_HID_DESCRIPTOR(1, 4, false, sizeof(hid_report_descriptor), 0x00, 8, 10)
};

uint8_t const *tud_hid_descriptor_report_cb(uint8_t instance) {
    return hid_report_descriptor;
}

void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t const* buffer, uint16_t bufsize) {
    ESP_LOGI(TAG, "Odebrano dane od hosta! Rozmiar bufora: %d", bufsize);

    ESP_LOGI(TAG, "Dane (ASCII): %.*s", bufsize, buffer);

    if (bufsize >= 8) {
        char char_a = buffer[8];  
        char char_b = buffer[9]; 
        ESP_LOGI(TAG, "2. znak (ASCII): %c, 3. znak (ASCII): %c", char_a, char_b);

        uint8_t value_a = isdigit(char_a) ? char_a - '0' : toupper(char_a) - 'A' + 10;
        uint8_t value_b = isdigit(char_b) ? char_b - '0' : toupper(char_b) - 'A' + 10;

        uint16_t hex_value = (value_a << 4) | value_b;

        vibration_strange(hex_value);
    }
}

void vibration_strange(uint8_t strength) {
    strength = strength / 2.55;  // Convert to percentage
    ESP_LOGI("VIBRATION", "Ustawiono siłę wibracji na: %d%%", strength);

    uint32_t duty_cycle = (strength * 255) / 100;  // Scale to 8-bit PWM resolution

    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, PWM_CHANNEL, duty_cycle));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, PWM_CHANNEL));
}

void app_main(void) {
    ESP_LOGI(TAG, "Inicjalizacja USB HID");

    const tinyusb_config_t tusb_cfg = {
        .device_descriptor = &hid_device_descriptor,
        .string_descriptor = hid_string_descriptor,
        .string_descriptor_count = sizeof(hid_string_descriptor) / sizeof(hid_string_descriptor[0]),
        .external_phy = false,
        .configuration_descriptor = hid_configuration_descriptor,
    };

    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));
    ESP_LOGI(TAG, "USB HID zainicjalizowane");

    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = PWM_TIMER,
        .duty_resolution = PWM_RESOLUTION,
        .freq_hz = PWM_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    ledc_channel_config_t ledc_channel = {
        .gpio_num = PWM_PIN,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = PWM_CHANNEL,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = PWM_TIMER,
        .duty = 0,
        .hpoint = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));

    while (1) {
        if (tud_mounted()) {
            // ESP_LOGI(TAG, "Urządzenie HID podłączone i działa, oczekiwanie na dane...");
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
