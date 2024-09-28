#include <stdlib.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "tinyusb.h"
#include "class/hid/hid_device.h"
#include "driver/gpio.h"
#include <ctype.h>  

#define APP_BUTTON (GPIO_NUM_0)
static const char *TAG = "example";

/************* TinyUSB descriptors ****************/
#define TUSB_DESC_TOTAL_LEN (TUD_CONFIG_DESC_LEN + TUD_HID_DESC_LEN)
#define VENDOR_ID  0x0b49
#define PRODUCT_ID 0x064f
#ifdef CFG_TUSB_DEBUG
#undef CFG_TUSB_DEBUG
#endif

#define CFG_TUSB_DEBUG 3  // Poziom debugowania, gdzie 3 to najbardziej szczegółow
// Deskryptor HID
const uint8_t hid_report_descriptor[] = {
    TUD_HID_REPORT_DESC_GENERIC_INOUT(8)
};


void vibration_strange(uint8_t strength);
// Deskryptor stringów
const char* hid_string_descriptor[5] = {
    (char[]){0x09, 0x04},  // Język angielski
    "ASCII CORPORATION",    // Producent
    "Drmn4ea Tech",         // Produkt
    "123456",               // Numer seryjny
    "ASCII Vib",            // Nazwa interfejsu
};

// Deskryptor urządzenia
static const tusb_desc_device_t hid_device_descriptor = {
    .bLength = sizeof(tusb_desc_device_t),
    .bDescriptorType = TUSB_DESC_DEVICE,
    .bcdUSB = 0x0100,      // USB 1.1
    .bDeviceClass = 0x00,
    .bDeviceSubClass = 0,
    .bDeviceProtocol = 0,
    .bMaxPacketSize0 = 0x08,
    .idVendor = VENDOR_ID,
    .idProduct = PRODUCT_ID,
    .bcdDevice = 0x0100,
    .iManufacturer = 1,
    .iProduct = 2,
    .iSerialNumber = 3,
    .bNumConfigurations = 1
};


// Funkcja obsługująca wszystkie żądania SETUP na EP0
// Funkcja obsługująca wszystkie żądania SETUP na EP0
bool tud_control_request_cb(uint8_t rhport, tusb_control_request_t const *request) {
    ESP_LOGI(TAG, "Obsługa Setup request: bmRequestType=0x%02X, bRequest=0x%02X, wValue=0x%04X, wIndex=0x%04X",
             request->bmRequestType, request->bRequest, request->wValue, request->wIndex);

    // Obsługa vendor-specific request (bmRequestType = 0x41)
    if (request->bmRequestType == 0x41) {
        ESP_LOGI(TAG, "Vendor-specific request otrzymano!");

        if (request->bRequest == 0x00) {
            const char* response = "Vendor-specific response";
            tud_control_xfer(rhport, request, (void*)response, strlen(response));
            return true;
        }
    }

    // Jeśli nie obsługujemy tego requesta, zwróć false
    return false;
}




// Funkcja do obsługi GET_REPORT
uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t* buffer, uint16_t reqlen) {
    ESP_LOGI(TAG, "GET_REPORT request: instance=%d, report_id=%d, report_type=%d", instance, report_id, report_type);
    
    // Wypełniamy bufor zerami, aby zwrócić pusty raport
    memset(buffer, 0, reqlen);
    
    // Zwracamy ilość danych do hosta
    return reqlen;
}

// Deskryptor konfiguracji
static const uint8_t hid_configuration_descriptor[] = {
    TUD_CONFIG_DESCRIPTOR(1, 1, 0, TUSB_DESC_TOTAL_LEN, 0x80, 98),
    TUD_HID_DESCRIPTOR(0, 4, false, sizeof(hid_report_descriptor), 0x81, 8, 10),  // IN (0x81)
    TUD_HID_DESCRIPTOR(1, 4, false, sizeof(hid_report_descriptor), 0x00, 8, 10)   // OUT (0x02)
};

/********* TinyUSB HID callbacks ***************/

// Deskryptor raportu HID
uint8_t const *tud_hid_descriptor_report_cb(uint8_t instance) {
    return hid_report_descriptor;
}

// Odbieranie danych od hosta (OUT endpoint)


void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t const* buffer, uint16_t bufsize) {
    ESP_LOGI(TAG, "Odebrano dane od hosta! Rozmiar bufora: %d", bufsize);

    // Wyświetlanie danych jako ciąg znaków (ASCII)
    ESP_LOGI(TAG, "Dane (ASCII): %.*s", bufsize, buffer);

    if (bufsize >= 8) {
        // Wyświetl znaki
        char char_a = buffer[6];  
        char char_b = buffer[7]; 

        ESP_LOGI(TAG, "2. znak (ASCII): %c, 3. znak (ASCII): %c", char_a, char_b);

        // Konwersja znaków ASCII '8' i 'E' na wartości liczbowe
        uint8_t value_a = isdigit(char_a) ? char_a - '0' : toupper(char_a) - 'A' + 10;
        uint8_t value_b = isdigit(char_b) ? char_b - '0' : toupper(char_b) - 'A' + 10;

        // Połączenie wartości w liczbę heksadecymalną
        uint16_t hex_value = (value_a << 4) | value_b;

        // Wyświetlenie wartości heksadecymalnej i dziesiętnej
        ESP_LOGI(TAG, "Połączona liczba heksadecymalna: 0x%02X, dziesiętnie: %d", hex_value, hex_value);

        // Przekazanie wartości do funkcji kontroli wibracji
        vibration_strange(hex_value);
    }
}


// Deklaracja funkcji do kontrolowania siły wibracji
void vibration_strange(uint8_t strength) {
    // Wykorzystaj wartość strength (0-255) do kontrolowania siły wibracji
    // np. poprzez PWM, sterownik silnika, itp.
	strength= strength/2.55;
    ESP_LOGI("VIBRATION", "Ustawiono siłę wibracji na: %d%%", strength);
}


/********* Application ***************/

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
    //dcd_event_setup_received(0, NULL, true);

    ESP_LOGI(TAG, "USB HID zainicjalizowane");

    while (1) {
        if (tud_mounted()) {
            //ESP_LOGI(TAG, "Urządzenie HID podłączone i działa, oczekiwanie na dane...");
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
