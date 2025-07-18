#include <stdio.h>
#include "bsp/device.h"
#include "bsp/display.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "hal/lcd_types.h"
#include "nvs_flash.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "rom/ets_sys.h"

#include "console.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_types.h"

#define BUF_SIZE (1024*16)

void dhrystone(void *pvParameters);
void whetstone(void *pvParameters);

// Constants
static char const TAG[] = "main";

// Global variables
static esp_lcd_panel_handle_t       display_lcd_panel    = NULL;
static esp_lcd_panel_io_handle_t    display_lcd_panel_io = NULL;
static size_t                       display_h_res        = 0;
static size_t                       display_v_res        = 0;
static lcd_color_rgb_pixel_format_t display_color_format;
static pax_buf_t g_pax_buf = {0};

static FILE *g_stdout_f;
char g_isb_buff[BUF_SIZE];

void main_pax_init()
{
  pax_buf_init(&g_pax_buf, NULL,
                display_h_res,
                display_v_res,
                PAX_BUF_16_565RGB);
  pax_buf_reversed(&g_pax_buf, false);
  pax_buf_set_orientation(&g_pax_buf, PAX_O_ROT_CW);
}

void main_draw()
{
  const void *pixels = pax_buf_get_pixels(&g_pax_buf);

  esp_lcd_panel_draw_bitmap(display_lcd_panel,
                            0, 0,
                            display_h_res,
                            display_v_res,
                            pixels);
}

void app_main(void) {
    // Start the GPIO interrupt service
    gpio_install_isr_service(0);

    // Initialize the Non Volatile Storage service
    esp_err_t res = nvs_flash_init();
    if (res == ESP_ERR_NVS_NO_FREE_PAGES || res == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        res = nvs_flash_init();
    }
    ESP_ERROR_CHECK(res);

    // Initialize the Board Support Package
    ESP_ERROR_CHECK(bsp_device_initialize());

    // Fetch the handle for using the screen, this works even when
    res = bsp_display_get_panel(&display_lcd_panel);
    ESP_ERROR_CHECK(res);                             // Check that the display handle has been initialized
    bsp_display_get_panel_io(&display_lcd_panel_io);  // Do not check result of panel IO handle: not all types of
                                                      // display expose a panel IO handle
    res = bsp_display_get_parameters(&display_h_res, &display_v_res, &display_color_format);
    ESP_ERROR_CHECK(res);  // Check that the display parameters have been initialized

    ESP_LOGW(TAG, "Hello world!");


	TaskHandle_t taskHandle = xTaskGetCurrentTaskHandle();
	char * my_name = pcTaskGetName(NULL);
	UBaseType_t my_prio = uxTaskPriorityGet(NULL);
	ESP_LOGD(my_name, "my_prio=%d", my_prio);

#if CONFIG_IDF_TARGET_ESP8266
	printf("Target is ESP8266\n");
#elif CONFIG_IDF_TARGET_ESP32
	printf("Target is ESP32@%"PRIu32"Mhz\n", ets_get_cpu_frequency());
#elif CONFIG_IDF_TARGET_ESP32S2
	printf("Target is ESP32S2@%"PRIu32"Mhz\n", ets_get_cpu_frequency());
#elif CONFIG_IDF_TARGET_ESP32S3
	printf("Target is ESP32S3@%"PRIu32"Mhz\n", ets_get_cpu_frequency());
#elif CONFIG_IDF_TARGET_ESP32C2
	printf("Target is ESP32C2@%"PRIu32"Mhz\n", ets_get_cpu_frequency());
#elif CONFIG_IDF_TARGET_ESP32C3
	printf("Target is ESP32C3@%"PRIu32"Mhz\n", ets_get_cpu_frequency());
#elif CONFIG_IDF_TARGET_ESP32C6
	printf("Target is ESP32C6@%"PRIu32"Mhz\n", ets_get_cpu_frequency());
#elif CONFIG_IDF_TARGET_ESP32H2
	printf("Target is ESP32H2@%"PRIu32"Mhz\n", ets_get_cpu_frequency());
#elif CONFIG_IDF_TARGET_ESP32P4
	printf("Target is ESP32P4@%"PRIu32"Mhz\n", ets_get_cpu_frequency());
#endif

	xTaskCreate(&dhrystone, "DHRY", 1024*4, (void *)taskHandle, my_prio+1, NULL);
	ulTaskNotifyTake( pdTRUE, portMAX_DELAY );
	ESP_LOGD(my_name, "ulTaskNotifyTake");

	xTaskCreate(&whetstone, "WHET", 1024*4, (void *)taskHandle, my_prio+1, NULL);
	ulTaskNotifyTake( pdTRUE, portMAX_DELAY );
	ESP_LOGD(my_name, "ulTaskNotifyTake");

}

