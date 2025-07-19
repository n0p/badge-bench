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

#include "esp_dsp.h"
#include "esp_timer.h"

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

const int N = 1024; // 128 for testing datasets that fit entirely in internal RAM
const int M = 1024; // 128

void calculateAndPrintMMACS(int numOps, int64_t startTime, int64_t endTime)
{
    int64_t total = endTime - startTime;
    double timeInSecs = total / 1000000.0;
    double macs = numOps / timeInSecs;
    double mmacs = macs / 1000000.0;
    printf("completed in %" PRId64 " us (%f MMACS)\n", total, mmacs);
}

/*
template<typename T>
void stdCTest(char* name, T* x, T* y)
{
    printf("Standard C test using %s\n", name);
    printf("---------------\n");

    printf("Performing %d multiply-accumulates...", N * M);
    T z[M];
    T* ptr;

    int64_t startTime = esp_timer_get_time();
    for (int i = 0; i < M; i++)
    {
        T acc = 0;
        ptr = &x[i*N];
        for (int j = 0; j < N; j++)
        {
            acc += *ptr++ * y[j];
        }
        z[i] = acc;
    }
    int64_t endTime = esp_timer_get_time();

    // To avoid optimizing out the above loop
    int tmpAcc = 0;
    for (int i = 0; i < N; i++)
    {
        tmpAcc += z[i];
    }

    // Print results
    printf("(accum %d--needed to avoid being optimized out)...", tmpAcc);
    calculateAndPrintMMACS(N * M, startTime, endTime);
    printf("---------------\n\n");
}
*/
#include <stdint.h>
#include <stdio.h>
#include <inttypes.h>

void stdCTest_int8(const char* name, int8_t* x, int8_t* y)
{
    printf("Standard C test using %s\n", name);
    printf("---------------\n");

    printf("Performing %d multiply-accumulates...", N * M);
    int32_t z[M];
    int8_t* ptr;

    int64_t startTime = esp_timer_get_time();
    for (int i = 0; i < M; i++)
    {
        int32_t acc = 0;
        ptr = &x[i * N];
        for (int j = 0; j < N; j++)
        {
            acc += (int32_t)(*ptr++) * y[j];
        }
        z[i] = acc;
    }
    int64_t endTime = esp_timer_get_time();

    int32_t tmpAcc = 0;
    for (int i = 0; i < N; i++)
    {
        tmpAcc += z[i];
    }

    printf("(accum %" PRId32 " -- needed to avoid being optimized out)...", tmpAcc);
    calculateAndPrintMMACS(N * M, startTime, endTime);
    printf("---------------\n\n");
}

void stdCTest_int16(const char* name, int16_t* x, int16_t* y)
{
    printf("Standard C test using %s\n", name);
    printf("---------------\n");

    printf("Performing %d multiply-accumulates...", N * M);
    int32_t z[M];
    int16_t* ptr;

    int64_t startTime = esp_timer_get_time();
    for (int i = 0; i < M; i++)
    {
        int32_t acc = 0;
        ptr = &x[i * N];
        for (int j = 0; j < N; j++)
        {
            acc += (int32_t)(*ptr++) * y[j];
        }
        z[i] = acc;
    }
    int64_t endTime = esp_timer_get_time();

    int32_t tmpAcc = 0;
    for (int i = 0; i < N; i++)
    {
        tmpAcc += z[i];
    }

    printf("(accum %" PRId32 " -- needed to avoid being optimized out)...", tmpAcc);
    calculateAndPrintMMACS(N * M, startTime, endTime);
    printf("---------------\n\n");
}

void stdCTest_int32(const char* name, int32_t* x, int32_t* y)
{
    printf("Standard C test using %s\n", name);
    printf("---------------\n");

    printf("Performing %d multiply-accumulates...", N * M);
    int64_t z[M];
    int32_t* ptr;

    int64_t startTime = esp_timer_get_time();
    for (int i = 0; i < M; i++)
    {
        int64_t acc = 0;
        ptr = &x[i * N];
        for (int j = 0; j < N; j++)
        {
            acc += (int64_t)(*ptr++) * y[j];
        }
        z[i] = acc;
    }
    int64_t endTime = esp_timer_get_time();

    int64_t tmpAcc = 0;
    for (int i = 0; i < N; i++)
    {
        tmpAcc += z[i];
    }

    printf("(accum %" PRId64 " -- needed to avoid being optimized out)...", tmpAcc);
    calculateAndPrintMMACS(N * M, startTime, endTime);
    printf("---------------\n\n");
}

void simdTestInt16(int16_t* x, int16_t* y)
{
    printf("SIMD test using int16\n");
    printf("---------------\n");

    printf("Performing %d multiply-accumulates...", N * M);
    int16_t z[M];

    int64_t startTime = esp_timer_get_time();
    for (int i = 0; i < M; i++)
    {
        dsps_dotprod_s16(&x[i*M], y, &z[i], N, 15);
    }
    int64_t endTime = esp_timer_get_time();

    // To avoid optimizing out the above loop
    int tmpAcc = 0;
    for (int i = 0; i < N; i++)
    {
        tmpAcc += z[i];
    }

    // Print results
    printf("(accum %d--needed to avoid being optimized out)...", tmpAcc);
    calculateAndPrintMMACS(N * M, startTime, endTime);
    printf("---------------\n\n");
}

void simdTestInt16Matrix(int16_t* x, int16_t* y)
{
    printf("SIMD test using int16 matrix fns\n");
    printf("---------------\n");

    printf("Performing %d multiply-accumulates...", N * M);
    int16_t z[M];

    int64_t startTime = esp_timer_get_time();
    dspm_mult_s16(x, y, z, M, N, 1, 15);
    int64_t endTime = esp_timer_get_time();

    // To avoid optimizing out the above loop
    int tmpAcc = 0;
    for (int i = 0; i < N; i++)
    {
        tmpAcc += z[i];
    }

    // Print results
    printf("(accum %d--needed to avoid being optimized out)...", tmpAcc);
    calculateAndPrintMMACS(N * M, startTime, endTime);
    printf("---------------\n\n");
}

/*
void simdTestInt8(int8_t* x, int8_t* y)
{
    printf("SIMD test using int8\n");
    printf("---------------\n");

    printf("Perfoming %d multiply-accumulates...", N * M);
    int64_t startTime = esp_timer_get_time();

    int8_t* ptr = x;
    int8_t* dataPtr = y;
    int8_t z[M];
    int8_t* zPtr = &z[0];

    int n = M;
    int p = N >> 4;

    asm volatile(
        "movi a9, 8\n"                                                       // a9 = 8
        "simd_loop_begin:\n"                                                 // while (n > 0) {
        "    movi a11, 0\n"                                                  //    a11 = 0
        "    loopgtz %[p], inner_loop_end\n"                                 //    while (p > 0) {
        "        ee.zero.accx\n"                                             //        Zero accx
        "        ld.qr q0, %[ptr], 0\n"                                      //        Load ptr into q0
        "        ld.qr q1, %[dataPtr], 0\n"                                  //        Load dataPtr into q1
        "        addi %[ptr], %[ptr], 16\n"                                  //        ptr += 16
        "        addi %[dataPtr], %[dataPtr], 16\n"                          //        dataPtr += 16
        "        ee.vmulas.s8.accx q0, q1\n"                                 //        accx += q0[0] * q1[0] + q0[1] * q1[1] + ...
        "        ee.srs.accx a10, a9, 0\n"                                   //        a10 = accx >> a9
        "        add a11, a11, a10\n"                                        //        a11 += a10
        "    inner_loop_end:\n"                                              //    }
        "    s8i a11, %[zPtr], 0\n"                                          //    Store value of a10 to memory location zPtr
        "    addi %[zPtr], %[zPtr], 1\n"                                     //    zPtr++
        "    addi %[dataPtr], %[dataPtr], -1024\n"                           //    dataPtr -= 1024
        "    addi %[n], %[n], -1\n"                                          //    n--
        "    bnez %[n], simd_loop_begin\n"                                   // }

        : // outputs
          [ptr] "=r"(ptr),
          [zPtr] "=r"(zPtr),
          [dataPtr] "=r"(dataPtr),
          [p] "=r"(p),
          [n] "=r"(n)

        : // inputs
          "0"(ptr),
          "1"(zPtr),
          "2"(dataPtr),
          "3"(p),
          "4"(n)

        :// clobbers
          "a9", "a10", "a11", "memory"
    );


    int tmpAcc = 0;
    for (int i = 0; i < N; i++)
    {
        tmpAcc += z[i];
    }

    printf("(accum %d)...", tmpAcc);
    int64_t endTime = esp_timer_get_time();
    calculateAndPrintMMACS(N * M, startTime, endTime);
    printf("\n");
}
*/

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


// https://github.com/tmiw/esp32s3-benchmark/blob/main/main/esp32s3-benchmark.cpp
// https://freedv.org/measuring-esp32-s3-performance-for-radae/
    // RADAE should need ~300MMACS

    printf("Initializing memory with random values...");

    int64_t startTime = esp_timer_get_time();
    int8_t* x = (int8_t*)malloc(N * M * sizeof(int));
    int y[N];

    assert(x != NULL);

    int8_t* ptr = x;
    for (int i = 0; i < N * M; i++)
    {
        *ptr++ = rand() * 128;
    }

    ptr = (int8_t*)y;
    for (int i = 0; i < N; i++)
    {
        *ptr++ = rand() * 128;
    }

    int64_t endTime = esp_timer_get_time();
    printf("completed in %" PRIu64 " us\n", endTime - startTime);

    //simdTestInt8(x, (int8_t*)y);
    simdTestInt16((int16_t*)x, (int16_t*)y);
    simdTestInt16Matrix((int16_t*)x, (int16_t*)y);

    stdCTest_int8("int8", x, (int8_t*)y);
    stdCTest_int16("int16", (int16_t*)x, (int16_t*)y);
    stdCTest_int32("int32", (int32_t*)x, (int32_t*)y);
    //stdCTest<float>("float");

    // Free memory
    free(x);

}

