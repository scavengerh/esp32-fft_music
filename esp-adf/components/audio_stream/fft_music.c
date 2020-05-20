#include "fft_music.h"
#include "freertos/FreeRTOS.h"
#include "freertos/ringbuf.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "dsps_fft2r.h"
#include "dsps_view.h"
#include <math.h>
#include <string.h>
#include "periph_ws2812.h"
#include "dsps_wind_hann.h"

static const char *TAG = "FFT_MUSIC";

#define N_SAMPLES 2048
//input audio raw data
float y_cf[N_SAMPLES * 2];
//output fft data
float sum_y[N_SAMPLES / 2];

//Led relate
periph_ws2812_ctrl_cfg_t *control_cfg = NULL;
esp_periph_handle_t handle_matrix = NULL;

#define GPIO_LEFT_NUMBER 12
#define GPIO_RIGHT_NUMBER 14

#define LED_HIGH_LEVEL 16
#define LED_NUM (LED_HIGH_LEVEL * 8)

//display freq index: 31.5, 45, 63, 90, 125, 180, 250, 360, 550, 680, 890, 1k, 2k, 4k, 8k, 16k
int16_t freq_offset[16] = {0, 1, 2, 3, 4, 5, 8, 11, 17, 25, 30, 41, 44, 87, 175, 711};

unsigned char led_data[LED_HIGH_LEVEL];

//for view test result
float led_data_view[LED_HIGH_LEVEL];

//update flow count
unsigned char led_updateCount[LED_HIGH_LEVEL];
#define SLOW_SPEED (2)

//Task period
#define FFT_TASK_DELAY (10) //Ms

//display review period: FFT_TASK_DELAY * LED_UPDATE_PERIOD
#define LED_UPDATE_PERIOD (2) //Ms

#define THRESHOLD_HIGH (0.1f)
#define THRESHOLD_LOW (0.00001f)
static float factor = 0.0f;

SemaphoreHandle_t xSemaphore = NULL;
int haveDataUpdated = 0;
int isCleanMatrixLed = 0;

static void fft_update_led(unsigned char display[], int size)
{
    if (size != LED_HIGH_LEVEL)
    {
        return;
    }

    if (handle_matrix != NULL)
    {
        //for update right matrix
        for (int i = 0; i < LED_NUM; i++)
        {
            control_cfg[i].color = (display[i / 8] > ((i % 8) * 32)) ? LED2812_COLOR_CYAN : LED2812_COLOR_BLACK;
            control_cfg[i].mode = PERIPH_WS2812_ONE;
            control_cfg[i].loop = 50;
            control_cfg[i].time_off_ms = 100;
            control_cfg[i].time_on_ms = 100;
        }
        periph_ws2812_control(handle_matrix, control_cfg, NULL);
    }
}

static void fft_update_data(void)
{
    int i = 0;
    float tempraw = 0.0f;
    unsigned char tempdata;

    for (i = 0; i < LED_HIGH_LEVEL; i++)
    {
        tempraw = sum_y[freq_offset[i]];

        if (tempraw < THRESHOLD_LOW)
            tempraw = THRESHOLD_LOW;
        if (tempraw > THRESHOLD_HIGH)
            tempraw = THRESHOLD_HIGH;

        tempdata = (unsigned char)(factor * (tempraw - THRESHOLD_LOW));

        if (tempdata > led_data[i])
        {
            led_data[i] = tempdata;
            led_updateCount[i] = SLOW_SPEED;
        }
        else
        {
            if (--led_updateCount[i] == 0)
            {
                if (led_data[i] - 32 < 0)
                {
                    led_data[i] = 0;
                }
                else
                {
                    led_data[i] = led_data[i] - 32;
                }
                led_updateCount[i] = SLOW_SPEED;
            }
        }
    }

#if  0 //send to ws2812 matrix for display
    fft_update_led(led_data, LED_HIGH_LEVEL);
#else //for test use disp_view
    for (i = 0; i < LED_HIGH_LEVEL; i++)
    {
        led_data_view[i] = (float)led_data[i];
    }
    dsps_view(led_data_view, LED_HIGH_LEVEL, 16, 8, 0.0f, 255.0f, '|');
#endif
}

static int checkHaveCharRepeat(char *buffer, int len)
{
    char prevTemp = 0.0;
    int filterCount = 0;
    int thresholdCount = len/16;

    //for check data is valid
    prevTemp = buffer[0];
    for (int i = 1; i < len; i += 3)
    {
        if (prevTemp == buffer[i])
        {
            filterCount++;
        }

        if (filterCount >thresholdCount)
        {
            //ESP_LOGE(TAG, "check  raw data, have repeat data!!!");
            return -1;
        }
        prevTemp = buffer[i];
    }

    return 0;
}

static void printMaxMinmun(float buffer[], int len)
{
    float maxmum, minmum;
    maxmum = buffer[0];
    minmum = buffer[0];

    for (int i = 1; i < len; i++)
    {
        if (buffer[i] > maxmum)
        {
            maxmum = buffer[i];
        }
        if (buffer[i] < minmum)
        {
            minmum = buffer[i];
        }
    }
    ESP_LOGE(TAG, "max: %f, min:%f", maxmum, minmum);
}

static void fft_music_task(void *params)
{
    esp_err_t ret;
    int updateLedPeriodCount = 0;

    ret = dsps_fft2r_init_fc32(NULL, CONFIG_DSP_MAX_FFT_SIZE);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Not possible to initialize FFT. Error = %i", ret);
        return;
    }

    while (1)
    {
        if (xSemaphore)
        {
            //obtain semaphore, portMAX_DELAY
            if (haveDataUpdated == 1 && pdTRUE == xSemaphoreTake(xSemaphore, 0))
            {
                //FFT
                dsps_fft2r_fc32(y_cf, N_SAMPLES);
                // Bit reverse
                dsps_bit_rev_fc32(y_cf, N_SAMPLES);

                for (int i = 0; i < N_SAMPLES / 2; i++)
                {
                    sum_y[i] = (float)(sqrt(y_cf[i * 2 + 0] * y_cf[i * 2 + 0] + y_cf[i * 2 + 1] * y_cf[i * 2 + 1]) / N_SAMPLES);
                }

                //Release semaphore
                haveDataUpdated = 0;
                updateLedPeriodCount++;

                // ESP_LOGE(TAG, "%s: calcuate fft completed", __func__);

                xSemaphoreGive(xSemaphore);
            }
            else
            {
                if (updateLedPeriodCount > LED_UPDATE_PERIOD)
                {
                    updateLedPeriodCount = 0;
                    fft_update_data();
                    // ESP_LOGE(TAG, "%s: prepare led display", __func__);
                    // printMaxMinmun(&sum_y[4], N_SAMPLES / 2 - 4);
                }
            }
        }

        vTaskDelay(FFT_TASK_DELAY / portTICK_PERIOD_MS);
    }

    vTaskDelete(NULL);
}

void fft_periph_init(esp_periph_set_handle_t set)
{
    periph_ws2812_cfg_t cfg = {
        .gpio_num = GPIO_LEFT_NUMBER,
        .led_num = LED_NUM,
    };

    ESP_LOGE(TAG, "initialize ws2812.");

    //init matrix
    handle_matrix = periph_ws2812_init(&cfg);

    //raw data nothing
    haveDataUpdated = 0;

    if (handle_matrix != NULL)
    {
        esp_periph_start(set, handle_matrix);
        control_cfg = malloc(sizeof(periph_ws2812_ctrl_cfg_t) * cfg.led_num);
        ESP_LOGE(TAG, "initialize ws2812 successed");
    }

    return;
}

void fft_music_init()
{
    ESP_LOGE(TAG, "fft_music_init:  Enter");

    //semaphore created for sync audio buffer.
    xSemaphore = xSemaphoreCreateMutex();
    if (xSemaphore != NULL)
    {
        xSemaphoreGive(xSemaphore);
        ESP_LOGE(TAG, "fft_music_init:  init xSemaphoreCreateMutex  successed");
    }

    xTaskCreate(fft_music_task, "fft_music_parse", 4096, NULL, 9, NULL);

    for (int i = 0; i < LED_HIGH_LEVEL; i++)
    {
        led_data[i] = 0;
    }

    factor = 256.0f / (THRESHOLD_HIGH - THRESHOLD_LOW);
}

void fft_music_push(char *buffer, int len)
{
    int i;
    short temp_left = 0;
    short temp_right = 0;
    static int samplePeriod = 0;
    float *y_cf_addr = NULL;

    if (len != 2048)
    {
        samplePeriod = 0;
        return;
    }

    if (checkHaveCharRepeat(buffer, len) != 0)
    {
        samplePeriod = 0;
        return;
    }

    if (xSemaphore)
    {
        if (haveDataUpdated == 0 && (pdTRUE == xSemaphoreTake(xSemaphore, 0)))
        {
            y_cf_addr = (float *)(&y_cf[samplePeriod * (N_SAMPLES / 2)]);
            for (i = 0; i < N_SAMPLES / 4; i++)
            {
                temp_left = (short)(buffer[i * 4 + 1] << 8 | buffer[i * 4 + 0]);
                temp_right = (short)(buffer[i * 4 + 3] << 8 | buffer[i * 4 + 2]);
                y_cf_addr[i * 2] = ((float)((temp_left + temp_right) / 32768.0f)) / 2.0f;
                y_cf_addr[i * 2 + 1] = 0;
            }

            // total raw data is N_SAMPLES
            if (++samplePeriod == 4)
            {
                haveDataUpdated = 1;
                samplePeriod = 0;
            }

            xSemaphoreGive(xSemaphore);
        }
    }
}
