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


#define LED_HIGH_LEVEL 16
#define LED_NUM (LED_HIGH_LEVEL * 8)

#define N_SAMPLES 512
#define N_FFT_SIZE 16 //(N_SAMPLES / 4 / LED_HIGH_LEVEL)

//input audio raw data
float y_cf[N_SAMPLES * 2];
//output fft data
float sum_y[N_SAMPLES / 2];

//Led relate
periph_ws2812_ctrl_cfg_t *control_cfg = NULL;
esp_periph_handle_t handle_matrix = NULL;

#define GPIO_Pin_NUM 14

//display freq index: 31.5, 45, 63, 90, 125, 180, 250, 360, 550, 680, 890, 1k, 2k, 4k, 8k, 16k
int16_t freq_offset[17] = {0, 1, 2, 3, 4, 5, 8, 11, 17, 25, 30, 41, 44, 80, 160, 200, 255};

unsigned char led_data[LED_HIGH_LEVEL];


//update flow count
unsigned char led_updateCount[LED_HIGH_LEVEL];
#define SLOW_SPEED (2)


//display review period: FFT_TASK_DELAY * LED_UPDATE_PERIOD
#define LED_UPDATE_PERIOD (2) //Ms

#define THRESHOLD_HIGH (0.1f)
#define THRESHOLD_LOW (0.0005f)
static float factor = 0.0f;

SemaphoreHandle_t xSemaphore = NULL;
int haveDataUpdated = 0;
int isCleanMatrixLed = 0;

uint8_t R_Color = 0;
uint8_t B_Color = 0;
uint8_t G_Color = 0;
uint32_t Temp_Color = 0;
float RGB_max;
float RGB_min;
float RGB_Adj; 
int difs;

// 其中的H、S、V分别代表色调（H）、饱和度（S）和明度（V）
uint32_t HSVtoRGB(uint16_t hh, uint16_t ss, uint16_t vv)      
{
    int i;
    uint16_t h = hh;
    uint16_t v = vv;
    uint16_t s = ss;
    if(h >= 360) h = 360;
    if(s >= 100) s = 100;
    if(v >= 100) v = 100;

    i = h / 60;
    difs = h % 60; 
    RGB_max = v * 2.55f;
    RGB_min = RGB_max * (100 - s) / 100.0f;
    RGB_Adj = (RGB_max - RGB_min) * difs / 60.0f; 
    switch(i)
    {
    case 0:
        R_Color = RGB_max;
        G_Color = RGB_min + RGB_Adj;
        B_Color = RGB_min;
        break;

    case 1:
        R_Color = RGB_max - RGB_Adj;
        G_Color = RGB_max;
        B_Color = RGB_min;
        break;

    case 2:
        R_Color = RGB_min;
        G_Color = RGB_max;
        B_Color = RGB_min + RGB_Adj;
        break;

    case 3:
        R_Color = RGB_min;
        G_Color = RGB_max - RGB_Adj;
        B_Color = RGB_max;
        break;

    case 4:
        R_Color = RGB_min + RGB_Adj;
        G_Color = RGB_min;
        B_Color = RGB_max;
        break;

    default:
        R_Color = RGB_max;
        G_Color = RGB_min;
        B_Color = RGB_max - RGB_Adj;
        break;
    }
    Temp_Color = ((uint32_t)R_Color << 8) | ((uint32_t)G_Color << 16) | ((uint32_t)B_Color << 0);
    return Temp_Color;
}


static void fft_update_led(unsigned char display[], int size)
{
    static int color_counter = 0;
    ++color_counter;
    uint32_t color_cur = HSVtoRGB(color_counter%360, color_counter%49 + 50, 65);
    if (size != LED_HIGH_LEVEL)
    {
       return;
    }

    if (handle_matrix != NULL)
    {
        for (int i = 0; i < LED_NUM; i++)
        {
            // control_cfg[i].color = (display[i / 8] > ((i % 8) * 32)) ? color_cur : LED2812_COLOR_BLACK;
            control_cfg[i].color = ((display[i / 8]) > ((i % 8) * 32)) ? color_cur : LED2812_COLOR_BLACK;
            control_cfg[i].loop = 50;
            if(control_cfg[i].color == LED2812_COLOR_BLACK){
            control_cfg[i].mode = PERIPH_WS2812_ONE;
                control_cfg[i].time_off_ms = 0;
                control_cfg[i].time_on_ms = 0;
            }else{
            control_cfg[i].mode = PERIPH_WS2812_FADE;
                control_cfg[i].time_off_ms = 100;
                control_cfg[i].time_on_ms = 100;
            }
        }
        periph_ws2812_control(handle_matrix, control_cfg, NULL);
    }
}
static float get_maximum(const float* start, int size){
    float maxValue = 0.0f;
    const float *ptr = start;
    if(ptr){
        for(int i = 0; i < size; i++, ptr++){
            if(*ptr > maxValue){
                maxValue = *ptr;
            }
        }
    }
    return maxValue;
}

static void fft_update_data(void)
{
    int i = 0;
    float tempraw = 0.0f;
    unsigned char tempdata;
    int fft_step = (N_SAMPLES/2) / LED_HIGH_LEVEL;
    for (i = 0; i < LED_HIGH_LEVEL; i++)
    {
        tempraw = get_maximum(&sum_y[freq_offset[i]], freq_offset[i + 1] - freq_offset[i]);
        // tempraw = get_maximum(&sum_y[i * fft_step], fft_step);

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

    fft_update_led(led_data, LED_HIGH_LEVEL);

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

                xSemaphoreGive(xSemaphore);
            }
            else
            {
                if (updateLedPeriodCount > LED_UPDATE_PERIOD)
                {
                    updateLedPeriodCount = 0;
                    fft_update_data();
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(LED_UPDATE_PERIOD*4));  //delay 2 ms
    }

    vTaskDelete(NULL);
}

void fft_periph_init(esp_periph_set_handle_t set)
{
    periph_ws2812_cfg_t cfg = {
        .gpio_num = GPIO_Pin_NUM,
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
    short temp_left = 0;
    short temp_right = 0;
    static int samplePeriod = 0;

    if(len < 2048){
        return;
    }


    if (xSemaphore)
    {
        if (haveDataUpdated == 0 && (pdTRUE == xSemaphoreTake(xSemaphore, 0)))
        {
            for (int i = 0; i < N_SAMPLES; i++)
            {
                temp_left = (short)(buffer[i * 4 + 1] << 8 | buffer[i * 4 + 0]);
                temp_right = (short)(buffer[i * 4 + 3] << 8 | buffer[i * 4 + 2]);
                y_cf[i * 2] = ((float)((temp_left + temp_right) / 32768.0f)) / 2.0f;
                y_cf[i * 2 + 1] = 0;
            }
            haveDataUpdated = 1;

            xSemaphoreGive(xSemaphore);
        }
    }
}
