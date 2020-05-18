#ifndef __FFT_MUSIC_H
#define __FFT_MUSIC_H

#include "esp_peripherals.h"

#ifdef __cplusplus
extern "C" {
#endif

void fft_periph_init(esp_periph_set_handle_t set);
void fft_music_init();
void fft_music_push(char* buffer, int len);

#ifdef __cplusplus
}
#endif

#endif
