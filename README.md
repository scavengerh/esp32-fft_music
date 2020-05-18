# esp32-fft_music

This is fft calcuate function for audio stream. please copy esp-dsp to esp-adf and add this patch, you can use it.

@Hardware:   ESP32-LyraT
@Software:  ESP-ADF, ESP-DSP, ESP-IDF

pipeline:
  I2s_stream -> fft_music-> fft_data -> led_matrix(ws2812 8*16)

Reference:
esp-adf:
      https://github.com/espressif/esp-adf.git
     
esp-dsp:
      https://github.com/espressif/esp-dsp.git
