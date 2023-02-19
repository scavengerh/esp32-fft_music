# esp32-fft music

# This is fft calcuate function for audio stream. please copy esp-dsp to esp-adf and add this patch, you can use it.

 ## Hardware:   ESP32-LyraT
 ## Software:  ESP-ADF, ESP-DSP, ESP-IDF

 ### pipeline:
  - I2s stream -> fft music-> fft data -> led matrix(ws2812 8x16) -> RMT send

  - Reference:
  #### esp-adf:
  - gitclone -b release/v2.4 --recursive https://github.com/espressif/esp-adf.git>

  #### esp-adf:
  - cd esp-adf
  - git clone -b release/v4.4 --recursive https://github.com/espressif/esp-idf.git
  - ./insall.sh
  - . ./export.sh

  #### esp-dsp: 
  - cd esp-adf
  - git clone https://github.com/espressif/esp-dsp.git

 # esp32-fft music install
 # 1, components install:
 ##  copy esp32-fft-music to esp-adf/componemts directory
 ### cp -af  esp-adf/components/audio_stream/*  $ADF_PATH/components/audio_stream/
 # 2, install ws2812 periph to main.c
 ##  fft_periph_init(set);


 ** you can use update_esp.sh script to auto install libraries **
<img src="img/fft_music.jpg">
