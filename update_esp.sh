#!/bin/bash
#


git clone -b release/v2.4 --recursive https://github.com/espressif/esp-adf.git 

#copy esp32-fft-music to esp-adf directory
#cp -af  esp-adf/components/audio_stream/*  $ADF_PATH/components/audio_stream/

cd esp-adf
rm -rf esp-idf
git clone -b release/v4.1 --recursive https://github.com/espressif/esp-idf.git
cd esp-idf
chmod +x install.sh
./install.sh

cd ..
rm -rf esp-dsp
git clone https://github.com/espressif/esp-dsp.git 
