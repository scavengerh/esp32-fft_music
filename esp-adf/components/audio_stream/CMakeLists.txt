set(COMPONENT_SRCS "fatfs_stream.c"
                    "i2s_stream.c"
                    "fft_stream.c"
                    "fft_music.c"
                    "http_stream.c"
                    "hls_playlist.c"
                    "raw_stream.c"
                    "spiffs_stream.c"
                    "tone_stream.c"
                    "algorithm_stream.c"                    
                     "../../esp-dsp/modules/common/misc/dsps_pwroftwo.cpp"
                    "../../esp-dsp/modules/fft/float/dsps_fft2r_fc32_ae32.S"
                    "../../esp-dsp/modules/fft/float/dsps_fft2r_fc32_ansi.c"
                    "../../esp-dsp/modules/fft/fixed/dsps_fft2r_sc16_ae32.S"
                    "../../esp-dsp/modules/fft/fixed/dsps_fft2r_sc16_ansi.c"
                    "../../esp-dsp/modules/support/view/dsps_view.cpp"
                    "../../esp-dsp/modules/windows/hann/float/dsps_wind_hann_f32.c")

set(COMPONENT_ADD_INCLUDEDIRS "include"
                "../esp_peripherals/include"
                "../../esp-dsp/modules/support/include"
                "../../esp-dsp/modules/common/include"
                "../../esp-dsp/modules/fft/include"
                "../../esp-dsp/modules/dotprod/float"
                "../../esp-dsp/modules/dotprod/fixed"
                "../../esp-dsp/modules/common/private_include"
                "../../esp-dsp/modules/windows/hann/include")


set(COMPONENT_REQUIRES audio_pipeline audio_sal esp_http_client spiffs esp-adf-libs audio_board esp-sr)

register_component()
