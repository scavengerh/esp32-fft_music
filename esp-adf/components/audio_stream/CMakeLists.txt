set(COMPONENT_SRCS "fatfs_stream.c"
                    "i2s_stream.c"
                    "http_stream.c"
                    "http_playlist.c"
                    "raw_stream.c"
                    "spiffs_stream.c"
                    "tone_stream.c"
                    "tcp_client_stream.c"
                    "embed_flash_stream.c"
                    "pwm_stream.c"
                    "fft_music.c"
                    "../../esp-dsp/modules/common/misc/dsps_pwroftwo.cpp"
                    "../../esp-dsp/modules/fft/float/dsps_fft2r_fc32_ae32_.S"
                    "../../esp-dsp/modules/fft/float/dsps_fft2r_fc32_ansi.c"
                    "../../esp-dsp/modules/fft/float/dsps_fft2r_bitrev_tables_fc32.c"
                    "../../esp-dsp/modules/fft/fixed/dsps_fft2r_sc16_ae32.S"
                    "../../esp-dsp/modules/fft/fixed/dsps_fft2r_sc16_ansi.c"
                    "../../esp-dsp/modules/fft/float/dsps_fft2r_bitrev_tables_fc32.c"
                    "../../esp-dsp/modules/support/view/dsps_view.cpp"
                    "../../esp-dsp/modules/windows/hann/float/dsps_wind_hann_f32.c")
set(COMPONENT_ADD_INCLUDEDIRS "include"
                "../esp_peripherals/include"
                "../../esp-dsp/modules/support/include"
                "../../esp-dsp/modules/common/include"
                "../../esp-dsp/modules/fft/include"
                "../../esp-dsp/modules/dotprod/float"
                "../../esp-dsp/modules/dotprod/fixed"
                "../../esp-dsp/modules/common/include"
                "../../esp-dsp/modules/windows/hann/include")

set(COMPONENT_PRIV_INCLUDEDIRS "lib/hls/include" "lib/gzip/include")
list(APPEND COMPONENT_SRCS  "lib/hls/hls_parse.c"
                            "lib/hls/hls_playlist.c"
                            "lib/hls/line_reader.c"
                            "lib/hls/join_path.c")

list(APPEND COMPONENT_SRCS  "lib/gzip/gzip_miniz.c")

set(COMPONENT_REQUIRES audio_pipeline audio_sal esp_http_client tcp_transport spiffs esp-adf-libs audio_board bootloader_support esp_dispatcher esp_actions tone_partition)

if((${IDF_TARGET} STREQUAL "esp32") OR (${IDF_TARGET} STREQUAL "esp32s3"))
    list(APPEND COMPONENT_SRCS "algorithm_stream.c" "tts_stream.c")
    list(APPEND COMPONENT_REQUIRES esp-sr)
endif()

register_component()
