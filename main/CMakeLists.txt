idf_component_register(
    SRCS "app.cpp" "settings.cpp" "storage.cpp" "sd_tools.cpp" "sd_scope.cpp" "fseq.cpp"
         "output.cpp" "dma_test.cpp" "strip_speed.cpp" "network.cpp" "web.cpp"
    INCLUDE_DIRS "." ".."
    REQUIRES driver esp_timer esp_psram esp_wifi esp_event esp_netif
             nvs_flash fatfs sdmmc app_update json esp_http_server esp_rom esp_lcd espcoredump mbedtls esp_adc
    EMBED_TXTFILES "web/index.html" "web/wifi.html" "web/leds.html" "web/speed.html" "web/sd.html" "web/recent.js" "web/scope.html" "web/duty.js")
target_compile_features(${COMPONENT_LIB} PUBLIC cxx_std_17)
