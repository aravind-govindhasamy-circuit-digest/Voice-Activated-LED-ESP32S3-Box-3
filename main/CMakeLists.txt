set(SRC_FILES
    "main.c"
    "light_state.c"
    "light_ctrl.c"
    "light_ui.c"

    "app/app_led.c"
    "app/app_sr.c"
    "app/app_sr_handler.c"
    "app/app_status.c"
    "app/app_wifi.c"
    "app/app_mqtt.c"

    "gui/ui_boot_animate.c"
    "gui/ui_sr.c"
    "gui/ui_main_min.c"
    "gui/ui_player_stub.c"

    # Images needed by boot animation + SR overlay
    "gui/image/esp_logo.c"
    "gui/image/esp_text.c"
    "gui/image/mic_logo.c"

    # Font used by ui_sr.c label
    "gui/font/font_cn_gb1_28.c"
)

idf_component_register(
    SRCS ${SRC_FILES}
    INCLUDE_DIRS
        "."
        "app"
        "gui"
        "gui/font"
        "gui/image"
)

target_compile_options(${COMPONENT_LIB} PRIVATE "-Wno-format" "-Wno-deprecated-declarations")

set_source_files_properties(
    ${SRC_FILES}
    PROPERTIES COMPILE_OPTIONS
    -DLV_LVGL_H_INCLUDE_SIMPLE)

spiffs_create_partition_image(storage ../spiffs FLASH_IN_PROJECT)
