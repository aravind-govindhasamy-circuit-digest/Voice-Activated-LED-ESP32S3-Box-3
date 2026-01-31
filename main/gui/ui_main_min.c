/*
 * Minimal UI helpers required by ui_sr.c / ui_boot_animate.c.
 *
 * We do not use the original factory `ui_main.c` menu system.
 */

#include "ui_main.h"

#include "bsp/esp-bsp.h"

void ui_acquire(void)
{
    bsp_display_lock(0);
}

void ui_release(void)
{
    bsp_display_unlock();
}

/* Unused stubs (keep linkers happy if referenced by legacy code that may linger) */
esp_err_t ui_main_start(void)
{
    return ESP_OK;
}

lv_group_t *ui_get_btn_op_group(void)
{
    return NULL;
}

button_style_t *ui_button_styles(void)
{
    return NULL;
}

lv_obj_t *ui_main_get_status_bar(void)
{
    return NULL;
}

void ui_main_status_bar_set_wifi(bool is_connected)
{
    (void)is_connected;
}

void ui_main_status_bar_set_cloud(bool is_connected)
{
    (void)is_connected;
}

