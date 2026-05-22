#ifndef AC_UI_H
#define AC_UI_H

#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

esp_err_t ac_ui_start(void);
void ac_ui_show_screen(void);
void ac_ui_update(bool power, uint8_t temp, uint8_t mode);

#endif // AC_UI_H
