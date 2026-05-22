#ifndef AC_CTRL_H
#define AC_CTRL_H

#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

esp_err_t ac_ctrl_init(void);
esp_err_t ac_ctrl_set_power(bool on);
bool ac_ctrl_get_power(void);
esp_err_t ac_ctrl_set_temp(uint8_t temp);
uint8_t ac_ctrl_get_temp(void);
esp_err_t ac_ctrl_set_mode(uint8_t mode); // 0=Cool, 1=Heat, 2=Fan, 3=Dry
uint8_t ac_ctrl_get_mode(void);

#endif // AC_CTRL_H
