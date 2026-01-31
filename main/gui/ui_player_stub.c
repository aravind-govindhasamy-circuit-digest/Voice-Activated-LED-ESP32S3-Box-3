/*
 * Minimal stub for ui_sr.c dependency.
 *
 * The original factory demo has a full music player page; we don't need it
 * for the light demo, but ui_sr.c hides/shows that page via get_player_page().
 */

#include "lvgl.h"
#include "ui_player.h"

lv_obj_t *get_player_page(void)
{
    return NULL;
}

