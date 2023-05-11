#pragma once

#define LV_CONF_SUPPRESS_DEFINE_CHECK
#define DISABLE_ALL_LIBRARY_WARNINGS

#include "OneButton.h"
#include "PIDControl.h"
#include "RotaryEncoder.h"
#include "TFT_eSPI.h"
#include "lv_conf.h"
#include "lvgl.h"
#include "pin_config.h"

class UI
{
public:
    UI(OneButton &button, RotaryEncoder &encoder, TFT_eSPI &tft, PIDControl *pid_control)
        : button_{button}, pid_control_{pid_control}
    {
        encoder_ = &encoder;
        tft_ = &tft;
        lv_input_event = xEventGroupCreate();
    }

    void Task();

    /* struct LabelUpdateStruct
    {
        lv_obj_t *label;
        char *text;
        PIDControl *pid_control;
    }; */

private:
    OneButton &button_;
    static RotaryEncoder *encoder_;
    static TFT_eSPI *tft_;
    PIDControl *pid_control_;

#define LV_BUTTON _BV(0)
#define LV_ENCODER_CW _BV(1)
#define LV_ENCODER_CCW _BV(2)
    static EventGroupHandle_t lv_input_event;

    static void lv_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p);
    static void lv_encoder_read(lv_indev_drv_t *indev_drv, lv_indev_data_t *data);

    lv_obj_t *create_btn(lv_obj_t *parent, const char *text);
};
