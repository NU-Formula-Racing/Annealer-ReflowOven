#include "UI.h"

RotaryEncoder *UI::encoder_;
TFT_eSPI *UI::tft_;
EventGroupHandle_t UI::lv_input_event;

void UI::lv_encoder_read(lv_indev_drv_t *indev_drv, lv_indev_data_t *data)
{
    EventGroupHandle_t *lv_input_event = (EventGroupHandle_t *)indev_drv->user_data;
    EventBits_t bit = xEventGroupGetBits(lv_input_event);
    data->state = LV_INDEV_STATE_RELEASED;
    if (bit & LV_BUTTON)
    {
        xEventGroupClearBits(lv_input_event, LV_BUTTON);
        data->state = LV_INDEV_STATE_PR;
    }
    else if (bit & LV_ENCODER_CW)
    {
        xEventGroupClearBits(lv_input_event, LV_ENCODER_CW);
        data->enc_diff = 1;
    }
    else if (bit & LV_ENCODER_CCW)
    {
        xEventGroupClearBits(lv_input_event, LV_ENCODER_CCW);
        data->enc_diff = -1;
    }
}

void UI::Task()
{
    static lv_disp_draw_buf_t draw_buf;
    static lv_color_t *buf1, *buf2;

    tft_->begin();
    Serial.println("TFT Begun");
    tft_->setRotation(3);
    tft_->fillScreen(TFT_WHITE);

    button_.attachClick(
        [](void *param)
        {
            EventGroupHandle_t *lv_input_event = (EventGroupHandle_t *)param;
            xEventGroupSetBits(lv_input_event, LV_BUTTON);
            // xEventGroupSetBits(global_event_group, WAV_RING_1);
        },
        lv_input_event);

    lv_init();
    buf1 = (lv_color_t *)heap_caps_malloc(LV_BUF_SIZE * sizeof(lv_color_t), MALLOC_CAP_8BIT | MALLOC_CAP_SPIRAM);
    assert(buf1);
    buf2 = (lv_color_t *)heap_caps_malloc(LV_BUF_SIZE * sizeof(lv_color_t), MALLOC_CAP_8BIT | MALLOC_CAP_SPIRAM);
    assert(buf2);
    lv_disp_draw_buf_init(&draw_buf, buf1, buf2, LV_BUF_SIZE);
    /*Initialize the display*/
    static lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);
    /*Change the following line to your display resolution*/
    disp_drv.hor_res = LV_SCREEN_WIDTH;
    disp_drv.ver_res = LV_SCREEN_HEIGHT;
    disp_drv.flush_cb = lv_disp_flush;
    disp_drv.draw_buf = &draw_buf;
    lv_disp_drv_register(&disp_drv);

    static lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_ENCODER;
    indev_drv.read_cb = lv_encoder_read;
    indev_drv.user_data = lv_input_event;
    static lv_indev_t *lv_encoder_indev = lv_indev_drv_register(&indev_drv);
    lv_group_t *g = lv_group_create();
    lv_indev_set_group(lv_encoder_indev, g);
    lv_group_set_default(g);
    // ui_boot_anim();
    /* lv_obj_t *self_test_btn = create_btn(lv_scr_act(), "self test");
    lv_obj_align(self_test_btn, LV_ALIGN_CENTER, -80, 0);
    lv_obj_add_event_cb(
        self_test_btn,
        [](lv_event_t *e) {
          lv_obj_clean(lv_scr_act());
          xTaskCreatePinnedToCore(mic_spk_task, "mic_spk_task", 1024 * 20, NULL, 3, NULL, 0);
          xEventGroupSetBits(lv_input_event, LV_SELF_TEST_START);
        },
        LV_EVENT_CLICKED, NULL); */

    /* lv_obj_t *into_ui_btn = create_btn(lv_scr_act(), "into ui");
    lv_obj_align(into_ui_btn, LV_ALIGN_CENTER, 80, 0);
    lv_obj_add_event_cb(
        into_ui_btn,
        [](lv_event_t *e) {
          lv_obj_clean(lv_scr_act());
          xTaskCreatePinnedToCore(wav_task, "wav_task", 1024 * 4, NULL, 2, NULL, 0);
          xTaskCreatePinnedToCore(mic_fft_task, "fft_task", 1024 * 20, NULL, 1, NULL, 0);
          xEventGroupSetBits(lv_input_event, LV_UI_DEMO_START);
        },
        LV_EVENT_CLICKED, NULL); */

    /* lv_obj_t *test_btn = create_btn(lv_scr_act(), "test");
    lv_obj_align(test_btn, LV_ALIGN_CENTER, 80, 0);
    lv_obj_add_event_cb(
        test_btn,
        [](lv_event_t *e)
        {
            lv_obj_clean(lv_scr_act());
            Serial.println("Test clicked");
        },
        LV_EVENT_CLICKED,
        NULL); */

    lv_obj_t *title = lv_label_create(lv_scr_act());
    lv_label_set_text(title, "NFR Annealer");
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 0);

    const char *wifi_info_fmt_str{"SSID: %s, Pass: %s\nhttp://formula.annealer.nu/\nIf on a phone, disable cellular\n"};
    char wifi_info_str[100];
    sprintf(wifi_info_str, wifi_info_fmt_str, WIFI_SSID, WIFI_PASSWORD);
    lv_obj_t *wifi_info = lv_label_create(lv_scr_act());
    lv_label_set_text(wifi_info, wifi_info_str);
    lv_obj_align_to(wifi_info, title, LV_ALIGN_OUT_BOTTOM_MID, 0, 0);

    const char *status_fmt_str{"Heater output: %5.2f"};
    char status_str[100];
    // sprintf(status_str, status_fmt_str, pid_control_->GetDutyCycle());
    lv_obj_t *status = lv_label_create(lv_scr_act());
    // lv_label_set_text(status, status_str);

    const char *temperature_fmt_str{"Temperature: %5.2fC"};
    char temperature_str[100];
    // sprintf(temperature_str, temperature_fmt_str, pid_control_->GetInput());
    lv_obj_t *temperature = lv_label_create(lv_scr_act());
    // lv_label_set_text(temperature, temperature_str);

    const char *temperature_setpoint_fmt_str{"Temperature setpoint: %5.2fC"};
    char temperature_setpoint_str[100];
    // sprintf(temperature_setpoint_str, temperature_setpoint_fmt_str, pid_control_->GetSetpoint());
    lv_obj_t *temperature_setpoint = lv_label_create(lv_scr_act());
    // lv_label_set_text(temperature_setpoint, temperature_setpoint_str);

    const char *tuning_fmt_str{"Kp: %5.5f, Ki: %5.5f, Kd: %5.5f"};
    char tuning_str[100];
    lv_obj_t *tuning = lv_label_create(lv_scr_act());

    attachInterrupt(
        digitalPinToInterrupt(PIN_ENCODE_A), []() { encoder_->tick(); }, CHANGE);
    attachInterrupt(
        digitalPinToInterrupt(PIN_ENCODE_B), []() { encoder_->tick(); }, CHANGE);

    while (1)
    {
        delay(1);
        // Serial.println("UI loop");
        if (pid_control_->GetEnabled())
        {
            sprintf(status_str, status_fmt_str, pid_control_->GetDutyCycle());
            lv_label_set_text(status, status_str);
        }
        else if (pid_control_->GetTuning())
        {
            sprintf(status_str, "Tuning, heater output: %5.2f", pid_control_->GetDutyCycle());
            lv_label_set_text(status, status_str);
        }
        else
        {
            lv_label_set_text(status, "Disabled");
        }
        sprintf(temperature_str, temperature_fmt_str, pid_control_->GetInput());
        lv_label_set_text(temperature, temperature_str);

        sprintf(temperature_setpoint_str, temperature_setpoint_fmt_str, pid_control_->GetSetpoint());
        lv_label_set_text(temperature_setpoint, temperature_setpoint_str);

        sprintf(tuning_str, tuning_fmt_str, pid_control_->GetKp(), pid_control_->GetKi(), pid_control_->GetKd());
        lv_label_set_text(tuning, tuning_str);

        lv_obj_align_to(status, wifi_info, LV_ALIGN_OUT_BOTTOM_MID, 0, 0);
        lv_obj_align_to(temperature, status, LV_ALIGN_OUT_BOTTOM_MID, 0, 0);
        lv_obj_align_to(temperature_setpoint, temperature, LV_ALIGN_OUT_BOTTOM_MID, 0, 0);
        lv_obj_align_to(tuning, temperature_setpoint, LV_ALIGN_OUT_BOTTOM_MID, 0, 0);

        // button.tick();
        // lv_task_handler();
        lv_refr_now(NULL);
        lv_timer_handler();

        /* RotaryEncoder::Direction dir = encoder.getDirection();
        if (dir != RotaryEncoder::Direction::NOROTATION) {
          if (dir != RotaryEncoder::Direction::CLOCKWISE) {
            xEventGroupSetBits(lv_input_event, LV_ENCODER_CW);
            xEventGroupSetBits(lv_input_event, LV_ENCODER_LED_CW);
          } else {
            xEventGroupSetBits(lv_input_event, LV_ENCODER_CCW);
            xEventGroupSetBits(lv_input_event, LV_ENCODER_LED_CCW);
          }
          if (is_self_check_completed())
            ui_switch_page();
        } */

        /* EventBits_t bit = xEventGroupGetBits(lv_input_event);
        if (bit & LV_SELF_TEST_START) {
          xEventGroupClearBits(lv_input_event, LV_SELF_TEST_START);
          uint16_t temp = 0x41;
          xQueueSend(led_setting_queue, &temp, 0);
          self_test();
          wifi_init = true;
          temp = (0xF << 6) | 0x02;
          xQueueSend(led_setting_queue, &temp, 0);
        } else if (bit & LV_UI_DEMO_START) {
          xEventGroupClearBits(lv_input_event, LV_UI_DEMO_START);
          ui_init();
        }*/
    }
}

void UI::lv_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p)
{
    uint32_t w = (area->x2 - area->x1 + 1);
    uint32_t h = (area->y2 - area->y1 + 1);
    tft_->setAddrWindow(area->x1, area->y1, w, h);
    tft_->pushColors((uint16_t *)&color_p->full, w * h);
    lv_disp_flush_ready(disp);
}

lv_obj_t *UI::create_btn(lv_obj_t *parent, const char *text)
{
    lv_obj_t *btn = lv_btn_create(parent);
    lv_obj_set_size(btn, 120, 100);

    lv_obj_t *label = lv_label_create(btn);
    lv_obj_center(label);
    lv_label_set_text(label, text);
    return btn;
}
