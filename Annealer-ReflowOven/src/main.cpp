#include <Arduino.h>

#include "Config.h"
#include "LEDStrip.h"
#include "OneButton.h"
#include "PIDControl.h"
#include "RotaryEncoder.h"
#include "TFT_eSPI.h"
#include "UI.h"
#include "WebInterface.h"
#include "max6675.h"
#include "pin_config.h"

// Cable pinout
//   2       4       6       8
//   1       3       5       7
//
//   SDA/THERM_DO/18      THERMO_CLK/16       GND     3V3
//   SCL/THERM_CS/8       17/SSR              GND     GND

uint8_t ssr_pin = 17;  // todo: check all pins

uint8_t thermo_do = PIN_IIC_SDA;
uint8_t thermo_cs = PIN_IIC_SCL;
uint8_t thermo_clk = 16;

TFT_eSPI tft = TFT_eSPI(170, 320);
RotaryEncoder encoder(PIN_ENCODE_A, PIN_ENCODE_B, RotaryEncoder::LatchMode::TWO03);
OneButton button{PIN_ENCODE_BTN};  // active low, pullup by default
UI *ui;

LEDStrip led_strip{};

MAX6675 thermocouple(thermo_clk, thermo_cs, thermo_do);
PIDControl *pid;
WebInterface *web_interface;

Config config{"/config.hex"};

void setup()
{
    pinMode(PIN_POWER_ON, OUTPUT);
    digitalWrite(PIN_POWER_ON, HIGH);
    // Serial.begin(115200);
    // Serial.println("Starting");

    lv_init();

    if (!SPIFFS.begin())
    {
        // Serial.println("Cannot mount SPIFFS volume...");
        /* while (1)
        {
            // onboard_led.on = millis() % 200 < 50;
            // onboard_led.update();
        } */
        config.config_struct = Config::ConfigStruct{.temperature = 0, .timer = 0, .kp = 0, .ki = 0, .kd = 0};
    }
    else if (!config.ReadConfig())
    {
        config.config_struct = Config::ConfigStruct{.temperature = 0, .timer = 0, .kp = 0, .ki = 0, .kd = 0};
        config.WriteConfig();
    }

    pid = new PIDControl(
        ssr_pin, 6, 1000, []() { return thermocouple.readCelsius(); }, config);

    ui = new UI{button, encoder, tft, pid};

    web_interface = new WebInterface{WIFI_SSID, WIFI_PASSWORD, *pid};

    web_interface->Init();

    xTaskCreate(
        [](void *param)
        {
            // Serial.println("PID Loop Task");
            pid->Task();
        },
        "pid_task",
        2048,  // stack size
        NULL,
        3,  // priority, higher is higher
        NULL);

    xTaskCreate([](void *param) { ui->Task(); }, "ui_task", 4096, NULL, 2, NULL);

    xTaskCreate([](void *param) { web_interface->Task(); }, "web_interface_task", 4096, NULL, 2, NULL);

    // vTaskDelete(NULL);  // skip main loop
    //  put your setup code here, to run once:
}

void loop()
{
    Serial.println("Loop is running");
    delay(1000);
    // put your main code here, to run repeatedly:
}
