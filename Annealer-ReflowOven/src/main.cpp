#include <Arduino.h>

#include "Config.h"
#include "LEDStrip.h"
#include "PIDControl.h"
#include "TFT_eSPI.h"
#include "WebInterface.h"
#include "max6675.h"
#include "pin_config.h"

uint8_t ssr_pin = 40;  // todo: change

uint8_t thermo_do = PIN_IIC_SDA;
uint8_t thermo_cs = PIN_IIC_SCL;
uint8_t thermo_clk = 16;

LEDStrip led_strip{};

MAX6675 thermocouple(thermo_clk, thermo_cs, thermo_do);
PIDControl* pid;

Config config{"/config.hex"};

void setup()
{
    pinMode(PIN_POWER_ON, OUTPUT);
    digitalWrite(PIN_POWER_ON, HIGH);
    Serial.begin(115200);

    if (!config.ReadConfig())
    {
        config.config_struct =
            Config::ConfigStruct{.default_temperature = 0, .default_timer = 0, .kp = 0, .ki = 0, .kd = 0};
        config.WriteConfig();
    }

    pid = new PIDControl(
        ssr_pin, 6, 1000, []() { return thermocouple.readCelsius(); }, config);

    if (!SPIFFS.begin())
    {
        Serial.println("Cannot mount SPIFFS volume...");
        while (1)
        {
            // onboard_led.on = millis() % 200 < 50;
            // onboard_led.update();
        }
    }

    xTaskCreate(
        [](void* param)
        {
            // Serial.println("PID Loop Task");
            pid->Task();
        },
        "pid_task",
        2048,  // stack size
        NULL,
        3,  // priority, higher is higher
        NULL);

    vTaskDelete(NULL);  // skip main loop
    // put your setup code here, to run once:
}

void loop()
{
    Serial.println("Loop is running");
    delay(1000);
    // put your main code here, to run repeatedly:
}