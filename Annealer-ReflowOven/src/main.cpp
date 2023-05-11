#include <Arduino.h>

#define DISABLE_ALL_LIBRARY_WARNINGS  // no warning about TFT_eSPI no cs pin

#include "ArduinoOTA.h"
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
    Serial.begin(115200);
    Serial.println("Starting");

    lv_init();

    if (!SPIFFS.begin())
    {
        Serial.println("Cannot mount SPIFFS volume...");
        /* while (1)
        {
            // onboard_led.on = millis() % 200 < 50;
            // onboard_led.update();
        } */
        config.config_struct =
            Config::ConfigStruct{.temperature = 0, .timer = 0, .kp = 5.502096176, .ki = 0.009857209, .kd = 0.047900259};
    }
    else if (!config.ReadConfig())
    {
        config.config_struct =
            Config::ConfigStruct{.temperature = 0, .timer = 0, .kp = 5.502096176, .ki = 0.009857209, .kd = 0.047900259};
        config.WriteConfig();
    }

    pid = new PIDControl(
        ssr_pin, 1000, []() { return thermocouple.readCelsius(); }, config, []() { web_interface->WriteConfig(); });

    ui = new UI{button, encoder, tft, pid};

    web_interface = new WebInterface{WIFI_SSID, WIFI_PASSWORD, *pid, config};

    web_interface->Init();

    ArduinoOTA.setPassword("annealer_admin_password");
    ArduinoOTA.setPort(3232);

    ArduinoOTA
        .onStart(
            []()
            {
                String type;
                if (ArduinoOTA.getCommand() == U_FLASH)
                    type = "sketch";
                else  // U_SPIFFS
                    type = "filesystem";

                // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
                Serial.println("Start updating " + type);
            })
        .onEnd([]() { Serial.println("\nEnd"); })
        .onProgress([](unsigned int progress, unsigned int total)
                    { Serial.printf("Progress: %u%%\r", (progress / (total / 100))); })
        .onError(
            [](ota_error_t error)
            {
                Serial.printf("Error[%u]: ", error);
                if (error == OTA_AUTH_ERROR)
                    Serial.println("Auth Failed");
                else if (error == OTA_BEGIN_ERROR)
                    Serial.println("Begin Failed");
                else if (error == OTA_CONNECT_ERROR)
                    Serial.println("Connect Failed");
                else if (error == OTA_RECEIVE_ERROR)
                    Serial.println("Receive Failed");
                else if (error == OTA_END_ERROR)
                    Serial.println("End Failed");
            });

    ArduinoOTA.begin();

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
