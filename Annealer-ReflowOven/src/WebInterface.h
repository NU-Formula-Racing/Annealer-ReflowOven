#pragma once

#include <Arduino.h>
#include <ArduinoJson.h>
#include <DNSServer.h>
#include <ESPAsyncWebServer.h>
#include <SPIFFS.h>
#include <WiFi.h>

#include "PIDControl.h"

class WebInterface
{
public:
    WebInterface(const char *ssid, const char *password, PIDControl &pid_control) : pid_control_{pid_control}
    {
        WiFi.softAP(ssid, password);
    }

    void Init()
    {
        initWebServer();
        initWebSocket();
        dnsServer.start(kDnsPort, "*", WiFi.softAPIP());
    }

    void Task()
    {
        uint32_t last_notify = millis();
        for (;;)
        {
            if (millis() > last_notify + 100)
            {
                notifyClients();
            }
            dnsServer.processNextRequest();
            taskYIELD();
        }
    }

private:
    PIDControl &pid_control_;
    const byte kDnsPort{53};
    DNSServer dnsServer;
    AsyncWebServer server{80};
    AsyncWebSocket ws{"/ws"};

    void onRootRequest(AsyncWebServerRequest *request) { request->send(SPIFFS, "/index.html", "text/html"); }

    void initWebServer()
    {
        server.on("/", [this](AsyncWebServerRequest *request) { this->onRootRequest(request); });
        server.serveStatic("/", SPIFFS, "/");
        server.begin();
    }

    void notifyClients()
    {
        static StaticJsonDocument<10000> doc;
        doc["temperature"] = pid_control_.GetInput();
        doc["setpoint"] = pid_control_.GetSetpoint();
        doc["enabled"] = pid_control_.GetEnabled();
        doc["duty_cycle"] = pid_control_.GetDutyCycle();
        doc["remaining_time"] = pid_control_.GetTimeRemaining();
        doc["timer"] = pid_control_.GetTimer();

        static char data[10000];
        size_t len = serializeJson(doc, data);
        ws.textAll(data, len);
    }

    void handleWebSocketMessage(void *arg, uint8_t *data, size_t len)
    {
        AwsFrameInfo *info = (AwsFrameInfo *)arg;
        if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT)
        {
            const uint8_t size = JSON_OBJECT_SIZE(1);
            StaticJsonDocument<size> json;
            DeserializationError err = deserializeJson(json, data);
            if (err)
            {
                Serial.print(F("deserializeJson() failed with code "));
                Serial.println(err.c_str());
                return;
            }

            if (json.containsKey("setpoint"))
            {
                pid_control_.SetSetpoint(json["setpoint"]);
            }

            if (json.containsKey("time"))
            {
                pid_control_.SetTimer(json["time"]);
            }

            if (json.containsKey("action"))
            {
                const char *action = json["action"];
                if (strcmp(action, "autotune") == 0)
                {
                    pid_control_.Autotune();
                }
                else if (strcmp(action, "enable") == 0)
                {
                    pid_control_.Enable();
                }
                else if (strcmp(action, "disable") == 0)
                {
                    pid_control_.Disable();
                }
                else if (strcmp(action, "write_config") == 0)
                {
                    pid_control_.WriteConfig();
                }
            }
        }
    }

    void onEvent(
        AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len)
    {
        switch (type)
        {
            case WS_EVT_CONNECT:
                Serial.printf(
                    "WebSocket client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
                break;
            case WS_EVT_DISCONNECT:
                Serial.printf("WebSocket client #%u disconnected\n", client->id());
                break;
            case WS_EVT_DATA:
                handleWebSocketMessage(arg, data, len);
                break;
            case WS_EVT_PONG:
            case WS_EVT_ERROR:
                break;
        }
    }

    void initWebSocket()
    {
        ws.onEvent([this](AsyncWebSocket *server,
                          AsyncWebSocketClient *client,
                          AwsEventType type,
                          void *arg,
                          uint8_t *data,
                          size_t len) { this->onEvent(server, client, type, arg, data, len); });
        server.addHandler(&ws);
    }
};
