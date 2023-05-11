#pragma once

#include "SPIFFS.h"

class Config
{
public:
    Config(String filename) : filename_{filename} {}

    struct ConfigStruct
    {
        float temperature;
        uint32_t timer;
        float kp;
        float ki;
        float kd;
    };

    void WriteConfig()
    {
        // SPIFFS.begin();
        SPIFFS.remove(filename_);
        File file = SPIFFS.open(filename_, FILE_WRITE, true);
        void *config_pointer = &config_struct;
        file.write(reinterpret_cast<uint8_t *>(config_pointer), sizeof(config_struct));
        /* for (uint16_t i = 0; i < sizeof(config_struct); i++)
        {
            file.write(reinterpret_cast<uint8_t *>(config_pointer)[i]);
        } */
        file.close();
        //    SPIFFS.end();
    }

    bool ReadConfig()
    {
        // SPIFFS.begin();
        if (!SPIFFS.exists(filename_))
        {
            return false;
        }
        File file = SPIFFS.open(filename_, FILE_READ);
        void *config_pointer = &config_struct;
        file.read(reinterpret_cast<uint8_t *>(config_pointer), sizeof(config_struct));
        /* for (uint16_t i = 0; i < sizeof(config_struct) && file.available(); i++)
        {
            reinterpret_cast<uint8_t *>(config_pointer)[i] = file.read();
        } */
        file.close();
        return true;
        // SPIFFS.end();
    }

    ConfigStruct config_struct{};

private:
    String filename_;
};
