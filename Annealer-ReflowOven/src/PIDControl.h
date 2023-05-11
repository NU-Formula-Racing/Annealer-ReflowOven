#pragma once

#include <Arduino.h>

#include <functional>

#include "Config.h"
#include "ESP32_FastPWM.h"
// #include "PID_v1.h"
//  #include "pidautotuner.h"
#include "QuickPID.h"
#include "sTune.h"

class PIDControl
{
public:
    PIDControl(uint8_t pwm_pin,
               float pwm_frequency,
               uint32_t control_loop_period,
               std::function<double(void)> get_input,
               Config* config)
        : kPwmPin{pwm_pin},
          kPwmFrequency{pwm_frequency},
          // pwm_{new ESP32_FAST_PWM(kPwmPin, kPwmFrequency, 0, 0, 14)},
          kControlLoopPeriod{control_loop_period},
          get_input_{get_input},
          config_{config},
          pid_controller_{&input_,
                          &output_,
                          &config_->config_struct.temperature,
                          config_->config_struct.kp,
                          config_->config_struct.ki,
                          config_->config_struct.kd,
                          QuickPID::Action::direct},
          pid_tuner_{
              &input_, &output_, sTune::TuningMethod::Mixed_PID, sTune::Action::directIP, sTune::SerialMode::printOFF}
    {
        // ledcSetup(0, 10, 20);
        pinMode(pwm_pin, OUTPUT);
        digitalWrite(pwm_pin, LOW);
        pid_controller_.SetOutputLimits(0, 100);
        // pid_controller_.SetSampleTime(kControlLoopPeriod * 0.9);  // less to make sure it actually runs every time
    }

    void SetSetpoint(double setpoint)
    {
        if (!tuning_)
        {
            config_->config_struct.temperature = setpoint;
        }
    }

    // FreeRTOS Task that runs every control loop period
    void Task()
    {
        for (;;)
        {
            input_ = get_input_();
            if (tuning_)
            {
                /* output_ = pid_tuner_.tunePID(input_, micros());
                if (pid_tuner_.isFinished())
                {
                    tuning_ = false;
                    SetTunings(pid_tuner_.getKp(), pid_tuner_.getKi(), pid_tuner_.getKd());
                    output_ = 0;
                } */
                switch (pid_tuner_.Run())
                {
                    case pid_tuner_.tunings:  // active just once when sTune is done
                        pid_tuner_.GetAutoTunings(&config_->config_struct.kp,
                                                  &config_->config_struct.ki,
                                                  &config_->config_struct.kd);  // sketch variables updated by sTune
                        pid_controller_.SetMode(QuickPID::Control::automatic);  // the PID is turned on
                        SetTunings(config_->config_struct.kp,
                                   config_->config_struct.ki,
                                   config_->config_struct.kd);  // update PID with the new tunings
                        tuning_ = false;
                        output_ = 0;
                        break;

                    case pid_tuner_.runPid:  // active once per sample after tunings
                        tuning_ = false;
                        output_ = 0;
                        break;

                    default:
                        break;
                }
            }
            else
            {
                if (enabled_)
                {
                    pid_controller_.SetMode(QuickPID::Control::automatic);
                    pid_controller_.Compute();
                    if (timer_ < (millis() - enable_time_))
                    {
                        enabled_ = false;
                    }
                }
                else
                {
                    output_ = 0;
                    pid_controller_.SetMode(QuickPID::Control::manual);
                }
            }
            // pwm_->setPWM(kPwmPin, kPwmFrequency, output_);

            static TickType_t xLastWakeTime = xTaskGetTickCount();
            // manual pwm
            uint32_t zero_crossing_per_period = 120 / (kControlLoopPeriod / 1000);
            for (uint32_t i = 0; i < zero_crossing_per_period;
                 i++)  // 120 zero-crossing points per second used to get resolution
            {
                if (output_ >= (100.0f * i / zero_crossing_per_period) && output_ != 0)
                {
                    digitalWrite(kPwmPin, HIGH);
                }
                else
                {
                    digitalWrite(kPwmPin, LOW);
                }
                xTaskDelayUntil(&xLastWakeTime, (1000 / 120) / portTICK_PERIOD_MS);  // ms per zero-crossing at 60hz
            }
            // SetTunings(1, 0, 0);
        }
    }

    double GetKp() { return /*config_->config_struct.kp; */ pid_controller_.GetKp(); }
    double GetKi() { return /*config_->config_struct.ki; */ pid_controller_.GetKi(); }
    double GetKd() { return /*config_->config_struct.kd; */ pid_controller_.GetKd(); }

    // autotunes at the setpoint
    void Autotune()
    {
        if (!tuning_ && !enabled_)
        {
            tuning_ = true;
            pid_tuner_.Configure(300, 100, 0, 100, 600, 0, 500);
            /* pid_tuner_.setTuningCycles(kTuningCycles);
            pid_tuner_.setTargetInputValue(config_->config_struct.temperature);
            pid_tuner_.setLoopInterval(kControlLoopPeriod * 1000);
            pid_tuner_.setOutputRange(0, 100);
            pid_tuner_.setZNMode(PIDAutotuner::ZNModeLessOvershoot);
            pid_tuner_.startTuningLoop(micros()); */
        }
    }

    bool GetTuning() { return tuning_; }

    void Enable() { enabled_ = true; }

    void Disable() { enabled_ = false; }

    double GetInput() { return input_; }

    bool GetEnabled() { return enabled_; }

    double GetDutyCycle() { return output_; }

    double GetSetpoint() { return config_->config_struct.temperature; }

    uint32_t GetTimeRemaining()
    {
        if (enabled_)
        {
            return timer_ < (millis() - enable_time_) ? 0 : timer_ - (millis() - enable_time_);
        }
        else
        {
            return timer_;
        }
    }

    void SetTimer(uint32_t time)
    {
        if (!tuning_)
        {
            timer_ = time;
            config_->config_struct.timer = time;
        }
    }

    uint32_t GetTimer() { return timer_; }

    void WriteConfig() { config_->WriteConfig(); }

private:
    bool enabled_ = false;

    const uint8_t kPwmPin;
    const float kPwmFrequency;
    // ESP32_FAST_PWM* pwm_;
    const uint32_t kControlLoopPeriod;
    std::function<double(void)> get_input_;
    Config* config_;
    QuickPID pid_controller_;
    sTune pid_tuner_;
    const uint8_t kTuningCycles = 2;
    bool tuning_ = false;

    uint32_t enable_time_ = 0;
    uint32_t timer_ = 0;

    float input_;
    float output_;

    void SetTunings(float kp, float ki, float kd)
    {
        pid_controller_.SetTunings(kp, ki, kd);
        config_->config_struct.kp = kp;
        config_->config_struct.kp = ki;
        config_->config_struct.kp = kd;
        // config_->WriteConfig(); //crashes
    }
};
