#pragma once

#include <Arduino.h>

#include <functional>

#include "Config.h"
#include "ESP32_FastPWM.h"
#include "PID_v1.h"
#include "pidautotuner.h"

class PIDControl
{
public:
    PIDControl(uint8_t pwm_pin,
               float pwm_frequency,
               uint32_t control_loop_period,
               std::function<double(void)> get_input,
               Config& config)
        : kPwmPin{pwm_pin},
          kPwmFrequency{pwm_frequency},
          pwm_{new ESP32_FAST_PWM(kPwmPin, kPwmFrequency, 0)},
          kControlLoopPeriod{control_loop_period},
          get_input_{get_input},
          config_{config},
          pid_controller_{&input_,
                          &output_,
                          &setpoint_,
                          config_.config_struct.kp,
                          config_.config_struct.ki,
                          config_.config_struct.kd,
                          DIRECT},
          pid_tuner_{}
    {
        pid_controller_.SetOutputLimits(0, 100);
        pid_controller_.SetSampleTime(kControlLoopPeriod * 0.9);  // less to make sure it actually runs every time
    }

    void SetSetpoint(double setpoint)
    {
        if (!tuning_)
        {
            setpoint_ = setpoint;
            config.config_struct.temperature = setpoint;
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
                output_ = pid_tuner_.tunePID(input_, micros());
                if (pid_tuner_.isFinished())
                {
                    tuning_ = false;
                    SetTunings(pid_tuner_.getKp(), pid_tuner_.getKi(), pid_tuner_.getKd());
                    output_ = 0;
                }
            }
            else
            {
                if (enabled_)
                {
                    pid_controller_.Compute();
                }
                else
                {
                    output_ = 0;
                }
            }
            pwm_->setPWM(kPwmPin, kPwmFrequency, output_);

            static TickType_t xLastWakeTime = xTaskGetTickCount();
            xTaskDelayUntil(&xLastWakeTime, kControlLoopPeriod * portTICK_PERIOD_MS);
        }
    }

    double GetKp() { return pid_controller_.GetKp(); }
    double GetKi() { return pid_controller_.GetKi(); }
    double GetKd() { return pid_controller_.GetKd(); }

    // autotunes at the setpoint
    void Autotune()
    {
        if (!tuning_)
        {
            tuning_ = true;
            pid_tuner_.setTargetInputValue(setpoint_);
            pid_tuner_.setLoopInterval(kControlLoopPeriod * 1000);
            pid_tuner_.setOutputRange(0, 100);
            pid_tuner_.setZNMode(PIDAutotuner::ZNModeBasicPID);
            pid_tuner_.startTuningLoop(micros());
        }
    }

    bool GetTuning() { return tuning_; }

    void Enable() { enabled_ = true; }

    void Disable() { enabled_ = false; }

    double GetInput() { return get_input_(); }

    bool GetEnabled() { return enabled_; }

    bool GetDutyCycle() { return output_; }

    double GetSetpoint() { return setpoint_; }

    uint32_t GetTimeRemaining()
    {
        if (enabled_)
        {
            return timer_ - (millis() - enable_time_);
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
            config.config_struct.timer = time;
        }
    }

    uint32_t GetTimer() { return timer_; }

    void WriteConfig() { config_.WriteConfig(); }

private:
    bool enabled_ = false;

    const uint8_t kPwmPin;
    const float kPwmFrequency;
    ESP32_FAST_PWM* pwm_;
    const uint32_t kControlLoopPeriod;
    std::function<double(void)> get_input_;
    Config& config_;
    PID pid_controller_;
    PIDAutotuner pid_tuner_;
    bool tuning_ = false;

    uint32_t enable_time_ = 0;
    uint32_t timer_ = 0;

    double input_;
    double output_;
    double setpoint_;

    void SetTunings(double kp, double ki, double kd)
    {
        pid_controller_.SetTunings(kp, ki, kd);
        config_.config_struct.kp = kp;
        config_.config_struct.kp = ki;
        config_.config_struct.kp = kd;
        config_.WriteConfig();
    }
};