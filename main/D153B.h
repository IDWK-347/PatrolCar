#pragma once
#include <driver/gpio.h>
#include <driver/ledc.h>
#include <driver/pulse_cnt.h>
#include <esp_adc/adc_oneshot.h>
#include <esp_adc_cal.h>

class MotorDriver {
  public:
    enum MotorChannel { MOTOR_A = 0, MOTOR_B = 1 };
    enum Direction { FORWARD = 0, BACKWARD = 1, STOP = 2 };

    struct PinConfig {
        gpio_num_t AIN1;
        gpio_num_t AIN2;
        gpio_num_t PWMA;
        gpio_num_t BIN1;
        gpio_num_t BIN2;
        gpio_num_t PWMB;
        gpio_num_t STBY;
        gpio_num_t E1A;            // 编码器A相 电机1
        gpio_num_t E1B;            // 编码器B相 电机1
        gpio_num_t E2A;            // 编码器A相 电机2
        gpio_num_t E2B;            // 编码器B相 电机2
        adc_channel_t voltage_adc; // 电源电压检测ADC通道（1/11分压）
    };

    MotorDriver(const PinConfig &config);
    ~MotorDriver();

    esp_err_t init();

    void set_speed(MotorChannel channel, float speed); // -100~100
    void set_raw(MotorChannel channel, Direction dir, uint32_t duty);
    void brake(MotorChannel channel);
    void standby(bool enable);

    int32_t get_encoder_count(MotorChannel channel);
    void clear_encoder(MotorChannel channel);
    uint32_t read_power_voltage();

  private:
    PinConfig cfg_;

    // PWM
    ledc_timer_t ledc_timer_  = LEDC_TIMER_0;
    ledc_channel_t ledc_ch_a_ = LEDC_CHANNEL_0;
    ledc_channel_t ledc_ch_b_ = LEDC_CHANNEL_1;

    // 脉冲计数器
    pcnt_unit_handle_t pcnt_unit_[2]    = {nullptr, nullptr};
    pcnt_channel_handle_t pcnt_chan_[2] = {nullptr, nullptr};

    // ADC 校准
    esp_adc_cal_characteristics_t adc_chars_;
    adc_oneshot_unit_handle_t adc_handle_ = nullptr;

    void init_gpio();
    void init_pwm();
    void init_encoder();
    void init_adc();
};
