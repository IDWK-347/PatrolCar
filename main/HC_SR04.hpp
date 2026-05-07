#pragma once
#include <algorithm>
#include <cstdint>
#include <driver/gpio.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h> // IWYU pragma: keep
#include <freertos/task.h>

class UltrasonicSensor {
  public:
    explicit UltrasonicSensor(gpio_num_t trig, gpio_num_t echo) : TRIG_GPIO(trig), ECHO_GPIO(echo) {};
    ~UltrasonicSensor() { end(); }

    void begin() {
        // 1. 配置 Trigger 引脚为输出
        gpio_config_t trig_conf = {
            .pin_bit_mask = (1ULL << TRIG_GPIO),
            .mode         = GPIO_MODE_OUTPUT,
            .pull_up_en   = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type    = GPIO_INTR_DISABLE,
        };
        gpio_config(&trig_conf);

        // 2. 配置 Echo 引脚为输入，并捕获任意边沿
        gpio_config_t echo_conf = {
            .pin_bit_mask = (1ULL << ECHO_GPIO),
            .mode         = GPIO_MODE_INPUT,
            .pull_up_en   = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type    = GPIO_INTR_ANYEDGE,
        };
        gpio_config(&echo_conf);

        // 3. 安装 GPIO 中断并挂载 ISR
        gpio_install_isr_service(0);
        gpio_isr_handler_add(ECHO_GPIO, echo_isr_handler, this);
    }

    void end() {
        gpio_isr_handler_remove(ECHO_GPIO);
        gpio_uninstall_isr_service();
        gpio_reset_pin(TRIG_GPIO);
        gpio_reset_pin(ECHO_GPIO);
    }

    // 发送一个 10us 的触发脉冲
    void send_trigger_pulse(void) {
        gpio_set_level(TRIG_GPIO, 1);
        esp_rom_delay_us(10);
        gpio_set_level(TRIG_GPIO, 0);
    }

    float get_distance_cm() const { return std::min(400.0f, pulse_width_us * 0.0343f / 2.0f); }

  private:
    // Echo 引脚的中断服务函数 (双边沿触发)
    static void echo_isr_handler(void *arg) {
        UltrasonicSensor *self = reinterpret_cast<UltrasonicSensor *>(arg);

        uint64_t now = esp_timer_get_time();

        // 直接读取 GPIO 电平
        if (gpio_get_level(self->ECHO_GPIO)) {
            self->start_time = now; // 上升沿
        } else {
            if (now > self->start_time) {
                uint32_t width = static_cast<uint32_t>(now - self->start_time);
                // 使用临界区保护共享数据
                self->pulse_width_us = width;
            }
        }
    }

    gpio_num_t TRIG_GPIO; // 触发引脚
    gpio_num_t ECHO_GPIO; // 回响引脚

    volatile uint64_t start_time     = 0;
    volatile uint32_t pulse_width_us = 0;
};
