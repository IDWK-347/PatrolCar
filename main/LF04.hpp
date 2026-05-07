#pragma once
#include <array>
#include <cassert>
#include <cstdint>
#include <driver/gpio.h>

template <uint8_t N = 1> class LineSensor {
  public:
    using Pins = std::array<gpio_num_t, N>;

    enum State : bool { Black = false, White = true };

    explicit LineSensor(const Pins &pins) : m_pins(pins) {
        static_assert(N > 0 && N <= 8, "N must be between 1 and 8");
    }
    ~LineSensor() { end(); }

    esp_err_t begin() {
        uint64_t pin_bit_mask = 0;
        for (int8_t i = 0; i < m_pins.size(); ++i) {
            pin_bit_mask |= (1ULL << m_pins[i]);
        }

        gpio_config_t io_conf = {
            .pin_bit_mask = pin_bit_mask,
            .mode         = GPIO_MODE_INPUT,
            .pull_up_en   = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type    = GPIO_INTR_DISABLE
        };
        return gpio_config(&io_conf);
    }

    void end() {
        for (auto &pin : m_pins) {
            ESP_ERROR_CHECK(gpio_reset_pin(pin));
        }
    }

    uint8_t read() const {
        uint8_t result = 0;
        for (uint8_t i = 0; i < m_pins.size(); ++i) {
            result |= (static_cast<uint8_t>(read(i)) << i);
        }
        return result;
    }
    State read(uint8_t bit) const { return static_cast<State>(gpio_get_level(m_pins[bit])); }

  private:
    const Pins m_pins;
};

using LineSensorState = LineSensor<>::State;
