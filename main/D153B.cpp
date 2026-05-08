#include "D153B.h"

#include <cmath>
#include <esp_log.h>

static const char *TAG = "MotorDriver";

#define LEDC_MODE LEDC_LOW_SPEED_MODE
#define LEDC_FREQ_HZ 10000
#define LEDC_RESOLUTION LEDC_TIMER_10_BIT
#define LEDC_DUTY_MAX 1023

#define PCNT_LOW_LIM -20000
#define PCNT_HIGH_LIM 20000

// ----------------------------------------------------------------------
// 构造函数 / 析构函数
// ----------------------------------------------------------------------
MotorDriver::MotorDriver(const PinConfig &config) : cfg_(config) {}

MotorDriver::~MotorDriver() {
    // 停止 PWM
    ledc_stop(LEDC_MODE, ledc_ch_a_, 0);
    ledc_stop(LEDC_MODE, ledc_ch_b_, 0);

    // 删除脉冲计数器
    for (int i = 0; i < 2; ++i) {
        if (pcnt_chan_[i]) {
            pcnt_unit_stop(pcnt_unit_[i]);
            pcnt_unit_disable(pcnt_unit_[i]);
            pcnt_del_channel(pcnt_chan_[i]);
            pcnt_del_unit(pcnt_unit_[i]);
        }
    }

    // 删除 ADC
    if (adc_handle_) {
        adc_oneshot_del_unit(adc_handle_);
    }
}

// ----------------------------------------------------------------------
// 初始化顶层
// ----------------------------------------------------------------------
esp_err_t MotorDriver::init() {
    init_gpio();
    init_pwm();
    init_encoder();
    init_adc();
    standby(false);
    ESP_LOGI(TAG, "TB6612 D153B initialized");
    return ESP_OK;
}

// ----------------------------------------------------------------------
// GPIO 初始化（与旧版相同）
// ----------------------------------------------------------------------
void MotorDriver::init_gpio() {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << cfg_.AIN1) | (1ULL << cfg_.AIN2) | (1ULL << cfg_.BIN1) |
                        (1ULL << cfg_.BIN2) | (1ULL << cfg_.STBY),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    gpio_set_level(cfg_.AIN1, 0);
    gpio_set_level(cfg_.AIN2, 0);
    gpio_set_level(cfg_.BIN1, 0);
    gpio_set_level(cfg_.BIN2, 0);
    gpio_set_level(cfg_.STBY, 0);
}

// ----------------------------------------------------------------------
// PWM 初始化（填充缺失字段以消除警告）
// ----------------------------------------------------------------------
void MotorDriver::init_pwm() {
    // 定时器
    ledc_timer_config_t timer_conf = {
        .speed_mode      = LEDC_MODE,
        .duty_resolution = LEDC_RESOLUTION,
        .timer_num       = ledc_timer_,
        .freq_hz         = LEDC_FREQ_HZ,
        .clk_cfg         = LEDC_AUTO_CLK,
        .deconfigure     = false,
    };
    ledc_timer_config(&timer_conf);

    // 通道 A
    ledc_channel_config_t ch_a = {
        .gpio_num   = cfg_.PWMA,
        .speed_mode = LEDC_MODE,
        .channel    = ledc_ch_a_,
        .intr_type  = LEDC_INTR_DISABLE,
        .timer_sel  = ledc_timer_,
        .duty       = 0,
        .hpoint     = 0,
        .sleep_mode = LEDC_SLEEP_MODE_NO_ALIVE_NO_PD,
        .flags      = {.output_invert = 0},
    };
    ledc_channel_config(&ch_a);

    // 通道 B
    ledc_channel_config_t ch_b = {
        .gpio_num   = cfg_.PWMB,
        .speed_mode = LEDC_MODE,
        .channel    = ledc_ch_b_,
        .intr_type  = LEDC_INTR_DISABLE,
        .timer_sel  = ledc_timer_,
        .duty       = 0,
        .hpoint     = 0,
        .sleep_mode = LEDC_SLEEP_MODE_NO_ALIVE_NO_PD,
        .flags      = {.output_invert = 0},
    };
    ledc_channel_config(&ch_b);
}

// ----------------------------------------------------------------------
// 编码器初始化（完全采用 v5.x 新 API）
// ----------------------------------------------------------------------
void MotorDriver::init_encoder() {
    struct {
        gpio_num_t a;
        gpio_num_t b;
    } enc[2] = {{cfg_.E1A, cfg_.E1B}, {cfg_.E2A, cfg_.E2B}};

    for (int i = 0; i < 2; ++i) {
        // 1. 创建单元
        pcnt_unit_config_t unit_cfg = {
            .low_limit     = PCNT_LOW_LIM,
            .high_limit    = PCNT_HIGH_LIM,
            .intr_priority = 0,
            .flags         = {.accum_count = 0},
        };
        ESP_ERROR_CHECK(pcnt_new_unit(&unit_cfg, &pcnt_unit_[i]));

        // 2. 创建通道（A相为 edge 信号，B相为 level 信号）
        pcnt_chan_config_t chan_cfg = {
            .edge_gpio_num  = enc[i].a,
            .level_gpio_num = enc[i].b,
            .flags =
                {
                    .invert_edge_input   = false,
                    .invert_level_input  = false,
                    .virt_edge_io_level  = true,
                    .virt_level_io_level = true,
                    .io_loop_back        = false,
                },
        };
        ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit_[i], &chan_cfg, &pcnt_chan_[i]));

        // 3. 设置边沿动作（A相发生边沿时计数）
        ESP_ERROR_CHECK(pcnt_channel_set_edge_action(
            pcnt_chan_[i], PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE
        ));

        // 4. 设置电平动作（B相高电平时方向为正，低电平时方向为反）
        ESP_ERROR_CHECK(pcnt_channel_set_level_action(
            pcnt_chan_[i], PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE
        ));

        // 5. 毛刺滤波器（100 ns）
        pcnt_glitch_filter_config_t filter_cfg = {.max_glitch_ns = 100};
        ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(pcnt_unit_[i], &filter_cfg));

        // 6. 启用并开始计数
        ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit_[i]));
        ESP_ERROR_CHECK(pcnt_unit_start(pcnt_unit_[i]));
        ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit_[i]));
    }
    ESP_LOGI(TAG, "Encoders initialized");
}

// ----------------------------------------------------------------------
// ADC 初始化（使用 oneshot 新 API）
// ----------------------------------------------------------------------
void MotorDriver::init_adc() {
    adc_oneshot_unit_init_cfg_t init_cfg = {
        .unit_id  = ADC_UNIT_1,
        .clk_src  = ADC_RTC_CLK_SRC_DEFAULT,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_cfg, &adc_handle_));

    adc_oneshot_chan_cfg_t chan_cfg = {.atten = ADC_ATTEN_DB_12, .bitwidth = ADC_BITWIDTH_12};
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle_, cfg_.voltage_adc, &chan_cfg));

    // 校准：注意第三个参数是 ADC_WIDTH_BIT_12
    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_12, ADC_WIDTH_BIT_12, 1100, &adc_chars_);
}

// ----------------------------------------------------------------------
// 电机控制函数（与旧版一致）
// ----------------------------------------------------------------------
void MotorDriver::set_speed(MotorChannel channel, float speed) {
    speed = std::max(-100.0f, std::min(100.0f, speed));
    Direction dir;
    if (speed > 0)
        dir = FORWARD;
    else if (speed < 0)
        dir = BACKWARD;
    else
        dir = STOP;

    uint32_t duty = static_cast<uint32_t>(LEDC_DUTY_MAX * (std::abs(speed) / 100.0f));
    set_raw(channel, dir, duty);
}

void MotorDriver::set_raw(MotorChannel channel, Direction dir, uint32_t duty) {
    if (duty > LEDC_DUTY_MAX)
        duty = LEDC_DUTY_MAX;

    gpio_num_t in1, in2;
    ledc_channel_t ledc_ch;

    if (channel == MOTOR_A) {
        in1     = cfg_.AIN1;
        in2     = cfg_.AIN2;
        ledc_ch = ledc_ch_a_;
    } else {
        in1     = cfg_.BIN1;
        in2     = cfg_.BIN2;
        ledc_ch = ledc_ch_b_;
    }

    switch (dir) {
    case FORWARD:
        gpio_set_level(in1, 1);
        gpio_set_level(in2, 0);
        break;
    case BACKWARD:
        gpio_set_level(in1, 0);
        gpio_set_level(in2, 1);
        break;
    default:
        gpio_set_level(in1, 1);
        gpio_set_level(in2, 1); // 刹车
        duty = LEDC_DUTY_MAX;
        break;
    }

    ledc_set_duty(LEDC_MODE, ledc_ch, duty);
    ledc_update_duty(LEDC_MODE, ledc_ch);
}

void MotorDriver::brake(MotorChannel channel) { set_raw(channel, STOP, LEDC_DUTY_MAX); }

void MotorDriver::standby(bool enable) { gpio_set_level(cfg_.STBY, enable ? 1 : 0); }

// ----------------------------------------------------------------------
// 编码器 / 电压读取
// ----------------------------------------------------------------------
int32_t MotorDriver::get_encoder_count(MotorChannel channel) {
    int count               = 0;
    pcnt_unit_handle_t unit = (channel == MOTOR_A) ? pcnt_unit_[0] : pcnt_unit_[1];
    if (unit)
        pcnt_unit_get_count(unit, &count);
    return count;
}

void MotorDriver::clear_encoder(MotorChannel channel) {
    pcnt_unit_handle_t unit = (channel == MOTOR_A) ? pcnt_unit_[0] : pcnt_unit_[1];
    if (unit)
        pcnt_unit_clear_count(unit);
}

uint32_t MotorDriver::read_power_voltage() {
    int adc_raw = 0;
    adc_oneshot_read(adc_handle_, cfg_.voltage_adc, &adc_raw);
    uint32_t voltage_mv = esp_adc_cal_raw_to_voltage(adc_raw, &adc_chars_);
    return voltage_mv * 11; // 1/11 分压
}
