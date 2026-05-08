#include <cstdint>
#include <freertos/FreeRTOS.h>
#include <iostream>

#include "HC_SR04.hpp"
#include "LF04.hpp"
#include "esp_timer.h"

// HC-SR04 超声波传感器引脚
#define ULTRASONIC_TRIG GPIO_NUM_27
#define ULTRASONIC_ECHO GPIO_NUM_35

// LF04 循迹传感器引脚
#define LINE_SENSOR_DH1 GPIO_NUM_32
#define LINE_SENSOR_DH2 GPIO_NUM_33
#define LINE_SENSOR_DH3 GPIO_NUM_4
#define LINE_SENSOR_DH4 GPIO_NUM_39

// TB6612 电机驱动引脚配置
// 左电机Motor B
#define MOTOR_LEFT_IN1 GPIO_NUM_21
#define MOTOR_LEFT_IN2 GPIO_NUM_22
#define MOTOR_LEFT_PWM GPIO_NUM_26
// 右电机Motor A
#define MOTOR_RIGHT_IN1 GPIO_NUM_18
#define MOTOR_RIGHT_IN2 GPIO_NUM_19
#define MOTOR_RIGHT_PWM GPIO_NUM_25

// LEDC PWM 通道配置
#define LEFT_MOTOR_CHANNEL LEDC_CHANNEL_0
#define RIGHT_MOTOR_CHANNEL LEDC_CHANNEL_1
#define PWM_TIMER LEDC_TIMER_0

UltrasonicSensor ultrasonic_sensor(ULTRASONIC_TRIG, ULTRASONIC_ECHO);

LineSensor<4> line_sensor({
    LINE_SENSOR_DH4,
    LINE_SENSOR_DH3,
    LINE_SENSOR_DH2,
    LINE_SENSOR_DH1,
});

extern "C" void app_main(void) {
    // Hello World
    std::cout << "Hello World!" << std::endl;

    ultrasonic_sensor.begin();
    line_sensor.begin();

    while (1) {
        std::cout << "[" << esp_timer_get_time() << "]" << std::endl;
        std::cout << "Distance: " << ultrasonic_sensor.get_distance_cm() << "cm" << std::endl;
        std::cout << "Line sensor: " << static_cast<int32_t>(line_sensor.read()) << " | "
                  << line_sensor.read(0) << line_sensor.read(1) << line_sensor.read(2) << line_sensor.read(3)
                  << std::endl;
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}
