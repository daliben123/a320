/* 文件名：drivers/touchpad/touchpad_a320.h */
#pragma once

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/input/input.h>

#define DT_DRV_COMPAT pixart_a320

/* 设备配置结构 */
struct touchpad_config {
    struct i2c_dt_spec bus;          // I2C总线配置[8](@ref)
    struct gpio_dt_spec motion_gpio; // 运动中断引脚[8](@ref)
    struct gpio_dt_spec reset_gpio;  // 可选复位引脚[8](@ref)
};

/* 设备数据结构 */
struct touchpad_data {
    struct gpio_callback motion_cb;
    struct k_work work;
    const struct device *input_dev;  // 输入设备指针[4](@ref)
};
