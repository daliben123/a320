#pragma once
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/mutex.h>

#define DT_DRV_COMPAT avago_a320

/* 精简传感器通道（移除虚假温度） */
enum a320_channel {
    A320_CHAN_DELTA_X = SENSOR_CHAN_PRIV_START, // 0x100
    A320_CHAN_DELTA_Y,
    A320_CHAN_MOTION_STATE,
    A320_CHAN_OVF_STATUS,     // 新增溢出状态通道
};

/* 完整设备配置结构 */
struct a320_config {
    struct i2c_dt_spec bus;
    struct gpio_dt_spec reset_gpio;
    struct gpio_dt_spec motion_gpio;
    struct gpio_dt_spec orientation_gpio;
    struct gpio_dt_spec shutdown_gpio;
    uint16_t polling_interval_ms;
};

/* 设备运行时数据 */
struct a320_data {
    struct k_mutex data_mutex;
    atomic_t is_scroll_mode;    // 滚轮模式
    atomic_t is_boost_mode;     // 加速模式
    atomic_t is_inverted_mode;  // 方向反转模式
    struct gpio_callback motion_cb; 
    struct k_sem motion_sem;    // 中断信号量
    struct k_thread thread;
    K_THREAD_STACK_MEMBER(thread_stack, CONFIG_A320_THREAD_STACK_SIZE);
    int8_t delta_x, delta_y;
    uint8_t motion_status;
};

/* 中断服务函数声明 */
void a320_motion_isr(const struct device *dev, struct gpio_callback *cb, uint32_t pins);
