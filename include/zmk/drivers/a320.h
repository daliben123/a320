#pragma once
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/input/input.h>
#include <sys/atomic.h>

/* 设备树兼容性标识 */
#define DT_DRV_COMPAT avago_a320

/* 传感器通道扩展 (基于Zephyr标准扩展) */
enum a320_channel {
    /* 基础位移通道 */
    A320_CHAN_DELTA_X = SENSOR_CHAN_PRIV_START,  // 0x100起始
    A320_CHAN_DELTA_Y,
    A320_CHAN_MOTION_STATE,
    
    /* 增强诊断通道 */
    A320_CHAN_OVF_STATUS,        // 数据溢出状态
    A320_CHAN_TEMPERATURE,       // 传感器温度
    A320_CHAN_POWER_MODE         // 当前功耗模式
};

/* 设备配置结构 (设备树动态适配) */
struct a320_config {
    struct i2c_dt_spec bus;      // I2C总线配置
    
    /* 可选GPIO (通过设备树条件编译) */
    #if DT_INST_NODE_HAS_PROP(0, reset_gpios)
        struct gpio_dt_spec reset_gpio;    // 硬件复位引脚
    #endif
    #if DT_INST_NODE_HAS_PROP(0, motion_gpios)
        struct gpio_dt_spec motion_gpio;   // 运动中断引脚
    #endif
    #if DT_INST_NODE_HAS_PROP(0, shutdown_gpios)
        struct gpio_dt_spec shutdown_gpio; // 低功耗控制引脚
    #endif
    
    uint16_t polling_interval;    // 轮询间隔(ms)，设备树可配置
};

/* 设备运行时数据 */
struct a320_data {
    struct k_mutex data_mutex;    // 数据互斥锁
    atomic_t is_scroll_mode;      // 滚动模式原子标志
    atomic_t is_boost_mode;       // 加速模式原子标志
    atomic_t is_slow_mode;        // 减速模式原子标志
    
    /* 运动检测 */
    struct gpio_callback motion_cb;   // 中断回调
    struct k_sem motion_sem;          // 事件信号量
    
    /* 线程控制 */
    struct k_thread thread;           // 数据处理线程
    K_THREAD_STACK_MEMBER(thread_stack, CONFIG_A320_THREAD_STACK_SIZE);
    
    /* 传感器状态 */
    uint8_t last_motion;     // 最近运动状态
    int8_t delta_x;          // X轴位移
    int8_t delta_y;          // Y轴位移
    uint32_t error_count;    // I2C错误计数器
};

/* ================= 寄存器定义 (完整版) ================= */
#define A320_REG_PRODUCT_ID       0x00
#define A320_REG_REVISION_ID      0x01
#define A320_REG_MOTION           0x02    // 运动状态
#define A320_REG_DELTA_X          0x03    // X轴位移
#define A320_REG_DELTA_Y          0x04    // Y轴位移
#define A320_REG_SQUAL            0x05    // 表面质量
#define A320_REG_CONFIG           0x11    // 工作模式配置
#define A320_REG_OBSERVATION      0x2E    // 诊断观测值
#define A320_REG_POWER_UP_RESET   0x3A    // 软复位控制
#define A320_REG_SHUTDOWN         0x42    // 低功耗控制

/* 状态位掩码 */
#define A320_BIT_MOTION_MOT   (1 << 7)  // 运动检测标志
#define A320_BIT_MOTION_OVF   (1 << 4)  // 位移溢出标志
#define A320_BIT_CONFIG_SLEEP (1 << 2)  // 休眠模式标志

/* ================= 模式参数 ================= */
#define A320_SCROLL_SPEED_DIVIDER    6   // 滚轮模式速度除数
#define A320_BOOST_SPEED_MULTIPLIER  2   // 加速模式倍数
#define A320_SLOW_SPEED_DIVIDER      2   // 慢速模式除数
#define A320_DEFAULT_POLLING_MS     10   // 默认采样间隔

/* ================== 驱动API声明 ================== */
int a320_init(const struct device *dev);
int a320_sample_fetch(const struct device *dev, enum sensor_channel chan);
int a320_channel_get(const struct device *dev, enum sensor_channel chan, 
                     struct sensor_value *val);

/* 模式控制API (线程安全) */
void a320_set_scroll_mode(const struct device *dev, bool enable);
void a320_set_boost_mode(const struct device *dev, bool enable);
void a320_set_slow_mode(const struct device *dev, bool enable);

/* 诊断API */
int a320_get_error_count(const struct device *dev);
