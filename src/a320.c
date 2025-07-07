/* SPDX-License-Identifier: MIT */
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/sys/util.h>
#include <zephyr/logging/log.h>
#include <zmk/event_manager.h>
#include <zmk/hid.h>
#include <sys/atomic.h>
#include <zephyr/sys/mutex.h>  // 新增互斥锁支持

LOG_MODULE_REGISTER(A320, CONFIG_SENSOR_LOG_LEVEL);

/* 寄存器定义 */
#define Motion         0x02
#define Delta_X        0x03
#define Delta_Y        0x04
#define Config         0x11
#define Observation    0x2E

/* 状态位定义 */
#define BIT_MOTION_MOT (1 << 7)
#define BIT_MOTION_OVF (1 << 4)

/* 模式配置参数 */
#define SCROLL_SPEED_DIVIDER   6
#define BOOST_SPEED_MULTIPLIER 2
#define SLOW_SPEED_DIVIDER     2
#define FIXED_POINT_FACTOR     64  // 固定点数缩放因子
#define POLLING_INTERVAL_MS    10  // 轮询间隔(ms)

/* 设备配置结构 */
struct a320_config {
    struct i2c_dt_spec bus;
    struct gpio_dt_spec reset_gpio;
};

/* 设备数据结构 */
struct a320_data {
    struct k_mutex data_mutex;  // 数据互斥锁[9](@ref)
    uint8_t motion;
    int8_t delta_x;
    int8_t delta_y;
    atomic_t is_scroll_mode;
    atomic_t is_boost_mode;
    atomic_t is_slow_mode;
    K_THREAD_STACK_MEMBER(thread_stack, 512);
    struct k_thread thread;
    k_tid_t thread_id;
};

/* 增强型I2C读取（带错误处理）*/
static int a320_read_reg(const struct device *dev, uint8_t reg_addr, uint8_t *value) {
    const struct a320_config *cfg = dev->config;
    int ret = i2c_reg_read_byte_dt(&cfg->bus, reg_addr, value);
    
    if (ret < 0) {
        LOG_ERR("读取寄存器0x%02x失败: %d", reg_addr, ret);
        return ret;
    }
    return 0;
}

/* 内部数据采集函数（无锁版）*/
static int a320_sample_fetch_internal(const struct device *dev) {
    struct a320_data *data = dev->data;
    uint8_t motion;
    int ret;
    
    // 读取运动状态
    ret = a320_read_reg(dev, Motion, &motion);
    if (ret < 0) return ret;
    
    data->motion = motion;
    
    // 仅当检测到运动时读取位移
    if (motion & BIT_MOTION_MOT) {
        ret = a320_read_reg(dev, Delta_X, (uint8_t*)&data->delta_x);
        if (ret < 0) return ret;
        
        ret = a320_read_reg(dev, Delta_Y, (uint8_t*)&data->delta_y);
        if (ret < 0) return ret;
    } else {
        // 无运动时清零数据
        data->delta_x = 0;
        data->delta_y = 0;
    }
    
    return 0;
}

/* 优化的数据采集（线程安全）*/
static int a320_sample_fetch(const struct device *dev, enum sensor_channel chan) {
    struct a320_data *data = dev->data;
    int ret;
    
    k_mutex_lock(&data->data_mutex, K_FOREVER);
    ret = a320_sample_fetch_internal(dev);
    k_mutex_unlock(&data->data_mutex);
    
    return ret;
}

/* 实现传感器通道数据获取 */
static int a320_channel_get(const struct device *dev, enum sensor_channel chan,
                            struct sensor_value *val) {
    struct a320_data *data = dev->data;
    int ret = 0;
    
    k_mutex_lock(&data->data_mutex, K_FOREVER);
    
    switch (chan) {
        case SENSOR_CHAN_POS_DX:
            val->val1 = data->delta_x;
            val->val2 = 0;
            break;
        case SENSOR_CHAN_POS_DY:
            val->val1 = data->delta_y;
            val->val2 = 0;
            break;
        case SENSOR_CHAN_MOTION:
            val->val1 = (data->motion & BIT_MOTION_MOT) ? 1 : 0;
            val->val2 = 0;
            break;
        default:
            LOG_WRN("不支持的通道: %d", chan);
            ret = -ENOTSUP;
    }
    
    k_mutex_unlock(&data->data_mutex);
    return ret;
}

/* 定点数缩放运算（优化精度与溢出保护）*/
static void apply_scaling(struct a320_data *data, int16_t *x, int16_t *y) {
    int32_t scaled_x = (int32_t)data->delta_x * FIXED_POINT_FACTOR;
    int32_t scaled_y = (int32_t)data->delta_y * FIXED_POINT_FACTOR;

    // 基础缩放（1.5倍）
    scaled_x = (scaled_x * 3) / 2;
    scaled_y = (scaled_y * 3) / 2;

    // 应用模式调节（加速/减速）
    if (atomic_get(&data->is_boost_mode)) {
        scaled_x *= BOOST_SPEED_MULTIPLIER;
        scaled_y *= BOOST_SPEED_MULTIPLIER;
    } else if (atomic_get(&data->is_slow_mode)) {
        scaled_x /= SLOW_SPEED_DIVIDER;
        scaled_y /= SLOW_SPEED_DIVIDER;
    }

    // 缩放回原始范围，并限制在int8_t范围内
    scaled_x /= FIXED_POINT_FACTOR;
    scaled_y /= FIXED_POINT_FACTOR;
    
    *x = CLAMP(scaled_x, INT8_MIN, INT8_MAX);
    *y = CLAMP(scaled_y, INT8_MIN, INT8_MAX);
}

/* HID报告生成与发送线程 */
static void a320_hid_thread(void *arg) {
    const struct device *dev = (const struct device *)arg;
    struct a320_data *data = dev->data;
    struct zmk_hid_report_mouse report = {0};
    
    while (1) {
        k_mutex_lock(&data->data_mutex, K_FOREVER);
        if (a320_sample_fetch_internal(dev) == 0) {
            if (data->motion & BIT_MOTION_MOT) {
                if (atomic_get(&data->is_scroll_mode)) {
                    // 滚动模式：转换为水平/垂直滚动值
                    report.h = CLAMP(data->delta_x / SCROLL_SPEED_DIVIDER, 
                                     INT8_MIN, INT8_MAX);
                    report.v = CLAMP(-data->delta_y / SCROLL_SPEED_DIVIDER,
                                     INT8_MIN, INT8_MAX);
                } else {
                    // 指针模式：应用缩放并设置X/Y位移
                    int16_t x, y;
                    apply_scaling(data, &x, &y);
                    report.x = (int8_t)x;
                    report.y = (int8_t)y;
                }
                
                // 发送HID报告
                zmk_hid_mouse_report_send(&report);
            }
        }
        k_mutex_unlock(&data->data_mutex);
        
        k_sleep(K_MSEC(POLLING_INTERVAL_MS));
    }
}

/* 线程安全的模式切换 */
void a320_toggle_scroll_mode(const struct device *dev) {
    struct a320_data *data = dev->data;
    atomic_set(&data->is_scroll_mode, !atomic_get(&data->is_scroll_mode));
    LOG_INF("滚轮模式: %s", atomic_get(&data->is_scroll_mode) ? "开启" : "关闭");
}

void a320_toggle_boost_mode(const struct device *dev) {
    struct a320_data *data = dev->data;
    atomic_set(&data->is_boost_mode, !atomic_get(&data->is_boost_mode));
    LOG_INF("加速模式: %s", atomic_get(&data->is_boost_mode) ? "开启" : "关闭");
}

void a320_toggle_slow_mode(const struct device *dev) {
    struct a320_data *data = dev->data;
    atomic_set(&data->is_slow_mode, !atomic_get(&data->is_slow_mode));
    LOG_INF("慢速模式: %s", atomic_get(&data->is_slow_mode) ? "开启" : "关闭");
}

/* 设备初始化 */
static int a320_init(const struct device *dev) {
    const struct a320_config *cfg = dev->config;
    struct a320_data *data = dev->data;
    
    // 初始化互斥锁[9](@ref)
    k_mutex_init(&data->data_mutex);
    
    // 检查I2C总线是否就绪
    if (!device_is_ready(cfg->bus.bus)) {
        LOG_ERR("I2C总线 %s 未就绪", cfg->bus.bus->name);
        return -EINVAL;
    }
    
    // 执行硬件复位（仅当设备树配置了复位GPIO）
    if (cfg->reset_gpio.port != NULL && device_is_ready(cfg->reset_gpio.port)) {
        LOG_DBG("执行硬件复位");
        gpio_pin_configure_dt(&cfg->reset_gpio, GPIO_OUTPUT_INACTIVE);
        k_sleep(K_MSEC(10));
        gpio_pin_set_dt(&cfg->reset_gpio, 1);
        k_sleep(K_MSEC(100));
    } else if (cfg->reset_gpio.port != NULL) {
        LOG_WRN("复位GPIO未就绪，跳过复位");
    }
    
    // 初始化模式状态
    atomic_set(&data->is_scroll_mode, 0);
    atomic_set(&data->is_boost_mode, 0);
    atomic_set(&data->is_slow_mode, 0);
    
    // 创建HID处理线程
    data->thread_id = k_thread_create(
        &data->thread,
        data->thread_stack,
        K_THREAD_STACK_SIZEOF(data->thread_stack),
        a320_hid_thread,
        (void *)dev, NULL, NULL,
        K_PRIO_PREEMPT(5),
        0,
        K_NO_WAIT
    );
    k_thread_name_set(data->thread_id, "a320_hid_thread");
    
    LOG_INF("A320初始化成功");
    return 0;
}

/* 传感器驱动API */
static const struct sensor_driver_api a320_driver_api = {
    .sample_fetch = a320_sample_fetch,
    .channel_get = a320_channel_get,
};

/* 设备定义宏 */
#define A320_DEFINE(inst)                                                                          \
    static struct a320_data a320_data_##inst;                                                      \
    static const struct a320_config a320_cfg_##inst = {                                           \
        .bus = I2C_DT_SPEC_INST_GET(inst),                                                         \
        .reset_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, reset_gpios, {0})                          \
    };                                                                                              \
    DEVICE_DT_INST_DEFINE(inst, a320_init, NULL, &a320_data_##inst, &a320_cfg_##inst,          \
                          POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY, &a320_driver_api);

/* 为设备树中所有状态为"okay"的A320设备生成驱动实例 */
DT_INST_FOREACH_STATUS_OKAY(A320_DEFINE)
