#define DT_DRV_COMPAT avago_a320

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/crc.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/input.h>

#include "a320.h"

LOG_MODULE_REGISTER(A320, CONFIG_SENSOR_LOG_LEVEL);

/* 寄存器定义 */
#define MOTION            0x02
#define DELTA_X           0x03
#define DELTA_Y           0x04
#define DELTA_XY_H        0x05
#define CONFIGURATION     0x11
#define OBSERVATION       0x2E
#define MBURST            0x42

/* 位定义 */
#define BIT_MOTION_MOT    (1 << 7)
#define BIT_MOTION_OVF    (1 << 4)
#define BIT_CONFIG_HIRES  (1 << 7)

/* 手势配置参数 */
#define SWIPE_COOLDOWN_TIME_MS 100  /* 两次滑动之间的冷却时间 */
#define SWIPE_RELEASE_DELAY_MS 10   /* 按键释放延迟 */
#define SWIPE_THRESHOLD        15   /* 触发滑动的最小移动量 */
#define SWIPE_PERPENDICULAR_MAX 5   /* 垂直方向允许的最大偏移 */

/* 输入键码定义 */
#define KEY_UP     103
#define KEY_DOWN   108
#define KEY_LEFT   105
#define KEY_RIGHT  106
#define KEY_LEFTALT 56

/* 触摸回调函数结构 */
struct touch_callback {
    void (*func)(int16_t x, int16_t y);
    struct touch_callback *next;
};

/* 设备私有数据结构 */
struct a320_data {
    struct k_work work;              /* 工作队列项 */
    struct k_timer swipe_timer;      /* 滑动冷却计时器 */
    uint32_t last_swipe_time;        /* 上次滑动时间 */
    bool alt_pressed;                /* Alt键状态 */
    struct touch_callback *callbacks;/* 触摸回调链表 */
};

/* 设备配置结构 */
struct a320_config {
    struct i2c_dt_spec bus;          /* I2C总线配置 */
    struct gpio_dt_spec motion_gpio; /* 运动检测GPIO */
    const char *input_dev_name;      /* 输入设备名称 */
};

/* 从A320传感器读取指定寄存器的值 */
static int a320_read_reg(const struct device *dev, uint8_t reg_addr) {
    const struct a320_config *cfg = dev->config;
    uint8_t value = 0;

    if (i2c_reg_read_byte_dt(&cfg->bus, reg_addr, &value) == 0) {
        return value;
    }

    LOG_ERR("Failed to read register 0x%x", reg_addr);
    return -EIO;
}

/* 向A320传感器写入指定寄存器的值 */
static int a320_write_reg(const struct device *dev, uint8_t reg_addr, uint8_t value) {
    const struct a320_config *cfg = dev->config;

    if (i2c_reg_write_byte_dt(&cfg->bus, reg_addr, value) == 0) {
        return 0;
    }

    LOG_ERR("Failed to write register 0x%x", reg_addr);
    return -EIO;
}

/* 检查是否为滑动手势 */
static bool is_swipe(int16_t primary, int16_t perpendicular) {
    return (abs(primary) >= SWIPE_THRESHOLD) && 
           (abs(perpendicular) <= SWIPE_PERPENDICULAR_MAX);
}

/* 释放按键工作处理函数 */
static void key_release_work_handler(struct k_work *work) {
    struct a320_data *data = CONTAINER_OF(work, struct a320_data, work);
    const struct device *input_dev = device_get_binding(data->input_dev_name);
    
    if (!input_dev) {
        LOG_ERR("Input device not found");
        return;
    }

    /* 这里应添加释放按键的代码 */
    input_report_key(input_dev, KEY_UP, 0, true);
    input_report_key(input_dev, KEY_DOWN, 0, true);
    input_report_key(input_dev, KEY_LEFT, 0, true);
    input_report_key(input_dev, KEY_RIGHT, 0, true);
}

/* 处理触摸数据工作函数 */
static void process_touch_data_work(struct k_work *work) {
    struct a320_data *data = CONTAINER_OF(work, struct a320_data, work);
    const struct device *dev = device_get_binding(DT_INST_LABEL(0));
    const struct a320_config *cfg = dev->config;
    const struct device *input_dev = device_get_binding(cfg->input_dev_name);
    
    int ifmotion, ovflow, dx_raw, dy_raw;
    int16_t dx, dy;
    uint32_t now = k_uptime_get();
    
    if (!input_dev) {
        LOG_ERR("Input device not found");
        return;
    }

    /* 读取运动状态 */
    ifmotion = a320_read_reg(dev, MOTION);
    if (ifmotion < 0) {
        return;
    }

    /* 读取状态寄存器 */
    ovflow = a320_read_reg(dev, OBSERVATION);
    if (ovflow < 0) {
        return;
    }

    /* 检查是否有有效运动数据且未溢出 */
    if ((ifmotion & BIT_MOTION_MOT) && !(ovflow & BIT_MOTION_OVF)) {
        dx_raw = a320_read_reg(dev, DELTA_X);
        dy_raw = a320_read_reg(dev, DELTA_Y);

        if (dx_raw < 0 || dy_raw < 0) {
            return;
        }

        /* 转换为有符号值并调整方向 */
        dx = ((dx_raw < 127) ? dx_raw : (dx_raw - 256)) * -1;
        dy = ((dy_raw < 127) ? dy_raw : (dy_raw - 256));

        LOG_DBG("X movement: %d, Y movement: %d", dx, dy);

        /* 检查Alt键状态 */
        if (data->alt_pressed) {
            /* 检查滑动冷却时间 */
            if (now - data->last_swipe_time > SWIPE_COOLDOWN_TIME_MS) {
                int key_code = 0;
                
                /* 检测滑动方向 */
                if (is_swipe(dy, dx)) {
                    key_code = (dy < 0) ? KEY_UP : KEY_DOWN;
                } else if (is_swipe(dx, dy)) {
                    key_code = (dx < 0) ? KEY_LEFT : KEY_RIGHT;
                }

                /* 如果检测到滑动，注入按键事件 */
                if (key_code) {
                    LOG_INF("Swipe detected: key 0x%x", key_code);
                    
                    /* 报告按键按下 */
                    input_report_key(input_dev, key_code, 1, true);
                    
                    /* 安排按键释放 */
                    k_work_submit_in(&data->work, K_MSEC(SWIPE_RELEASE_DELAY_MS));
                    
                    /* 更新最后滑动时间 */
                    data->last_swipe_time = now;
                }
            }
        } else {
            /* Alt键未按下，调用回调函数 */
            if (data->callbacks) {
                struct touch_callback *cb = data->callbacks;

                while (cb) {
                    cb->func(dx, dy);
                    cb = cb->next;
                }
            }
        }
    }
}

/* 运动检测GPIO中断处理函数 */
static void motion_gpio_callback(const struct device *dev, struct gpio_callback *cb,
                                uint32_t pins) {
    struct a320_data *data = CONTAINER_OF(cb, struct a320_data, gpio_cb);
    
    /* 提交工作队列处理触摸数据 */
    k_work_submit(&data->work);
}

/* Alt键状态变化回调 */
static void alt_key_callback(const struct device *dev, struct input_event *evt) {
    struct a320_data *data = dev->data;
    
    if (evt->type == INPUT_EVENT_TYPE_KEY && evt->code == KEY_LEFTALT) {
        data->alt_pressed = evt->value;
        LOG_DBG("Alt key state: %d", data->alt_pressed);
    }
}

/* 从传感器获取采样数据 */
static int a320_sample_fetch(const struct device *dev, enum sensor_channel chan) {
    if (chan != SENSOR_CHAN_ALL && chan != SENSOR_CHAN_POS_DX_DY) {
        return -ENOTSUP;
    }

    /* 实际采样逻辑已在中断处理中完成 */
    return 0;
}

/* 获取指定通道的传感器数据 */
static int a320_channel_get(const struct device *dev, enum sensor_channel chan,
                            struct sensor_value *val) {
    if (chan != SENSOR_CHAN_POS_DX_DY) {
        return -ENOTSUP;
    }

    /* 这里可以添加直接读取最新数据的逻辑 */
    val->val1 = 0;  /* X方向移动 */
    val->val2 = 0;  /* Y方向移动 */
    
    return 0;
}

/* 添加触摸回调函数 */
void a320_add_touch_callback(const struct device *dev, struct touch_callback *callback) {
    struct a320_data *data = dev->data;
    
    if (!data->callbacks) {
        data->callbacks = callback;
        callback->next = NULL;
        return;
    }

    /* 找到链表末尾并添加新回调 */
    struct touch_callback *cb = data->callbacks;
    while (cb->next)
        cb = cb->next;

    cb->next = callback;
    callback->next = NULL;
}

/* A320传感器驱动API结构 */
static const struct sensor_driver_api a320_driver_api = {
    .sample_fetch = a320_sample_fetch,
    .channel_get = a320_channel_get,
};

/* 初始化A320传感器设备 */
static int a320_init(const struct device *dev) {
    struct a320_data *data = dev->data;
    const struct a320_config *cfg = dev->config;
    
    /* 检查I2C总线是否就绪 */
    if (!device_is_ready(cfg->bus.bus)) {
        LOG_ERR("I2C bus %s is not ready!", cfg->bus.bus->name);
        return -EINVAL;
    }

    /* 检查运动检测GPIO是否就绪 */
    if (!device_is_ready(cfg->motion_gpio.port)) {
        LOG_ERR("Motion GPIO device not ready");
        return -EINVAL;
    }

    /* 配置运动检测GPIO */
    int ret = gpio_add_callback(cfg->motion_gpio.port, &data->gpio_cb,
                               motion_gpio_callback);
    if (ret < 0) {
        LOG_ERR("Cannot add GPIO callback");
        return ret;
    }

    ret = gpio_pin_configure_dt(&cfg->motion_gpio, GPIO_INPUT | GPIO_PULL_UP);
    if (ret < 0) {
        LOG_ERR("Cannot configure motion GPIO");
        return ret;
    }

    ret = gpio_add_callback(cfg->motion_gpio.port, &data->gpio_cb,
                           motion_gpio_callback);
    if (ret < 0) {
        LOG_ERR("Cannot add motion GPIO callback");
        return ret;
    }

    ret = gpio_pin_interrupt_configure_dt(&cfg->motion_gpio, GPIO_INT_EDGE_TO_ACTIVE);
    if (ret < 0) {
        LOG_ERR("Cannot configure motion GPIO interrupt");
        return ret;
    }

    /* 初始化工作队列 */
    k_work_init(&data->work, process_touch_data_work);
    
    /* 初始化滑动计时器 */
    k_timer_init(&data->swipe_timer, NULL, NULL);
    
    /* 初始化状态变量 */
    data->last_swipe_time = 0;
    data->alt_pressed = false;
    data->callbacks = NULL;

    /* 读取初始值以清除可能的旧数据 */
    a320_read_reg(dev, MOTION);
    a320_read_reg(dev, OBSERVATION);
    a320_read_reg(dev, DELTA_X);
    a320_read_reg(dev, DELTA_Y);

    /* 配置传感器（设置为高分辨率模式） */
    a320_write_reg(dev, CONFIGURATION, BIT_CONFIG_HIRES);

    LOG_INF("A320 initialized successfully, ready to detect touch and gestures");
    
    /* 注册Alt键事件监听 */
    const struct device *input_dev = device_get_binding(cfg->input_dev_name);
    if (input_dev) {
        input_set_callback(input_dev, alt_key_callback);
        input_enable_callback(input_dev);
        LOG_DBG("Alt key monitoring enabled");
    } else {
        LOG_WRN("Input device not found, gesture functionality disabled");
    }
    
    return 0;
}

/* 为每个设备树实例定义A320设备 */
#define A320_DEFINE(inst)                                                                          \
    static struct a320_data a320_data_##inst;                                                      \
    static struct gpio_callback a320_gpio_cb_##inst;                                               \
    static const struct a320_config a320_cfg_##inst = {                                            \
        .bus = I2C_DT_SPEC_INST_GET(inst),                                                         \
        .motion_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, motion_gpios, {0}),                        \
        .input_dev_name = DT_INST_PROP_OR(inst, input_dev_name, "input0"),                       \
    };                                                                                               \
    \
    static int a320_init_##inst(const struct device *dev) {                                        \
        a320_data_##inst.gpio_cb = a320_gpio_cb_##inst;                                            \
        return a320_init(dev);                                                                       \
    }                                                                                               \
    \
    DEVICE_DT_INST_DEFINE(inst, a320_init_##inst, NULL, &a320_data_##inst, &a320_cfg_##inst,  \
                          POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY, &a320_driver_api);

DT_INST_FOREACH_STATUS_OKAY(A320_DEFINE)   