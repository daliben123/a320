/* 文件名：touchpad_a320.c */
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/input/input.h> // 输入子系统头文件
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#define DT_DRV_COMPAT pixart_a320
LOG_MODULE_REGISTER(TOUCHPAD_A320, CONFIG_SENSOR_LOG_LEVEL);

/* 寄存器定义 */
#define REG_MOTION         0x02
#define REG_DELTA_X        0x03
#define REG_DELTA_Y        0x04
#define BIT_MOTION_MOT     (1 << 7)
#define BIT_MOTION_OVF     (1 << 4)

/* 设备配置结构 */
struct touchpad_config {
    struct i2c_dt_spec bus;
    struct gpio_dt_spec motion_gpio;  // 运动中断引脚
    struct gpio_dt_spec reset_gpio;   // 复位引脚（可选）
};

/* 设备数据结构 */
struct touchpad_data {
    struct gpio_callback motion_cb;
    struct k_work work;
    const struct device *input_dev;   // 输入设备指针
};

/* 工作队列处理函数 */
static void touchpad_work_handler(struct k_work *work) {
    struct touchpad_data *data = CONTAINER_OF(work, struct touchpad_data, work);
    const struct device *dev = data->input_dev;
    const struct touchpad_config *cfg = dev->config;

    uint8_t motion;
    int8_t x, y;

    /* 读取运动状态寄存器 */
    if (i2c_reg_read_byte_dt(&cfg->bus, REG_MOTION, &motion) < 0) {
        LOG_ERR("Failed to read motion register");
        return;
    }

    /* 检查有效运动事件 */
    if ((motion & BIT_MOTION_MOT) && !(motion & BIT_MOTION_OVF)) {
        /* 读取位移数据 */
        i2c_reg_read_byte_dt(&cfg->bus, REG_DELTA_X, (uint8_t*)&x);
        i2c_reg_read_byte_dt(&cfg->bus, REG_DELTA_Y, (uint8_t*)&y);

        /* 符号扩展转换（原始数据为8位有符号） */
        int16_t dx = (x < 127) ? -x : -(x - 256); // X轴位移
        int16_t dy = (y < 127) ? y : (y - 256);   // Y轴位移

        /* 仅上报鼠标移动事件 */
        input_report_rel(dev, REL_X, dx, true, K_FOREVER);
        input_report_rel(dev, REL_Y, dy, true, K_FOREVER);
    }
}

/* 中断处理函数 */
static void touchpad_gpio_cb(const struct device *dev, 
                            struct gpio_callback *cb, 
                            uint32_t pins) {
    struct touchpad_data *data = CONTAINER_OF(cb, struct touchpad_data, motion_cb);
    k_work_submit(&data->work);  // 提交到工作队列
}

/* 设备初始化 */
static int touchpad_init(const struct device *dev) {
    struct touchpad_data *data = dev->data;
    const struct touchpad_config *cfg = dev->config;
    
    /* 初始化工作队列 */
    k_work_init(&data->work, touchpad_work_handler);
    data->input_dev = dev; // 关联输入设备

    /* I2C总线检查 */
    if (!device_is_ready(cfg->bus.bus)) {
        LOG_ERR("I2C bus not ready");
        return -ENODEV;
    }

    /* 复位序列（可选） */
    if (cfg->reset_gpio.port) {
        gpio_pin_configure_dt(&cfg->reset_gpio, GPIO_OUTPUT_INACTIVE);
        gpio_pin_set_dt(&cfg->reset_gpio, 0);
        k_msleep(100);
        gpio_pin_set_dt(&cfg->reset_gpio, 1);
    }

    /* 中断配置 */
    gpio_pin_configure_dt(&cfg->motion_gpio, GPIO_INPUT);
    gpio_pin_interrupt_configure_dt(&cfg->motion_gpio, GPIO_INT_EDGE_FALLING);
    gpio_init_callback(&data->motion_cb, touchpad_gpio_cb, BIT(cfg->motion_gpio.pin));
    gpio_add_callback(cfg->motion_gpio.port, &data->motion_cb);

    /* 注册输入设备能力 */
    input_set_capability(data->input_dev, INPUT_EV_REL, REL_X);
    input_set_capability(data->input_dev, INPUT_EV_REL, REL_Y);

    LOG_INF("Touchpad initialized (Move-only mode)");
    return 0;
}

/* 设备实例定义 */
#define TOUCHPAD_DEFINE(inst) \
    static struct touchpad_data touchpad_data_##inst; \
    static const struct touchpad_config touchpad_config_##inst = { \
        .bus = I2C_DT_SPEC_INST_GET(inst), \
        .motion_gpio = GPIO_DT_SPEC_INST_GET(inst, motion_gpios), \
        .reset_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, reset_gpios, {0}), \
    }; \
    DEVICE_DT_INST_DEFINE(inst, \
                          touchpad_init, \
                          NULL, \
                          &touchpad_data_##inst, \
                          &touchpad_config_##inst, \
                          POST_KERNEL, \
                          CONFIG_SENSOR_INIT_PRIORITY, \
                          NULL); // 无需sensor_driver_api

DT_INST_FOREACH_STATUS_OKAY(TOUCHPAD_DEFINE)
