/* a320.c - Avago A320 Optical Motion Sensor Driver */

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zmk/hid.h>
#include <sys/atomic.h>
#include <sys/mutex.h>

LOG_MODULE_REGISTER(A320, CONFIG_SENSOR_LOG_LEVEL);

/* 寄存器定义 */
#define A320_REG_PRODUCT_ID    0x00
#define A320_REG_REVISION_ID   0x01
#define A320_REG_MOTION        0x02  // 运动状态寄存器
#define A320_REG_DELTA_X       0x03  // X轴位移
#define A320_REG_DELTA_Y       0x04  // Y轴位移
#define A320_REG_CONFIG        0x11  // 工作模式配置
#define A320_REG_SHUTDOWN      0x42  // 低功耗控制

/* 状态位掩码 */
#define A320_BIT_MOTION_MOT   (1 << 7)  // 运动检测标志
#define A320_BIT_CONFIG_SLEEP (1 << 2)  // 休眠模式标志

/* 模式参数 */
#define SCROLL_SPEED_DIVIDER     6     // 滚轮模式速度除数
#define BOOST_SPEED_MULTIPLIER   2     // 加速模式倍数
#define SLOW_SPEED_DIVIDER       2     // 慢速模式除数
#define FIXED_POINT_FACTOR       64    // 定点数缩放因子

/* 设备配置结构（设备树动态生成） */
struct a320_config {
    struct i2c_dt_spec bus;           // I2C总线配置
    struct gpio_dt_spec motion_gpio;   // 运动中断引脚（必须）
    struct gpio_dt_spec reset_gpio;    // 硬件复位引脚（可选）
    struct gpio_dt_spec shutdown_gpio; // 低功耗控制引脚（可选）
    struct gpio_dt_spec orient_gpio;  // 方向控制引脚（可选）
    uint16_t polling_interval_ms;      // 轮询间隔（无中断时使用）
};

/* 设备运行时数据 */
struct a320_data {
    struct k_mutex data_mutex;        // 数据互斥锁
    atomic_t is_scroll_mode;           // 滚轮模式标志
    atomic_t is_boost_mode;            // 加速模式标志
    atomic_t is_inverted_mode;         // 方向反转模式
    struct gpio_callback motion_cb;    // 中断回调
    struct k_sem motion_sem;           // 事件信号量
    struct k_thread thread;            // 数据处理线程
    K_THREAD_STACK_MEMBER(thread_stack, CONFIG_A320_THREAD_STACK_SIZE);
    int8_t delta_x;                    // X轴位移
    int8_t delta_y;                    // Y轴位移
    uint8_t motion_status;             // 运动状态
};

/* 安全I2C读取函数（带错误重试） */
static int a320_read_reg(const struct device *dev, uint8_t reg, uint8_t *val) {
    const struct a320_config *cfg = dev->config;
    int retry = 3;
    int ret;

    while (retry--) {
        ret = i2c_reg_read_byte_dt(&cfg->bus, reg, val);
        if (ret == 0) return 0;
        k_sleep(K_MSEC(1));
    }
    LOG_ERR("I2C read error: reg 0x%02x, code %d", reg, ret);
    return ret;
}

/* 内部数据采集（无锁版） */
static int a320_fetch_data(const struct device *dev) {
    struct a320_data *data = dev->data;
    uint8_t motion;
    int ret;

    // 读取运动状态寄存器
    ret = a320_read_reg(dev, A320_REG_MOTION, &motion);
    if (ret < 0) return ret;

    data->motion_status = motion;

    // 仅当检测到运动时读取位移数据
    if (motion & A320_BIT_MOTION_MOT) {
        ret = a320_read_reg(dev, A320_REG_DELTA_X, (uint8_t*)&data->delta_x);
        if (ret < 0) return ret;

        ret = a320_read_reg(dev, A320_REG_DELTA_Y, (uint8_t*)&data->delta_y);
        if (ret < 0) return ret;
    } else {
        data->delta_x = 0;
        data->delta_y = 0;
    }
    return 0;
}

/* 传感器驱动API：数据采集 */
static int a320_sample_fetch(const struct device *dev, enum sensor_channel chan) {
    struct a320_data *data = dev->data;
    int ret;

    k_mutex_lock(&data->data_mutex, K_FOREVER);
    ret = a320_fetch_data(dev);
    k_mutex_unlock(&data->data_mutex);

    return ret;
}

/* 传感器驱动API：通道数据获取 */
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
            val->val1 = (data->motion_status & A320_BIT_MOTION_MOT) ? 1 : 0;
            val->val2 = 0;
            break;
        default:
            ret = -ENOTSUP;
            LOG_WRN("Unsupported channel: %d", chan);
    }
    k_mutex_unlock(&data->data_mutex);
    return ret;
}

/* 定点数位移缩放（精度优化） */
static void apply_scaling(struct a320_data *data, int16_t *x, int16_t *y) {
    int32_t scaled_x = (int32_t)data->delta_x * FIXED_POINT_FACTOR;
    int32_t scaled_y = (int32_t)data->delta_y * FIXED_POINT_FACTOR;

    // 基础灵敏度调整（1.5倍）
    scaled_x = (scaled_x * 3) / 2;
    scaled_y = (scaled_y * 3) / 2;

    // 应用模式调节
    if (atomic_get(&data->is_boost_mode)) {
        scaled_x *= BOOST_SPEED_MULTIPLIER;
        scaled_y *= BOOST_SPEED_MULTIPLIER;
    } else if (atomic_get(&data->is_slow_mode)) {
        scaled_x /= SLOW_SPEED_DIVIDER;
        scaled_y /= SLOW_SPEED_DIVIDER;
    }

    // 方向反转控制
    if (atomic_get(&data->is_inverted_mode)) {
        scaled_x = -scaled_x;
        scaled_y = -scaled_y;
    }

    // 缩放回原始范围并限幅
    scaled_x /= FIXED_POINT_FACTOR;
    scaled_y /= FIXED_POINT_FACTOR;
    *x = CLAMP(scaled_x, INT8_MIN, INT8_MAX);
    *y = CLAMP(scaled_y, INT8_MIN, INT8_MAX);
}

/* 中断服务函数 */
static void a320_motion_isr(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
    struct a320_data *data = CONTAINER_OF(cb, struct a320_data, motion_cb);
    k_sem_give(&data->motion_sem);  // 唤醒处理线程
}

/* HID报告生成线程 */
static void a320_hid_thread(void *dev_ptr) {
    const struct device *dev = dev_ptr;
    struct a320_data *data = dev->data;
    const struct a320_config *cfg = dev->config;
    struct zmk_hid_report_mouse report = {0};

    while (1) {
        // 等待中断或轮询超时
        if (k_sem_take(&data->motion_sem, K_MSEC(cfg->polling_interval_ms)) == 0) {
            k_mutex_lock(&data->data_mutex, K_FOREVER);
            if (a320_fetch_data(dev) == 0 && (data->motion_status & A320_BIT_MOTION_MOT)) {
                if (atomic_get(&data->is_scroll_mode)) {
                    // 滚轮模式：转换为水平/垂直滚动
                    report.h = CLAMP(data->delta_x / SCROLL_SPEED_DIVIDER, INT8_MIN, INT8_MAX);
                    report.v = CLAMP(-data->delta_y / SCROLL_SPEED_DIVIDER, INT8_MIN, INT8_MAX);
                } else {
                    // 指针模式：应用动态缩放
                    int16_t x, y;
                    apply_scaling(data, &x, &y);
                    report.x = x;
                    report.y = y;
                }
                zmk_hid_mouse_report_send(&report);  // 发送HID事件
            }
            k_mutex_unlock(&data->data_mutex);
        }
    }
}

/* ========== 公开API函数 ========== */
void a320_set_scroll_mode(const struct device *dev, bool enable) {
    struct a320_data *data = dev->data;
    atomic_set(&data->is_scroll_mode, enable);
    LOG_INF("Scroll mode %s", enable ? "ON" : "OFF");
}

void a320_set_boost_mode(const struct device *dev, bool enable) {
    struct a320_data *data = dev->data;
    atomic_set(&data->is_boost_mode, enable);
    LOG_INF("Boost mode %s", enable ? "ON" : "OFF");
}

void a320_set_inverted_mode(const struct device *dev, bool enable) {
    struct a320_data *data = dev->data;
    const struct a320_config *cfg = dev->config;
    
    atomic_set(&data->is_inverted_mode, enable);
    if (cfg->orient_gpio.port) {
        gpio_pin_set_dt(&cfg->orient_gpio, enable ? 1 : 0);
    }
    LOG_INF("Inverted mode %s", enable ? "ON" : "OFF");
}

/* 设备初始化 */
static int a320_init(const struct device *dev) {
    const struct a320_config *cfg = dev->config;
    struct a320_data *data = dev->data;

    // 初始化互斥锁和信号量
    k_mutex_init(&data->data_mutex);
    k_sem_init(&data->motion_sem, 0, 1);

    // 检查I2C总线就绪状态
    if (!device_is_ready(cfg->bus.bus)) {
        LOG_ERR("I2C bus not ready");
        return -ENODEV;
    }

    // 配置运动中断引脚
    if (device_is_ready(cfg->motion_gpio.port)) {
        gpio_pin_configure_dt(&cfg->motion_gpio, GPIO_INPUT);
        gpio_init_callback(&data->motion_cb, a320_motion_isr, BIT(cfg->motion_gpio.pin));
        gpio_add_callback(cfg->motion_gpio.port, &data->motion_cb);
        gpio_pin_interrupt_configure_dt(&cfg->motion_gpio, GPIO_INT_EDGE_RISING);
    } else {
        LOG_WRN("Motion GPIO not configured, using polling");
    }

    // 配置方向控制引脚（可选）
    if (cfg->orient_gpio.port && device_is_ready(cfg->orient_gpio.port)) {
        gpio_pin_configure_dt(&cfg->orient_gpio, GPIO_OUTPUT_INACTIVE);
    }

    // 执行硬件复位（可选）
    if (cfg->reset_gpio.port && device_is_ready(cfg->reset_gpio.port)) {
        gpio_pin_configure_dt(&cfg->reset_gpio, GPIO_OUTPUT_INACTIVE);
        gpio_pin_set_dt(&cfg->reset_gpio, 1);
        k_msleep(100);
        gpio_pin_set_dt(&cfg->reset_gpio, 0);
        k_msleep(10);
        gpio_pin_set_dt(&cfg->reset_gpio, 1);
        k_msleep(50);
    }

    // 创建HID处理线程
    k_thread_create(&data->thread, data->thread_stack,
                   K_THREAD_STACK_SIZEOF(data->thread_stack),
                   a320_hid_thread, (void *)dev, NULL, NULL,
                   K_PRIO_PREEMPT(5), 0, K_NO_WAIT);
    k_thread_name_set(&data->thread, "a320_hid");

    LOG_INF("A320 initialized successfully");
    return 0;
}

/* 传感器驱动API结构体 */
static const struct sensor_driver_api a320_driver_api = {
    .sample_fetch = a320_sample_fetch,
    .channel_get = a320_channel_get,
};

/* 设备实例化宏 */
#define A320_DEFINE(inst)                                                     \
    static struct a320_data a320_data_##inst;                                 \
    static const struct a320_config a320_cfg_##inst = {                       \
        .bus = I2C_DT_SPEC_INST_GET(inst),                                    \
        .motion_gpio = GPIO_DT_SPEC_INST_GET(inst, motion_gpios),             \
        .reset_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, reset_gpios, {0}),        \
        .shutdown_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, shutdown_gpios, {0}), \
        .orient_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, orientation_gpios, {0}),\
        .polling_interval_ms = DT_INST_PROP(inst, polling_interval_ms),       \
    };                                                                        \
    DEVICE_DT_INST_DEFINE(inst, a320_init, NULL,                              \
                          &a320_data_##inst, &a320_cfg_##inst,                \
                          POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY,           \
                          &a320_driver_api);

/* 为设备树中所有A320设备生成实例 */
DT_INST_FOREACH_STATUS_OKAY(A320_DEFINE)
