#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/input/input.h>
#include <dt-bindings/input/input-event-codes.h>
#include "driver/trackpad.h"

/* 寄存器定义 */
enum {
    REG_PID        = 0x00,
    REG_REV        = 0x01,
    REG_MOTION     = 0x02,
    REG_DELTA_X    = 0x03,
    REG_DELTA_Y    = 0x04,
};
#define BIT_MOTION_MOT  (1 << 7)
#define BIT_MOTION_OVF  (1 << 4)

/* 设备配置结构 */
struct trackpad_config {
    struct i2c_dt_spec i2c;                 // I2C设备规格
    struct gpio_dt_spec shtdwn_gpio;        // 关机控制引脚
    struct gpio_dt_spec reset_gpio;         // 复位控制引脚
    struct gpio_dt_spec motion_gpio;        // 运动检测引脚
    uint8_t scroll_speed_div;               // 滚动速度除数
};

/* 设备运行时数据 */
struct trackpad_data {
    struct k_spinlock lock;                 // 自旋锁保护共享数据
    int8_t acc_delta_x;                     // X轴位移累计值
    int8_t acc_delta_y;                     // Y轴位移累计值
    bool is_scroll_mode;                    // 滚动模式标志
    struct k_work work;                     // 工作队列
    struct gpio_callback isr_cb;            // 中断回调
    const struct device *dev;               // 设备实例
};

/* I2C安全读取 */
static int read_register8(const struct device *dev, uint8_t reg, uint8_t *val) {
    const struct trackpad_config *cfg = dev->config;
    return i2c_write_read_dt(&cfg->i2c, &reg, 1, val, 1);
}

/* 工作队列处理函数（关键修改点）*/
static void trackpad_work_handler(struct k_work *work) {
    struct trackpad_data *data = CONTAINER_OF(work, struct trackpad_data, work);
    const struct device *dev = data->dev;
    const struct trackpad_config *cfg = dev->config;
    k_spinlock_key_t key;
    int8_t x = 0, y = 0;
    bool is_scroll;

    key = k_spin_lock(&data->lock);
    x = data->acc_delta_x;
    y = data->acc_delta_y;
    is_scroll = data->is_scroll_mode;
    data->acc_delta_x = 0;
    data->acc_delta_y = 0;
    k_spin_unlock(&data->lock, key);

    if (x != 0 || y != 0) {
        if (is_scroll) {
            input_report_rel(dev, INPUT_REL_HWHEEL, x / cfg->scroll_speed_div);
            input_report_rel(dev, INPUT_REL_WHEEL, -y / cfg->scroll_speed_div);
        } else {
            /* 新增X轴方向反转：-x * 3 / 2 */
            input_report_rel(dev, INPUT_REL_X, (int32_t)(-x) * 3 / 2); // 反转X轴方向
            input_report_rel(dev, INPUT_REL_Y, (int32_t)y * 3 / 2);
        }
        input_sync(dev);
    }
}

/* 中断处理函数（无改动）*/
static void trackpad_isr(const struct device *dev, 
                         struct gpio_callback *cb, 
                         uint32_t pins) {
    struct trackpad_data *data = CONTAINER_OF(cb, struct trackpad_data, isr_cb);
    uint8_t motion;
    int8_t x, y;
    
    if (read_register8(dev, REG_MOTION, &motion) != 0) 
        return;

    if (motion & BIT_MOTION_MOT) {
        if (read_register8(dev, REG_DELTA_X, (uint8_t*)&x) != 0) return;
        if (read_register8(dev, REG_DELTA_Y, (uint8_t*)&y) != 0) return;
        
        x = (x < 127) ? x : (x - 256);
        y = (y < 127) ? y : (y - 256);
        
        k_spinlock_key_t key = k_spin_lock(&data->lock);
        data->acc_delta_x += x;
        data->acc_delta_y += y;
        k_spin_unlock(&data->lock, key);
    }
    k_work_submit(&data->work);
}

/* 模式切换API（无改动）*/
void trackpad_set_mode(const struct device *dev, uint8_t mode) {
    struct trackpad_data *data = dev->data;
    k_spinlock_key_t key = k_spin_lock(&data->lock);
    data->is_scroll_mode = (mode == TRACKPAD_MODE_SCROLL);
    k_spin_unlock(&data->lock, key);
}

/* 设备初始化（无改动）*/
static int trackpad_init(const struct device *dev) {
    struct trackpad_data *data = dev->data;
    const struct trackpad_config *cfg = dev->config;
    uint8_t pid;

    if (!device_is_ready(cfg->i2c.bus)) {
        LOG_ERR("I2C bus not ready");
        return -ENODEV;
    }

    gpio_pin_configure_dt(&cfg->shtdwn_gpio, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&cfg->reset_gpio, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&cfg->motion_gpio, 
                         GPIO_INPUT | cfg->motion_gpio.dt_flags | GPIO_INT_DEBOUNCE);

    gpio_pin_set_dt(&cfg->shtdwn_gpio, 0);
    k_msleep(10);
    gpio_pin_set_dt(&cfg->reset_gpio, 0);
    k_msleep(100);
    gpio_pin_set_dt(&cfg->reset_gpio, 1);
    k_msleep(50);
    gpio_pin_set_dt(&cfg->shtdwn_gpio, 1);
    k_msleep(20);

    for (int i = 0; i < 5; i++) {
        k_msleep(10);
        if (read_register8(dev, REG_PID, &pid) == 0 && pid == 0x30)
            break;
        if (i == 4) {
            LOG_ERR("Device ID mismatch");
            return -EIO;
        }
    }

    k_work_init(&data->work, trackpad_work_handler);
    data->dev = dev;
    data->acc_delta_x = 0;
    data->acc_delta_y = 0;
    data->is_scroll_mode = false;

    gpio_init_callback(&data->isr_cb, trackpad_isr, BIT(cfg->motion_gpio.pin));
    gpio_add_callback_dt(&cfg->motion_gpio, &data->isr_cb);
    gpio_pin_interrupt_configure_dt(&cfg->motion_gpio, GPIO_INT_EDGE_FALLING);

    return 0;
}

/* 设备树定义宏（无改动）*/
#define TRACKPAD_DEFINE(inst) \
    static struct trackpad_data trackpad_data_##inst; \
    static const struct trackpad_config trackpad_config_##inst = { \
        .i2c = I2C_DT_SPEC_GET(DT_DRV_INST(inst)), \
        .shtdwn_gpio = GPIO_DT_SPEC_GET(DT_DRV_INST(inst), shtdwn_gpios), \
        .reset_gpio = GPIO_DT_SPEC_GET(DT_DRV_INST(inst), reset_gpios), \
        .motion_gpio = GPIO_DT_SPEC_GET(DT_DRV_INST(inst), motion_gpios), \
        .scroll_speed_div = DT_INST_PROP(inst, scroll_speed_divider) \
    }; \
    DEVICE_DT_INST_DEFINE(inst, trackpad_init, NULL, \
        &trackpad_data_##inst, &trackpad_config_##inst, \
        POST_KERNEL, CONFIG_INPUT_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(TRACKPAD_DEFINE)
