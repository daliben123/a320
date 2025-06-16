#pragma once
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/input.h>
#include <zephyr/sys/util.h>

/**
 * @brief A320光学运动传感器数据结构
 */
struct a320_data {
    uint16_t x_position;
    uint16_t y_position;
    uint32_t last_swipe_time;
    struct k_work work;
    struct touch_callback *callbacks;
    bool alt_pressed;
};

/**
 * @brief A320设备配置结构
 */
struct a320_config {
    struct i2c_dt_spec bus;
    struct k_mutex polling_mutex;
#if DT_INST_NODE_HAS_PROP(0, reset_gpios)
    struct gpio_dt_spec reset_gpio;  /* 复位引脚配置 */
#endif
#if DT_INST_NODE_HAS_PROP(0, motion_gpios)
    struct gpio_dt_spec motion_gpio;  /* 运动检测引脚配置 */
#endif
#if DT_INST_NODE_HAS_PROP(0, orientation_gpios)
    struct gpio_dt_spec orientation_gpio;  /* 方向控制引脚配置 */
#endif
#if DT_INST_NODE_HAS_PROP(0, shutdown_gpios)
    struct gpio_dt_spec shutdown_gpio;  /* 关机控制引脚配置 */
#endif
    const char *input_dev_name;      /* 输入设备名称 */
};

/**
 * @brief 触摸回调函数结构
 */
struct touch_callback {
    void (*func)(int16_t x, int16_t y);
    struct touch_callback *next;
};

/**
 * @brief A320设备驱动API函数原型
 */
int a320_sample_fetch(const struct device *dev, enum sensor_channel chan);
int a320_channel_get(const struct device *dev, enum sensor_channel chan,
                    struct sensor_value *val);
int a320_init(const struct device *dev);

/**
 * @brief 注册触摸事件回调函数
 * 
 * @param dev 设备指针
 * @param callback 回调函数结构指针
 */
void a320_add_touch_callback(const struct device *dev, struct touch_callback *callback);

/**
 * @brief A320寄存器地址定义
 */
#define PRODUCT_ID      0x00    /* 产品ID寄存器 */
#define REVISION_ID     0x01    /* 版本ID寄存器 */
#define MOTION          0x02    /* 运动状态寄存器 */
#define DELTA_X         0x03    /* X轴移动量寄存器 */
#define DELTA_Y         0x04    /* Y轴移动量寄存器 */
#define SQUAL           0x05    /* 表面质量寄存器 */
#define SHUTTER_UPPER   0x06    /* 快门值高字节 */
#define SHUTTER_LOWER   0x07    /* 快门值低字节 */
#define MAXIMUM_PIXEL   0x08    /* 最大像素值 */
#define PIXEL_SUM       0x09    /* 像素总和 */
#define MINIMUM_PIXEL   0x0a    /* 最小像素值 */
#define PIXEL_GRAB      0x0b    /* 像素抓取寄存器 */
#define CRC0            0x0c    /* CRC校验字节0 */
#define CRC1            0x0d    /* CRC校验字节1 */
#define CRC2            0x0e    /* CRC校验字节2 */
#define CRC3            0x0f    /* CRC校验字节3 */

/* 控制和配置寄存器 */
#define SELF_TEST       0x10    /* 自检寄存器 */
#define CONFIG_BITS     0x11    /* 配置位寄存器 */
#define LED_CONTROL     0x1a    /* LED控制寄存器 */
#define IO_MODE         0x1c    /* IO模式寄存器 */
#define MOTION_CONTROL  0x1d    /* 运动控制寄存器 */
#define OBSERVATION     0x2e    /* 观察寄存器 */
#define SOFT_RESET      0x3a    /* 软件复位寄存器 */
#define SHUTTER_MAX_HI  0x3b    /* 最大快门值高字节 */
#define SHUTTER_MAX_LO  0x3c    /* 最大快门值低字节 */
#define INV_REVISION_ID 0x3e    /* 反转版本ID */
#define INV_PRODUCT_ID  0x3f    /* 反转产品ID */

/* OFN引擎相关寄存器 */
#define OFN_ENGINE      0x60    /* OFN引擎控制 */
#define OFN_RESOLUTION  0x62    /* OFN分辨率设置 */
#define OFN_SPEED_CTRL  0x63    /* OFN速度控制 */
/* 速度状态转换阈值寄存器 */
#define OFN_SPEED_ST12  0x64
#define OFN_SPEED_ST21  0x65
#define OFN_SPEED_ST23  0x66
#define OFN_SPEED_ST32  0x67
#define OFN_SPEED_ST34  0x68
#define OFN_SPEED_ST43  0x69
#define OFN_SPEED_ST45  0x6a
#define OFN_SPEED_ST54  0x6b
/* 自适应检测相关寄存器 */
#define OFN_AD_CTRL     0x6d
#define OFN_AD_ATH_HIGH 0x6e
#define OFN_AD_DTH_HIGH 0x6f
#define OFN_AD_ATH_LOW  0x70
#define OFN_AD_DTH_LOW  0x71
/* 量化控制相关寄存器 */
#define OFN_QUANTIZE_CTRL 0x73
#define OFN_XYQ_THRESH    0x74
#define OFN_FPD_CTRL      0x75
#define OFN_ORIENTATION_CTRL 0x77

/* MOTION寄存器位定义 */
#define BIT_MOTION_MOT  (1 << 7)  /* 运动检测位 */
#define BIT_MOTION_OVF  (1 << 4)  /* 溢出标志位 */

/* CONFIG_BITS寄存器位定义 */
#define BIT_CONFIG_HIRES  (1 << 7)  /* 高分辨率模式位 */

/* OBSERVATION寄存器位定义 */
#define BIT_OBSERV_RUN    (0 << 6)  /* 运行状态 */
#define BIT_OBSERV_REST1  (1 << 6)  /* 休眠状态1 */
#define BIT_OBSERV_REST2  (2 << 6)  /* 休眠状态2 */
#define BIT_OBSERV_REST3  (3 << 6)  /* 休眠状态3 */

/* 滑动检测相关定义 */
#define SWIPE_COOLDOWN_TIME_MS 100  /* 滑动冷却时间 */
#define SWIPE_RELEASE_DELAY_MS 10   /* 按键释放延迟 */
#define MOTION_IS_SWIPE(i, j) (((i >= 15) || (i <= -15)) && ((j >= -5) && (j <= 5)))

/* 按键定义 */
#define KEY_JOY_UP      INPUT_KEY_UP
#define KEY_JOY_DOWN    INPUT_KEY_DOWN
#define KEY_JOY_LEFT    INPUT_KEY_LEFT
#define KEY_JOY_RIGHT   INPUT_KEY_RIGHT
#define KEY_MOD_ALT     INPUT_KEY_LEFTALT    