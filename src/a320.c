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

#include "a320.h"

LOG_MODULE_REGISTER(A320, CONFIG_SENSOR_LOG_LEVEL);

// 将无符号8位值转换为有符号8位值
static inline int8_t convert_to_signed(uint8_t value) {
    return (int8_t)((value ^ 0x80) - 0x80);
}

static int a320_read_reg(const struct device *dev, uint8_t reg_addr) {
    const struct a320_config *cfg = dev->config;
    uint8_t value = 0;

    if (i2c_reg_read_byte_dt(&cfg->bus, reg_addr, &value) == 0) {
        return value;
    }

    LOG_ERR("failed to read 0x%x register", reg_addr);
    return -EIO;
}

static int a320_sample_fetch(const struct device *dev, enum sensor_channel chan) {
    const struct a320_config *cfg = dev->config;
    
    // 检查是否只请求了特定通道
    if (chan != SENSOR_CHAN_ALL && chan != SENSOR_CHAN_DISTANCE) {
        return -ENOTSUP;
    }
    
    // 读取Motion寄存器以清除运动状态
    int ret = a320_read_reg(dev, Motion);
    if (ret < 0) {
        return ret;
    }
    
    return 0;
}

static int a320_channel_get(const struct device *dev, enum sensor_channel chan,
                            struct sensor_value *val) {
    if (chan != SENSOR_CHAN_DISTANCE_X && chan != SENSOR_CHAN_DISTANCE_Y) {
        return -ENOTSUP;
    }
    
    // 读取Motion寄存器检测运动
    int motion_val = a320_read_reg(dev, Motion);
    if (motion_val < 0) {
        return motion_val;
    }
    
    // 读取Overflow寄存器检查溢出
    int overflow_val = a320_read_reg(dev, Overflow);
    if (overflow_val < 0) {
        return overflow_val;
    }
    
    // 检查是否有有效运动数据且无溢出
    if ((motion_val & BIT_MOTION_MOT) && !(overflow_val & BIT_MOTION_OVF)) {
        int delta_x, delta_y;
        
        // 读取X和Y位移
        delta_x = a320_read_reg(dev, Delta_X);
        delta_y = a320_read_reg(dev, Delta_Y);
        
        if (delta_x < 0 || delta_y < 0) {
            return (delta_x < 0) ? delta_x : delta_y;
        }
        
        // 转换为有符号值
        int8_t x = convert_to_signed(delta_x);
        int8_t y = convert_to_signed(delta_y);
        
        // 根据请求的通道返回相应值
        if (chan == SENSOR_CHAN_DISTANCE_X) {
            val->val1 = x;
            val->val2 = 0; // 没有小数部分
            LOG_DBG("X displacement: %d", x);
        } else {
            val->val1 = y;
            val->val2 = 0; // 没有小数部分
            LOG_DBG("Y displacement: %d", y);
        }
        
        return 0;
    }
    
    // 没有运动或有溢出，返回0
    val->val1 = 0;
    val->val2 = 0;
    return 0;
}

static const struct sensor_driver_api a320_driver_api = {
    .sample_fetch = a320_sample_fetch,
    .channel_get = a320_channel_get,
};

static int a320_init(const struct device *dev) {
    const struct a320_config *cfg = dev->config;
    
    // 检查I2C总线是否就绪
    if (!device_is_ready(cfg->bus.bus)) {
        LOG_ERR("I2C bus %s is not ready!", cfg->bus.bus->name);
        return -EINVAL;
    }
    
    // 执行初始化序列
    int ret = a320_read_reg(dev, Product_ID);
    if (ret < 0) {
        LOG_ERR("Failed to read Product ID");
        return ret;
    }
    
    LOG_INF("A320 Product ID: 0x%02X", ret);
    
    // 读取初始值以清除可能的旧数据
    a320_read_reg(dev, Motion);
    a320_read_reg(dev, Delta_X);
    a320_read_reg(dev, Delta_Y);
    
    LOG_INF("A320 Init done, Ready to read data.");
    return 0;
}

#define A320_DEFINE(inst)                                                                          \
    struct a320_data a3200_data_##inst;                                                            \
    static const struct a320_config a3200_cfg_##inst = {.bus = I2C_DT_SPEC_INST_GET(inst)};        \
    DEVICE_DT_INST_DEFINE(inst, a320_init, NULL, &a3200_data_##inst, &a3200_cfg_##inst,            \
                          POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY, &a320_driver_api);

DT_INST_FOREACH_STATUS_OKAY(A320_DEFINE)
