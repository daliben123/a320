#ifndef ZEPHYR_DRIVER_TRACKPAD_H_
#define ZEPHYR_DRIVER_TRACKPAD_H_

#include <zephyr/device.h>
#include <zephyr/input/input.h>

/* 模式定义 */
enum trackpad_mode {
    TRACKPAD_MODE_POINTER,  // 指针模式
    TRACKPAD_MODE_SCROLL    // 滚动模式
};

/* 驱动API */
void trackpad_set_mode(const struct device *dev, uint8_t mode);

/* 设备树兼容性标识 */
#define DT_TRACKPAD_COMPAT "vendor,trackpad"

#endif /* ZEPHYR_DRIVER_TRACKPAD_H_ */
