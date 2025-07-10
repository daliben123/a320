#pragma once

#include <zephyr/device.h>

#define TRACKPAD_MODE_NORMAL 0
#define TRACKPAD_MODE_SCROLL 1

void trackpad_set_mode(const struct device *dev, uint8_t mode);
