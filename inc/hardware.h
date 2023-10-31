#ifndef DOGGEDNESS_HARDWARE
#define DOGGEDNESS_HARDWARE

/* zephyr device and driver */
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>

/* std headers */
#include <array>

namespace RbfpidBalbot {
namespace hardware {

extern const struct gpio_dt_spec run_led, err_led;
extern const struct device *imu;

int CheckHardware();
int InitHardware();
int ReadIMU(std::array<double, 3> &accel, std::array<double, 3> &gyro, std::array<double, 3> &magn);

};  // namespace hardware
};  // namespace RbfpidBalbot

#endif