#ifndef DOGGEDNESS_HARDWARE
#define DOGGEDNESS_HARDWARE

/* zephyr device and driver */
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/ring_buffer.h>

/* std headers */
#include <array>

namespace Doggedness {
namespace hardware {

extern const struct gpio_dt_spec run_led, err_led, tx_enable;
extern const struct device *imu, *motor_uart, *telemetry_uart, *console_uart;
extern struct ring_buf jetson_uart_rx_buf;

int CheckHardware();
int InitHardware();
int ReadIMU(std::array<double, 3> &accel, std::array<double, 3> &gyro, std::array<double, 3> &magn);

};  // namespace hardware
};  // namespace Doggedness

#endif