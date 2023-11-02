#include "../inc/hardware.h"

#include <errno.h>
#include <stdint.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>

#include <cmath>
#include <vector>

LOG_MODULE_REGISTER(hardware);

using namespace Doggedness;

const struct gpio_dt_spec hardware::run_led =
    GPIO_DT_SPEC_GET(DT_NODELABEL(run_led), gpios);
const struct gpio_dt_spec hardware::err_led =
    GPIO_DT_SPEC_GET(DT_NODELABEL(err_led), gpios);
const struct gpio_dt_spec hardware::tx_enable =
    GPIO_DT_SPEC_GET(DT_NODELABEL(motor_tx_enable), gpios);

const struct device *hardware::imu/* = DEVICE_DT_GET_ONE(invensense_mpu9250)*/;

const struct device *hardware::motor_uart = DEVICE_DT_GET(DT_NODELABEL(usart1));
const struct device *hardware::console_uart =
    DEVICE_DT_GET(DT_NODELABEL(usart2));
const struct device *hardware::telemetry_uart =
    DEVICE_DT_GET(DT_NODELABEL(usart3));

int hardware::CheckHardware() {
  std::vector<const device *> check_list = {run_led.port,   err_led.port,
                                            tx_enable.port, motor_uart,
                                            telemetry_uart, console_uart};

  for (const auto l : check_list) {
    if (l == NULL) return -EINVAL;
    if (!device_is_ready(l)) return -ENODEV;
  }
  return 0;
}

int hardware::InitHardware() {
  gpio_pin_configure_dt(&hardware::run_led, GPIO_OUTPUT);
  gpio_pin_configure_dt(&hardware::err_led, GPIO_OUTPUT);
  gpio_pin_configure_dt(&hardware::tx_enable, GPIO_OUTPUT);

  return 0;
}

int hardware::ReadIMU(std::array<double, 3> &accel, std::array<double, 3> &gyro,
                      std::array<double, 3> &magn) {
  struct sensor_value tmp_a[3], tmp_g[3], tmp_m[3];
  int rc = sensor_sample_fetch(hardware::imu);

  if (rc == 0) {
    rc = sensor_channel_get(hardware::imu, SENSOR_CHAN_ACCEL_XYZ, tmp_a);
  }
  if (rc == 0) {
    rc = sensor_channel_get(hardware::imu, SENSOR_CHAN_GYRO_XYZ, tmp_g);
  }
  if (rc == 0) {
    rc = sensor_channel_get(hardware::imu, SENSOR_CHAN_MAGN_XYZ, tmp_m);
  }
  if (rc == 0) {
    for (int i = 0; i < 3; i++) {
      accel[i] = sensor_value_to_double(&tmp_a[i]);
      gyro[i] = sensor_value_to_double(&tmp_g[i]);
      magn[i] = sensor_value_to_double(&tmp_m[i]);
    }
  } else {
    LOG_ERR("sample fetch/get failed: %d\n", rc);
  }
  return rc;
}