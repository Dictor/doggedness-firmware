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

const struct device *hardware::imu = DEVICE_DT_GET_ONE(invensense_mpu9250);

const struct device *hardware::motor_uart = DEVICE_DT_GET(DT_NODELABEL(usart1));
const struct device *hardware::console_uart =
    DEVICE_DT_GET(DT_NODELABEL(usart2));
const struct device *hardware::telemetry_uart =
    DEVICE_DT_GET(DT_NODELABEL(usart3));

uint8_t jetson_uart_rx_dma_buf[64];
struct ring_buf hardware::jetson_uart_rx_buf;
uint8_t jetson_uart_rx_buf_data[512];

int hardware::CheckHardware() {
  std::vector<const device *> check_list = {run_led.port,   err_led.port,
                                            tx_enable.port, motor_uart,
                                            telemetry_uart, console_uart, imu};

  for (const auto l : check_list) {
    if (l == NULL) return -EINVAL;
    if (!device_is_ready(l)) return -ENODEV;
  }
  return 0;
}

void JetsonUartRxCallback(const struct device *dev, struct uart_event *evt,
                          void *user_data) {
  switch (evt->type) {
    case UART_RX_RDY:
      LOG_DBG("%d bytes recieved", evt->data.rx.len);
      ring_buf_put(&hardware::jetson_uart_rx_buf, evt->data.rx.buf + evt->data.rx.offset,
                   evt->data.rx.len);
      break;
    case UART_RX_DISABLED:
      uart_rx_enable(dev, jetson_uart_rx_dma_buf,
                     sizeof(jetson_uart_rx_dma_buf), 5);
      break;
    case UART_RX_STOPPED:
      LOG_ERR("uart rx dma is unexpectly stopped : %d",
              evt->data.rx_stop.reason);
      break;
  }
}

int hardware::InitHardware() {
  gpio_pin_configure_dt(&hardware::run_led, GPIO_OUTPUT);
  gpio_pin_configure_dt(&hardware::err_led, GPIO_OUTPUT);
  gpio_pin_configure_dt(&hardware::tx_enable, GPIO_OUTPUT);

  int ret;
  ring_buf_init(&jetson_uart_rx_buf, sizeof(jetson_uart_rx_buf_data),
                jetson_uart_rx_buf_data);
  ret = uart_rx_enable(console_uart, jetson_uart_rx_dma_buf,
                       sizeof(jetson_uart_rx_buf), 5);
  if (ret < 0) return ret;
  ret = uart_callback_set(console_uart, JetsonUartRxCallback, NULL);
  if (ret < 0) return ret;

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