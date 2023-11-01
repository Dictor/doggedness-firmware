#include "../inc/app_main.h"
#include "../inc/hardware.h"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <stdio.h>

#include "../inc/hardware.h"

#include "../inc/dynamixel_sdk/dynamixel_sdk.h"

LOG_MODULE_REGISTER(app_main);

using namespace Doggedness;

void AppMain(void) {
  int ret;

  LOG_INF("hardware initialization start");
  if (ret = hardware::CheckHardware(); ret < 0) {
    LOG_ERR("fail to check hardware, ret=%d", ret);
    gpio_pin_set_dt(&hardware::err_led, 1);
    return;
  }
  if (ret = hardware::InitHardware(); ret < 0) {
    LOG_ERR("fail to initiate hardware, ret=%d", ret);
    gpio_pin_set_dt(&hardware::err_led, 1);
    return;
  }
  LOG_INF("hardware initialization complete");

  /* application logic */
  LOG_INF("application started");
  LOG_INF("Doggedness firmware");

  dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler("");
  dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler();

  portHandler->openPort();
  int dxl_comm_result;
  uint8_t dxl_error;

  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, 0, 562, 1, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    LOG_ERR("failed to communicate with motor : %d, %d", dxl_comm_result, dxl_error);
  }

  for (;;) {
    gpio_pin_toggle_dt(&hardware::run_led);
    k_sleep(K_MSEC(1000));
  }
}