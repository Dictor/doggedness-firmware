#include "../inc/app_main.h"

#include <stdio.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <vector>

#include "../inc/dynamixel_sdk/dynamixel_sdk.h"
#include "../inc/hardware.h"

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

  dynamixel::PortHandler *portHandler =
      dynamixel::PortHandler::getPortHandler("");
  dynamixel::PacketHandler *packetHandler =
      dynamixel::PacketHandler::getPacketHandler();

  portHandler->openPort();
  int dxl_comm_result;
  uint8_t dxl_error;

  k_sleep(K_MSEC(500));

  std::vector<uint8_t> found_id;
  std::vector<int> try_baudrate = {57600};

  for (const auto baud : try_baudrate) {
    portHandler->setBaudRate(baud);
    LOG_INF("try baudrate %d", baud);

    dxl_comm_result = packetHandler->broadcastPing(portHandler, found_id);
    if (dxl_comm_result != COMM_SUCCESS) {
      LOG_ERR("failed to communicate with motor : %d", dxl_comm_result);
    }

    LOG_INF("%d motors found", found_id.size());
    for (const auto id : found_id) {
      LOG_INF("motor id : %d", id);
    }

    k_sleep(K_MSEC(200));
  }

  for (;;) {
    gpio_pin_toggle_dt(&hardware::run_led);
    k_sleep(K_MSEC(1000));
  }
}