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


  for (;;) {
    gpio_pin_toggle_dt(&hardware::run_led);
    k_sleep(K_MSEC(1000));
  }
}