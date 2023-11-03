#include "../inc/app_main.h"

#include <stdio.h>
#include <stdlib.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/ring_buffer.h>

#include <vector>

#include "../inc/dynamixel.h"
#include "../inc/dynamixel_sdk/dynamixel_sdk.h"
#include "../inc/hardware.h"

LOG_MODULE_REGISTER(app_main);

using namespace Doggedness;

void threadJetsonDataParseLoop(void *, void *, void *);

struct k_thread jetson_parse_thread_data;
K_THREAD_STACK_DEFINE(jetson_parse, 1024);

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

  k_thread_create(&jetson_parse_thread_data, jetson_parse,
                  K_THREAD_STACK_SIZEOF(jetson_parse),
                  threadJetsonDataParseLoop, NULL, NULL, NULL, 10, 0, K_NO_WAIT);

  dynamixel::PortHandler *portHandler =
      dynamixel::PortHandler::getPortHandler("");
  dynamixel::PacketHandler *packetHandler =
      dynamixel::PacketHandler::getPacketHandler();
  DynamixelController controller(portHandler, packetHandler);

  portHandler->openPort();
  int dxl_comm_result;
  uint8_t dxl_error;

  k_sleep(K_MSEC(500));

  std::vector<uint8_t> found_id;
  dxl_comm_result = packetHandler->broadcastPing(portHandler, found_id);
  if (dxl_comm_result != COMM_SUCCESS) {
    LOG_ERR("failed to communicate with motor : %d", dxl_comm_result);
  }
  LOG_INF("%d motors found", found_id.size());
  for (const auto id : found_id) {
    LOG_INF("motor id : %d", id);
  }

  controller.SetID(1, 8);

  for (;;) {
    gpio_pin_toggle_dt(&hardware::run_led);
    k_sleep(K_MSEC(1000));
  }
}

void threadJetsonDataParseLoop(void *, void *, void *) {
  char tmp[128], sentence[129];
  int tmp_cnt = 0;
  bool line_found = false;
  int line_end = 0;
  double angles[8];

  memset(tmp, 0, sizeof(tmp));
  for (;;) {
    if (ring_buf_size_get(&hardware::jetson_uart_rx_buf) > 0) {
      tmp_cnt += ring_buf_get(&hardware::jetson_uart_rx_buf,
                              (uint8_t *)tmp + tmp_cnt, sizeof(tmp) - tmp_cnt);

      LOG_INF("tmp_cnt : %d", tmp_cnt);
      for (int i = 0; i < tmp_cnt; i++) {
        if (tmp[i] == '\n') {
          line_found = true;
          line_end = i;
          break;
        }
      }

      if (line_found) {
        memcpy(sentence, tmp, line_end + 1);
        sentence[line_end + 1] = 0;
        LOG_DBG("found : %s", sentence);

        char *ptr = strtok(sentence, ",");
        int tok_cnt = 0;
        while (ptr != NULL) {
          LOG_DBG("tok : %s", ptr);
          angles[tok_cnt] = atof(ptr);
          ptr = strtok(NULL, ",");
          tok_cnt++;
        }
        if (tok_cnt != 8) {
          LOG_ERR("invalid tok cnt! : %d", tok_cnt);
        } else {
          for (int i = 0; i < 8; i++) {
            LOG_INF("set motor %d to %f", i, angles[i]);
          }
        }

        if (line_end + 1 >= tmp_cnt) {
          memcpy(tmp, tmp + tmp_cnt, tmp_cnt - line_end - 1);
          tmp_cnt -= (line_end + 1);
        }
      } else {
        if (tmp_cnt >= sizeof(tmp)) {
          tmp_cnt = 0;
        }
      }
    }
    k_sleep(K_MSEC(5));
  }
}
