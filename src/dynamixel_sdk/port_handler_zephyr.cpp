#include "../../inc/dynamixel_sdk/port_handler_zephyr.h"

#include <zephyr/logging/log.h>

#define LATENCY_TIMER 4  // msec (USB latency timer)

LOG_MODULE_REGISTER(port_handler);

void uartRxDmaGlobalCallback(const struct device *dev, struct uart_event *evt, void *user_data) {
  ((dynamixel::PortHandlerZephyr*)user_data)->uartRxDmaCallback(evt);
}

using namespace dynamixel;

PortHandlerZephyr::PortHandlerZephyr(const struct device *dev,
                                     const struct gpio_dt_spec *txe,
                                     k_thread_stack_t *stack, size_t stack_size)
    : baudrate_(DEFAULT_BAUDRATE_),
      packet_start_time_(0.0),
      packet_timeout_(0.0),
      tx_time_per_byte(0.0),
      dev_(dev),
      tx_enable_(txe) {
  is_using_ = false;
  setPortName("");
  setTxDisable();

  int ret;
  ring_buf_init(&read_buffer_, sizeof(read_buffer_data_), read_buffer_data_);

  ret = uart_rx_enable(dev_, read_dma_buffer_, sizeof(read_dma_buffer_), 10);
  if (ret < 0) {
    LOG_ERR("failed to init uart rx dma : %d", ret);
  }
  uart_callback_set(dev, uartRxDmaGlobalCallback, (void *)this);
  /*
    read_thread_id_ =
        k_thread_create(&read_thread_data_, stack, stack_size,
    threadReadHandler, (void *)this, NULL, NULL, 2, 0, K_NO_WAIT);
  */
}

void PortHandlerZephyr::uartRxDmaCallback(struct uart_event *evt) {
  switch (evt->type) {
    case UART_RX_RDY:
      LOG_DBG("%d bytes recieved", evt->data.rx.len);
      ring_buf_put(&read_buffer_, evt->data.rx.buf + evt->data.rx.offset,
                   evt->data.rx.len);
      break;
    case UART_RX_DISABLED:
      uart_rx_enable(dev_, read_dma_buffer_, READ_DMA_BUFFER_SIZE, 10);
      break;
    case UART_RX_STOPPED:
      LOG_ERR("uart rx dma is unexpectly stopped : %d",
              evt->data.rx_stop.reason);
      break;
  }
}

bool PortHandlerZephyr::openPort() { return setBaudRate(baudrate_); }

void PortHandlerZephyr::closePort() { setPowerOff(); }

void PortHandlerZephyr::clearPort() {
  /* by polling api, no need flushing logic */
  return;
}

void PortHandlerZephyr::setPortName(const char *port_name) {
  strcpy(port_name_, port_name);
}

char *PortHandlerZephyr::getPortName() { return port_name_; }

bool PortHandlerZephyr::setBaudRate(const int baudrate) {
  baudrate_ = checkBaudrateAvailable(baudrate);

  if (baudrate_ == -1) return false;

  setupPort(baudrate_);

  return true;
}

int PortHandlerZephyr::getBaudRate() { return baudrate_; }

int PortHandlerZephyr::getBytesAvailable() {
  return ring_buf_size_get(&read_buffer_);
}

int PortHandlerZephyr::readPort(uint8_t *packet, int length) {
  return ring_buf_get(&read_buffer_, packet, length);
}

int PortHandlerZephyr::writePort(uint8_t *packet, int length) {
  setTxEnable();
  k_sleep(K_MSEC(1));
  for (int i = 0; i < length; i++) {
    uart_poll_out(dev_, (unsigned char)packet[i]);
    LOG_DBG("send : %d", packet[i]);
  }
  k_sleep(K_MSEC(1));
  setTxDisable();

  return length;
}

void PortHandlerZephyr::setPacketTimeout(uint16_t packet_length) {
  packet_start_time_ = getCurrentTime();
  packet_timeout_ =
      (tx_time_per_byte * (double)packet_length) + (LATENCY_TIMER * 2.0) + 2.0;
}

void PortHandlerZephyr::setPacketTimeout(double msec) {
  packet_start_time_ = getCurrentTime();
  packet_timeout_ = msec;
}

bool PortHandlerZephyr::isPacketTimeout() {
  if (getTimeSinceStart() > packet_timeout_) {
    packet_timeout_ = 0;
    return true;
  }

  return false;
}

double PortHandlerZephyr::getCurrentTime() { return (double)k_uptime_get(); }

double PortHandlerZephyr::getTimeSinceStart() {
  double elapsed_time;

  elapsed_time = getCurrentTime() - packet_start_time_;
  if (elapsed_time < 0.0) packet_start_time_ = getCurrentTime();

  return elapsed_time;
}

bool PortHandlerZephyr::setupPort(int baudrate) {
  struct uart_config cfg;
  int ret;

  ret = uart_config_get(dev_, &cfg);
  if (ret < 0) {
    LOG_ERR("failed to get uart config : %d", ret);
    return false;
  }

  cfg.baudrate = baudrate;
  ret = uart_configure(dev_, &cfg);
  if (ret < 0) {
    LOG_ERR("failed to set uart config : %d", ret);
    return false;
  }
  LOG_INF("set baudrate as %d", baudrate);

  tx_time_per_byte = (1000.0 / (double)baudrate) * 10.0;
  return true;
}

int PortHandlerZephyr::checkBaudrateAvailable(int baudrate) {
  switch (baudrate) {
    case 9600:
      return 9600;
    case 57600:
      return 57600;
    case 115200:
      return 115200;
    case 1000000:
      return 1000000;
    case 2000000:
      return 2000000;
    case 3000000:
      return 3000000;
    case 4000000:
      return 4000000;
    case 4500000:
      return 4500000;
    default:
      return -1;
  }
}

void PortHandlerZephyr::setPowerOn() {}

void PortHandlerZephyr::setPowerOff() {}

void PortHandlerZephyr::setTxEnable() { gpio_pin_set_dt(tx_enable_, 1); }

void PortHandlerZephyr::setTxDisable() { gpio_pin_set_dt(tx_enable_, 0); }