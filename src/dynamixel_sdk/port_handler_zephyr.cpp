#include "../../inc/dynamixel_sdk/port_handler_zephyr.h"

#include <zephyr/kernel.h>

#define LATENCY_TIMER 4  // msec (USB latency timer)

using namespace dynamixel;

PortHandlerZephyr::PortHandlerZephyr(const struct device *dev)
    : baudrate_(DEFAULT_BAUDRATE_),
      packet_start_time_(0.0),
      packet_timeout_(0.0),
      tx_time_per_byte(0.0),
      dev(dev) {
  is_using_ = false;
  setPortName(port_name);
  setTxDisable();
}

bool PortHandlerZephyr::openPort() { return setBaudRate(baudrate_); }

void PortHandlerZephyr::closePort() { setPowerOff(); }

void PortHandlerZephyr::clearPort() { /* TODO*/
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
  int bytes_available;

  /* TODO*/

  return bytes_available;
}

int PortHandlerZephyr::readPort(uint8_t *packet, int length) {
  int rx_length;

  /* TODO*/

  return rx_length;
}

int PortHandlerZephyr::writePort(uint8_t *packet, int length) {
  int length_written;

  setTxEnable();

  /* TODO*/

  setTxDisable();

  return length_written;
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

void PortHandlerZephyr::setTxEnable() { /* TODO */
}

void PortHandlerZephyr::setTxDisable() { /* TODO */
}