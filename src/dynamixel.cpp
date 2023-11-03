#include "../inc/dynamixel.h"

#include <zephyr/logging/log.h>

using namespace Doggedness;

LOG_MODULE_REGISTER(dynamixel_controller);

DynamixelController::DynamixelController(dynamixel::PortHandler* port,
                                         dynamixel::PacketHandler* packet)
    : port_handler_(port), packet_handler_(packet) {}

void DynamixelController::LogError(int comm_ret, uint8_t motor_err) {
  LOG_ERR("failed to comm with motor: comm=%d motor=%d", comm_ret, motor_err);
}

int DynamixelController::SetTorque(uint8_t id, bool enable) {
  int cret;
  uint8_t dret;

  cret = packet_handler_->write1ByteTxRx(
      port_handler_, id, Address::TorqueEnable, enable ? 1 : 0, &dret);
  if (cret != COMM_SUCCESS) LogError(cret, dret);
  return cret;
}

int DynamixelController::SetGoalPosition(uint8_t id, double degree) {
  int cret;
  uint8_t dret;

  cret =
      packet_handler_->write4ByteTxRx(port_handler_, id, Address::GoalPosition,
                                      (uint32_t)(degree / 0.088F), &dret);
  if (cret != COMM_SUCCESS) LogError(cret, dret);
  return cret;
}

int DynamixelController::SetID(uint8_t origin, uint8_t target) {
  int cret;
  uint8_t dret;

  cret = packet_handler_->write1ByteTxRx(port_handler_, origin, Address::ID,
                                         target, &dret);
  if (cret != COMM_SUCCESS) LogError(cret, dret);
  return cret;
}

int DynamixelController::SetOperatingMode(uint8_t id, OperatingModeValue mode) {
  int cret;
  uint8_t dret;

  cret = packet_handler_->write1ByteTxRx(port_handler_, id, Address::OperatingMode,
                                         (uint8_t)mode, &dret);
  if (cret != COMM_SUCCESS) LogError(cret, dret);
  return cret;
}
