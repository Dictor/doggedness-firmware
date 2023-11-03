#ifndef DOGGEDNESS_DYNAMIXEL_CONTROLLER
#define DOGGEDNESS_DYNAMIXEL_CONTROLLER

#include "dynamixel_sdk/dynamixel_sdk.h"

namespace Doggedness {
class DynamixelController {
 public:
  enum class OperatingModeValue : uint8_t {
    Current = 0,
    Velocity = 1,
    Position = 3,
    ExtendedPosition = 4,
    CurrentPosition = 5,
    PWM = 16,
  };

  enum Address : uint16_t {
    ID = 7,
    OperatingMode = 11,
    TorqueEnable = 64,
    GoalPosition = 116,
  };

  DynamixelController(dynamixel::PortHandler*, dynamixel::PacketHandler*);
  int SetOperatingMode(uint8_t id, OperatingModeValue mode);
  int SetTorque(uint8_t id, bool enable);
  int SetGoalPosition(uint8_t id, double degree);
  int SetID(uint8_t origin, uint8_t target);

 private:
  dynamixel::PortHandler* port_handler_;
  dynamixel::PacketHandler* packet_handler_;
  void LogError(int comm_ret, uint8_t motor_err);
};
};  // namespace Doggedness

#endif