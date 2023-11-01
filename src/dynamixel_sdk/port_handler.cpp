/*******************************************************************************
* Copyright 2017 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Author: zerom, Ryu Woon Jung (Leon) */

#include "../../inc/hardware.h"
#include "../../inc/dynamixel_sdk/port_handler.h"
#include "../../inc/dynamixel_sdk/port_handler_zephyr.h"

using namespace dynamixel;

K_THREAD_STACK_DEFINE(port_stack, 1024);

PortHandler *PortHandler::getPortHandler(const char *port_name)
{
  return (PortHandler*)new PortHandlerZephyr(Doggedness::hardware::motor_uart, &Doggedness::hardware::tx_enable, port_stack, K_THREAD_STACK_SIZEOF(port_stack));
}
