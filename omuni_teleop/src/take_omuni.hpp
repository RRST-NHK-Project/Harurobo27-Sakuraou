#pragma once

#include <chrono>
#include <iostream>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"

#include "common/Ros2CanPacketController.hpp"
#include ""


double wheel_deg[4] = {0.0,pi/2.0,pi,3.0*pi/2.0};  //