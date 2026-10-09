#pragma once

#include <mc_rtc/Configuration.h>
#include <mc_rtc/logging.h>

namespace mc_unitree {

enum class ControlMode { Position, Velocity, Torque };
}

namespace mc_rtc {

template <>
struct ConfigurationLoader<mc_unitree::ControlMode> {
  static Configuration save(const mc_unitree::ControlMode& mode) {
    Configuration c;
    switch (mode) {
      case mc_unitree::ControlMode::Position:
        c.add("mode", "Position");
        break;
      case mc_unitree::ControlMode::Velocity:
        c.add("mode", "Velocity");
        break;
      case mc_unitree::ControlMode::Torque:
        c.add("mode", "Torque");
        break;
      default:
        log::error_and_throw<std::runtime_error>(
            "ControlMode has unexpected value");
    }
    return c("mode");
  }

  static mc_unitree::ControlMode load(const Configuration& conf) {
    std::string mode = conf;
    if (mode == "Position") {
      return mc_unitree::ControlMode::Position;
    }
    if (mode == "Velocity") {
      return mc_unitree::ControlMode::Velocity;
    }
    if (mode == "Torque") {
      return mc_unitree::ControlMode::Torque;
    }
    log::error_and_throw<std::runtime_error>(
        "ControlMode has unexpected value {}", mode);
  }
};

}  // namespace mc_rtc
