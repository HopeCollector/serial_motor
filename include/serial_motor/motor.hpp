#pragma once
#include "serial_motor/types/general_motor.hpp"
#include "serial_motor/types/rmd.hpp"
#include <memory>

namespace smotor {
std::shared_ptr<motor> create_rmd_motor(std::string dev, uint8_t id);
};