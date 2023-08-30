#include "serial_motor/motor.hpp"

namespace smotor {
std::shared_ptr<motor> create_rmd_motor(std::string dev, uint8_t id) {
  return std::make_shared<rmd>(dev, id);
}
};