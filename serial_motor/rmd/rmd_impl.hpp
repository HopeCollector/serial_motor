#pragma once
#include <string>
#include <memory>
#include <vector>
#include "serial_motor/types/general_motor.hpp"

namespace serial {
  class Serial;
};

namespace smotor {
class commander;
class rmd_impl final : public motor {
public:
  rmd_impl() = delete;
  rmd_impl(std::string dev, uint8_t id);
  ~rmd_impl() override;
  bool rotate(double speed) override;
  bool pause() override;
  bool to(double degree, double speed) override;
  bool more(double degree, double speed) override;
  std::optional<double> cur_pose() override;

private:
  bool is_busy_;
  uint8_t id_;
  std::vector<uint8_t> buffer_;
  std::unique_ptr<commander> cmder_;
  std::unique_ptr<serial::Serial> sender_;
};
};
