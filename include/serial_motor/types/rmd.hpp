#pragma once
#include "serial_motor/types/general_motor.hpp"
#include <string>
#include <memory>

namespace smotor {
  
  class rmd_impl;

  class rmd final : public motor {
  public:
    rmd() = delete;
    rmd(std::string dev, uint8_t id);
    ~rmd() override;
    bool rotate(double speed) override;
    bool pause() override;
    bool to(double degree, double speed) override;
    bool more(double degree, double speed) override;
    std::optional<double> cur_pose() override;
  
  private:
    std::unique_ptr<rmd_impl> impl_;
  };
}
