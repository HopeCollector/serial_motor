#include "serial_motor/rmd/rmd_impl.hpp"
#include "serial_motor/types/rmd.hpp"

using namespace smotor;

rmd::rmd(std::string dev, uint8_t id) : impl_(new rmd_impl{dev, id}) {}

rmd::~rmd() {}

bool rmd::rotate(double speed) { return impl_->rotate(speed); }

bool rmd::pause() { return impl_->pause(); }

bool rmd::to(double degree, double speed) { return impl_->to(degree, speed); }

bool rmd::more(double degree, double speed) {
  return impl_->more(degree, speed);
}

std::optional<double> rmd::cur_pose() { return impl_->cur_pose(); }