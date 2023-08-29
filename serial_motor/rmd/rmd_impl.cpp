#include "serial_motor/rmd/rmd_cmd.hpp"
#include "serial_motor/commander.hpp"
#include "serial_motor/rmd/rmd_impl.hpp"
#include "serial/serial.h"
#include <cstdlib>

using namespace smotor;

rmd_impl::rmd_impl(std::string dev, uint8_t id)
    : sender_(
          new serial::Serial{dev, 115200, serial::Timeout::simpleTimeout(5)}),
      id_(id), buffer_(100), cmder_(new commander{100}) {
  (*cmder_).init = [&](uint8_t cmd) -> commander & {
    (*cmder_).resize(cmd::rmd::FRAME_LEN);
    (*cmder_)[cmd::rmd::IDX_FRAME_HEAD] = cmd::rmd::FRAME_HEAD;
    (*cmder_)[cmd::rmd::IDX_CMD] = cmd;
    (*cmder_)[cmd::rmd::IDX_ID] = id_;
    (*cmder_)[cmd::rmd::IDX_HEAD_CHK] = 0;
    return *cmder_;
  };
  (*cmder_).build = [&]() -> commander & {
    if ((*cmder_).size() > cmd::rmd::FRAME_LEN) {
      (*cmder_).push_back(0);
      (*cmder_)[cmd::rmd::IDX_DATA_LEN] =
          (*cmder_).size() - cmd::rmd::FRAME_LEN - 1;
      for (size_t i = 0; i < (*cmder_)[cmd::rmd::IDX_DATA_LEN]; i++) {
        (*cmder_).back() += (*cmder_)[cmd::rmd::IDX_DATA_START + i];
      }
    } else
      (*cmder_)[cmd::rmd::IDX_DATA_LEN] = 0;

    for (size_t i = 0; i < cmd::rmd::IDX_HEAD_CHK; i++) {
      (*cmder_)[cmd::rmd::IDX_HEAD_CHK] += (*cmder_)[i];
    }
    return (*cmder_);
  };
  if (!sender_->isOpen()) {
    sender_->open();
  }
}

rmd_impl::~rmd_impl() {
  if (sender_->isOpen()) {
    sender_->close();
  }
}

bool rmd_impl::rotate(double speed) {
  if (is_busy_ || std::abs(speed) > cmd::rmd::MAX_SPEED)
    return false;
  is_busy_ = true;
  const int rev_len = 13;
  int32_t _speed = static_cast<int32_t>(speed * 100);
  cmder_->init(cmd::rmd::W_SPEED).append(_speed).build();
  sender_->write(*cmder_);
  buffer_.clear();
  auto ret = sender_->read(buffer_, rev_len);
  is_busy_ = false;
  return ret == rev_len;
}

bool rmd_impl::pause() {
  cmder_->init(cmd::rmd::MOTOR_PAUSE).build();
  sender_->write(*cmder_);
  buffer_.clear();
  auto rev = sender_->read(buffer_, 5);
  return rev == 5;
}

bool rmd_impl::to(double degree, double speed) {
  constexpr int rev_len = 13;
  if (is_busy_ || std::abs(speed) > cmd::rmd::MAX_SPEED || speed == 0)
    return false;
  is_busy_ = true;
  int sign = degree * speed > 0.0 ? 1 : -1;
  int64_t _degree = sign * static_cast<int64_t>(degree * 100);
  uint32_t _speed = static_cast<uint32_t>(std::abs(speed) * 100);
  cmder_->init(cmd::rmd::W_MLANG2).append(_degree).append(_speed).build();
  buffer_.clear();
  sender_->write(*cmder_);
  auto ret = sender_->read(buffer_, rev_len);
  is_busy_ = false;
  return ret == rev_len;
}

bool rmd_impl::more(double degree, double speed) {
  constexpr int rev_len = 13;
  if (is_busy_ || std::abs(speed) > cmd::rmd::MAX_SPEED)
    return false;
  is_busy_ = true;
  int32_t _degree = static_cast<int32_t>(degree * 100);
  uint32_t _speed = static_cast<uint32_t>(speed * 100);
  cmder_->init(cmd::rmd::W_ADDANG2).append(_degree).append(_speed).build();
  sender_->write(*cmder_);
  buffer_.clear();
  auto ret = sender_->read(buffer_, rev_len);
  is_busy_ = false;
  return ret == rev_len;
}

std::optional<double> rmd_impl::cur_pose() {
  constexpr int rev_len = 8;
  if (is_busy_)
    return {};
  is_busy_ = true;
  double angle = 600.0;
  if ((*cmder_)[cmd::rmd::IDX_CMD] != cmd::rmd::R_SINGALLOOPANG) {
    (*cmder_).init(cmd::rmd::R_SINGALLOOPANG).build();
  }
  sender_->write((*cmder_));
  buffer_.clear();
  auto ret = sender_->read(buffer_, rev_len);
  auto tmp = reinterpret_cast<uint16_t *>(&(buffer_[cmd::rmd::IDX_DATA_START]));
  angle = (static_cast<double>(*tmp)) / 100.0;
  is_busy_ = false;
  return ret == rev_len ? angle : std::optional<double>{};
}