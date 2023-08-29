#include "serial_motor/RMDS.h"
#include "serial/serial.h"

using namespace Motor;

RMDS::RMDS(std::string dev, uint8_t id)
    : m_sender(new serial::Serial{dev, 115200, serial::Timeout::simpleTimeout(50)}), m_id(id),
      Motor(180.0, 100, 100) {
  m_snd.init = [&](uint8_t cmd) -> CMD::Command & {
    m_snd.resize(CMD::RMDS::FRAME_LEN);
    m_snd[CMD::RMDS::IDX_FRAME_HEAD] = CMD::RMDS::FRAME_HEAD;
    m_snd[CMD::RMDS::IDX_CMD] = cmd;
    m_snd[CMD::RMDS::IDX_ID] = m_id;
    m_snd[CMD::RMDS::IDX_HEAD_CHK] = 0;
    return m_snd;
  };

  m_snd.build = [&]() -> CMD::Command & {
    if (m_snd.size() > CMD::RMDS::FRAME_LEN) {
      m_snd.push_back(0);
      m_snd[CMD::RMDS::IDX_DATA_LEN] = m_snd.size() - CMD::RMDS::FRAME_LEN - 1;
      for (size_t i = 0; i < m_snd[CMD::RMDS::IDX_DATA_LEN]; i++) {
        m_snd.back() += m_snd[CMD::RMDS::IDX_DATA_START + i];
      }
    } else
      m_snd[CMD::RMDS::IDX_DATA_LEN] = 0;

    for (size_t i = 0; i < CMD::RMDS::IDX_HEAD_CHK; i++) {
      m_snd[CMD::RMDS::IDX_HEAD_CHK] += m_snd[i];
    }
    return m_snd;
  };

  if (!m_sender->isOpen()) {
    try {
      m_sender->open();
    } catch (serial::IOException &e) {
      std::cerr << "Unable to open port " << std::endl;
      exit(-1);
    }
  }

  // find motor
  {}
}

RMDS::~RMDS() {
  if (m_sender->isOpen()) {
    m_sender->close();
  }
}

bool RMDS::rotate(double speed) {
  if (is_busy || speed > CMD::RMDS::MAX_SPEED)
    return false;

  is_busy = true;

  const int rev_len = 13;
  updateSpeed(speed);
  int32_t _speed = static_cast<int32_t>(speed * 100);

  m_snd.init(CMD::RMDS::W_SPEED).append(_speed).build();

  m_sender->write(m_snd);

  m_rev.clear();
  auto ret = m_sender->read(m_rev, rev_len);
  is_busy = false;
  return ret == rev_len;
}

bool RMDS::pause() {
  m_snd.init(CMD::RMDS::MOTOR_PAUSE).build();
  m_sender->write(m_snd);
  m_rev.clear();
  auto rev = m_sender->read(m_rev, 5);
  return rev == 5;
}

bool RMDS::rotateTo(double degree, double speed) {
  if (is_busy || speed > CMD::RMDS::MAX_SPEED || speed == 0 || degree < 0 ||
      degree > 359.99)
    return false;

  is_busy = true;

  const int rev_len = 13;
  updateSpeed(speed);
  uint32_t _speed = static_cast<uint32_t>(std::abs(speed) * 100);
  uint16_t _degree = static_cast<uint16_t>(degree * 100);
  uint8_t direction = speed < 0;

  m_snd.init(CMD::RMDS::W_SLANG2)
      .append(direction)
      .append(_degree)
      .append(static_cast<uint8_t>(0))
      .append(_speed)
      .build();

  m_sender->write(m_snd);
  m_rev.clear();
  auto ret = m_sender->read(m_rev, rev_len);
  is_busy = false;
  return ret == rev_len;
}

bool RMDS::rotateMore(double degree, double speed) {
  if (is_busy || speed > CMD::RMDS::MAX_SPEED)
    return false;

  is_busy = true;

  const int rev_len = 13;
  updateSpeed(speed);
  int32_t _degree = static_cast<int32_t>(degree * 100);
  uint32_t _speed = static_cast<uint32_t>(speed * 100);

  m_snd.init(CMD::RMDS::W_ADDANG2).append(_degree).append(_speed).build();

  m_rev.clear();
  m_sender->write(m_snd);
  auto ret = m_sender->read(m_rev, rev_len);
  is_busy = false;
  return ret == rev_len;
}

double RMDS::getCurrentPose() {
  if (is_busy)
    return 400.0;
  is_busy = true;

  const int rev_len = 8;
  double angle = 600.0;

  if (m_snd[CMD::RMDS::IDX_CMD] != CMD::RMDS::R_SINGALLOOPANG) {
    m_snd.init(CMD::RMDS::R_SINGALLOOPANG).build();
  }

  auto wt_len = m_sender->write(m_snd);
  if (wt_len != m_snd.size()) {
    std::cerr << "ERROR: Only write " << wt_len << " bytes" << std::endl;
  }

  m_rev.clear();
  auto ret = m_sender->read(m_rev, rev_len);
  if (ret == rev_len) {
    auto tmp = reinterpret_cast<uint16_t *>(&m_rev[CMD::RMDS::IDX_DATA_START]);
    angle = (static_cast<double>(*tmp)) / 100.0;
  } else {
    std::cerr << "ERROR: Only read " << ret << " bytes" << std::endl;
  }
  is_busy = false;
  return angle;
}

bool RMDS::rotateMTo(double degree, double speed) {
  if (is_busy || speed > CMD::RMDS::MAX_SPEED || speed == 0)
    return false;

  is_busy = true;

  const int rev_len = 13;
  updateSpeed(speed);
  uint32_t _speed = static_cast<uint32_t>(std::abs(speed) * 100);
  int64_t _degree = static_cast<int64_t>(degree * 100);

  m_snd.init(CMD::RMDS::W_MLANG2).append(_degree).append(_speed).build();

  m_rev.clear();
  m_sender->write(m_snd);
  auto ret = m_sender->read(m_rev, rev_len);
  is_busy = false;
  return ret == rev_len;
}

bool RMDS::rotateMMore(double degree, double speed) {
  return rotateMore(degree, speed);
}

double RMDS::getCurrentMPose() {
  if (is_busy)
    return -1.0;
  is_busy = true;

  const int rev_len = 14;
  double angle = -2.0;

  if (m_snd[CMD::RMDS::IDX_CMD] != CMD::RMDS::R_MULTILOOPANG) {
    m_snd.init(CMD::RMDS::R_MULTILOOPANG).build();
  }

  m_rev.clear();
  auto wt_len = m_sender->write(m_snd);
  if (wt_len != m_snd.size()) {
    std::cerr << "ERROR: Only write " << wt_len << " bytes" << std::endl;
  }

  auto ret = m_sender->read(m_rev, rev_len);
  if (ret == rev_len) {
    auto tmp = reinterpret_cast<int64_t *>(&m_rev[CMD::RMDS::IDX_DATA_START]);
    angle = (static_cast<double>(*tmp)) / 100.0;
  } else {
    std::cerr << "ERROR: Only read " << ret << " bytes" << std::endl;
  }
  is_busy = false;
  return angle;
}