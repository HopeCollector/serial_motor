#pragma once
#include <optional>

namespace smotor {
class motor {
public:
  virtual ~motor(){};

  /**
   * @brief make the motor rotate with the specific speed
   * @param speed rotation speed (degree / s)
  */
  virtual bool rotate(double speed) = 0;

  /**
   * @brief stop motor's action immediately
   */
  virtual bool pause() = 0;

  /**
   * @brief to specific position(degree)
   * @param degree the degree want to achive
   * @param speed rotation speed (degree / s)
  */
  virtual bool to(double degree, double speed) = 0;

  /**
   * @brief more base on current position
   * @param degree the degree want to rotate more
   * @param speed rotation speed (degree / s)
   */
  virtual bool more(double degree, double speed) = 0;

  virtual std::optional<double> cur_pose() = 0;
};
} // namespace smotor