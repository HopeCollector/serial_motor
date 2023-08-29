#pragma once
#include <vector>
#include <cstdint>
#include <functional>

namespace smotor {
class commander : public std::vector<uint8_t> {
public:
  commander(size_t s = 0, bool big_endian = false)
      : vector<uint8_t>(s), need_big_endian(big_endian) {
    union {
      uint32_t i;
      char c[4];
    } bint = {0x01020304};

    is_big_endian = bint.c[0] == 1;
  }

  template <typename T> commander &append(T data) {
    static auto reverse_endian = [&](T u) -> T {
      union {
        T u;
        unsigned char u8[sizeof(T)];
      } source, dest;

      source.u = u;

      for (size_t k = 0; k < sizeof(T); k++)
        dest.u8[k] = source.u8[sizeof(T) - k - 1];

      return dest.u;
    };

    if (is_big_endian != need_big_endian)
      data = reverse_endian(data);

    auto ptr = reinterpret_cast<uint8_t *>(&data);
    for (size_t i = 0; i < sizeof(T); i++) {
      push_back(*ptr);
      ptr++;
    }

    return *this;
  }

public:
  std::function<commander &(uint8_t)> init;
  std::function<commander &()> build;

private:
  bool is_big_endian;
  bool need_big_endian;
};
};