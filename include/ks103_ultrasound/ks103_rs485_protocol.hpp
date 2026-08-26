#pragma once

#include <array>
#include <cstdint>
#include <stdexcept>

namespace ks103_ultrasound {

enum class Ks103ResultUnit { MILLIMETRES, MICROSECONDS };

struct Ks103ParsedMeasurement {
  uint16_t raw;
  Ks103ResultUnit unit;
};

class Ks103Rs485Protocol {
public:
  static constexpr uint8_t kRegister = 0x02;
  static bool valid_address(uint8_t address);
  static bool supported_command(uint8_t command);
  static Ks103ResultUnit result_unit(uint8_t command);
  static std::array<uint8_t, 3> build_measurement_request(
    uint8_t address, uint8_t command = 0xB0);
  static Ks103ParsedMeasurement parse_measurement_response(
    uint8_t command, uint8_t high, uint8_t low);
};

}  // namespace ks103_ultrasound
