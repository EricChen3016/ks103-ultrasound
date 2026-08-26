#include "ks103_ultrasound/ks103_rs485_protocol.hpp"

namespace ks103_ultrasound {

bool Ks103Rs485Protocol::valid_address(uint8_t address) {
  return address >= 0xD0 && address <= 0xFE && address != 0xF0 &&
         address != 0xF2 && address != 0xF4 && address != 0xF6;
}

bool Ks103Rs485Protocol::supported_command(uint8_t command) {
  return (command >= 0x01 && command <= 0x2F) || command == 0xB0 ||
         command == 0xB2 || command == 0xB4 || command == 0xB8 ||
         command == 0xBA || command == 0xBC;
}

Ks103ResultUnit Ks103Rs485Protocol::result_unit(uint8_t command) {
  if (!supported_command(command)) throw std::invalid_argument("unsupported KS103 command");
  return (command <= 0x2F || command == 0xB2 || command == 0xBA) ?
    Ks103ResultUnit::MICROSECONDS : Ks103ResultUnit::MILLIMETRES;
}

std::array<uint8_t, 3> Ks103Rs485Protocol::build_measurement_request(
  uint8_t address, uint8_t command) {
  if (!valid_address(address)) throw std::invalid_argument("invalid KS103 RS485 address");
  (void)result_unit(command);
  return {address, kRegister, command};
}

Ks103ParsedMeasurement Ks103Rs485Protocol::parse_measurement_response(
  uint8_t command, uint8_t high, uint8_t low) {
  return {static_cast<uint16_t>((static_cast<uint16_t>(high) << 8) | low),
          result_unit(command)};
}

}  // namespace ks103_ultrasound
