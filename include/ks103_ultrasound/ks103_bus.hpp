#pragma once

#include <chrono>
#include <cstdint>
#include <mutex>
#include <optional>
#include <string>

namespace ks103_ultrasound {

class IKs103Bus {
public:
  virtual ~IKs103Bus() = default;
  virtual bool configure_noise_filter(uint8_t address, uint8_t level) = 0;
  virtual std::optional<uint16_t> measure_mm(
    uint8_t address, std::chrono::milliseconds timeout) = 0;
  virtual void close() = 0;
};

using IKs103Transport = IKs103Bus;

class Ks103I2cBus final : public IKs103Bus {
public:
  explicit Ks103I2cBus(const std::string &device);
  ~Ks103I2cBus() override;
  bool configure_noise_filter(uint8_t address, uint8_t level) override;
  std::optional<uint16_t> measure_mm(
    uint8_t address, std::chrono::milliseconds timeout) override;
  void close() override;

private:
  bool write_register_locked(uint8_t address, uint8_t reg, uint8_t value);
  bool read_register_locked(uint8_t address, uint8_t reg, uint8_t &value);
  std::mutex mutex_;
  int fd_{-1};
};
}  // namespace ks103_ultrasound
