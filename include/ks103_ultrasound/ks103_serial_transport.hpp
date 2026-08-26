#pragma once

#include "ks103_ultrasound/ks103_bus.hpp"
#include "ks103_ultrasound/ks103_rs485_protocol.hpp"
#include <functional>
#include <memory>

namespace ks103_ultrasound {

class ISerialIo {
public:
  virtual ~ISerialIo() = default;
  virtual bool discard_input() = 0;
  virtual bool write_byte(uint8_t value) = 0;
  virtual int read_some(uint8_t *data, size_t size,
    std::chrono::steady_clock::time_point deadline) = 0;
  virtual void close() = 0;
};

class PosixSerialIo final : public ISerialIo {
public:
  PosixSerialIo(const std::string &device, int baud, const std::string &parity,
    int stop_bits, bool kernel_rs485);
  ~PosixSerialIo() override;
  bool discard_input() override;
  bool write_byte(uint8_t value) override;
  int read_some(uint8_t *data, size_t size,
    std::chrono::steady_clock::time_point deadline) override;
  void close() override;
private:
  int fd_{-1};
};

class Ks103Rs485Transport final : public IKs103Bus {
public:
  Ks103Rs485Transport(std::shared_ptr<ISerialIo> io, uint8_t command = 0xB0,
    unsigned command_byte_delay_us = 50,
    std::function<void(std::chrono::microseconds)> sleeper = {});
  bool configure_noise_filter(uint8_t, uint8_t) override { return true; }
  std::optional<uint16_t> measure_mm(uint8_t address,
    std::chrono::milliseconds timeout) override;
  void close() override;
private:
  std::shared_ptr<ISerialIo> io_;
  uint8_t command_;
  unsigned delay_us_;
  std::function<void(std::chrono::microseconds)> sleeper_;
  std::mutex transaction_mutex_;
};

}  // namespace ks103_ultrasound
