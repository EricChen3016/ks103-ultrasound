#include "ks103_ultrasound/ks103_bus.hpp"
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <stdexcept>
#include <thread>

namespace ks103_ultrasound {
Ks103I2cBus::Ks103I2cBus(const std::string &device) {
  fd_ = ::open(device.c_str(), O_RDWR | O_CLOEXEC);
  if (fd_ < 0) throw std::runtime_error("cannot open I2C device " + device);
}
Ks103I2cBus::~Ks103I2cBus() { close(); }

bool Ks103I2cBus::write_register_locked(uint8_t address, uint8_t reg, uint8_t value) {
  uint8_t bytes[]{reg, value};
  i2c_msg message{address, 0, 2, bytes};
  i2c_rdwr_ioctl_data transaction{&message, 1};
  return fd_ >= 0 && ::ioctl(fd_, I2C_RDWR, &transaction) >= 0;
}
bool Ks103I2cBus::read_register_locked(uint8_t address, uint8_t reg, uint8_t &value) {
  i2c_msg messages[]{{address, 0, 1, &reg}, {address, I2C_M_RD, 1, &value}};
  i2c_rdwr_ioctl_data transaction{messages, 2};
  return fd_ >= 0 && ::ioctl(fd_, I2C_RDWR, &transaction) >= 0;
}
bool Ks103I2cBus::configure_noise_filter(uint8_t address, uint8_t level) {
  std::lock_guard<std::mutex> lock(mutex_);
  return level <= 6 && write_register_locked(address, 2, static_cast<uint8_t>(0x69 + level));
}
std::optional<uint16_t> Ks103I2cBus::measure_mm(uint8_t address, std::chrono::milliseconds timeout) {
  std::lock_guard<std::mutex> lock(mutex_);  // Trigger and reads are one serialized transaction.
  constexpr auto conversion_time = std::chrono::milliseconds(100);
  if (!write_register_locked(address, 2, 0xbc) || timeout < conversion_time) return std::nullopt;
  std::this_thread::sleep_for(conversion_time);
  uint8_t high{}, low{};
  if (!read_register_locked(address, 2, high) || !read_register_locked(address, 3, low)) return std::nullopt;
  return static_cast<uint16_t>((static_cast<uint16_t>(high) << 8) | low);
}
void Ks103I2cBus::close() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
}
}  // namespace ks103_ultrasound
