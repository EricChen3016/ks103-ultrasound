#include "ks103_ultrasound/ks103_serial_transport.hpp"
#include <asm-generic/ioctls.h>
#include <fcntl.h>
#include <linux/serial.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>
#include <cerrno>
#include <stdexcept>
#include <thread>

namespace ks103_ultrasound {
namespace {
speed_t baud_flag(int baud) {
  switch (baud) { case 9600: return B9600; case 19200: return B19200;
    case 38400: return B38400; case 57600: return B57600; case 115200: return B115200;
    default: throw std::invalid_argument("unsupported serial baud"); }
}
}

PosixSerialIo::PosixSerialIo(const std::string &device, int baud,
  const std::string &parity, int stop_bits, bool kernel_rs485) {
  if (parity != "none" && parity != "even" && parity != "odd")
    throw std::invalid_argument("serial parity must be none, even, or odd");
  if (stop_bits != 1 && stop_bits != 2) throw std::invalid_argument("stop_bits must be 1 or 2");
  fd_ = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
  if (fd_ < 0) throw std::runtime_error("cannot open serial device " + device);
  termios tty{};
  if (tcgetattr(fd_, &tty) != 0) { close(); throw std::runtime_error("tcgetattr failed"); }
  cfmakeraw(&tty); const auto speed = baud_flag(baud);
  cfsetispeed(&tty, speed); cfsetospeed(&tty, speed);
  tty.c_cflag = (tty.c_cflag & ~(CSIZE | PARENB | PARODD | CSTOPB)) | CS8 | CLOCAL | CREAD;
  if (parity != "none") tty.c_cflag |= PARENB;
  if (parity == "odd") tty.c_cflag |= PARODD;
  if (stop_bits == 2) tty.c_cflag |= CSTOPB;
  if (tcsetattr(fd_, TCSANOW, &tty) != 0) { close(); throw std::runtime_error("tcsetattr failed"); }
  if (kernel_rs485) {
    serial_rs485 config{}; config.flags = SER_RS485_ENABLED | SER_RS485_RTS_ON_SEND;
    if (ioctl(fd_, TIOCSRS485, &config) != 0) { close(); throw std::runtime_error("TIOCSRS485 unsupported or failed"); }
  }
}
PosixSerialIo::~PosixSerialIo() { close(); }
bool PosixSerialIo::discard_input() { return fd_ >= 0 && tcflush(fd_, TCIFLUSH) == 0; }
bool PosixSerialIo::write_byte(uint8_t value) { return fd_ >= 0 && ::write(fd_, &value, 1) == 1; }
int PosixSerialIo::read_some(uint8_t *data, size_t size,
  std::chrono::steady_clock::time_point deadline) {
  const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(deadline - std::chrono::steady_clock::now());
  if (remaining.count() < 0) return 0;
  pollfd pfd{fd_, POLLIN, 0};
  const int ready = ::poll(&pfd, 1, static_cast<int>(remaining.count() + 1));
  if (ready <= 0) return ready;
  const auto count = ::read(fd_, data, size);
  if (count < 0 && (errno == EAGAIN || errno == EINTR)) return 0;
  return static_cast<int>(count);
}
void PosixSerialIo::close() { if (fd_ >= 0) { ::close(fd_); fd_ = -1; } }

Ks103Rs485Transport::Ks103Rs485Transport(std::shared_ptr<ISerialIo> io,
  uint8_t command, unsigned delay, std::function<void(std::chrono::microseconds)> sleeper)
: io_(std::move(io)), command_(command), delay_us_(delay), sleeper_(std::move(sleeper)) {
  (void)Ks103Rs485Protocol::result_unit(command_);
  if (delay_us_ < 20 || delay_us_ > 100) throw std::invalid_argument("command_byte_delay_us must be 20..100");
  if (!sleeper_) sleeper_ = [](auto duration) { std::this_thread::sleep_for(duration); };
}
std::optional<uint16_t> Ks103Rs485Transport::measure_mm(uint8_t address,
  std::chrono::milliseconds timeout) {
  std::lock_guard<std::mutex> lock(transaction_mutex_);
  const auto request = Ks103Rs485Protocol::build_measurement_request(address, command_);
  if (!io_->discard_input()) return std::nullopt;
  for (size_t i = 0; i < request.size(); ++i) {
    if (!io_->write_byte(request[i])) return std::nullopt;
    if (i + 1 < request.size()) sleeper_(std::chrono::microseconds(delay_us_));
  }
  uint8_t response[2]{}; size_t received = 0;
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (received < 2 && std::chrono::steady_clock::now() < deadline) {
    const int count = io_->read_some(response + received, 2 - received, deadline);
    if (count < 0) return std::nullopt;
    received += static_cast<size_t>(count);
  }
  if (received != 2) return std::nullopt;
  const auto parsed = Ks103Rs485Protocol::parse_measurement_response(command_, response[0], response[1]);
  if (parsed.unit != Ks103ResultUnit::MILLIMETRES) return std::nullopt;
  return parsed.raw;
}
void Ks103Rs485Transport::close() { std::lock_guard<std::mutex> lock(transaction_mutex_); io_->close(); }

}  // namespace ks103_ultrasound
