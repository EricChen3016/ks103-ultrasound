#include "ks103_ultrasound/ks103_node.hpp"
#include "ks103_ultrasound/ks103_rs485_protocol.hpp"
#include "ks103_ultrasound/ks103_serial_transport.hpp"
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace ks103_ultrasound {
Ks103Node::Ks103Node(const rclcpp::NodeOptions &options, std::shared_ptr<IKs103Bus> bus)
: Node("ks103_ultrasound", options) {
  const auto transport = declare_parameter<std::string>("transport", "i2c");
  const auto device = declare_parameter<std::string>("i2c_device", "/dev/i2c-1");
  const std::array<std::string, 4> names{"left_front", "right_front", "left_rear", "right_rear"};
  const bool serial_transport = transport == "rs485_usb" || transport == "rs485_uart";
  if (!serial_transport && transport != "i2c") throw std::invalid_argument("transport must be i2c, rs485_usb, or rs485_uart");
  const std::array<int64_t, 4> i2c_defaults{0x74, 0x75, 0x76, 0x77};
  const std::array<int64_t, 4> rs485_defaults{0xD0, 0xD2, 0xD4, 0xD6};
  const std::array<std::string, 4> default_frames{"ultrasound_left_front", "ultrasound_right_front", "ultrasound_left_rear", "ultrasound_right_rear"};
  for (size_t i = 0; i < names.size(); ++i) {
    const auto address = declare_parameter<int64_t>("sensors." + names[i] + ".address", serial_transport ? rs485_defaults[i] : i2c_defaults[i]);
    if (address < 0 || address > 255 || (serial_transport && !Ks103Rs485Protocol::valid_address(static_cast<uint8_t>(address))) ||
        (!serial_transport && (address < 0x03 || address > 0x77)))
      throw std::invalid_argument("address out of range for " + names[i]);
    sensors_[i] = {names[i], static_cast<uint8_t>(address),
      declare_parameter<std::string>("sensors." + names[i] + ".frame_id", default_frames[i]),
      declare_parameter<std::string>("sensors." + names[i] + ".topic", "/ultrasound/" + names[i])};
    indices_[names[i]] = i;
  }
  std::array<uint8_t, 4> addresses{sensors_[0].address, sensors_[1].address, sensors_[2].address, sensors_[3].address};
  auto unique_addresses = addresses; std::sort(unique_addresses.begin(), unique_addresses.end());
  if (std::adjacent_find(unique_addresses.begin(), unique_addresses.end()) != unique_addresses.end())
    throw std::invalid_argument("sensor addresses must be unique");
  const auto legacy_order = declare_parameter<std::vector<std::string>>(
    "firing_order", {"left_front", "right_front", "left_rear", "right_rear"});
  firing_order_ = declare_parameter<std::vector<std::string>>("poll_order", legacy_order);
  if (firing_order_.size() != 4) throw std::invalid_argument("firing_order must contain four sensors");
  auto sorted = firing_order_; std::sort(sorted.begin(), sorted.end());
  auto expected = std::vector<std::string>(names.begin(), names.end()); std::sort(expected.begin(), expected.end());
  if (sorted != expected) throw std::invalid_argument("firing_order must contain each sensor exactly once");
  field_of_view_ = declare_parameter<double>("field_of_view", 0.52);
  min_range_ = declare_parameter<double>("min_range", 0.02);
  max_range_ = declare_parameter<double>("max_range", 11.28);
  const auto legacy_timeout = declare_parameter<double>("measurement_timeout", 0.15);
  measurement_timeout_ = std::chrono::milliseconds(
    declare_parameter<int64_t>("response_timeout_ms", static_cast<int64_t>(legacy_timeout * 1000.0)));
  const auto cycle_period = std::chrono::milliseconds(declare_parameter<int64_t>("firing_interval_ms", 125));
  const auto noise = declare_parameter<int64_t>("noise_filtering", 1);
  if (measurement_timeout_.count() <= 0 || cycle_period.count() <= 0 || min_range_ < 0 || max_range_ <= min_range_ || field_of_view_ <= 0)
    throw std::invalid_argument("invalid timing or range metadata parameter");
  if (noise < 0 || noise > 6) throw std::invalid_argument("noise_filtering must be between 0 and 6");
  const auto timestamp_mode = declare_parameter<std::string>("timestamp_mode", "response_complete");
  if (timestamp_mode != "response_complete") throw std::invalid_argument("only response_complete timestamp_mode is supported");
  if (bus) bus_ = std::move(bus);
  else if (!serial_transport) bus_ = std::make_shared<Ks103I2cBus>(device);
  else {
    const auto serial_device = declare_parameter<std::string>("serial.device", "/dev/ttyUSB0");
    const auto baud = declare_parameter<int64_t>("serial.baud", 9600);
    const auto parity = declare_parameter<std::string>("serial.parity", "none");
    const auto stop_bits = declare_parameter<int64_t>("serial.stop_bits", 1);
    const auto command = declare_parameter<int64_t>("measurement_command", 0xB0);
    const auto delay = declare_parameter<int64_t>("command_byte_delay_us", 50);
    if (command < 0 || command > 255 || delay < 0) throw std::invalid_argument("invalid serial command setting");
    auto io = std::make_shared<PosixSerialIo>(serial_device, static_cast<int>(baud), parity,
      static_cast<int>(stop_bits), transport == "rs485_uart");
    bus_ = std::make_shared<Ks103Rs485Transport>(io, static_cast<uint8_t>(command), static_cast<unsigned>(delay));
  }
  diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("diagnostics", 10);
  for (const auto &sensor : sensors_) {
    publishers_[indices_.at(sensor.name)] = create_publisher<sensor_msgs::msg::Range>(sensor.topic, 10);
    if (!bus_->configure_noise_filter(sensor.address, static_cast<uint8_t>(noise)))
      RCLCPP_WARN(get_logger(), "Could not configure sensor %s", sensor.name.c_str());
  }
  timer_ = create_wall_timer(cycle_period, [this] { measure_next(); });
}
Ks103Node::~Ks103Node() { if (timer_) timer_->cancel(); if (bus_) bus_->close(); }
void Ks103Node::measure_next() {
  if (next_ == 0) cycle_start_ = std::chrono::steady_clock::now();
  const auto index = indices_.at(firing_order_[next_]);
  next_ = (next_ + 1) % firing_order_.size();
  const auto transaction_start = std::chrono::steady_clock::now();
  const auto distance = bus_->measure_mm(sensors_[index].address, measurement_timeout_);
  const auto response_complete = std::chrono::steady_clock::now();
  last_measurement_ms_ = std::chrono::duration<double, std::milli>(response_complete - transaction_start).count();
  ++measurement_count_; average_measurement_ms_ += (last_measurement_ms_ - average_measurement_ms_) / measurement_count_;
  if (next_ == 0) {
    last_cycle_ms_ = std::chrono::duration<double, std::milli>(response_complete - cycle_start_).count();
    ++cycle_count_; average_cycle_ms_ += (last_cycle_ms_ - average_cycle_ms_) / cycle_count_;
  }
  publish_timing();
  if (!distance) { RCLCPP_WARN(get_logger(), "No valid measurement from %s", sensors_[index].name.c_str()); return; }
  sensor_msgs::msg::Range msg;
  msg.header.stamp = now(); msg.header.frame_id = sensors_[index].frame;
  msg.radiation_type = sensor_msgs::msg::Range::ULTRASOUND;
  msg.field_of_view = static_cast<float>(field_of_view_);
  msg.min_range = static_cast<float>(min_range_); msg.max_range = static_cast<float>(max_range_);
  msg.range = static_cast<float>(*distance) / 1000.0F;
  if (!std::isfinite(msg.range) || msg.range < msg.min_range || msg.range > msg.max_range) return;
  publishers_[index]->publish(msg);
}

void Ks103Node::publish_timing() {
  diagnostic_msgs::msg::DiagnosticArray array; array.header.stamp = now();
  diagnostic_msgs::msg::DiagnosticStatus status; status.name = "ks103_poll_timing";
  status.hardware_id = "ks103"; status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  const auto add = [&status](const char *key, double value) {
    diagnostic_msgs::msg::KeyValue item; item.key = key; item.value = std::to_string(value); status.values.push_back(item);
  };
  add("last_measurement_duration_ms", last_measurement_ms_);
  add("average_measurement_duration_ms", average_measurement_ms_);
  add("last_full_poll_cycle_ms", last_cycle_ms_);
  add("average_full_poll_cycle_ms", average_cycle_ms_);
  add("effective_sensor_rate_hz", average_cycle_ms_ > 0.0 ? 1000.0 / average_cycle_ms_ : 0.0);
  array.status.push_back(status); diagnostics_pub_->publish(array);
}
}  // namespace ks103_ultrasound
