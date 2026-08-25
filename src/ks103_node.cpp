#include "ks103_ultrasound/ks103_node.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace ks103_ultrasound {
Ks103Node::Ks103Node(const rclcpp::NodeOptions &options, std::shared_ptr<IKs103Bus> bus)
: Node("ks103_ultrasound", options) {
  const auto device = declare_parameter<std::string>("i2c_device", "/dev/i2c-1");
  const std::array<std::string, 4> names{"left_front", "right_front", "left_rear", "right_rear"};
  const std::array<int64_t, 4> defaults{0x74, 0x75, 0x76, 0x77};
  const std::array<std::string, 4> default_frames{"ultrasound_left_front", "ultrasound_right_front", "ultrasound_left_rear", "ultrasound_right_rear"};
  for (size_t i = 0; i < names.size(); ++i) {
    const auto address = declare_parameter<int64_t>("sensors." + names[i] + ".address", defaults[i]);
    if (address < 0x03 || address > 0x77) throw std::invalid_argument("I2C address out of range for " + names[i]);
    sensors_[i] = {names[i], static_cast<uint8_t>(address),
      declare_parameter<std::string>("sensors." + names[i] + ".frame_id", default_frames[i]),
      "/ultrasound/" + names[i]};
    indices_[names[i]] = i;
  }
  std::array<uint8_t, 4> addresses{sensors_[0].address, sensors_[1].address, sensors_[2].address, sensors_[3].address};
  auto unique_addresses = addresses; std::sort(unique_addresses.begin(), unique_addresses.end());
  if (std::adjacent_find(unique_addresses.begin(), unique_addresses.end()) != unique_addresses.end())
    throw std::invalid_argument("sensor I2C addresses must be unique");
  firing_order_ = declare_parameter<std::vector<std::string>>(
    "firing_order", {"left_front", "right_front", "left_rear", "right_rear"});
  if (firing_order_.size() != 4) throw std::invalid_argument("firing_order must contain four sensors");
  auto sorted = firing_order_; std::sort(sorted.begin(), sorted.end());
  auto expected = std::vector<std::string>(names.begin(), names.end()); std::sort(expected.begin(), expected.end());
  if (sorted != expected) throw std::invalid_argument("firing_order must contain each sensor exactly once");
  field_of_view_ = declare_parameter<double>("field_of_view", 0.52);
  min_range_ = declare_parameter<double>("min_range", 0.02);
  max_range_ = declare_parameter<double>("max_range", 11.28);
  measurement_timeout_ = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::duration<double>(declare_parameter<double>("measurement_timeout", 0.15)));
  const auto cycle_period = std::chrono::milliseconds(declare_parameter<int64_t>("firing_interval_ms", 125));
  const auto noise = declare_parameter<int64_t>("noise_filtering", 1);
  if (measurement_timeout_.count() <= 0 || cycle_period.count() <= 0 || min_range_ < 0 || max_range_ <= min_range_ || field_of_view_ <= 0)
    throw std::invalid_argument("invalid timing or range metadata parameter");
  if (noise < 0 || noise > 6) throw std::invalid_argument("noise_filtering must be between 0 and 6");
  bus_ = bus ? std::move(bus) : std::make_shared<Ks103I2cBus>(device);
  for (const auto &sensor : sensors_) {
    publishers_[indices_.at(sensor.name)] = create_publisher<sensor_msgs::msg::Range>(sensor.topic, 10);
    if (!bus_->configure_noise_filter(sensor.address, static_cast<uint8_t>(noise)))
      RCLCPP_WARN(get_logger(), "Could not configure sensor %s", sensor.name.c_str());
  }
  timer_ = create_wall_timer(cycle_period, [this] { measure_next(); });
}
Ks103Node::~Ks103Node() { if (timer_) timer_->cancel(); if (bus_) bus_->close(); }
void Ks103Node::measure_next() {
  const auto index = indices_.at(firing_order_[next_]);
  next_ = (next_ + 1) % firing_order_.size();
  const auto distance = bus_->measure_mm(sensors_[index].address, measurement_timeout_);
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
}  // namespace ks103_ultrasound
