#pragma once

#include "ks103_ultrasound/ks103_bus.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <array>
#include <memory>
#include <string>
#include <unordered_map>

namespace ks103_ultrasound {
struct SensorConfig { std::string name; uint8_t address; std::string frame; std::string topic; };

class Ks103Node : public rclcpp::Node {
public:
  explicit Ks103Node(const rclcpp::NodeOptions &options = rclcpp::NodeOptions(),
                     std::shared_ptr<IKs103Bus> bus = nullptr);
  ~Ks103Node() override;
  const std::array<SensorConfig, 4> &sensors() const { return sensors_; }
  void measure_next_for_test() { measure_next(); }

private:
  void measure_next();
  void publish_timing();
  std::shared_ptr<IKs103Bus> bus_;
  std::array<SensorConfig, 4> sensors_;
  std::unordered_map<std::string, size_t> indices_;
  std::vector<std::string> firing_order_;
  std::array<rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr, 4> publishers_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  size_t next_{0};
  double field_of_view_, min_range_, max_range_;
  std::chrono::milliseconds measurement_timeout_;
  std::chrono::steady_clock::time_point cycle_start_;
  double last_measurement_ms_{0.0}, average_measurement_ms_{0.0};
  double last_cycle_ms_{0.0}, average_cycle_ms_{0.0};
  uint64_t measurement_count_{0}, cycle_count_{0};
};
}  // namespace ks103_ultrasound
