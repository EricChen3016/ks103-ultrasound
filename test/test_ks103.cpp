#include "ks103_ultrasound/ks103_node.hpp"
#include <gtest/gtest.h>
#include <deque>

using namespace std::chrono_literals;
namespace ks103_ultrasound {
class MockBus final : public IKs103Bus {
public:
  bool configure_noise_filter(uint8_t address, uint8_t) override { configured.push_back(address); return true; }
  std::optional<uint16_t> measure_mm(uint8_t address, std::chrono::milliseconds timeout) override {
    measured.push_back(address); timeouts.push_back(timeout);
    if (answers.empty()) return std::nullopt;
    auto answer = answers.front(); answers.pop_front(); return answer;
  }
  void close() override { closed = true; }
  std::vector<uint8_t> configured, measured;
  std::vector<std::chrono::milliseconds> timeouts;
  std::deque<std::optional<uint16_t>> answers;
  bool closed{false};
};

class Ks103Test : public testing::Test {
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }
  rclcpp::NodeOptions options(std::vector<rclcpp::Parameter> overrides = {}) {
    overrides.emplace_back("firing_interval_ms", 60000);
    return rclcpp::NodeOptions().parameter_overrides(overrides);
  }
};

TEST_F(Ks103Test, ParsesAddressesFramesAndFiringOrder) {
  auto bus = std::make_shared<MockBus>();
  auto node = std::make_shared<Ks103Node>(options({
    {"sensors.left_front.address", 0x61}, {"sensors.left_front.frame_id", "lf_link"},
    {"firing_order", std::vector<std::string>{"right_rear", "left_rear", "right_front", "left_front"}},
    {"measurement_timeout", 0.234}}), bus);
  EXPECT_EQ(node->sensors()[0].address, 0x61);
  EXPECT_EQ(node->sensors()[0].frame, "lf_link");
  for (int i = 0; i < 4; ++i) node->measure_next_for_test();
  EXPECT_EQ(bus->measured, (std::vector<uint8_t>{0x77, 0x76, 0x75, 0x61}));
  EXPECT_TRUE(std::all_of(bus->timeouts.begin(), bus->timeouts.end(), [](auto t) { return t == 234ms; }));
}

TEST_F(Ks103Test, PublishesRangeMetadataWithNonzeroStamp) {
  auto bus = std::make_shared<MockBus>(); bus->answers.push_back(1234);
  auto node = std::make_shared<Ks103Node>(options({
    {"field_of_view", 0.4}, {"min_range", 0.1}, {"max_range", 5.0},
    {"sensors.left_front.frame_id", "lf_sensor"}}), bus);
  auto listener = std::make_shared<rclcpp::Node>("listener");
  sensor_msgs::msg::Range::SharedPtr received;
  auto sub = listener->create_subscription<sensor_msgs::msg::Range>(
    "/ultrasound/left_front", 10, [&](sensor_msgs::msg::Range::SharedPtr msg) { received = msg; });
  node->measure_next_for_test();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node); executor.add_node(listener);
  for (int i = 0; i < 10 && !received; ++i) executor.spin_some(10ms);
  ASSERT_TRUE(received);
  EXPECT_EQ(received->radiation_type, sensor_msgs::msg::Range::ULTRASOUND);
  EXPECT_NE(received->header.stamp.nanosec + received->header.stamp.sec, 0u);
  EXPECT_EQ(received->header.frame_id, "lf_sensor");
  EXPECT_FLOAT_EQ(received->field_of_view, 0.4F);
  EXPECT_FLOAT_EQ(received->min_range, 0.1F); EXPECT_FLOAT_EQ(received->max_range, 5.0F);
  EXPECT_FLOAT_EQ(received->range, 1.234F);
}

TEST_F(Ks103Test, TimeoutAndReadFailureDoNotPublish) {
  auto bus = std::make_shared<MockBus>();
  bus->answers.push_back(std::nullopt); bus->answers.push_back(std::nullopt);
  auto node = std::make_shared<Ks103Node>(options({{"measurement_timeout", 0.05}}), bus);
  auto listener = std::make_shared<rclcpp::Node>("failure_listener");
  int publications = 0;
  auto sub = listener->create_subscription<sensor_msgs::msg::Range>(
    "/ultrasound/left_front", 10, [&](sensor_msgs::msg::Range::SharedPtr) { ++publications; });
  node->measure_next_for_test();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node); executor.add_node(listener); executor.spin_some(20ms);
  EXPECT_EQ(publications, 0); EXPECT_EQ(bus->timeouts.front(), 50ms);
}

TEST_F(Ks103Test, RejectsInvalidAddressAndDuplicateOrder) {
  auto bus = std::make_shared<MockBus>();
  EXPECT_THROW(Ks103Node(options({{"sensors.left_front.address", 0x78}}), bus), std::invalid_argument);
  EXPECT_THROW(Ks103Node(options({{"firing_order", std::vector<std::string>{"left_front", "left_front", "left_rear", "right_rear"}}}), bus), std::invalid_argument);
}
}  // namespace ks103_ultrasound
