#include "ks103_ultrasound/ks103_node.hpp"
#include "ks103_ultrasound/ks103_rs485_protocol.hpp"
#include "ks103_ultrasound/ks103_serial_transport.hpp"
#include <gtest/gtest.h>
#include <deque>

using namespace std::chrono_literals;
namespace ks103_ultrasound {
class FakeSerial final : public ISerialIo {
public:
  bool discard_input() override { ++flushes; return flush_ok; }
  bool write_byte(uint8_t value) override { writes.push_back(value); return true; }
  int read_some(uint8_t *data, size_t size, std::chrono::steady_clock::time_point) override {
    if (chunks.empty()) return 0;
    auto chunk = chunks.front(); chunks.pop_front();
    if (chunk.empty()) return 0;
    const auto count = std::min(size, chunk.size());
    std::copy_n(chunk.begin(), count, data); return static_cast<int>(count);
  }
  void close() override { closed = true; }
  std::vector<uint8_t> writes; std::deque<std::vector<uint8_t>> chunks;
  int flushes{0}; bool flush_ok{true}, closed{false};
};

TEST(Ks103Protocol, GoldenVectorAddressesCommandsAndUnits) {
  EXPECT_EQ(Ks103Rs485Protocol::build_measurement_request(0xE8),
    (std::array<uint8_t, 3>{0xE8, 0x02, 0xB0}));
  EXPECT_FALSE(Ks103Rs485Protocol::valid_address(0xCF));
  for (const auto excluded : {0xF0, 0xF2, 0xF4, 0xF6})
    EXPECT_FALSE(Ks103Rs485Protocol::valid_address(excluded));
  EXPECT_EQ(Ks103Rs485Protocol::parse_measurement_response(0xB0, 0x12, 0x34).raw, 0x1234);
  EXPECT_EQ(Ks103Rs485Protocol::result_unit(0xB4), Ks103ResultUnit::MILLIMETRES);
  EXPECT_EQ(Ks103Rs485Protocol::result_unit(0x2F), Ks103ResultUnit::MICROSECONDS);
  EXPECT_EQ(Ks103Rs485Protocol::result_unit(0xBA), Ks103ResultUnit::MICROSECONDS);
  EXPECT_THROW(Ks103Rs485Protocol::build_measurement_request(0xE8, 0xB1), std::invalid_argument);
}

TEST(Ks103Serial, PartialZeroByteCompleteTimeoutAndByteDelay) {
  auto serial = std::make_shared<FakeSerial>();
  serial->chunks = {{}, {0x04}, {0xD2}};
  std::vector<std::chrono::microseconds> delays;
  Ks103Rs485Transport transport(serial, 0xB0, 50,
    [&](auto delay) { delays.push_back(delay); });
  EXPECT_EQ(transport.measure_mm(0xD0, 5ms), 1234);
  EXPECT_EQ(serial->writes, (std::vector<uint8_t>{0xD0, 0x02, 0xB0}));
  EXPECT_EQ(delays, (std::vector<std::chrono::microseconds>{50us, 50us}));
  EXPECT_EQ(serial->flushes, 1);
  EXPECT_EQ(transport.measure_mm(0xD2, 0ms), std::nullopt);
  EXPECT_THROW(Ks103Rs485Transport(serial, 0xB0, 19), std::invalid_argument);
}

TEST(Ks103Serial, TravelTimeCommandIsNotMisreportedAsMillimetres) {
  auto serial = std::make_shared<FakeSerial>(); serial->chunks = {{0x00, 0x64}};
  Ks103Rs485Transport transport(serial, 0xB2, 50, [](auto) {});
  EXPECT_EQ(transport.measure_mm(0xD0, 5ms), std::nullopt);
}
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
