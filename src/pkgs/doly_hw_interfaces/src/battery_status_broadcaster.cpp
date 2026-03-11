#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <limits>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <ros2_fmt_logger/ros2_fmt_logger.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <string>

namespace battery_status_broadcaster
{

using namespace std::chrono_literals;

class BatteryStatusBroadcaster : public rclcpp::Node
{
public:
  explicit BatteryStatusBroadcaster(const rclcpp::NodeOptions & options = rclcpp::NodeOptions{})
  : rclcpp::Node("battery_status_broadcaster", options), logger_(this->get_logger())
  {
    current_path_ =
      this->declare_parameter<std::string>("current_path", "/sys/class/hwmon/hwmon1/curr1_input");
    bus_voltage_path_ =
      this->declare_parameter<std::string>("bus_voltage_path", "/sys/class/hwmon/hwmon1/in1_input");
    shunt_voltage_path_ = this->declare_parameter<std::string>(
      "shunt_voltage_path", "/sys/class/hwmon/hwmon1/in0_input");

    publisher_ = this->create_publisher<sensor_msgs::msg::BatteryState>("battery_status", 1);
    timer_ =
      this->create_wall_timer(1s, std::bind(&BatteryStatusBroadcaster::publishBatteryState, this));

    logger_.info("BatteryStatusBroadcaster started");
  }

private:
  static bool readDoubleFromFile(const std::string & path, double & value)
  {
    std::ifstream stream(path);
    if (!stream.is_open()) {
      return false;
    }

    stream >> value;
    return !stream.fail();
  }

  void publishBatteryState()
  {
    double current_na = 0.0;
    double bus_voltage_mv = 0.0;
    double shunt_voltage_mv = 0.0;

    const bool has_current = readDoubleFromFile(current_path_, current_na);
    const bool has_bus_voltage = readDoubleFromFile(bus_voltage_path_, bus_voltage_mv);
    const bool has_shunt_voltage = readDoubleFromFile(shunt_voltage_path_, shunt_voltage_mv);

    sensor_msgs::msg::BatteryState message;
    message.header.stamp = this->now();
    message.header.frame_id = "battery";
    message.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN;
    message.power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;
    message.power_supply_technology = sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_LION;
    message.design_capacity = std::numeric_limits<float>::quiet_NaN();
    message.charge = std::numeric_limits<float>::quiet_NaN();
    message.capacity = std::numeric_limits<float>::quiet_NaN();
    message.temperature = std::numeric_limits<float>::quiet_NaN();

    if (!has_current || !has_bus_voltage || !has_shunt_voltage) {
      message.present = false;
      message.voltage = std::numeric_limits<float>::quiet_NaN();
      message.current = std::numeric_limits<float>::quiet_NaN();
      message.percentage = std::numeric_limits<float>::quiet_NaN();
      message.cell_voltage.clear();

      logger_.warn_throttle(
        1s, "Failed to read battery telemetry from sysfs paths: current=%s, bus=%s, shunt=%s",
        current_path_.c_str(), bus_voltage_path_.c_str(), shunt_voltage_path_.c_str());

      publisher_->publish(message);
      return;
    }

    const double bus_voltage_v = bus_voltage_mv / 1000.0;
    const double shunt_voltage_v = shunt_voltage_mv / 1000.0;
    const double current_a = current_na / 1000000000.0;

    // Doly-provided capacity heuristic: 4.1 V ~= full, 3.0 V ~= empty.
    const double capacity_percent = std::clamp(((bus_voltage_v - 3.0) / 1.1) * 100.0, 0.0, 100.0);

    message.present = true;
    message.voltage = static_cast<float>(bus_voltage_v);
    message.current = static_cast<float>(current_a);
    message.percentage = static_cast<float>(capacity_percent / 100.0);
    message.cell_voltage = {static_cast<float>(bus_voltage_v), static_cast<float>(shunt_voltage_v)};

    publisher_->publish(message);
  }

  ros2_fmt_logger::Logger logger_;

  std::string current_path_;
  std::string bus_voltage_path_;
  std::string shunt_voltage_path_;

  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace battery_status_broadcaster

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<battery_status_broadcaster::BatteryStatusBroadcaster>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return EXIT_SUCCESS;
}