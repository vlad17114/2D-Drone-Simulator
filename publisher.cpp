#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <regex>
#include <sstream>
#include <iostream>


#include "rclcpp/rclcpp.hpp"
#include "px4_msgs/msg/goto_setpoint.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/offboard_control_mode.hpp"

using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher()
  : Node("publish_xyz"), count_(0)
  {
    std::string input;
    std::vector<float> coordinates;

    // Read input line
    std::getline(std::cin, input);

    // Call the parsing function
    if (parseThreeFloats(input, coordinates)) {
      std::cout << "Parsed numbers: " << coordinates[0] << ", " << coordinates[1] << ", " << coordinates[2] << std::endl;
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to parse input. Exiting.");
      rclcpp::shutdown();
      return;
    }

    // Store coordinates
    coordinates_ = coordinates;

    // Initialize publishers
    setpoint_publisher_ = this->create_publisher<px4_msgs::msg::GotoSetpoint>("/fmu/in/goto_setpoint", 10);
    command_publisher_ = this->create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", 10);
    offboard_signal_pub_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", 10);

    // Switch to OFFBOARD mode and arm the drone
    switch_to_offboard_mode();
    std::this_thread::sleep_for(2s);
    arm_drone();
    std::this_thread::sleep_for(2s);

    // Set up timers
    auto offboard_signal_callback = [this]() -> void {
      auto mmessage = px4_msgs::msg::OffboardControlMode();
      mmessage.position = true;
      mmessage.velocity = false;
      mmessage.acceleration = false;
      mmessage.attitude = false;
      mmessage.body_rate = false;
      mmessage.thrust_and_torque = false;
      mmessage.direct_actuator = false;
      offboard_signal_pub_->publish(mmessage);
    };

    auto timer_callback = [this]() -> void {
      auto message = px4_msgs::msg::GotoSetpoint();
      message.position[0] = coordinates_[0]; // x
      message.position[1] = coordinates_[1]; // y
      message.position[2] = coordinates_[2]; // z
      /*RCLCPP_INFO(this->get_logger(), "Publishing setpoint: x = %.2f, y = %.2f, z = %.2f",
                  message.position[0], message.position[1], message.position[2]);*/
      this->setpoint_publisher_->publish(message);
    };

    timer_ = this->create_wall_timer(std::chrono::milliseconds(50), timer_callback);
    offboard_signal_timer_ = this->create_wall_timer(std::chrono::milliseconds(450), offboard_signal_callback);
  }

private:
  bool parseThreeFloats(const std::string& input, std::vector<float>& result) {
    std::stringstream ss(input);
    float x, y, z;
    if (ss >> x >> y >> z && ss.eof()) {
      result = {x, y, z};
      return true;
    }
    std::cerr << "Error: Input must be three floating-point numbers separated by spaces." << std::endl;
    return false;
  }

  void switch_to_offboard_mode() {
    auto mmessage = px4_msgs::msg::OffboardControlMode();
    mmessage.position = true;
    mmessage.velocity = false;
    mmessage.acceleration = false;
    mmessage.attitude = false;
    mmessage.body_rate = false;
    mmessage.thrust_and_torque = false;
    mmessage.direct_actuator = false;
    for (int i = 0; i < 5; ++i) {
      offboard_signal_pub_->publish(mmessage);
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    auto command = px4_msgs::msg::VehicleCommand();
    command.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE;
    command.param1 = 1.0; // Custom mode
    command.param2 = 6.0; // OFFBOARD mode
    command.target_system = 1;
    command.target_component = 1;
    command.source_system = 1;
    command.source_component = 1;
    command.from_external = true;
    command.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    command_publisher_->publish(command);
    RCLCPP_INFO(this->get_logger(), "Switching to OFFBOARD mode");
  }

  void arm_drone() {
    auto command = px4_msgs::msg::VehicleCommand();
    command.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM;
    command.param1 = 1.0; // 1 = arm
    command.target_system = 1;
    command.target_component = 1;
    command.source_system = 1;
    command.source_component = 1;
    command.from_external = true;
    command.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    command_publisher_->publish(command);
    RCLCPP_INFO(this->get_logger(), "Arming drone");
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr offboard_signal_timer_;
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_signal_pub_;
  rclcpp::Publisher<px4_msgs::msg::GotoSetpoint>::SharedPtr setpoint_publisher_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr command_publisher_;
  size_t count_;
  std::vector<float> coordinates_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
