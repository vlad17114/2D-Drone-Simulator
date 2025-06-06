#include <chrono>
#include <memory>
#include <vector>
#include <sstream>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <regex.h>
#include <string.h>

#include "rclcpp/rclcpp.hpp"
#include "px4_msgs/msg/goto_setpoint.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"


using namespace std::chrono_literals;
using namespace std::placeholders;

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher()
  : Node("publish_xyz"), count_(0)
  {

    // Initialize publishers
    setpoint_publisher_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
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
      mmessage.position = false;
      mmessage.velocity = true;

      mmessage.acceleration = false;
      mmessage.attitude = false;
      mmessage.body_rate = false;
      mmessage.thrust_and_torque = false;
      mmessage.direct_actuator = false;
      offboard_signal_pub_->publish(mmessage);
    };

    

    std::thread(&MinimalPublisher::main_loop,this).detach();
    velocity_signal_timer_ = this->create_wall_timer(std::chrono::milliseconds(200),std::bind(&MinimalPublisher::publish_xyz,this));
    offboard_signal_timer_ = this->create_wall_timer(std::chrono::milliseconds(450), offboard_signal_callback);
  }

private:
    

  volatile float x_ = 0,y_ = 0,z_ = 0;
  void main_loop(){

    while(rclcpp::ok){
      parse_3fp_registers();
      RCLCPP_INFO(this->get_logger(), "Publishing trajectory setpoint: x = %.2f, y = %.2f, z = %.2f",
                x_,y_,z_);
    }
  }
  void publish_xyz() {
    auto message = px4_msgs::msg::TrajectorySetpoint();
    message.velocity[0] =  x_;// x
    message.velocity[1] =  y_;// y
    message.velocity[2] =  z_;// z
    //RCLCPP_INFO(this->get_logger(), "Publishing setpoint: x = %.2f, y = %.2f, z = %.2f",
    //            message.position[0], message.position[1], message.position[2]);
     
  setpoint_publisher_->publish(message);
  };

  

  

  int parse_3fp_registers() {
      // Buffer to store input line
      char line[256];
      std::cout << "Input 3 floating points sepparated by a whitespace\n";
      // Read a line from stdin
      if (!fgets(line, sizeof(line), stdin)) {
          return 0; // Failed to read input (EOF or error)
      }

      // Remove trailing newline if present
      line[strcspn(line, "\n")] = 0;

      // Regex for three floating-point numbers separated by whitespace
      regex_t regex;
      regmatch_t matches[4]; // 3 submatches for numbers + whole match
      const char *pattern = "^\\s*([-+]?[0-9]*\\.?[0-9]+)\\s+([-+]?[0-9]*\\.?[0-9]+)\\s+([-+]?[0-9]*\\.?[0-9]+)\\s*$";

      if (regcomp(&regex, pattern, REG_EXTENDED) != 0) {
          return 0; // Regex compilation failed
      }

      // Execute regex
      if (regexec(&regex, line, 4, matches, 0) != 0) {
          regfree(&regex);
          return 0; // No match
      }

      // Convert matched strings to floats
      char *endptr;
      char num1[32], num2[32], num3[32];
      
      // Copy matched substrings (null-terminate them)
      strncpy(num1, line + matches[1].rm_so, matches[1].rm_eo - matches[1].rm_so);
      num1[matches[1].rm_eo - matches[1].rm_so] = 0;
      strncpy(num2, line + matches[2].rm_so, matches[2].rm_eo - matches[2].rm_so);
      num2[matches[2].rm_eo - matches[2].rm_so] = 0;
      strncpy(num3, line + matches[3].rm_so, matches[3].rm_eo - matches[3].rm_so);
      num3[matches[3].rm_eo - matches[3].rm_so] = 0;

      // Convert to floats
      x_ = strtof(num1, &endptr);
      if (*endptr != 0) {
          regfree(&regex);
          return 0; // Conversion failed
      }
      y_ = strtof(num2, &endptr);
      if (*endptr != 0) {
          regfree(&regex);
          return 0; // Conversion failed
      }
      z_ = strtof(num3, &endptr);
      if (*endptr != 0) {
          regfree(&regex);
          return 0; // Conversion failed
      }

      regfree(&regex);
      return 1; // Success
  }

  void switch_to_offboard_mode() {
    auto mmessage = px4_msgs::msg::OffboardControlMode();
    mmessage.position = false;
    mmessage.velocity = true;
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

  
  rclcpp::TimerBase::SharedPtr offboard_signal_timer_;
  rclcpp::TimerBase::SharedPtr velocity_signal_timer_;
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_signal_pub_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr setpoint_publisher_;
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
