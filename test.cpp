#include "nav_msgs/msg/path.hpp"
#include "px4_msgs/msg/goto_setpoint.hpp"
#include <iostream>
#include <memory>
#include <rclcpp/logging.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <regex>
#include <px4_msgs/msg/goto_setpoint.hpp>

using namespace nav_msgs::msg;
using namespace geometry_msgs::msg;

#define optional_vector std::optional<std::vector<float>>

optional_vector get_three_floats()
{
  std::string line;
  std::cout << "Enter 3 floats (max 2 decimal places), separated by spaces: ";
  std::getline(std::cin, line);

  std::istringstream iss(line);
  std::vector<float> values;
  std::string token;

  std::regex float_pattern(R"(^-?\d+(\.\d{1,2})?$)");

  while (iss >> token) {
    if (!std::regex_match(token, float_pattern)) {
      std::cerr << "Invalid input: '" << token << "' is not a valid float with up to 2 decimal places.\n";
      return std::nullopt;
    }

    try {
      float value = std::stof(token);
      values.push_back(value);
    } catch (const std::exception &e) {
      std::cerr << "Error converting to float: " << e.what() << "\n";
      return std::nullopt;
    }
  }

  if (values.size() != 3) {
    std::cerr << "Error: Expected exactly 3 numbers, got " << values.size() << ".\n";
    return std::nullopt;
  }

  return values;
}

class SetPointPub : public rclcpp::Node{
  private:
  std::shared_ptr<rclcpp::Publisher<Path>> path_pub_;
  PoseStamped pose;
  Path path;

  void input_loop() {
    while (rclcpp::ok()) {
      optional_vector input = get_three_floats();
      if(!input || input->size() != 3) continue;
      publishSetPoint(input);
    }
  }

  void publishSetPoint(optional_vector input){
      
    path.header.stamp = this->get_clock()->now();
    path.header.frame_id = "map";
    
    pose.header = path.header;
    pose.pose.position.x = input->at(0);
    pose.pose.position.y = input->at(1);
    pose.pose.position.z = input->at(2);

    path.poses.clear();
    path.poses.push_back(pose);
    

    path_pub_->publish(path);
  }

  public:
    SetPointPub() : rclcpp::Node("SetPointPub"){
      rclcpp::QoS waypoint_qos_profile(10);
      waypoint_qos_profile.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
      path_pub_ = this->create_publisher<Path>("/waypoint_generator/waypoints", waypoint_qos_profile);
    
      

      RCLCPP_INFO(this->get_logger(), "Setpoint Publisher running.");

      std::thread(&SetPointPub::input_loop, this).detach();
    }
};


int main(int argc, char ** argv)
{

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SetPointPub>());
  rclcpp::shutdown();

  return 0;
}
