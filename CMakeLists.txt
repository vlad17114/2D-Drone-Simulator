cmake_minimum_required(VERSION 3.8)
project(twoDdrone)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS_ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(twoDdrone src/publisher.cpp)
add_executable(publisher_velocity src/publisher_velocity.cpp)

ament_target_dependencies(twoDdrone
  rclcpp
  px4_msgs
)

ament_target_dependencies(publisher_velocity
  rclcpp
  px4_msgs
)

install(TARGETS 
  twoDdrone
  publisher_velocity
  DESTINATION lib/${PROJECT_NAME})

ament_package()
