#pragma once

#include <string>

#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/bool.hpp> // This is deprecated in newer versions of ROS.
#include <abb_librws/rws_state_machine_interface.h>


class SmartGripper : public rclcpp::Node {
public:
    SmartGripper(std::string node_name, const std::string& robot_ip);
    bool Init();
private:
    std::string ip_;
    std::shared_ptr<abb::rws::RWSStateMachineInterface> 
    rws_state_machine_interface_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr grip_sub_;
    bool grip_callback(std_msgs::msg::Bool::SharedPtr msg);
};