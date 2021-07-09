#pragma once

#include <chrono>
#include <functional>
#include <string>

#include <rclcpp/rclcpp.hpp>


void ros2_init(std::string node_name = "jupyter_ros_hub"){
    int argc = 1;
    char *argv[1];
    argv[0] = node_name.data();
    rclcpp::init(argc, argv);
}

template <class T>
std::shared_ptr<rclcpp::Node> get_ros2_node() {
    auto node_ptr = std::make_shared<T>();
    return node_ptr->shared_from_this();
}