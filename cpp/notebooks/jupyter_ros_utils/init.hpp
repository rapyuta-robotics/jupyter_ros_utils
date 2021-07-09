#pragma once

#include <string>

#include <ros/ros.h>
#include <ros/spinner.h>

void ros_init(std::string node_name = "jupyter_ros_hub"){
    int argc = 1;
    char *argv[1];
    argv[0] = node_name.data();
    ros::init(argc, argv, node_name);

    static ros::AsyncSpinner spinner(4);
    spinner.start();
}
