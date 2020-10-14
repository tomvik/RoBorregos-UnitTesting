#include <iostream>
#include "ros/ros.h"
#include "geometry/geometry.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "unit_testing_tutorial_node");

    ROS_INFO("[aFunction]: %d", geometry::aFunction(10));
    ROS_INFO("[aFunction]: %d", geometry::aFunction(50));
    ROS_INFO("[aFunction]: %d", geometry::aFunction(100));
    ROS_INFO("[aFunction]: %d", geometry::aFunction(1000));
    return 0;
}