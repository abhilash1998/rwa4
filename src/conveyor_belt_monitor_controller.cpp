#include "conveyor_belt_monitor.hpp"

#include <ros/ros.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "conveyor_belt_monitor_controller");
    ros::NodeHandle nh;

    // Create our node to monitor parts on the conveyor belt
    ConveyorBeltMonitor conveyor_belt_monitor(&nh);

    // Spin indefinitely
    ros::spin();

    return 0;
}
