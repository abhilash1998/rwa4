#include "blackout.hpp"

#include <ros/ros.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "blackout_controller");
    ros::NodeHandle nh;

    // Create our node to monitor for sensor blackouts
    BlackoutDetector blackout_detector(&nh);

    // Spin indefinitely
    ros::spin();

    return 0;
}
