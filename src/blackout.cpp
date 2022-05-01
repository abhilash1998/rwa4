#include "blackout.hpp"

#include <ros/node_handle.h>
#include <ros/duration.h>

#include <std_msgs/Bool.h>

void BlackoutDetector::competition_state_callback(const std_msgs::String::ConstPtr& msg)
{
    if (msg->data != current_competition_state)
    {
        current_competition_state = msg->data;
        ROS_INFO_STREAM("Competition state updated: " << current_competition_state);
    }
}

void BlackoutDetector::logical_camera_image_callback(const nist_gear::LogicalCameraImage::ConstPtr& msg)
{
    watch_for_blackouts_tmr.stop();

    if (in_sensor_blackout)
    {
        publish_update(false);
    }

    sensors_started = true;
    in_sensor_blackout = false;

    watch_for_blackouts_tmr.start();
}

void BlackoutDetector::sensor_blackout_detected_callback(const ros::TimerEvent& evt)
{
    if ((current_competition_state == "go") && sensors_started && !in_sensor_blackout)
    {
        in_sensor_blackout = true;
        publish_update(true);
        ROS_WARN_STREAM("SENSOR BLACK OUT DETECTED");
    }
}

void BlackoutDetector::publish_update(const bool active)
{
    std_msgs::Bool msg;
    msg.data = active;
    blackout_status_pub.publish(msg);
}

BlackoutDetector::BlackoutDetector(ros::NodeHandle* const nh) :
    current_competition_state(""),
    sensors_started(false),
    in_sensor_blackout(false)
{
    competition_state_sub = nh->subscribe<std_msgs::String>(
        "/ariac/competition_state",
        1,
        &BlackoutDetector::competition_state_callback,
        this
    );
    for (int i = 0; i < logical_camera_subs.size(); i++)
    {
        logical_camera_subs[i] = nh->subscribe<nist_gear::LogicalCameraImage>(
            std::string("/ariac/logical_camera_") + std::to_string(i+1),
            1,
            &BlackoutDetector::logical_camera_image_callback,
            this
        );
    }
    blackout_status_pub = nh->advertise<std_msgs::Bool>(
        "/group3/blackout_active",
        1,
        true
    );
    watch_for_blackouts_tmr = nh->createTimer(
        ros::Duration(1.0),
        &BlackoutDetector::sensor_blackout_detected_callback,
        this
    );
}

BlackoutDetector::~BlackoutDetector()
{
}
