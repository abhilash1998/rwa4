#include "conveyor_belt_monitor.hpp"

#include <ros/node_handle.h>
#include <ros/duration.h>

#include <geometry_msgs/TransformStamped.h>

#include <utility>
#include "std_msgs/Float32.h"
#define BELT_SPEED_MPS 0.2
#define BELT_LENGTH_M 9.0
#define DURATION_ON_BELT ros::Duration(BELT_LENGTH_M/BELT_SPEED_MPS)

#define PART_THRESHOLD 0.55

ConveyorBeltPart::ConveyorBeltPart(const ros::Time& time, const geometry_msgs::Pose& pose) :
    time_of_detection(time),
    pick_pose_at_detection(pose)


{
    
}
ConveyorBeltPart::~ConveyorBeltPart()
{
}
geometry_msgs::Pose ConveyorBeltPart::get_estimated_pose(const double dt) const
{
    geometry_msgs::Pose adjusted = pick_pose_at_detection;
    adjusted.position.y -= (BELT_SPEED_MPS * (ros::Time::now() + ros::Duration(dt) - time_of_detection).toSec());
    return adjusted;
}
bool ConveyorBeltPart::is_part_on_belt(const double dt) const
{
    return ros::Time::now() + ros::Duration(dt) - time_of_detection < DURATION_ON_BELT;
}





void ConveyorBeltMonitor::sensor_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    const std::vector<float> ranges = msg->ranges;
    if (!ranges.empty())
    {
        const float center_range = ranges[ranges.size()/2];

        // If not currently profiling a part, but there is a part there, start
        // profiling it
        if ((PART_THRESHOLD > center_range) && !profiling_active)
        {
            ROS_DEBUG_STREAM("Started profiling part");
            active_profile_start = ros::Time::now();
            active_profile_min = center_range;
            profiling_active = true;
        }
        // If we were profiling, but now there is no part, finish profiling
        else if ((PART_THRESHOLD < center_range) && profiling_active)
        {
            ROS_DEBUG_STREAM("Finished profiling part");
            profiling_active = false;

            // Get an estimate of the time at which the center of the part was
            // under the sensor, and therefore an estimate of its displacement.
            // Then get that point as a pose, build it from the sensor frame
            geometry_msgs::Pose pick_pose_estimate;
            if (get_sensor_pose(pick_pose_estimate))
            {
            
                // Account for the difference between the sensor and the belt
                pick_pose_estimate.position.z -= active_profile_min;

                // The conveyor belt moves the part in the -y direction
                auto current_time=(ros::Time::now()).toSec();
                const double dt = (ros::Time::now() - active_profile_start).toSec() / 2;
                const ros::Time time_avg = active_profile_start + ros::Duration(dt);
                const double dx = BELT_SPEED_MPS * dt;
                pick_pose_estimate.position.y -= dx;
                std_msgs::Float32 pose_y,spawn_time;
                pose_y.data=pick_pose_estimate.position.y;
                spawn_time.data=current_time;
                
                if (ConveyorBeltMonitor::conveyor_part  ){
                location_of_obj_on_conveyor.publish(pose_y);
                time_of_spawn.publish(spawn_time);
                ConveyorBeltMonitor::conveyor_part =false;
                }

                current_parts.emplace_back(time_avg, pick_pose_estimate);
                ROS_INFO_STREAM("Detected a part on the conveyor belt");
            }
            else
            {
                ROS_FATAL_STREAM("Failed to get sensor frame?!");
            }
        }
        // If profiling is already active, see if this is the portion of the
        // part that is closest to the sensor
        else if (profiling_active)
        {
            active_profile_min = (center_range < active_profile_min) ? center_range : active_profile_min;
        }
    }
}


void ConveyorBeltMonitor::expire_parts_callback(const ros::TimerEvent& evt)
{
    // Create a new list of parts from the current list. Start by getting the
    // current time
    const int old_size = current_parts.size();
    const ros::Time time_now = ros::Time::now();
    const ConveyorBeltPartList old_parts = ConveyorBeltPartList(current_parts.begin(), current_parts.end());
    current_parts.clear();
    for (const ConveyorBeltPart& part : old_parts)
    {
        // Keep this part if it's still on the belt
        if (part.is_part_on_belt())
        {
            current_parts.push_back(part);
            ROS_INFO_STREAM("Part at y = "
                            << part.pick_pose_at_detection.position.y
                            << " => "
                            << part.get_estimated_pose().position.y);
        }
    }

    const int new_size = current_parts.size();
    if (old_size != new_size)
    {
        ROS_INFO_STREAM("Conveyor belt part count went from "
                        << old_size
                        << " to "
                        << new_size);
    }
}



bool ConveyorBeltMonitor::handle_get_part_pick_pose(
    group3_rwa4::GetConveyorBeltPartPickPose::Request &req,
    group3_rwa4::GetConveyorBeltPartPickPose::Response &res)
{
    // Prove this otherwise
    res.part_available = false;

    // Loop forward, since the list is ordered from "older" parts to younger,
    // we want to get the oldest part possible
    for (const ConveyorBeltPart& part : current_parts)
    {
        if (part.is_part_on_belt(req.dt))
        {
            res.pick_pose = part.get_estimated_pose(req.dt);
            res.part_available = true;
            break;
        }
    }

    return true;
}

ConveyorBeltMonitor::ConveyorBeltMonitor(ros::NodeHandle* const nh) :
    tf_listener(tf_buffer),
    profiling_active(false),
    active_profile_min(0.0)
{
    sensor_sub = nh->subscribe<sensor_msgs::LaserScan>(
        "/ariac/laser_profiler_0",
        1,
        &ConveyorBeltMonitor::sensor_callback,
        this
    );
    get_part_pick_pose_srv = nh->advertiseService(
        "/group3/get_conveyor_belt_part_pick_pose",
        &ConveyorBeltMonitor::handle_get_part_pick_pose,
        this
    );
    expire_parts_tmr = nh->createTimer(
        ros::Duration(0.5),
        &ConveyorBeltMonitor::expire_parts_callback,
        this
    );
    location_of_obj_on_conveyor=nh->advertise<std_msgs::Float32>("/ariac/conveyor/part_position_y", 10);
    time_of_spawn=nh->advertise<std_msgs::Float32>("/ariac/conveyor/part_position_time", 10);
}

ConveyorBeltMonitor::~ConveyorBeltMonitor()
{
}

bool ConveyorBeltMonitor::get_sensor_pose(geometry_msgs::Pose& pose, const double timeout) const
{
    try {
        const geometry_msgs::TransformStamped world_pose_tf = tf_buffer.lookupTransform(
            "world",
            "laser_profiler_0_frame",
            ros::Time(0),
            ros::Duration(timeout)
        );

        pose.position.x = world_pose_tf.transform.translation.x;
        pose.position.y = world_pose_tf.transform.translation.y;
        pose.position.z = world_pose_tf.transform.translation.z;
        pose.orientation.x = world_pose_tf.transform.rotation.x;
        pose.orientation.y = world_pose_tf.transform.rotation.y;
        pose.orientation.z = world_pose_tf.transform.rotation.z;
        pose.orientation.w = world_pose_tf.transform.rotation.w;

        return true;
    } catch (tf2::TransformException& ex) {
        return false;
    }
}
