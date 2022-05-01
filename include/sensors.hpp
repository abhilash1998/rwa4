#include <algorithm>
#include <vector>
#include <string>
#include <ros/ros.h>

#include <nist_gear/LogicalCameraImage.h>
#include <nist_gear/Order.h>
#include <nist_gear/Proximity.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <trajectory_msgs/JointTrajectory.h>

namespace ros {
    class NodeHandle;
}

class Sensors {

protected:
	ros::Subscriber breakbeam0_sub;
	ros::Subscriber breakbeam0_change_sub;
	ros::Subscriber logical_camera_bins0_sub;
	ros::Subscriber logical_camera_station2_sub;
	ros::Subscriber proximity_sensor_0_sub;
	ros::Subscriber laser_profiler_0_sub;
	ros::Subscriber quality_control_sensor_1_sub;
	ros::Subscriber quality_control_sensor_2_sub;
	ros::Subscriber quality_control_sensor_3_sub;
	ros::Subscriber quality_control_sensor_4_sub;

public:
explicit Sensors(ros::NodeHandle* const nh);

void break_beam_callback(const nist_gear::Proximity::ConstPtr& msg);

void break_beam_change_callback(const nist_gear::Proximity::ConstPtr& msg);

void laser_profiler_callback(const sensor_msgs::LaserScan::ConstPtr& msg);

void proximity_sensor_callback(const sensor_msgs::Range::ConstPtr& msg);

void quality_callback1(
    const nist_gear::LogicalCameraImage::ConstPtr& image_msg);

void quality_callback2(
    const nist_gear::LogicalCameraImage::ConstPtr& image_msg);

void quality_callback3(
    const nist_gear::LogicalCameraImage::ConstPtr& image_msg);

void quality_callback4(
    const nist_gear::LogicalCameraImage::ConstPtr& image_msg);

void logical_camera_callback(
    const nist_gear::LogicalCameraImage::ConstPtr& image_msg);

void logical_camera_callback2(
    const nist_gear::LogicalCameraImage::ConstPtr& image_msg);

void startdetect();

private:
ros::NodeHandle* n;

};
