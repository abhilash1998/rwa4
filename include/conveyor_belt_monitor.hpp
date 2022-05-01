#include <ros/subscriber.h>
#include <ros/timer.h>

#include <group3_rwa4/GetConveyorBeltPartPickPose.h>

#include <geometry_msgs/Pose.h>
#include <sensor_msgs/LaserScan.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <vector>

// Forward declare
namespace ros {
    class NodeHandle;
}

// A struct that represents parts on the conveyor belt
struct ConveyorBeltPart
{
public:

    // The average time of when the part was initially detected
    const ros::Time time_of_detection;
    // The estimated center of the part after initial detection
    const geometry_msgs::Pose pick_pose_at_detection;

    // Constructor, all fields populated
    ConveyorBeltPart(const ros::Time& time, const geometry_msgs::Pose& pose);
    // Destructor
    ~ConveyorBeltPart();

    // Given a lapse in time, get the estimated pose of the part a specified
    // number of seconds in the future.
    // @param dt The number of seconds in the future, defaults to 0.0 (now) if
    // unspecified.
    // @return The estimated pose of this part resolved in world frame
    geometry_msgs::Pose get_estimated_pose(const double dt=0.0) const;

    // Get whether or not this part will still be on the belt a specified
    // number of seconds in the future.
    // @param dt The number of seconds in the future, defaults to 0.0 (now) if
    // unspecified.
    // @param True if the part is / will be, false otherwise.
    bool is_part_on_belt(const double dt=0.0) const;
};

// A list of conveyor belt parts
using ConveyorBeltPartList = std::vector<ConveyorBeltPart>;

// A ROS utility to monitor parts that enter and leave the conveyor belt
class ConveyorBeltMonitor
{
protected:
ros::Publisher location_of_obj_on_conveyor;
ros::Publisher time_of_spawn;
    // TF buffer for \a tf_listener
    tf2_ros::Buffer tf_buffer;
    // TF listener to monitor the sensor's frame
    tf2_ros::TransformListener tf_listener;

    // Subscriber to the sensor data that describes the detected parts
    ros::Subscriber sensor_sub;
    // The server for the service which collects a pick point for a conveyor
    // belt part
    ros::ServiceServer get_part_pick_pose_srv;
    // A timer used to periodically reevaluate if the parts are still valid
    ros::Timer expire_parts_tmr;
    
    



    // True if a part is currently passing the sensor, false otherwise
    bool profiling_active;
    // The time profiling the current part started
    ros::Time active_profile_start;
    // The min distance from the sensor for the part currently being profiled
    float active_profile_min;
    // A list of parts currently on the belt
    ConveyorBeltPartList current_parts;

    // Callback for the sensor which profiles the conveyor belt parts
    // @param msg The sensor data
    void sensor_callback(const sensor_msgs::LaserScan::ConstPtr& msg);
    // Callback for the reevaluation timer being triggered
    // @param evt The timer event
    void expire_parts_callback(const ros::TimerEvent& evt);

    // Service callback to handle requests to get a pick pose for a part on the
    // conveyor belt.
    // @param req The service request
    // @param res The service response
    // @return True
    bool handle_get_part_pick_pose(
        group3_rwa4::GetConveyorBeltPartPickPose::Request &req,
        group3_rwa4::GetConveyorBeltPartPickPose::Response &res);

    // Helper function to get the pose of the sensor
    // @param pose Overwritten with the pose if this method returns true
    // @param timeout The timeout to look up the transform, defaults to 0.0
    // @return True if the pose was found, false otherwise
    bool get_sensor_pose(geometry_msgs::Pose& pose, const double timeout=0.0) const;
    bool conveyor_part=true;

public:
    // Constructor
    ConveyorBeltMonitor(ros::NodeHandle* const nh);
    // Destructor
    ~ConveyorBeltMonitor();
};
