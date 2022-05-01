#ifndef __ARM_H__
#define __ARM_H__

// ros
#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <std_msgs/String.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <sensor_msgs/JointState.h>
#include <control_msgs/JointTrajectoryControllerState.h>
#include <trajectory_msgs/JointTrajectory.h>
// standard library
#include <string>
#include <vector>
#include <array>
#include <cstdarg>
// nist
#include <nist_gear/VacuumGripperState.h>
#include <nist_gear/VacuumGripperControl.h>
#include <nist_gear/LogicalCameraImage.h>
// custom
#include "utils.hpp"

/**
 * @brief class for the Gantry robot
 *
 */
class Arm {
public:
    /**
    * @brief Struct for preset locations
    * @todo Add new preset locations here if needed
    */
    typedef struct ArmPresetLocation {
        std::vector<double> arm_preset;  //9 joints
        std::string name;
    } start, bin, agv, grasp;

    Arm();

    /**
     * @brief Initialize the object
     */
     bool conveyorPickPart(geometry_msgs::Pose part_pose);
    bool pickPart(std::string part_type, geometry_msgs::Pose part_pose, int ss);
    bool placePart(geometry_msgs::Pose part_init_pose, geometry_msgs::Pose part_goal_pose, std::string agv,bool flip_);
    void testPreset(const std::vector<ArmPresetLocation>& preset_list);
    void movePart(std::string part_type, std::string camera_frame, geometry_msgs::Pose goal_in_tray_frame, std::string agv);
    void activateGripper();
    void deactivateGripper();

    /**
     * @brief Move the joint linear_arm_actuator_joint only
     *
     * The joint position for this joint corresponds to the y value
     * in the world frame. For instance, a value of 0 for this joint
     * moves the base of the robot to y = 0.
     *
     * @param location A preset location
     */
    void moveBaseTo(double linear_arm_actuator_joint_position);
    nist_gear::VacuumGripperState getGripperState();

    // Send command message to robot controller
    bool sendJointPosition(trajectory_msgs::JointTrajectory command_msg);
     void goToPresetLocation(std::string location_name,int condition = 0 ,double y = 0.0);

    // Use this robot's tf buffer to resolve an AGV in the world frame
    geometry_msgs::Pose transform_to_world_frame(const geometry_msgs::Pose& target, const std::string& agv_id);
    geometry_msgs::Pose transform_to_world_frame(const std::string& part_in_camera_frame);

    //--preset locations;
    start home1_, home2_,home3_,home4_;
    agv agv1_, agv2_, agv3_, agv4_;
    bin bin1_, bin2_,bin5_, bin6_;
    bin bin1_e, bin2_e,bin5_e, bin6_e;

private:
    std::vector<double> joint_group_positions_;
    std::vector<double> joint_arm_positions_;
    ros::NodeHandle node_;
    std::string planning_group_;
    moveit::planning_interface::MoveGroupInterface::Options arm_options_;
    moveit::planning_interface::MoveGroupInterface arm_group_;
    sensor_msgs::JointState current_joint_states_;
    control_msgs::JointTrajectoryControllerState arm_controller_state_;

    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener;

    nist_gear::VacuumGripperState gripper_state_;
    // gripper state subscriber
    ros::Subscriber gripper_state_subscriber_;
    // service client
    ros::ServiceClient gripper_control_client_;
    // publishers
    ros::Publisher arm_joint_trajectory_publisher_;
    // joint states subscribers
    ros::Subscriber arm_joint_states_subscriber_;
    // controller state subscribers
    ros::Subscriber arm_controller_state_subscriber_;

    // callbacks
    void gripper_state_callback(const nist_gear::VacuumGripperState::ConstPtr& gripper_state_msg);
    void arm_joint_states_callback_(const sensor_msgs::JointState::ConstPtr& joint_state_msg);
    void arm_controller_state_callback(const control_msgs::JointTrajectoryControllerState::ConstPtr& msg);
};

#endif
