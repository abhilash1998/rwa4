#ifndef __GANTRY_H__
#define __GANTRY_H__

// ros
#include <ros/ros.h>
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
#include <nist_gear/LogicalCameraImage.h>

    /**
     * @brief class for the Gantry robot
     *
     */
    class Gantry {
    public:
        /**
        * @brief Struct for preset locations
        * @todo Add new preset locations here if needed
        */
        typedef struct GantryPresetLocation {
            std::vector<double> gantry_preset;  //9 joints
            std::string name;
        } start, bin, agv, agv_pos1, agv_pos2, grasp, as, arm;

        
        Gantry();
        /**
         * @brief Returns true if the part is sucessfully picked up by the robotic arm.
         * 
         * @param part_type - Name of the part which needs to be picked up
         * @param part_pose - Pose of the part from which it need to be picked up
         * @param ss - integer to switch values of between bin and tray.
         after which it slows down for attaching the object.
         * @return true - True if sucessfully picked
         * @return false - False if not picked up 
         */
        bool pickPart(std::string part_type, geometry_msgs::Pose part_pose, int ss);
        
        /**
         * @brief Places the part to the given desired location on the given agv nad return true when the part is sucessfully kept. 
         * 
         * @param part_init_pose - Global pose of the part in the bin or current position at the starting of this function
         * @param part_goal_pose - The pose at which the part needs to be kept on the agv with reference to the camera frame.
         * @param agv - AGV name on which the part needs to be kept
         * @return true - Part sucessfully picked up
         * @return false - Part was not picked up sucessfully
         */
        bool placePart(geometry_msgs::Pose part_init_pose, geometry_msgs::Pose part_pose_in_frame,std::string part_type, std::string agv,bool flip_);
        void testPreset(const std::vector<GantryPresetLocation>& preset_list);
        void movePart(std::string part_type, std::string camera_frame, geometry_msgs::Pose goal_in_tray_frame, std::string agv);
        void activateGripper();
        void deactivateGripper();


        
        void moveBaseTo(double small_long_joint,double torso_rail_joint);
        nist_gear::VacuumGripperState getGripperState();
	
	geometry_msgs::Pose transform_to_world_frame(const geometry_msgs::Pose& target, const std::string& agv_id);
        

        // Send command message to robot controller
        bool sendJointPosition(trajectory_msgs::JointTrajectory command_msg);
        void goToPresetLocation(std::string location_name);
        void gantryArmPreset();

        //--preset locations;
        start home1_, home2_;
        agv agv1_, agv2_, agv3_, agv4_;
        agv_pos1 agv1_as1, agv2_as1, agv3_as3, agv4_as3;
        agv_pos2 agv1_as2, agv2_as2, agv3_as4, agv4_as4;
        as as1_ , as2_ , as3_ , as4_;
        bin bin3_, bin4_, bin7_, bin8_ ;
        arm goodArm_;

        //-- pointer variable for the current part number
        int* counter;

        //-- boolean variables for faulty parts detected
        

    private:
        std::vector<double> joint_group_positions_;
        std::vector<double> joint_arm_positions_;
        ros::NodeHandle node_;
        std::string planning_group_;
        moveit::planning_interface::MoveGroupInterface::Options gantry_options_;
        moveit::planning_interface::MoveGroupInterface gantry_group_;
        moveit::planning_interface::MoveGroupInterface::Options gantry_arm_options_;
        moveit::planning_interface::MoveGroupInterface gantry_arm_group_;
        moveit::planning_interface::MoveGroupInterface::Options gantry_torso_options_;
        moveit::planning_interface::MoveGroupInterface gantry_torso_group_;
        sensor_msgs::JointState current_joint_states_;
        control_msgs::JointTrajectoryControllerState gantry_arm_controller_state_;
        control_msgs::JointTrajectoryControllerState gantry_controller_state_;

	tf2_ros::Buffer tf_buffer;
    	tf2_ros::TransformListener tf_listener;

        nist_gear::VacuumGripperState gripper_state_;
        // gripper state subscriber
        ros::Subscriber gripper_state_subscriber_;
        // service client
        ros::ServiceClient gripper_control_client_;
        // publishers
        ros::Publisher Gantry_arm_joint_trajectory_publisher_;
        ros::Publisher Gantry_torso_joint_trajectory_publisher_;
        // joint states subscribers
        ros::Subscriber Gantry_joint_states_subscriber_;
        // controller state subscribers
        ros::Subscriber Gantry_arm_controller_state_subscriber_;
        ros::Subscriber Gantry_torso_controller_state_subscriber_;

        

        // callbacks
        void gripper_state_callback(const nist_gear::VacuumGripperState::ConstPtr& gripper_state_msg);
        void gantry_joint_states_callback_(const sensor_msgs::JointState::ConstPtr& joint_state_msg);
        void gantry_arm_controller_state_callback(const control_msgs::JointTrajectoryControllerState::ConstPtr& msg);
        void gantry_controller_state_callback(const control_msgs::JointTrajectoryControllerState::ConstPtr& msg);
    
    };

#endif
