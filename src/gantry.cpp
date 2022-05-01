#include "gantry.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>
#include <eigen_conversions/eigen_msg.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <Eigen/Geometry>
#include <tf2/convert.h>
#include "utils.hpp"
#include <math.h>

/////////////////////////////////////////////////////
Gantry::Gantry() : 
    node_("/ariac/gantry"),
    planning_group_("/ariac/gantry/robot_description"),
    gantry_options_("gantry_full", planning_group_, node_),
    gantry_torso_options_("gantry_torso", planning_group_, node_),
    gantry_arm_options_("gantry_arm", planning_group_, node_),
    gantry_group_(gantry_options_),
    gantry_arm_group_(gantry_arm_options_),
    gantry_torso_group_(gantry_torso_options_),
    tf_listener(tf_buffer)
{
       
        // publishers to directly control the joints without moveit
        Gantry_arm_joint_trajectory_publisher_ =
            node_.advertise<trajectory_msgs::JointTrajectory>("/ariac/gantry/gantry_arm_controller/command", 10);
        Gantry_torso_joint_trajectory_publisher_ =
            node_.advertise<trajectory_msgs::JointTrajectory>("/ariac/gantry/gantry_controller/command", 10);
        
        // joint state subscribers
        Gantry_joint_states_subscriber_ =
            node_.subscribe("/ariac/gantry/joint_states", 10, &Gantry::gantry_joint_states_callback_, this);
        
        // controller state subscribers
        Gantry_arm_controller_state_subscriber_ = node_.subscribe(
            "/ariac/gantry/gantry_arm_controller/state", 10, &Gantry::gantry_arm_controller_state_callback, this);
        Gantry_torso_controller_state_subscriber_ = node_.subscribe(
            "/ariac/gantry/gantry_controller/state", 10, &Gantry::gantry_controller_state_callback, this);
        
        // gripper state subscriber
        gripper_state_subscriber_ = node_.subscribe(
            "/ariac/gantry/arm/gripper/state", 10, &Gantry::gripper_state_callback, this);
        
        // controller state subscribers
        gripper_control_client_ =
            node_.serviceClient<nist_gear::VacuumGripperControl>("/ariac/gantry/arm/gripper/control");
        gripper_control_client_.waitForExistence();
        
        // Preset locations
        // ^^^^^^^^^^^^^^^^
        // Joints for the arm are in this order:
        // - linear_arm_actuator_joint
        // - shoulder_pan_joint
        // - shoulder_lift_joint
        // - elbow_joint
        // - wrist_1_joint
        // - wrist_2_joint
        // - wrist_3_joint

        double small_long_joint{ -1.45 };   //long rail 2 to small rail (prismatic)
        double torso_rail_joint{ 0 };       // small rail to torso base(prismatic)
        double torso_base_main_joint{ 0 };  //base to main (revolute)
        double gantry_arm_shoulder_pan_joint{ 0 };
        double gantry_arm_shoulder_lift_joint{ -1.13 };
        double gantry_arm_elbow_joint{ 1.88 };
        double gantry_arm_wrist_1_joint{ 0.72 };
        double gantry_arm_wrist_2_joint{ 1.55 };
        double gantry_arm_wrist_3_joint{ 0.83 };
        
        

        //home position
        home1_.gantry_preset = { -2, 0, 0, 0.0, -1.13, 1.88, -0.72, 1.55, 0.83 };
        home1_.name = "home1";
        home2_.gantry_preset = { -8, 0, 0, 0.0, -1.13, 1.88, -0.72, 1.55, 0.83 };
        home2_.name = "home2";
        agv1_.gantry_preset = { 0.03, -4.0, 0.0, 0.0, -1.13, 1.88, -0.72, 1.55, 0.83};
        agv1_.name = "agv1";
        agv2_.gantry_preset = { 0.03, -2.0, -3.14, 0.0, -1.13, 1.88, -0.72, 1.55, 0.83};
        agv2_.name = "agv2";
        agv3_.gantry_preset = { 0.03, 2.0, 0.0, 0.0, -1.13, 1.88, -0.72, 1.55, 0.83};
        agv3_.name = "agv3";
        agv4_.gantry_preset = { 0.03, 4.0, -3.14, 0.0, -1.13, 1.88, -0.72, 1.55, 0.83};
        agv4_.name = "agv4";
        double x,y,z;
        y = -4.50;
        agv1_as1.gantry_preset = { -2.75, -4.50, 1.57, 0.0, -1.13, 1.88, -0.72, 1.55, 0.83};
        agv1_as1.name = "agv1_as1";
        agv2_as1.gantry_preset = { -2.75, -1.35, 1.57, 0.0, -1.13, 1.88,- 0.72, 1.55, 0.83};
        agv2_as1.name = "agv2_as1";
        agv3_as3.gantry_preset = { -2.75, 1.35, 1.57, 0.0, -1.13, 1.88, -0.72, 1.55, 0.83};
        agv3_as3.name = "agv3_as3";
        agv4_as3.gantry_preset = { -2.75, 4.50, 1.57, 0.0, -1.13, 1.88, -0.72, 1.55, 0.83};
        agv4_as3.name = "agv4_as3";

        agv1_as2.gantry_preset = { -7.52, -4.50, 1.57, 0.0, -1.13, 1.88, -0.72, 1.55, 0.83 };
        agv1_as2.name = "agv1_as2";
        agv2_as2.gantry_preset = { -7.52, -1.35, 1.57,  0.0, -1.13, 1.88, -0.72, 1.55, 0.83 };
        agv2_as2.name = "agv2_as2";
        agv3_as4.gantry_preset = { -7.52, 1.35, 1.57, 0.0, -1.13, 1.88, -0.72, 1.55, 0.83};
        agv3_as4.name = "agv3_as4";
        agv4_as4.gantry_preset = { -7.52, 4.50, 1.57, 0.0, -1.13, 1.88, -0.72, 1.55, 0.83 };
        agv4_as4.name = "agv4_as4";

        as1_.gantry_preset = { -3.96, -2.88, 1.63, -1.38, -2.14, 1.88, -0.38, 0.13, 0.83 };
        as1_.name = "as1";
        as2_.gantry_preset = { -8.85, -2.88, 1.57, -1.38, -2.14, 1.88, -0.38, 0.13, 0.83};
        as2_.name = "as2";
        as3_.gantry_preset = { -3.96, 2.97, 1.63, -1.38, -2.14, 1.88, -0.38, 0.13, 0.83};
        as3_.name = "as3";
        as4_.gantry_preset = { -8.85, 3.24, 1.57, -1.38, -2.14, 1.88, -0.38, 0.13, 0.83};
        as4_.name = "as4";

        bin4_.gantry_preset = { -1.20, -3.24, -1.57, 0.0, -1.13, 1.88, -0.72, 1.55, 0.83 };
        bin4_.name = "bin4";
        bin3_.gantry_preset = { -1.20, -2.43, -1.57, 0.0, -1.13, 1.88, -0.72, 1.55, 0.83 };
        bin3_.name = "bin3";
        bin7_.gantry_preset = { -1.20, 2.61, -1.57, 0.0, -1.13, 1.88, -0.72, 1.55, 0.83 };
        bin7_.name = "bin7";
        bin8_.gantry_preset = { -1.20, 3.42, -1.57, 0.0, -1.13, 1.88, -0.72, 1.55, 0.83 };
        bin8_.name = "bin8";

        goodArm_.gantry_preset = { -0.13, -2.89, 2.13, 0.72, 1.55, 0.83 };
        goodArm_.name = "arm";

        // raw pointers are frequently used to refer to the planning group for improved performance.
        // to start, we will create a pointer that references the current robot’s state.
    const moveit::core::JointModelGroup* joint_model_group = gantry_group_.getCurrentState()->getJointModelGroup("gantry_full");
    moveit::core::RobotStatePtr current_state = gantry_group_.getCurrentState();
    // next get the current set of joint values for the group.
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions_);

}
    //////////////////////////////////////////////////////
    void Gantry::moveBaseTo(double part_pose_x, double part_pose_y) {
        // get the current joint positions
        ROS_INFO_STREAM("part x :"<< part_pose_x << "part y: "<< part_pose_y);

        
            if (3.079 <= part_pose_y && part_pose_y <= 3.679){
                goToPresetLocation("bin4");
            }
            else if(2.265 <= part_pose_y && part_pose_y <= 2.865){
                goToPresetLocation("bin3");
            }
            else if(-2.865 <= part_pose_y && part_pose_y <= -2.265){
                goToPresetLocation("bin7");                
            }
            else if(-3.679 <= part_pose_y && part_pose_y <= -3.079){
                goToPresetLocation("bin8");                
            }
        
        //////
        else if (-6.00 <= part_pose_x && part_pose_x <= -4.00){
            if (part_pose_y >= 3.800){
                goToPresetLocation("agv1_as1");
            }
            else if (0 <= part_pose_y && part_pose_y <= 3.000){
                goToPresetLocation("agv2_as1");
            }
            else if (-3.00 <= part_pose_y && part_pose_y <= 0){
                goToPresetLocation("agv3_as3");
            }
            else if (-3.800 >= part_pose_y){
                goToPresetLocation("agv4_as3");
            }
        }
        //////
         else if (-7.00 >= part_pose_x){
            if (part_pose_y >= 3.800){
                goToPresetLocation("agv1_as2");
            }
            else if (0 <= part_pose_y && part_pose_y <= 3.000){
                goToPresetLocation("agv2_as2");
            }
            else if (-3.00 <= part_pose_y && part_pose_y <= 0){
                goToPresetLocation("agv3_as4");
            }
            else if (-3.800 >= part_pose_y){
                goToPresetLocation("agv4_as4");
            }
        }

        // else {
        //     const moveit::core::JointModelGroup* joint_model_group = gantry_torso_group_.getCurrentState()->getJointModelGroup("gantry_torso");
        //     moveit::core::RobotStatePtr current_state = gantry_torso_group_.getCurrentState();
        //     // next get the current set of joint values for the group.
        //     current_state->copyJointGroupPositions(joint_model_group, joint_group_positions_);
        //     joint_group_positions_.at(0) = part_pose_x;
        //     joint_group_positions_.at(1) = -part_pose_y;
        //     gantry_torso_group_.setJointValueTarget(joint_group_positions_);
        //     gantry_torso_group_.move();
        // }
  
    }
    void Gantry::movePart(std::string part_type, std::string camera_frame, geometry_msgs::Pose goal_in_tray_frame, std::string agv) {
        //convert goal_in_tray_frame into world frame
        int type =0;
        bool flip_=false;
        if((agv.compare("as1") == 0) or (agv.compare("as2") == 0) or (agv.compare("as3") == 0) or (agv.compare("as4") == 0))
        {
            type = 1;
        }
        auto init_pose_in_world = utils::transformToWorldFrame(camera_frame, tf_buffer);
        if (pickPart(part_type, init_pose_in_world, type)) {
            placePart(init_pose_in_world, goal_in_tray_frame,part_type, agv,flip_); /// Changed function.

        }
    }

    bool Gantry::pickPart(std::string part_type, geometry_msgs::Pose part_init_pose, int ss) {
        gantry_group_.setMaxVelocityScalingFactor(1.0);
        gantry_arm_group_.setMaxVelocityScalingFactor(1.0);
        // gantryArmPreset();

        moveBaseTo(part_init_pose.position.x, part_init_pose.position.y);

        // // move the arm above the part to grasp
        // // gripper stays at the current z
        // // only modify its x and y based on the part to grasp
        // // In this case we do not need to use preset locations
        // // everything is done dynamically
        // arm_ee_link_pose.position.x = part_init_pose.position.x;
        // arm_ee_link_pose.position.y = part_init_pose.position.y;
        // arm_ee_link_pose.position.z = arm_ee_link_pose.position.z;
        // // move the arm
        // arm_group_.setPoseTarget(arm_ee_link_pose);
        // arm_group_.move();

        // Make sure the wrist is facing down
        // otherwise it will have a hard time attaching a part
        geometry_msgs::Pose gantry_arm_ee_link = gantry_arm_group_.getCurrentPose().pose;
        auto flat_orientation = utils::quaternionFromEuler(0, 1.57, 0);
        gantry_arm_ee_link.orientation.x = flat_orientation.getX();
        gantry_arm_ee_link.orientation.y = flat_orientation.getY();
        gantry_arm_ee_link.orientation.z = flat_orientation.getZ();
        gantry_arm_ee_link.orientation.w = flat_orientation.getW();
        
        // post-grasp pose 3
        // store the pose of the arm before it goes down to pick the part
        // we will bring the arm back to this pose after picking up the part
        auto postgrasp_pose3 = part_init_pose;
        postgrasp_pose3.orientation = gantry_arm_ee_link.orientation;
        postgrasp_pose3.position.z = gantry_arm_ee_link.position.z;

        // preset z depending on the part type
        // some parts are bigger than others
        // TODO: Add new z_pos values for the regulator and the battery
        double z_pos{};
        if (ss == 0){
        
        if (part_type.find("pump") != std::string::npos) {
            z_pos = 0.863;
        }
        if (part_type.find("sensor") != std::string::npos) {
            z_pos = 0.833;
        }
        if (part_type.find("regulator") != std::string::npos) {
            z_pos = 0.813;
        }
        if (part_type.find("battery") != std::string::npos) {
            z_pos = 0.793;
        }}
        else{
            if (part_type.find("pump") != std::string::npos) {
            z_pos = 0.883;
        }
        if (part_type.find("sensor") != std::string::npos) {
            z_pos = 0.853;
        }
        if (part_type.find("regulator") != std::string::npos) {
            z_pos = 0.833;
        }
        if (part_type.find("battery") != std::string::npos) {
            z_pos = 0.823;
        }
        }

        // flat_orientation = utils::quaternionFromEuler(0, 1.57, 0);
        // arm_ee_link_pose = arm_group_.getCurrentPose().pose;
        // arm_ee_link_pose.orientation.x = flat_orientation.getX();
        // arm_ee_link_pose.orientation.y = flat_orientation.getY();
        // arm_ee_link_pose.orientation.z = flat_orientation.getZ();
        // arm_ee_link_pose.orientation.w = flat_orientation.getW();

        
        // set of waypoints the arm will go through
        std::vector<geometry_msgs::Pose> waypoints;
        // pre-grasp pose: somewhere above the part
        auto pregrasp_pose = part_init_pose;
        pregrasp_pose.orientation = gantry_arm_ee_link.orientation;
        pregrasp_pose.position.z = z_pos + 0.03;

        // grasp pose: right above the part
        auto grasp_pose = part_init_pose;
        grasp_pose.orientation = gantry_arm_ee_link.orientation;
        grasp_pose.position.z = z_pos - 0.001;

        waypoints.push_back(pregrasp_pose);
        waypoints.push_back(grasp_pose);

        // activate gripper
        // sometimes it does not activate right away
        // so we are doing this in a loop
        while (!gripper_state_.enabled) {
            activateGripper();
        }

        // move the arm to the pregrasp pose
        gantry_arm_group_.setPoseTarget(pregrasp_pose);
        // ros::Duration(sleep(2.0));
        gantry_arm_group_.move();

        
        /* Cartesian motions are frequently needed to be slower for actions such as approach
        and retreat grasp motions. Here we demonstrate how to reduce the speed and the acceleration
        of the robot arm via a scaling factor of the maxiumum speed of each joint.
        */
        gantry_arm_group_.setMaxVelocityScalingFactor(0.05); // 0.05
        gantry_arm_group_.setMaxAccelerationScalingFactor(0.05);
        // plan the cartesian motion and execute it
        moveit_msgs::RobotTrajectory trajectory;
        const double jump_threshold = 0.0;
        const double eef_step = 0.01;
        double fraction = gantry_arm_group_.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        plan.trajectory_ = trajectory;
        gantry_arm_group_.execute(plan);

        // ros::Duration(sleep(2.0));

        // move the arm 1 mm down until the part is attached
        while (!gripper_state_.attached) {
            grasp_pose.position.z -= 0.001; // 0.001
            gantry_arm_group_.setPoseTarget(grasp_pose);
            gantry_arm_group_.move();
            ros::Duration(sleep(0.5));
        }
        
            gantry_arm_group_.setMaxVelocityScalingFactor(1.0);
            gantry_arm_group_.setMaxAccelerationScalingFactor(1.0);
            ROS_INFO_STREAM("[Gripper] = object attached");
            // ros::Duration(sleep(1.0));
            gantry_arm_group_.setPoseTarget(postgrasp_pose3);
            gantry_arm_group_.move();
            ros::Duration(sleep(1.0));

            // gantryArmPreset();

            return true;
        
    }
    bool Gantry::placePart(geometry_msgs::Pose part_init_pose, geometry_msgs::Pose part_pose_in_frame,std::string part_type, std::string agv,bool flip_)
    {
            // gantryArmPreset();       
            goToPresetLocation(agv);
              geometry_msgs::Pose target_pose_in_world;
    if(flip_){
         target_pose_in_world =  part_pose_in_frame;
    }else{
     target_pose_in_world = utils::transformToWorldFrame(
        part_pose_in_frame,
        agv);
    }
            // get the target pose of the part in the world frame
            
            // auto target_pose_in_world = utils::transformToWorldFrame(
            //     part_pose_in_frame,
            //     agv);

            geometry_msgs::Pose gantry_arm_ee_link = gantry_arm_group_.getCurrentPose().pose;
            auto flat_orientation = utils::quaternionFromEuler(0, 1.57, 0);
            gantry_arm_ee_link = gantry_arm_group_.getCurrentPose().pose;
            gantry_arm_ee_link.orientation.x = flat_orientation.getX();
            gantry_arm_ee_link.orientation.y = flat_orientation.getY();
            gantry_arm_ee_link.orientation.z = flat_orientation.getZ();
            gantry_arm_ee_link.orientation.w = flat_orientation.getW();

            // store the current orientation of the end effector now
            // so we can reuse it later
            tf2::Quaternion q_current(
                gantry_arm_ee_link.orientation.x,
                gantry_arm_ee_link.orientation.y,
                gantry_arm_ee_link.orientation.z,
                gantry_arm_ee_link.orientation.w);
            
            // move the arm above the agv
            // gripper stays at the current z
            // only modify its x and y based on the part to grasp
            // In this case we do not need to use preset locations
            // everything is done dynamically
            gantry_arm_ee_link.position.x = target_pose_in_world.position.x;
            gantry_arm_ee_link.position.y = target_pose_in_world.position.y;
            // move the arm
            gantry_arm_group_.setMaxVelocityScalingFactor(1.0);
            gantry_arm_group_.setPoseTarget(gantry_arm_ee_link);
            gantry_arm_group_.move();

            // orientation of the part in the bin, in world frame
            tf2::Quaternion q_init_part(
                part_init_pose.orientation.x,
                part_init_pose.orientation.y,
                part_init_pose.orientation.z,
                part_init_pose.orientation.w);
            // orientation of the part in the tray, in world frame
            tf2::Quaternion q_target_part(
                target_pose_in_world.orientation.x,
                target_pose_in_world.orientation.y,
                target_pose_in_world.orientation.z,
                target_pose_in_world.orientation.w);

            // relative rotation between init and target
            tf2::Quaternion q_rot = q_target_part * q_init_part.inverse();
            // apply this rotation to the current gripper rotation
            tf2::Quaternion q_rslt = q_rot * q_current;
            q_rslt.normalize();

            // orientation of the gripper when placing the part in the tray
            target_pose_in_world.orientation.x = q_rslt.x();
            target_pose_in_world.orientation.y = q_rslt.y();
            target_pose_in_world.orientation.z = q_rslt.z();
            target_pose_in_world.orientation.w = q_rslt.w();
            target_pose_in_world.position.z += 0.15;

            gantry_arm_group_.setMaxVelocityScalingFactor(0.3);
            gantry_arm_group_.setPoseTarget(target_pose_in_world);
            gantry_arm_group_.move();
            // ros::Duration(2.0).sleep();
            deactivateGripper();
            gantry_arm_group_.setMaxVelocityScalingFactor(1.0);
            // gantryArmPreset();
            goToPresetLocation(agv);

        return true;
    }

    /////////////////////////////////////////////////////
    nist_gear::VacuumGripperState Gantry::getGripperState()
    {
        return gripper_state_;
    }

    /**
     * @brief Pick up a part from a bin
     *
     * @param part Part to pick up
     * @return true Part was picked up
     * @return false Part was not picked up
     *
     * We use the group full_gantry_group_ to allow the robot more flexibility
     */

    /////////////////////////////////////////////////////
    void Gantry::gripper_state_callback(const nist_gear::VacuumGripperState::ConstPtr& gripper_state_msg)
    {
        gripper_state_ = *gripper_state_msg;
    }
    /////////////////////////////////////////////////////
    void Gantry::activateGripper()
    {
        nist_gear::VacuumGripperControl srv;
        srv.request.enable = true;
        gripper_control_client_.call(srv);

        ROS_INFO_STREAM("[Gantry][activateGripper] DEBUG: srv.response =" << srv.response);
    }

    /////////////////////////////////////////////////////
    void Gantry::deactivateGripper()
    {
        nist_gear::VacuumGripperControl srv;
        srv.request.enable = false;
        gripper_control_client_.call(srv);

        ROS_INFO_STREAM("[Gantry][deactivateGripper] DEBUG: srv.response =" << srv.response);
    }

    /////////////////////////////////////////////////////
    void Gantry::goToPresetLocation(std::string location_name)
    {

        GantryPresetLocation location;
        if (location_name.compare("home1") == 0) {
            location = home1_;
        }
        else if (location_name.compare("home2") == 0) {
            location = home2_;
        }
        else if (location_name.compare("agv1") == 0) {
            location = agv1_;
        }
        else if (location_name.compare("agv2") == 0) {
            location = agv2_;
        }
        else if (location_name.compare("agv3") == 0) {
            location = agv3_;
        }
        else if (location_name.compare("agv4") == 0) {
            location = agv4_;
        }
        
        //////////////
        else if (location_name.compare("agv1_as1") == 0) {
            location = agv1_as1;
        }
        else if (location_name.compare("agv2_as1") == 0) {
            location = agv2_as1;
        }
        else if (location_name.compare("agv3_as3") == 0) {
            location = agv3_as3;
        }
        else if (location_name.compare("agv4_as3") == 0) {
            location = agv4_as3;
        }
        ///////////
        else if (location_name.compare("agv1_as2") == 0) {
            // goToPresetLocation("home2");
            location = agv1_as2;
        }
        else if (location_name.compare("agv2_as2") == 0) {
            // goToPresetLocation("home2");
            location = agv2_as2;
        }
        else if (location_name.compare("agv3_as4") == 0) {
            // goToPresetLocation("home2");
            location = agv3_as4;
        }
        else if (location_name.compare("agv4_as4") == 0) {
            // goToPresetLocation("home2");
            location = agv4_as4;
        }
        ////////
        else if (location_name.compare("as1") == 0) {
            // goToPresetLocation("as1_waypoint");
            location = as1_;
        }
        else if (location_name.compare("as2") == 0) {
            // goToPresetLocation("as2_waypoint");
            location = as2_;
        }
        else if (location_name.compare("as3") == 0) {
            // goToPresetLocation("as3_waypoint");
            location = as3_;
        }
        else if (location_name.compare("as4") == 0) {
            // goToPresetLocation("as4_waypoint");
            location = as4_;
        }
        ///////////
        else if (location_name.compare("bin3") == 0) {
            location = bin3_;
        }
        else if (location_name.compare("bin4") == 0) {
            location = bin4_;
        }
        else if (location_name.compare("bin7") == 0) {
            location = bin7_;
        }
        else if (location_name.compare("bin8") == 0) {
            location = bin8_;
        }
        ////////////
        // else if (location_name.compare("as1_waypoint") == 0) {
        //     location = as1_waypoint;
        // }
        // else if (location_name.compare("as2_waypoint") == 0) {
        //     location = as2_waypoint;
        // }
        // else if (location_name.compare("as3_waypoint") == 0) {
        //     location = as3_waypoint;
        // }
        // else if (location_name.compare("as4_waypoint") == 0) {
        //     location = as4_waypoint;
        // }

        joint_group_positions_.at(0) = location.gantry_preset.at(0);
        joint_group_positions_.at(1) = location.gantry_preset.at(1);
        joint_group_positions_.at(2) = location.gantry_preset.at(2);
        joint_group_positions_.at(3) = location.gantry_preset.at(3);
        joint_group_positions_.at(4) = location.gantry_preset.at(4);
        joint_group_positions_.at(5) = location.gantry_preset.at(5);
        joint_group_positions_.at(6) = location.gantry_preset.at(6);
        joint_group_positions_.at(7) = location.gantry_preset.at(7);
        joint_group_positions_.at(8) = location.gantry_preset.at(8);

        gantry_group_.setJointValueTarget(joint_group_positions_);

        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        // check a plan is found first then execute the action
        bool success1 = (gantry_group_.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if (success1)
            gantry_group_.move();

    }

     void Gantry::gantryArmPreset()
    {
        GantryPresetLocation location;
        location = goodArm_;
       
        joint_group_positions_.at(0) = location.gantry_preset.at(0);
        joint_group_positions_.at(1) = location.gantry_preset.at(1);
        joint_group_positions_.at(2) = location.gantry_preset.at(2);
        joint_group_positions_.at(3) = location.gantry_preset.at(3);
        joint_group_positions_.at(4) = location.gantry_preset.at(4);
        joint_group_positions_.at(5) = location.gantry_preset.at(5);

        gantry_arm_group_.setJointValueTarget(joint_group_positions_);

        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        // check a plan is found first then execute the action
        bool success1 = (gantry_arm_group_.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if (success1)
            gantry_arm_group_.move();
    }

    ///////////////////////////
    ////// Callback Functions
    ///////////////////////////
geometry_msgs::Pose Gantry::transform_to_world_frame(const geometry_msgs::Pose& target, const std::string& agv_id)
{
    return utils::transformToWorldFrame(target, agv_id, tf_buffer);
}
    

    /////////////////////////////////////////////////////
    void Gantry::gantry_joint_states_callback_(const sensor_msgs::JointState::ConstPtr& joint_state_msg)
    {
        if (joint_state_msg->position.size() == 0) {
            ROS_ERROR("[Gantry][gantry_joint_states_callback_] joint_state_msg->position.size() == 0!");
        }
        current_joint_states_ = *joint_state_msg;
    }

    /////////////////////////////////////////////////////
    void Gantry::gantry_arm_controller_state_callback(const control_msgs::JointTrajectoryControllerState::ConstPtr& msg)
    {
        gantry_arm_controller_state_ = *msg;
    }
    void Gantry::gantry_controller_state_callback(const control_msgs::JointTrajectoryControllerState::ConstPtr& msg)
    {
        gantry_controller_state_ = *msg;
    }
