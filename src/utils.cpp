#include "utils.hpp"

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TransformStamped.h>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <array>

namespace utils
{
    tf2::Quaternion quaternionFromEuler(double r, double p, double y)
    {
        tf2::Quaternion q;
        q.setRPY(r, p, y);
        return q;
    }

    std::array<double, 3> eulerFromQuaternion(double x, double y, double z, double w)
    {
        const tf2::Matrix3x3 m(tf2::Quaternion(x, y, z, w));
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        return {roll, pitch, yaw};
    }

    std::array<double, 3> eulerFromQuaternion(const tf2::Quaternion& quat)
    {
        return eulerFromQuaternion(
            quat.getX(),
            quat.getY(),
            quat.getZ(),
            quat.getW()
        );
    }

    std::array<double, 3> eulerFromQuaternion(const geometry_msgs::Pose& pose)
    {
        return eulerFromQuaternion(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w
        );
    }

    geometry_msgs::Pose transformToWorldFrame(const std::string& part_in_camera_frame, tf2_ros::Buffer& tfBuffer)
    {
        ros::Rate rate(10);
        ros::Duration timeout(1.0);

        geometry_msgs::TransformStamped world_target_tf;
        geometry_msgs::TransformStamped ee_target_tf;

        for (int i = 0; i < 10; i++)
        {
            try {
                world_target_tf = tfBuffer.lookupTransform(
                    "world",
                    part_in_camera_frame,
                    ros::Time(0),
                    timeout
                );
            } catch (tf2::TransformException& ex) {
                ROS_WARN("%s", ex.what());
                ros::Duration(1.0).sleep();
                continue;
            }
        }

        geometry_msgs::Pose world_target{};
        world_target.position.x = world_target_tf.transform.translation.x;
        world_target.position.y = world_target_tf.transform.translation.y;
        world_target.position.z = world_target_tf.transform.translation.z;
        world_target.orientation.x = world_target_tf.transform.rotation.x;
        world_target.orientation.y = world_target_tf.transform.rotation.y;
        world_target.orientation.z = world_target_tf.transform.rotation.z;
        world_target.orientation.w = world_target_tf.transform.rotation.w;

        return world_target;
    }

    geometry_msgs::Pose transformToWorldFrame(const std::string& part_in_camera_frame)
    {
        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener tfListener(tfBuffer);
        return transformToWorldFrame(part_in_camera_frame, tfBuffer);
    }

    geometry_msgs::Pose transformToWorldFrame(const geometry_msgs::Pose& target, const std::string& agv, tf2_ros::Buffer& tfBuffer)
    {
        static tf2_ros::StaticTransformBroadcaster br;
        geometry_msgs::TransformStamped transformStamped;

    std::string kit_tray;
        if (agv.compare("agv1") == 0)
            kit_tray = "kit_tray_1";
        else if (agv.compare("agv2") == 0)
            kit_tray = "kit_tray_2";
        else if (agv.compare("agv3") == 0)
            kit_tray = "kit_tray_3";
        else if (agv.compare("agv4") == 0)
            kit_tray = "kit_tray_4";
        else if (agv.compare("as1") == 0)
            kit_tray = "briefcase_1";
        else if (agv.compare("as2") == 0)
            kit_tray = "briefcase_2";
        else if (agv.compare("as3") == 0)
            kit_tray = "briefcase_3";
        else if (agv.compare("as4") == 0)
            kit_tray = "briefcase_4";
        else if (agv.compare("bin1") == 0)
            kit_tray = "bin_1";
        else if (agv.compare("bin2") == 0)
            kit_tray = "bin_2";
        else if (agv.compare("bin5") == 0)
            kit_tray = "bin_5";
        else if (agv.compare("bin6") == 0)
            kit_tray = "bin_6";

        transformStamped.header.stamp = ros::Time::now();
        transformStamped.header.frame_id = kit_tray;
        transformStamped.child_frame_id = "target_frame";
        transformStamped.transform.translation.x = target.position.x;
        transformStamped.transform.translation.y = target.position.y;
        transformStamped.transform.translation.z = target.position.z;
        transformStamped.transform.rotation.x = target.orientation.x;
        transformStamped.transform.rotation.y = target.orientation.y;
        transformStamped.transform.rotation.z = target.orientation.z;
        transformStamped.transform.rotation.w = target.orientation.w;

        for (int i{ 0 }; i < 15; ++i)
            br.sendTransform(transformStamped);

        ros::Rate rate(10);
        ros::Duration timeout(1.0);

        geometry_msgs::TransformStamped world_pose_tf;
        geometry_msgs::TransformStamped ee_target_tf;

        for (int i = 0; i < 10; i++)
        {
            try {
                world_pose_tf = tfBuffer.lookupTransform(
                    "world",
                    "target_frame",
                    ros::Time(0),
                    timeout
                );
            } catch (tf2::TransformException& ex) {
                ROS_WARN("%s", ex.what());
                ros::Duration(1.0).sleep();
                continue;
            }
        }

        geometry_msgs::Pose world_pose{};
        world_pose.position.x = world_pose_tf.transform.translation.x;
        world_pose.position.y = world_pose_tf.transform.translation.y;
        world_pose.position.z = world_pose_tf.transform.translation.z;
        world_pose.orientation.x = world_pose_tf.transform.rotation.x;
        world_pose.orientation.y = world_pose_tf.transform.rotation.y;
        world_pose.orientation.z = world_pose_tf.transform.rotation.z;
        world_pose.orientation.w = world_pose_tf.transform.rotation.w;

        return world_pose;
    }

    geometry_msgs::Pose transformToWorldFrame(const geometry_msgs::Pose& target, const std::string& agv)
    {
        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener tfListener(tfBuffer);
        return transformToWorldFrame(target, agv, tfBuffer);
    }
}
