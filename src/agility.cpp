#include "agility.hpp"

#include <ros/node_handle.h>

#include <geometry_msgs/Pose.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <algorithm>
#include <numeric>
#include <cmath>

void AgilityChallenger::order_callback(const nist_gear::Order::ConstPtr& msg)
{
    // Temporary functionality, where order_1 is 'hardcoded' to be a high
    // priority order, others are not
    pending_order = *msg;
    pending_order_priority = (pending_order.order_id == "order_1") ? 2 : 1;

    ROS_INFO_STREAM("Received order with ID '"
                    << pending_order.order_id
                    << "' with priority "
                    << pending_order_priority);
}

void AgilityChallenger::blackout_status_callback(const std_msgs::Bool::ConstPtr& msg)
{
    in_sensor_blackout = msg->data;
    ROS_INFO_STREAM("Blackout status updated: " << std::to_string(in_sensor_blackout));
}

void AgilityChallenger::help_logical_camera_image_callback(const nist_gear::LogicalCameraImage::ConstPtr& msg, const int bin_idx)
{
    // Clear the list of parts that this camera currently sees, and repopulate
    // it with updated data
    // Notice the vector is a reference
    std::vector<std::string>& current_parts_bin_idx = current_detected_parts[bin_idx];
    current_parts_bin_idx.clear();
    for (auto iter_model = msg->models.begin(); iter_model != msg->models.end(); ++iter_model)
    {
        current_parts_bin_idx.push_back(iter_model->type);
    }
}

void AgilityChallenger::help_logical_camera_as_callback(const nist_gear::LogicalCameraImage::ConstPtr& msg, const int bin_idx)
{
    // Clear the list of parts that this camera currently sees, and repopulate
    // it with updated data
    // Notice the vector is a reference
    std::vector<std::string>& current_parts_bin_idx = current_detected_parts_as1[bin_idx];
    current_parts_bin_idx.clear();
    for (auto iter_model = msg->models.begin(); iter_model != msg->models.end(); ++iter_model)
    {
        current_parts_bin_idx.push_back(iter_model->type);
    }
}

void AgilityChallenger::help_quality_control_sensor_callback(const nist_gear::LogicalCameraImage::ConstPtr& msg, const int lc_idx)
{
    const nist_gear::LogicalCameraImage new_results = *msg;
    if (new_results.models.size() != current_qc_results[lc_idx].models.size())
    {
        ROS_INFO_STREAM("Number of faulty models from logical camera #"
                        << lc_idx+1
                        << " changed from "
                        << current_qc_results[lc_idx].models.size()
                        << " to "
                        << new_results.models.size());
    }

    current_qc_results[lc_idx] = new_results;

    const std::string agv_id = std::string("agv") + std::to_string(lc_idx+1);

    // Loop through each of the existing parts that need verification. Create a
    // new list, where parts are only added if they show up as faulty by the QC
    // sensor.
    std::vector<PartForFaultVerification> new_list;
    for (const PartForFaultVerification& part : parts_for_fault_verification[agv_id])
    {
        // We have the part already resolved in world frame, store its 3D
        // position in ppt
        bool found_part = false;
        const geometry_msgs::Point ppt = std::get<2>(part).position;
        for (const nist_gear::Model model : new_results.models)
        {
            // Get this model in world frame, store its 3D position in mpt
            geometry_msgs::Pose mpose;
            tf2::Transform qc_camera_tf, part_tf;
            tf2::fromMsg(current_qc_results[lc_idx].pose, qc_camera_tf);
            tf2::fromMsg(model.pose, part_tf);
            tf2::toMsg(qc_camera_tf * part_tf, mpose);
            const geometry_msgs::Point mpt = mpose.position;

            // If its within (0.1m, 0.1m) on the X/Y plane, consider it the same model
            const double dx = std::abs(mpt.x - ppt.x);
            const double dy = std::abs(mpt.y - ppt.y);
            ROS_DEBUG_STREAM("ppt=[x:" << ppt.x << ",y:" << ppt.y << "] vs. mpt=[x:" << mpt.x << ",y:" << mpt.y << "]");
            ROS_DEBUG_STREAM("dx=" << dx << ", dy=" << dy);
            if ((dx < 0.1) && (dy < 0.1))
            {
                // We found the part we were interested in
                found_part = true;
                new_list.push_back(part);
                break;
            }
        }
        if (!found_part)
        {
            // If we didn't find the part, it means the model wasn't picked up
            // by the QC camera, meaning that either 1.) it was never faulty
            // and this confirmed that, or 2.) it was faulty but was removed
            // from the tray by the robot
            ROS_INFO_STREAM("Removed '"
                            << std::get<0>(part).type
                            << "' of order '"
                            << std::get<1>(part)
                            << "' at [x:"
                            << ppt.x
                            << ",y:"
                            << ppt.y
                            << "] from faulty part verification queue");
        }
    }
    parts_for_fault_verification[agv_id] = new_list;
}

void AgilityChallenger::logical_camera_as11_callback(const nist_gear::LogicalCameraImage::ConstPtr& msg)
{
    // Callback for bin #1 with index 0
    help_logical_camera_as_callback(msg, 0);
}

void AgilityChallenger::logical_camera_as12_callback(const nist_gear::LogicalCameraImage::ConstPtr& msg)
{
    // Callback for bin #1 with index 0
    help_logical_camera_as_callback(msg, 1);
}
void AgilityChallenger::logical_camera_as21_callback(const nist_gear::LogicalCameraImage::ConstPtr& msg)
{
    // Callback for bin #1 with index 0
    help_logical_camera_as_callback(msg, 2);
}

void AgilityChallenger::logical_camera_as22_callback(const nist_gear::LogicalCameraImage::ConstPtr& msg)
{
    // Callback for bin #1 with index 0
    help_logical_camera_as_callback(msg, 3);
}

void AgilityChallenger::logical_camera_as31_callback(const nist_gear::LogicalCameraImage::ConstPtr& msg)
{
    // Callback for bin #1 with index 0
    help_logical_camera_as_callback(msg, 4);
}

void AgilityChallenger::logical_camera_as32_callback(const nist_gear::LogicalCameraImage::ConstPtr& msg)
{
    // Callback for bin #1 with index 0
    help_logical_camera_as_callback(msg, 5);
}
void AgilityChallenger::logical_camera_as41_callback(const nist_gear::LogicalCameraImage::ConstPtr& msg)
{
    // Callback for bin #1 with index 0
    help_logical_camera_as_callback(msg, 6);
}

void AgilityChallenger::logical_camera_as42_callback(const nist_gear::LogicalCameraImage::ConstPtr& msg)
{
    // Callback for bin #1 with index 0
    help_logical_camera_as_callback(msg, 7);
}

void AgilityChallenger::logical_camera_image1_callback(const nist_gear::LogicalCameraImage::ConstPtr& msg)
{
    // Callback for bin #1 with index 0
    help_logical_camera_image_callback(msg, 0);
}

void AgilityChallenger::logical_camera_image2_callback(const nist_gear::LogicalCameraImage::ConstPtr& msg)
{
    // Callback for bin #2 with index 1
    help_logical_camera_image_callback(msg, 1);
}

void AgilityChallenger::logical_camera_image3_callback(const nist_gear::LogicalCameraImage::ConstPtr& msg)
{
    // Callback for bin #3 with index 2
    help_logical_camera_image_callback(msg, 2);
}

void AgilityChallenger::logical_camera_image4_callback(const nist_gear::LogicalCameraImage::ConstPtr& msg)
{
    // Callback for bin #4 with index 3
    help_logical_camera_image_callback(msg, 3);
}

void AgilityChallenger::quality_control_sensor1_callback(const nist_gear::LogicalCameraImage::ConstPtr& msg)
{
    // Callback for agv #1 with index 0
    help_quality_control_sensor_callback(msg, 0);
}

void AgilityChallenger::quality_control_sensor2_callback(const nist_gear::LogicalCameraImage::ConstPtr& msg)
{
    // Callback for agv #2 with index 1
    help_quality_control_sensor_callback(msg, 1);
}

void AgilityChallenger::quality_control_sensor3_callback(const nist_gear::LogicalCameraImage::ConstPtr& msg)
{
    // Callback for agv #3 with index 2
    help_quality_control_sensor_callback(msg, 2);
}

void AgilityChallenger::quality_control_sensor4_callback(const nist_gear::LogicalCameraImage::ConstPtr& msg)
{
    // Callback for agv #4 with index 3
    help_quality_control_sensor_callback(msg, 3);
}

AgilityChallenger::AgilityChallenger(ros::NodeHandle* const nh) :
    pending_order_priority(0),
    in_sensor_blackout(false)
{
    orders_subs = nh->subscribe<nist_gear::Order>(
        "/ariac/orders",
        1,
        &AgilityChallenger::order_callback,
        this
    );
    blackout_sub = nh->subscribe<std_msgs::Bool>(
        "/group3/blackout_active",
        1,
        &AgilityChallenger::blackout_status_callback,
        this
    );
    logical_camera_as1[0] = nh->subscribe<nist_gear::LogicalCameraImage>(
        "/ariac/logical_camera_9",
        1,
        &AgilityChallenger::logical_camera_as11_callback,
        this
    );
    logical_camera_as1[1] = nh->subscribe<nist_gear::LogicalCameraImage>(
        "/ariac/logical_camera_10",
        1,
        &AgilityChallenger::logical_camera_as12_callback,
        this
    );
    logical_camera_as2[0] = nh->subscribe<nist_gear::LogicalCameraImage>(
        "/ariac/logical_camera_11",
        1,
        &AgilityChallenger::logical_camera_as21_callback,
        this
    );
    logical_camera_as2[1] = nh->subscribe<nist_gear::LogicalCameraImage>(
        "/ariac/logical_camera_12",
        1,
        &AgilityChallenger::logical_camera_as22_callback,
        this
    );
    logical_camera_as3[0] = nh->subscribe<nist_gear::LogicalCameraImage>(
        "/ariac/logical_camera_13",
        1,
        &AgilityChallenger::logical_camera_as31_callback,
        this
    );
    logical_camera_as3[1] = nh->subscribe<nist_gear::LogicalCameraImage>(
        "/ariac/logical_camera_14",
        1,
        &AgilityChallenger::logical_camera_as32_callback,
        this
    );
    logical_camera_as4[0] = nh->subscribe<nist_gear::LogicalCameraImage>(
        "/ariac/logical_camera_15",
        1,
        &AgilityChallenger::logical_camera_as41_callback,
        this
    );
    logical_camera_as4[1] = nh->subscribe<nist_gear::LogicalCameraImage>(
        "/ariac/logical_camera_16",
        1,
        &AgilityChallenger::logical_camera_as42_callback,
        this
    );
    logical_camera_subs[0] = nh->subscribe<nist_gear::LogicalCameraImage>(
        "/ariac/logical_camera_1",
        1,
        &AgilityChallenger::logical_camera_image1_callback,
        this
    );
    logical_camera_subs[1] = nh->subscribe<nist_gear::LogicalCameraImage>(
        "/ariac/logical_camera_2",
        1,
        &AgilityChallenger::logical_camera_image2_callback,
        this
    );
    logical_camera_subs[2] = nh->subscribe<nist_gear::LogicalCameraImage>(
        "/ariac/logical_camera_3",
        1,
        &AgilityChallenger::logical_camera_image3_callback,
        this
    );
    logical_camera_subs[3] = nh->subscribe<nist_gear::LogicalCameraImage>(
        "/ariac/logical_camera_4",
        1,
        &AgilityChallenger::logical_camera_image4_callback,
        this
    );
    quality_control_sensor_subs[0] = nh->subscribe<nist_gear::LogicalCameraImage>(
        "/ariac/quality_control_sensor_1",
        1,
        &AgilityChallenger::quality_control_sensor1_callback,
        this
    );
    quality_control_sensor_subs[1] = nh->subscribe<nist_gear::LogicalCameraImage>(
        "/ariac/quality_control_sensor_2",
        1,
        &AgilityChallenger::quality_control_sensor2_callback,
        this
    );
    quality_control_sensor_subs[2] = nh->subscribe<nist_gear::LogicalCameraImage>(
        "/ariac/quality_control_sensor_3",
        1,
        &AgilityChallenger::quality_control_sensor3_callback,
        this
    );
    quality_control_sensor_subs[3] = nh->subscribe<nist_gear::LogicalCameraImage>(
        "/ariac/quality_control_sensor_4",
        1,
        &AgilityChallenger::quality_control_sensor4_callback,
        this
    );

    parts_for_fault_verification = {
        {"agv1", {}},
        {"agv2", {}},
        {"agv3", {}},
        {"agv4", {}}
    };
}

AgilityChallenger::~AgilityChallenger()
{
}

bool AgilityChallenger::is_sensor_blackout_active() const
{
    return in_sensor_blackout;
}

void AgilityChallenger::queue_for_fault_verification(const nist_gear::Product& product,
                                                     const std::string& order_id,
                                                     const std::string& agv_id,
                                                     const geometry_msgs::Pose& objective_pose_in_world)
{
    ROS_INFO_STREAM("Queueing part for fault verification: "
                    << order_id
                    << ", "
                    << product.type
                    << ", "
                    << agv_id
                    << ", [x:"
                    << objective_pose_in_world.position.x
                    << ",y:"
                    << objective_pose_in_world.position.y
                    << "]");
    parts_for_fault_verification[agv_id].push_back(std::make_tuple(
        product,
        order_id,
        objective_pose_in_world
    ));
}

bool AgilityChallenger::needs_fault_verification(const std::string& agv_id)
{
    return !parts_for_fault_verification[agv_id].empty();
}

int AgilityChallenger::consume_pending_order(nist_gear::Order& order)
{
    // 'Consume' the current order
    order = pending_order;
    pending_order = nist_gear::Order();

    // 'Consume' its priority
    const int priority = pending_order_priority;
    pending_order_priority = 0;

    return priority;
}

bool AgilityChallenger::higher_priority_order_requested(const int current_priority) const
{
    return pending_order_priority > current_priority;
}

std::vector<int> AgilityChallenger::get_camera_indices_of(const std::string& product_type) const
{
    std::vector<int> indices;
    for (int i = 0; i < current_detected_parts.size(); i++)
    {
        const std::vector<std::string>& lcd = current_detected_parts[i];
        if (lcd.cend() != std::find(lcd.cbegin(), lcd.cend(), product_type))
        {
            indices.push_back(i+1);
        }
    }
    return indices;
}

std::vector<int> AgilityChallenger::get_as1_indices_of(const std::string& product_type) const
{
    std::vector<int> indices;
    for (int i = 0; i < current_detected_parts_as1.size(); i++)
    {
        const std::vector<std::string>& lcd = current_detected_parts_as1[i];
        if (lcd.cend() != std::find(lcd.cbegin(), lcd.cend(), product_type))
        {
            indices.push_back(i+9);
        }
    }
    return indices;
}

std::string AgilityChallenger::get_logical_camera_contents() const
{
    std::string str = "{";
    for (int i = 0; i < current_detected_parts.size(); i++)
    {
        if (i != 0)
        {
            str += ", ";
        }

        const std::vector<std::string>& lcd = current_detected_parts[i];
        str += (std::to_string(i+1) + ": [");
        if (!lcd.empty())
        {
            str += std::accumulate(
                std::next(lcd.begin()),
                lcd.end(),
                lcd[0],
                [](const std::string& a, const std::string& b) { return std::move(a) + ',' + b; }
            );
        }
        str += "]";
    }
    return str;
}

bool AgilityChallenger::get_agv_faulty_part(const std::string& order_id,
                                            std::string& agv_id,
                                            nist_gear::Product& product,
                                            geometry_msgs::Pose& pick_frame) const
{
    for (const auto& map_iter : parts_for_fault_verification)
    {
        for (const PartForFaultVerification& part : map_iter.second)
        {
            if (std::get<1>(part) == order_id)
            {
                agv_id = map_iter.first;
                product = std::get<0>(part);
                pick_frame = std::get<2>(part);
                return true;
            }
        }
    }
    return false;
}
