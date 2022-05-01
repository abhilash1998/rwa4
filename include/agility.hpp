#include <ros/subscriber.h>

#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <nist_gear/Order.h>
#include <nist_gear/LogicalCameraImage.h>
#include <nist_gear/Product.h>

#include <string>
#include <array>
#include <vector>
#include <unordered_map>
#include <tuple>

class AgilityChallenger
{
protected:
    // A tuple that describes a part that has been placed but needs quality
    // control checking. First param is the product that was placed, second arg
    // is the order ID it is a part of, third arg is the objective pose for the
    // part in world frame.
    using PartForFaultVerification = std::tuple<nist_gear::Product, std::string, geometry_msgs::Pose>;

    ros::Subscriber orders_subs;
    ros::Subscriber blackout_sub;
    std::array<ros::Subscriber, 4> logical_camera_subs;
    std::array<ros::Subscriber, 2> logical_camera_as1;
    std::array<ros::Subscriber, 2> logical_camera_as2;
    std::array<ros::Subscriber, 2> logical_camera_as3;
    std::array<ros::Subscriber, 2> logical_camera_as4;
    std::array<ros::Subscriber, 4> quality_control_sensor_subs;

    // The relative priority of the order at \a pending_order. If this is 0,
    // the \a pending_order is not populated / not a valid order.
    int pending_order_priority;

    // An order that was received and is pending to be catered. This is
    // consumed by callers of consume_pending_order(nist_gear::Order&).
    nist_gear::Order pending_order;

    // The parts currently detected by the logical cameras
    std::array<std::vector<std::string>, 4> current_detected_parts;
    std::array<std::vector<std::string>, 8> current_detected_parts_as1;

    // True if a sensor blackout was detected right now, false otherwise
    bool in_sensor_blackout;

    // A map that relates an AGV ID (which itself corresponds to an logical
    // camera index) to a list of parts that are pending fault verification.
    std::unordered_map<std::string, std::vector<PartForFaultVerification>> parts_for_fault_verification;

    // The most recently heard quality control updates heard from QC cameras
    // 1-4 (with indices [0,3])
    std::array<nist_gear::LogicalCameraImage, 4> current_qc_results;

    void help_logical_camera_image_callback(const nist_gear::LogicalCameraImage::ConstPtr& msg, const int bin_idx);
    void help_logical_camera_as_callback(const nist_gear::LogicalCameraImage::ConstPtr& msg, const int bin_idx);
    void help_quality_control_sensor_callback(const nist_gear::LogicalCameraImage::ConstPtr& msg, const int lc_idx);

    void annouce_world_tf(const std::string part_name, const std::string frame);
    void order_callback(const nist_gear::Order::ConstPtr& msg);
    void blackout_status_callback(const std_msgs::Bool::ConstPtr& msg);
    void logical_camera_image1_callback(const nist_gear::LogicalCameraImage::ConstPtr& msg);
    void logical_camera_image2_callback(const nist_gear::LogicalCameraImage::ConstPtr& msg);
    void logical_camera_image3_callback(const nist_gear::LogicalCameraImage::ConstPtr& msg);
    void logical_camera_image4_callback(const nist_gear::LogicalCameraImage::ConstPtr& msg);
    void logical_camera_as11_callback(const nist_gear::LogicalCameraImage::ConstPtr& msg);
    void logical_camera_as12_callback(const nist_gear::LogicalCameraImage::ConstPtr& msg);
    void logical_camera_as21_callback(const nist_gear::LogicalCameraImage::ConstPtr& msg);
    void logical_camera_as22_callback(const nist_gear::LogicalCameraImage::ConstPtr& msg);
    void logical_camera_as31_callback(const nist_gear::LogicalCameraImage::ConstPtr& msg);
    void logical_camera_as32_callback(const nist_gear::LogicalCameraImage::ConstPtr& msg);
    void logical_camera_as41_callback(const nist_gear::LogicalCameraImage::ConstPtr& msg);
    void logical_camera_as42_callback(const nist_gear::LogicalCameraImage::ConstPtr& msg);
    void quality_control_sensor1_callback(const nist_gear::LogicalCameraImage::ConstPtr& msg);
    void quality_control_sensor2_callback(const nist_gear::LogicalCameraImage::ConstPtr& msg);
    void quality_control_sensor3_callback(const nist_gear::LogicalCameraImage::ConstPtr& msg);
    void quality_control_sensor4_callback(const nist_gear::LogicalCameraImage::ConstPtr& msg);

public:
    AgilityChallenger(ros::NodeHandle* const nh);
    ~AgilityChallenger();

    // Getter for \a sensor_blackout_active.
    // @return The value of \a sensor_blackout_active (true if a blackout is
    // active, false otherwise).
    bool is_sensor_blackout_active() const;

    // Queue a part for fault verification. Uses the given values to create an
    // instance of PartForFaultVerification in \a parts_for_fault_verification.
    // @param product The product that needs verification
    // @param order_id The ID of the order this part is associated with
    // @param agv_id The ID of the AGV this part was placed on
    // @param objective_pose_in_world The objective pose of this product that
    // was placed, in world frame
    void queue_for_fault_verification(const nist_gear::Product& product,
                                      const std::string& order_id,
                                      const std::string& agv_id,
                                      const geometry_msgs::Pose& objective_pose_in_world);

    // Check if any parts of the given AGV still need verification
    // @param agv_id The ID of the AGV
    // @return True if there are parts pending verification, false otherwise
    bool needs_fault_verification(const std::string& agv_id);

    // Pass ownership of \a pending_order off to the caller of this method if
    // it is populated.
    // @param order If there is a pending order (\a pending_order_priority is
    // nonzero), this is overwritten with \a pending_order.
    // @return The current value of \a pending_order_priority.
    int consume_pending_order(nist_gear::Order& order);

    // Get whether or not there is an order with a priority that is higher than
    // the given one.
    // @param current_priority The priority of the order being catered by the
    // caller of this method.
    // @return Whether or not \a pending_order_priority is greater than
    // \a current_priority.
    bool higher_priority_order_requested(const int current_priority) const;

    std::vector<int> get_camera_indices_of(const std::string& product_type) const;
    std::vector<int> get_as1_indices_of(const std::string& product_type) const;
    std::string get_logical_camera_contents() const;

    // If there are any faulty parts, get the pick pose for one of them.
    // @param agv_id If this method returns true, then this value is
    // overwritten with the ID of the AGV that this part is placed on
    // @param product If this method returns true, then this value is
    // overwritten with the product that requires this type of part
    // @param pick_frame If this method returns true, then this value is
    // overwritten with the pose of a faulty part resolved in world frame,
    // if false then this value is not overwritten
    // @return True if there is a faulty part, false otherwise.
    bool get_agv_faulty_part(const std::string& order_id,
                             std::string& agv_id,
                             nist_gear::Product& product,
                             geometry_msgs::Pose& pick_frame) const;
};
