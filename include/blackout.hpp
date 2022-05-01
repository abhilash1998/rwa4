#include <ros/subscriber.h>
#include <ros/publisher.h>
#include <ros/timer.h>

#include <std_msgs/String.h>
#include <nist_gear/LogicalCameraImage.h>

#include <string>
#include <array>

#define NUM_LOGICAL_CAMERAS 4

// Forward declare
namespace ros {
    class NodeHandle;
}

class BlackoutDetector
{
protected:
    ros::Subscriber competition_state_sub;
    std::array<ros::Subscriber, NUM_LOGICAL_CAMERAS> logical_camera_subs;
    ros::Publisher blackout_status_pub;
    ros::Timer watch_for_blackouts_tmr;

    std::string current_competition_state;
    bool sensors_started;
    bool in_sensor_blackout;

    void competition_state_callback(const std_msgs::String::ConstPtr& msg);
    void logical_camera_image_callback(const nist_gear::LogicalCameraImage::ConstPtr& msg);
    void sensor_blackout_detected_callback(const ros::TimerEvent& evt);

    void publish_update(const bool active);

public:
    BlackoutDetector(ros::NodeHandle* const nh);
    ~BlackoutDetector();
};
