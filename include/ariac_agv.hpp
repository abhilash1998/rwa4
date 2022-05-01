#include <ros/subscriber.h>
#include <ros/service_client.h>
#include <std_msgs/String.h>

#include <string>

// Forward declare
namespace ros {
    class NodeHandle;
}

class AriacAgv
{
protected:
    const int number;
    const std::string id;

    ros::Subscriber state_sub;
    ros::Subscriber station_sub;
    ros::ServiceClient submit_shipment_scl;

    std::string curr_state;
    std::string curr_station;

    void state_callback(const std_msgs::String::ConstPtr& msg);
    void station_callback(const std_msgs::String::ConstPtr& msg);

public:
    AriacAgv(ros::NodeHandle* const nh, const int agv_number);
    ~AriacAgv();

    std::string get_id() const;
    bool is_ready_to_deliver() const;

    bool submit_shipment(const std::string& assembly_station_name,
                         const std::string& shipment_type);
};
