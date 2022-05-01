#include <ros/ros.h>

#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>

#include <memory>
#include <unordered_map>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include "agility.hpp"
#include "ariac_agv.hpp"
#include "arm.hpp"
#include "gantry.hpp"

#define NUM_AGVS 4
std::vector<std::string> hit_list;
using AriacAgvMap = std::unordered_map<std::string, std::shared_ptr<AriacAgv>>;

namespace {
    std::string build_part_frame(const std::string& product_type, const int camera_index, const int counter)
    {
        return std::string("logical_camera_")
            + std::to_string(camera_index)
            + "_"
            + product_type
            + "_"
            + std::to_string(counter)
            + "_frame"
        ;
    }

    bool does_frame_exist(const std::string& part_in_camera_frame, const double timeout)
    {
        bool rc = true;
        static tf2_ros::Buffer tfBuffer;
        static tf2_ros::TransformListener tfListener(tfBuffer);
        try {
            tfBuffer.lookupTransform(
                "world",
                part_in_camera_frame,
                ros::Time(0),
                ros::Duration(timeout)
            );
        } catch (tf2::TransformException& ex) {
            rc = false;
        }
        return rc;
    }

    void cater_higher_priority_order_if_necessary(const AriacAgvMap& agv_map,
                                                  AgilityChallenger* const agility,
                                                  Gantry* const arm,
                                                  const int current_order_priority);

    

    void cater_assembly_shipments(const AriacAgvMap& agv_map,
                                 AgilityChallenger* const agility,
                                 Gantry* const arm,
                                 const int order_priority,
                                 const std::string& order_id,
                                 std::vector<nist_gear::AssemblyShipment>& assembly_shipments)
    {
        // Ignore request if there are no assembly shipments
        if (assembly_shipments.empty())
        {
            return;
        }

        ROS_INFO_STREAM("Catering "
                        << assembly_shipments.size()
                        << " assembly shipments from order with priority of "
                        << order_priority);

        int counter = 0;
        // parse each assembly shipment
        for (const auto& as : assembly_shipments)
        {
            std::vector<nist_gear::Product> products = as.products;
            if (products.empty())
            {
                ROS_FATAL_STREAM("Assembly shipment had no products?");
                ros::shutdown();
                return;
            }

            // loop through each product in this shipment
            while (!products.empty())
            {
                // Remove this product from the list, with the intention that
                // it will be catered to
                const nist_gear::Product product = products.front();
                products.erase(products.begin());
                ROS_INFO_STREAM("Catering product '"
                                << product.type
                                << "', "
                                << products.size()
                                << " remaining afterwards");

                // Get the bins in which this part appears
                const std::vector<int> bin_indices = agility->get_as1_indices_of(product.type);
                if (bin_indices.empty())
                {
                    ROS_FATAL_STREAM(
                        "No matching part '"
                        << product.type
                        << "' found by any logical camera with contents "
                        << agility->get_logical_camera_contents()
                    );
                    // ros::shutdown();
                    // return;
                }
                // ROS_ERROR_STREAM(bin_indices.first);

                // counter++;

                for (auto iter = bin_indices.cbegin(); iter != bin_indices.cend(); ++iter)
                {
                    bool item_ready = false;
                    if( as.station_id.compare("as1")==0 && ( *iter == 9 or *iter == 10 ) )
                    {
                        item_ready = true;
                    }
                    else if( as.station_id.compare("as2")==0 && ( *iter == 11 or *iter == 12 ) )
                    {
                        item_ready = true;
                    }
                    else if( as.station_id.compare("as3")==0 && ( *iter == 13 or *iter == 14 ) )
                    {
                        item_ready = true;
                    }
                    else if( as.station_id.compare("as4")==0 && ( *iter == 15 or *iter == 16 ) )
                    {
                        item_ready = true;
                    }
                    
                    if(!item_ready)
                    {
                        continue;
                    }

                    std::string part_frame;
                    for (counter = 1; counter <= 12; counter++)
                    {
                        bool hit = false;
                        part_frame = build_part_frame(product.type, *iter, counter);
                        if (does_frame_exist(part_frame, 0.5))
                        {
                            if (hit_list.empty())
                            {   
                                // ROS_INFO_STREAM("Hit List Updated");
                                hit_list.emplace_back(product.type + std::to_string(counter));
                            }
                            else
                            {
                                for (auto s : hit_list)
                                {
                                    if(s.compare(product.type + std::to_string(counter)) == 0)
                                    {
                                        hit = true;
                                        break;
                                    }
                                }
                            }
                            if (hit) {continue;}
                            else 
                            {
                                hit_list.emplace_back(product.type + std::to_string(counter));
                                break;
                            }
                        }
                    }
                    if (!does_frame_exist(part_frame, 0.5))
                    {
                        continue;
                    }

                    // Move the part from where it is to the AGV bed
                    ROS_INFO_STREAM("Moving part '" << product.type << "' to '" << as.station_id << "' (" << part_frame << ")");
                    arm->movePart(product.type, part_frame, product.pose, as.station_id);
                    ROS_INFO_STREAM("Placed part '" << product.type << "' at '" << as.station_id << "'");
                    // agility->queue_for_fault_verification(
                    //     product,
                    //     order_id,
                    //     as.station_id,
                    //     arm->transform_to_world_frame(product.pose, as.agv_id)
                    // );
                    ros::Duration(0.2).sleep();

                    // Give an opportunity for higher priority orders
                    cater_higher_priority_order_if_necessary(agv_map, agility, arm, order_priority);

                    break;
                }

            }

        }
    }

    void cater_order(const AriacAgvMap& agv_map,
                     AgilityChallenger* const agility,
                     Gantry* const arm,
                     const int order_priority,
                     nist_gear::Order& order)
    {
        cater_assembly_shipments(
            agv_map,
            agility,
            arm,
            order_priority,
            order.order_id,
            order.assembly_shipments
        );
    }

    void cater_higher_priority_order_if_necessary(const AriacAgvMap& agv_map,
                                                  AgilityChallenger* const agility,
                                                  Gantry* const arm,
                                                  const int current_order_priority)
    {
        if (agility->higher_priority_order_requested(current_order_priority))
        {
            ROS_INFO_STREAM("Catering higher priority order...");
            int new_order_priority;
            nist_gear::Order new_order;
            new_order_priority = agility->consume_pending_order(new_order);
            cater_order(agv_map, agility, arm, new_order_priority, new_order);
            ROS_INFO_STREAM("Finished higher priority order, returning to previous order");
        }
    }
}

void add_objects(ros::NodeHandle* const nh)
{
    ros::Publisher planning_scene_diff_publisher = nh->advertise<moveit_msgs::PlanningScene>("/ariac/gantry/planning_scene", 1);
    ros::WallDuration sleep_t(0.5);
    while (planning_scene_diff_publisher.getNumSubscribers() < 1) {
        sleep_t.sleep();
    }
    // Now let's define a collision object ROS message for the robot to avoid.
    moveit_msgs::PlanningScene planning_scene;
    moveit_msgs::CollisionObject collision_object;
    shape_msgs::SolidPrimitive primitive;
    geometry_msgs::Pose box_pose;

    // AS1
    collision_object.header.frame_id = "world";
    collision_object.id = "as1";
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = 1.2;
    primitive.dimensions[primitive.BOX_Y] = 1.2;
    primitive.dimensions[primitive.BOX_Z] = 1.2;  //agv = 1.5 //as = 1.2
    box_pose.orientation.w = 1.0;
    box_pose.position.x = -7.3; //-12.3
    box_pose.position.y = 3.0;  //3 -3
    box_pose.position.z = 0;
    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;
    planning_scene.world.collision_objects.push_back(collision_object);

    // AS1
    collision_object.header.frame_id = "world";
    collision_object.id = "as1";
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = 1.2;
    primitive.dimensions[primitive.BOX_Y] = 1.2;
    primitive.dimensions[primitive.BOX_Z] = 1.2;  //agv = 1.5 //as = 1.2
    box_pose.orientation.w = 1.0;
    box_pose.position.x = -7.3; //-12.3
    box_pose.position.y = 3.0;  //3 -3
    box_pose.position.z = 0;
    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;
    planning_scene.world.collision_objects.push_back(collision_object);

    collision_object.header.frame_id = "world";
    collision_object.id = "case1";
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = 0.6;
    primitive.dimensions[primitive.BOX_Y] = 0.2;
    primitive.dimensions[primitive.BOX_Z] = 0.75;  //agv = 1.5 //as = 1.2
    box_pose.orientation.w = 1.0;
    box_pose.position.x = -7.26; //-12.3
    box_pose.position.y = 3.5;  //3 -3
    box_pose.position.z = 1.25;
    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;
    planning_scene.world.collision_objects.push_back(collision_object);

    planning_scene.is_diff = true;
    planning_scene_diff_publisher.publish(planning_scene);
    ros::Duration(20.0).sleep();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gantry_controller");
    ros::NodeHandle nh;

    // add_objects(&nh);
    
    // Create interfaces to each of the AGVs
    AriacAgvMap agv_map;
    for (int i = 0; i < NUM_AGVS; i++)
    {
        // AGV topics use identifiers in the range [1,NUM_AGVS], but this loop
        // is [0,NUM_AGVS-1], so add 1 to all indices when creating them here
        auto agv = std::shared_ptr<AriacAgv>(new AriacAgv(&nh, i+1));
        agv_map[agv->get_id()] = agv;
    }

    ros::AsyncSpinner spinner(0);
    spinner.start();

    AgilityChallenger agility(&nh);
    Gantry arm;

    //
    // Start the competition
    //

    ros::Duration wait_for_competition_state(0.1);
    bool competition_state_valid = false;
    std::string competition_state;
    ros::Subscriber competition_state_sub = nh.subscribe<std_msgs::String>(
        "/ariac/competition_state",
        1,
        [&](const std_msgs::String::ConstPtr& msg) 
        {
            competition_state = msg->data;
            competition_state_valid = true;
        }
    );

    // Spin the node until we've collected the competition state
    while (!competition_state_valid)
    {
        wait_for_competition_state.sleep();
        ros::spinOnce();
    }

    // If the competition has not started, we place a request to start it
    if (competition_state == "go")
    {
        ROS_INFO_STREAM("Competition is already started.");
    }
    else
    {
        // Create the client
        assert(competition_state == "init");
        ros::ServiceClient start_competition_scl = nh.serviceClient<std_srvs::Trigger>("/ariac/start_competition");
        // Wait a very long time, because this service server may take a while
        // to be created
        assert(start_competition_scl.waitForExistence(ros::Duration(30.0)));

        // Place the request
        std_srvs::Trigger start_competition_srv;
        if (!(start_competition_scl.call(start_competition_srv) && start_competition_srv.response.success))
        {
            ROS_ERROR_STREAM("Failed to start the competition: '"<< start_competition_srv.response.message);
            return 1;
        }

        // Wait a little and collect the updated competition state
        competition_state_valid = false;
        ros::Duration(0.25).sleep();
        ros::spinOnce();
        if (competition_state == "go")
        {
            ROS_INFO_STREAM("Competition was started.");
        }
        else
        {
            ROS_ERROR_STREAM("Competition state did not update as expected: '"<< competition_state);
            return 2;
        }
    }

    //
    // If we're here, the competition has started
    //

    arm.goToPresetLocation("home1");
    // arm.goToPresetLocation("home2");

    int current_order_priority;
    nist_gear::Order current_order;
    ros::Duration rate(0.1);
    while (ros::ok())
    {
        current_order_priority = agility.consume_pending_order(current_order);
        if (0 != current_order_priority)
        {
            cater_order(
                agv_map,
                &agility,
                &arm,
                current_order_priority,
                current_order
            );
        }
        else
        {
            rate.sleep();
        }
    }
    
    return 0;
}
