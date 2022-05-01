#include <ros/ros.h>

#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>

#include <memory>
#include <unordered_map>

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
                                                  Arm* const arm,
                                                  Gantry* const garm,
                                                  const int current_order_priority);

    void cater_faulty_parts(AgilityChallenger* const agility,
                            Arm* const arm,
                            const std::string& order_id,
                            std::vector<nist_gear::Product>& products)
    {
        // If this part is faulty, move it
        std::string faulty_part_agv_id;
        nist_gear::Product faulty_part_product;
        geometry_msgs::Pose faulty_part_pick_frame;
        while (agility->get_agv_faulty_part(order_id,
                                            faulty_part_agv_id,
                                            faulty_part_product,
                                            faulty_part_pick_frame))
        {
            ROS_INFO_STREAM("Moving faulty part: "
                            << faulty_part_agv_id
                            << ", "
                            << faulty_part_product.type);

            // Move this part to the disposal bin
            // TODO: tweak all the stops/sleeps?
            arm->goToPresetLocation(faulty_part_agv_id);
            if (arm->pickPart(faulty_part_product.type, faulty_part_pick_frame, 1))
            {
                ros::Duration(2.0).sleep();
                arm->goToPresetLocation(faulty_part_agv_id);
                arm->goToPresetLocation("home2");
                ros::Duration(0.5).sleep();
                arm->deactivateGripper();

                // Place this product back in the queue to be
                // picked again elsewhere
                products.push_back(faulty_part_product);
            }
            else
            {
                ROS_ERROR_STREAM("Failed to pick the faulty part");
            }
        }
    }

    void cater_kitting_shipments(const AriacAgvMap& agv_map,
                                 AgilityChallenger* const agility,
                                 Arm* const arm,
                                 Gantry* const garm,
                                 const int order_priority,
                                 const std::string& order_id,
                                 std::vector<nist_gear::KittingShipment>& kitting_shipments)
    {
        // Ignore request if there are no kitting shipments
        if (kitting_shipments.empty())
        {
            return;
        }

        ROS_INFO_STREAM("Catering "
                        << kitting_shipments.size()
                        << " kitting shipments from order with priority of "
                        << order_priority);

        int counter = 0;
        // parse each kitting shipment
        for (const auto& ks : kitting_shipments)
        {
            std::vector<nist_gear::Product> products = ks.products;
            if (products.empty())
            {
                ROS_FATAL_STREAM("Kitting shipment had no products?");
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
                const std::vector<int> bin_indices = agility->get_camera_indices_of(product.type);
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

                // counter++;

                for (auto iter = bin_indices.cbegin(); iter != bin_indices.cend(); ++iter)
                {
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
                    //check TF for Gantry or Arm
                    auto world_pose = arm->transform_to_world_frame(part_frame);
                    if (world_pose.position.x < -2.3)
                    {
                        arm->goToPresetLocation("home2");
                        ROS_INFO_STREAM("GANTRY WILL DO THE TASK");
                        ROS_INFO_STREAM("Moving part '" << product.type << "' to '" << ks.agv_id << "' (" << part_frame << ")");
                        garm->movePart(product.type, part_frame, product.pose, ks.agv_id);
                        ROS_INFO_STREAM("Placed part '" << product.type << "' at '" << ks.agv_id << "'");
                        garm->moveBaseTo(world_pose.position.x, world_pose.position.y);
                    }
                    else{
                        ROS_INFO_STREAM("KITTING ARM WILL DO THE TASK");
                        ROS_INFO_STREAM("Moving part '" << product.type << "' to '" << ks.agv_id << "' (" << part_frame << ")");
                        arm->movePart(product.type, part_frame, product.pose, ks.agv_id);
                        ROS_INFO_STREAM("Placed part '" << product.type << "' at '" << ks.agv_id << "'");
                    }
                    agility->queue_for_fault_verification(
                        product,
                        order_id,
                        ks.agv_id,
                        arm->transform_to_world_frame(product.pose, ks.agv_id)
                    );
                    ros::Duration(0.2).sleep();

                    // Give an opportunity for higher priority orders
                    cater_higher_priority_order_if_necessary(agv_map, agility, arm, garm, order_priority);

                    // If there is no sensor blackout, check for faulty parts
                    if (!agility->is_sensor_blackout_active())
                    {
                        // After checking, give an opportunity for higher
                        // priority orders
                        cater_faulty_parts(agility, arm, order_id, products);
                        cater_higher_priority_order_if_necessary(agv_map, agility, arm, garm, order_priority);
                    }
                    else
                    {
                         ROS_WARN_STREAM("SENSOR BLCKOUT");
                    }

                    // It may be faulty, but we placed the part. Whether it was
                    // already declared faulty and moved, or there was a sensor
                    // blackout, we're done with the product for now.
                    break;
                }

                // If we have finished placing all of the parts but parts still
                // need verification for faults, wait here until the sensor
                // blackout is done, then cater them. If any of them are
                // faulty, it'll add the product back into the products vector.
                if (products.empty() && agility->needs_fault_verification(ks.agv_id))
                {
                    ROS_INFO_STREAM("Waiting for sensor blackout to finish...");
                    do {
                        static ros::Duration d(0.1);
                        d.sleep();
                    } while (agility->needs_fault_verification(ks.agv_id));

                    cater_faulty_parts(agility, arm, order_id, products);

                    // If products were added back, first give an opportunity
                    // for higher priority orders before resuming
                    if (!products.empty())
                    {
                        cater_higher_priority_order_if_necessary(agv_map, agility, arm, garm, order_priority);
                    }
                }
            }

            // If we're here, we have placed all products in this shipment
            ros::Duration(1.0).sleep();
            const auto agv_iter = agv_map.find(ks.agv_id);
            if (agv_map.cend() != agv_iter)
            {
                const std::shared_ptr<AriacAgv> agv = agv_iter->second;
                if (agv->is_ready_to_deliver())
                {
                    agv->submit_shipment(
                        ks.station_id,
                        ks.shipment_type
                    );
                    ROS_INFO_STREAM("Submitted AGV with ID " << ks.agv_id);
                }
                else
                {
                    ROS_ERROR_STREAM("AGV with ID " << ks.agv_id << " is not ready to ship");
                }
            }
            else
            {
                ROS_FATAL_STREAM("Unknown AGV with ID " << ks.agv_id);
                ros::shutdown();
                return;
            }
        }
    }

    void cater_order(const AriacAgvMap& agv_map,
                     AgilityChallenger* const agility,
                     Arm* const arm,
                     Gantry* const garm,
                     const int order_priority,
                     nist_gear::Order& order)
    {
        cater_kitting_shipments(
            agv_map,
            agility,
            arm,
            garm,
            order_priority,
            order.order_id,
            order.kitting_shipments
        );
    }

    void cater_higher_priority_order_if_necessary(const AriacAgvMap& agv_map,
                                                  AgilityChallenger* const agility,
                                                  Arm* const arm,
                                                  Gantry* const garm,
                                                  const int current_order_priority)
    {
        if (agility->higher_priority_order_requested(current_order_priority))
        {
            ROS_INFO_STREAM("Catering higher priority order...");
            int new_order_priority;
            nist_gear::Order new_order;
            new_order_priority = agility->consume_pending_order(new_order);
            cater_order(agv_map, agility, arm, garm, new_order_priority, new_order);
            ROS_INFO_STREAM("Finished higher priority order, returning to previous order");
        }
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "arm_controller");
    ros::NodeHandle nh;

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
    Arm arm;
    Gantry garm;

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
    garm.goToPresetLocation("home1");
    arm.goToPresetLocation("home2");

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
                &garm,
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
