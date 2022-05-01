#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "obstacle_controller");

    ros::AsyncSpinner spinner(0);
    spinner.start();

    ros::NodeHandle nh;
    ros::Publisher pub_scene;
    ROS_INFO("Wait");
      
    pub_scene = nh.advertise<moveit_msgs::PlanningScene>("/ariac/gantry/planning_scene", 1);
    
    robot_model_loader::RobotModelLoader::Options opt;
    opt.robot_description_ = "/ariac/gantry/robot_description";
    opt.load_kinematics_solvers_ = false;

    robot_model_loader::RobotModelLoaderPtr rml(new robot_model_loader::RobotModelLoader(opt));
    planning_scene::PlanningScene ps(rml->getModel());

    std::ifstream f("/home/sameep/ariac_ws/src/group3_rwa4/config/ariac.scene");
    if(f.fail()){
        ROS_INFO("HIT");
    }
    else{
      ROS_INFO("OPEN");
    }
    
    if (ps.loadGeometryFromStream(f))
    {
      ROS_INFO("Publishing geometry from file");
      moveit_msgs::PlanningScene ps_msg;
      ps.getPlanningSceneMsg(ps_msg);
      ps_msg.is_diff = true;

      ros::WallDuration dt(0.5);
      unsigned int attempts = 0;
      while (pub_scene.getNumSubscribers() < 1 && ++attempts < 100)
        dt.sleep();

      pub_scene.publish(ps_msg);
      ros::Duration(1).sleep();
    }


  ros::shutdown();
  return 0;
}