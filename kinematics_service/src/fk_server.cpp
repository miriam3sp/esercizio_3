#include <ros/ros.h>
#include <service_msg/GetFKSolutions.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf2/convert.h>



bool compute_fk_server(service_msg::GetFKSolutionsRequest & request, service_msg::GetFKSolutionsResponse  & response){

    //Start by instantiating a RobotModelLoader object, 
    //which will look up the robot description on the ROS parameter server and construct a RobotModel for us to use.
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    const moveit::core::RobotModelPtr& robot_model = robot_model_loader.getModel();
    ROS_INFO("Model frame: %s", robot_model->getModelFrame().c_str());

    //Using the RobotModel, we can construct a RobotState that maintains the configuration of the robot. 
    moveit::core::RobotStatePtr robot_state(new moveit::core::RobotState(robot_model));
 
    moveit::core::robotStateToRobotStateMsg(*robot_state,request.robot_state);

    //We will set all joints in the state to their default values.
    //robot_state->setToDefaultValues();

    robot_state->updateLinkTransforms();

    //We can then get a JointModelGroup, which represents the robot model for a particular group, in this case “smartisx” of the Comau robot.
    const moveit::core::JointModelGroup* joint_model_group = robot_model->getJointModelGroup("smartsix");


    //const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();

    const Eigen::Isometry3d& end_effector_state = robot_state->getGlobalLinkTransform("link6");


    tf::poseEigenToMsg(end_effector_state,response.pose);
  


    return true;
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "f_kinematics_server");
  
    ros::NodeHandle nh;
    ros::ServiceServer service = nh.advertiseService("forward_k", compute_fk_server);
    ros::spin();

    ros::shutdown();
    return 0;
}





