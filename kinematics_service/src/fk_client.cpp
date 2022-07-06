#include <ros/ros.h>
#include <service_msg/GetFKSolutions.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


int main(int argc, char** argv)
{
    ros::init(argc, argv, "f_kinematics_client");

    ros::NodeHandle nh;
    ros::ServiceClient client = nh.serviceClient<service_msg::GetFKSolutions>("forward_k");


    //Start by instantiating a RobotModelLoader object, 
    //which will look up the robot description on the ROS parameter server and construct a RobotModel for us to use.
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    const moveit::core::RobotModelPtr& robot_model = robot_model_loader.getModel();
    ROS_INFO("Model frame: %s", robot_model->getModelFrame().c_str());

     //Using the RobotModel, we can construct a RobotState that maintains the configuration of the robot. 
    moveit::core::RobotStatePtr robot_state(new moveit::core::RobotState(robot_model));

    //We can then get a JointModelGroup, which represents the robot model for a particular group, in this case “smartisx” of the Comau robot.
    const moveit::core::JointModelGroup* joint_model_group = robot_model->getJointModelGroup("smartsix");


    service_msg::GetFKSolutions service;
    moveit::core::robotStateToRobotStateMsg(*robot_state, service.request.robot_state);

       
    if(client.call(service)) {
      tf2::Quaternion quaternion;
      tf2::fromMsg(service.response.pose.orientation, quaternion);
      
      tf2::Matrix3x3 rotation_matrix(quaternion);
      tf2Scalar roll, pitch, yaw;
      rotation_matrix.getRPY(roll, pitch, yaw);

  
      std::ostringstream std_out1;

      std_out1 << "Forward kinematic:"  << std::endl;
      std_out1 << "Position x:"; 
      std_out1 << service.response.pose.position.x << std::endl;
      std_out1 << "Position y:"; 
      std_out1 << service.response.pose.position.y << std::endl;
      std_out1 << "Position z:"; 
      std_out1 << service.response.pose.position.z << std::endl;
      std_out1 << "Orientation (RPY): [" << roll << ", " << pitch << ", " << yaw << "]";

      ROS_INFO_STREAM(std_out1.str());
    } else{
      ROS_ERROR("Failed to call service");
      return 1;
    }


    return 0;
}