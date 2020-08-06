/* 
-------------------Follow Object Manipulation interface implementation-------------------------
This node reads the incoming pose by subscribing to the sample_pose topic. It also performs the 
computation of the path, according to the result from the IK solvers, and if exists it executes 
the manipulator motion by moving the planning group accordingly such that the end-tool is in the 
required pose. A further set of cycles and flags is used to keep the interface up over time.
*/

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

// Variables iniaitalisation
geometry_msgs::Pose poseUtility;
bool flag = false;

// Callback definition for the topic subscription, it performs an assignment of the incoming pose 
// to a support variable used within this script
void subscribeTopicCB(const geometry_msgs::Pose& targetPose)
{
  poseUtility.position.x = targetPose.position.x; 
  poseUtility.position.y = targetPose.position.y; 
  poseUtility.position.z = targetPose.position.z; 
  poseUtility.orientation.x = targetPose.orientation.x;
  poseUtility.orientation.y = targetPose.orientation.y;
  poseUtility.orientation.z = targetPose.orientation.z;
  poseUtility.orientation.w = targetPose.orientation.w;

  // Flag update to keep the interface looping
  flag = true;
}


int main(int argc, char** argv)
{
  // Node, subscriber, rviz and MoveGroup class iniatialisation
  ros::init(argc, argv, "follow_object_interface");
  ros::NodeHandle nodeHandle_;
  ros::Subscriber subscriber_ = nodeHandle_.subscribe("sampled_pose", 1, subscribeTopicCB);

  ros::AsyncSpinner spinner(4);
  spinner.start();

  static const std::string PLANNING_GROUP = "panda_arm";
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("panda_link0");
  visual_tools.deleteAllMarkers();
  visual_tools.loadRemoteControl();
  visual_tools.trigger();

  // Path planning and execution for the incoming sampled poses
  while (ros::ok())
  {
    // ros::Duration d(2.0);
    ros::Rate r(1);
   
    if (flag)
    {
        // pathPlan iniatialisation
        moveit::planning_interface::MoveGroupInterface::Plan pathPlan;

        // Pose set up
        move_group.setPoseTarget(poseUtility); 

        // Pose visualisation
        ROS_INFO("Pose w to compute %f", poseUtility.orientation.w);
        ROS_INFO("Pose x to compute %f", poseUtility.position.x);
        ROS_INFO("Pose y to compute %f", poseUtility.position.y);
        ROS_INFO("Pose z to compute %f", poseUtility.position.z);

        // Path existence and motion execution if any
        bool path_success = (move_group.plan(pathPlan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        ROS_INFO_NAMED("Path is %s", path_success ? "found":"NOT found");
        move_group.move(); 

        // Rviz trajectory visualisation
        visual_tools.publishTrajectoryLine(pathPlan.trajectory_, joint_model_group);

        // Flag to keep the interface running over time
        flag = false;
    }
  }

  ros::waitForShutdown();
  return 0;

}



