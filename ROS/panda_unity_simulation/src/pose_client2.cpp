/*
Action client implementation for sending goals to the related server without having to rely on goals from Unity
*/
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <panda_unity_simulation/FollowObjectMoveitAction.h>

int main (int argc, char **argv)
{
  ros::init(argc, argv, "pose_handle");

  actionlib::SimpleActionClient<panda_unity_simulation::FollowObjectMoveitAction> actionClient("follow_object_moveit", true);

  ROS_INFO("Action server not started");
  actionClient.waitForServer(); 
  ROS_INFO("Action server started");

  // Sending one single goal for testing
  panda_unity_simulation::FollowObjectMoveitGoal goal;
  goal.targetPose.position.x = 0.4;
  goal.targetPose.position.y = - 0.4;
  goal.targetPose.position.z = 0.8;
  goal.targetPose.orientation.x = 0;
  goal.targetPose.orientation.y = 0;
  goal.targetPose.orientation.z = 0;
  goal.targetPose.orientation.w = 1.0;  
  actionClient.sendGoal(goal);

  // Wait for the result
  bool finished_before_timeout = actionClient.waitForResult(ros::Duration(30.0));

  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = actionClient.getState();
    ROS_INFO("Action successfully completed: %s", state.toString().c_str());
  }
  else
    ROS_INFO("Action went out of time");

  return 0;
}