/* 
--------Action server implementation and publisher for follow object manipulation---------
To assist the follow object manipulation implementation, the updated marker position sent 
throughtout ROS# from Unity in the form of an action goal, is received and read by this 
server, that also acts as a filter for the incoming poses, by publishing them at a desire 
rate (the value can be freely chosen), on the topic sampled_pose
*/

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <panda_unity_simulation/FollowObjectMoveitAction.h>


class FollowObjectMoveitAction
{
protected:

  // Standard action initialisation, node creation where to publish the goals and variables initialisation
  ros::NodeHandle nodeHandle_;
  ros::Publisher publisher_ = nodeHandle_.advertise<geometry_msgs::Pose>("sampled_pose", 1);
  actionlib::SimpleActionServer<panda_unity_simulation::FollowObjectMoveitAction> actionServer_;
  std::string action_name_;
  geometry_msgs::Pose goal_;
  geometry_msgs::Pose poseFromGoal;

public:

  // Server, node, action name and callback function initialisation
  FollowObjectMoveitAction(std::string name) :
    actionServer_(nodeHandle_, name, boost::bind(&FollowObjectMoveitAction::callback, this, _1), false),
    action_name_(name)
  {
    actionServer_.start();
  }

  ~FollowObjectMoveitAction(void)
  {
  }

  // Callback definition that implements the publisher paradigm 
  void callback(const panda_unity_simulation::FollowObjectMoveitGoalConstPtr &goal)
  {

    // Sampling rate selection
    ros::Rate r(1);
    bool success = true;
    goal_ = goal-> targetPose; 

    if (actionServer_.isPreemptRequested() || !ros::ok())
    {
        ROS_INFO("%s: Preempted", action_name_.c_str());
        actionServer_.setPreempted();
        success = false;
    }

   // Sampled pose that will be written to the allocated topic
    poseFromGoal.position.x = goal_.position.x;
    poseFromGoal.position.y = goal_.position.y;
    poseFromGoal.position.z = goal_.position.z;
    poseFromGoal.orientation.x = goal_.orientation.x;
    poseFromGoal.orientation.y = goal_.orientation.y;
    poseFromGoal.orientation.z = goal_.orientation.z;
    poseFromGoal.orientation.w = goal_.orientation.w;

    // Sampled pose visualisation
    ROS_INFO("Pose w published %f", poseFromGoal.orientation.w);
    ROS_INFO("Pose x published %f", poseFromGoal.position.x);
    ROS_INFO("Pose y published %f", poseFromGoal.position.y);
    ROS_INFO("Pose z published %f", poseFromGoal.position.z);

    // Sample pose publishing on the ad-hoc topic
    publisher_.publish(poseFromGoal);

    r.sleep();

    if(success)
    {
      ROS_INFO("%s: Succeeded", action_name_.c_str());
      actionServer_.setSucceeded();
    }
  }
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "follow_object_moveit");
  FollowObjectMoveitAction pose("follow_object_moveit");
  ros::spin();

  return 0;
}
