using System.Collections;
using System.Collections.Generic;
using RosSharp.RosBridgeClient.Actionlib;
using RosSharp;
using RosSharp.RosBridgeClient.MessageTypes.Geometry;
using UnityEngine;
using RosSharp.RosBridgeClient.MessageTypes.Actionlib;
using System.Reflection;
using RosSharp.RosBridgeClient.MessageTypes.FollowObject;

// This script implements the action client for the follow_object_moveit action,
// used to continuosly send marker pose (converted to ROS coordinate) to the 
// follow interface in the ROS side

namespace RosSharp.RosBridgeClient.Actionlib
{
    public class FollowObjectActionClient : ActionClient<FollowObjectMoveitAction, FollowObjectMoveitActionGoal, FollowObjectMoveitActionResult, FollowObjectMoveitActionFeedback, FollowObjectMoveitGoal, FollowObjectMoveitResult, FollowObjectMoveitFeedback>
    {
        // goal definition, type geometry_msgs/Pose in the ROS side, i.e. 3 for position, 4 for orientation (3 + w)
        public MessageTypes.Geometry.Pose targetPose;
        // standard definition
        public string status = "";
        public string result = "";
        public string feedback = "";

        // goal components declaration
        public Point targetPosition;
        public MessageTypes.Geometry.Quaternion targetOrientation; // check why is read

        // standard initialisation for the Client
        public FollowObjectActionClient(string actionName, RosSocket rosSocket)
        {
            this.actionName = actionName;
            this.rosSocket = rosSocket;
            action = new FollowObjectMoveitAction();
            goalStatus = new MessageTypes.Actionlib.GoalStatus();

            // goal components
            targetPosition = new Point();
            targetOrientation = new MessageTypes.Geometry.Quaternion();

            // goal
            targetPose = new MessageTypes.Geometry.Pose();
        }


        // needed by SendGoal 
        protected override FollowObjectMoveitActionGoal GetActionGoal()
        {
            // setting of the two pose components
            targetPose.position = targetPosition;
            targetPose.orientation = targetOrientation;

            // give to the goal the required update
            action.action_goal.goal.targetPose = targetPose;
            return action.action_goal;

        }


        protected override void OnStatusUpdated()
        {
        }

        protected override void OnFeedbackReceived()
        {
        }

        protected override void OnResultReceived()
        {
        }

        // status implementation, already done out of the box
        public string GetStatusString()
        {
            if (goalStatus != null)
            {
                return ((ActionStatus)(goalStatus.status)).ToString();
            }
            return "";
        }

        public int GetFeedbackString()
        {
            return 0;
        }

        public string GetResultString()
        {
            return "";
        }


    }

}

