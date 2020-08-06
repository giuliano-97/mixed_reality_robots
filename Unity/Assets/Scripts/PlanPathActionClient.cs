using System;
using RosSharp.RosBridgeClient;
using RosSharp.RosBridgeClient.Actionlib;
using RosSharp.RosBridgeClient.MessageTypes.Actionlib;
using RosSharp.RosBridgeClient.MessageTypes.Geometry;
using RosSharp.RosBridgeClient.MessageTypes.PandaUnitySimulation;

namespace MixedRealityRobot.ActionClients
{
    public class PlanPathActionClient : 
        ActionClient<
            PlanPathMoveitAction, 
            PlanPathMoveitActionGoal, 
            PlanPathMoveitActionResult,
            PlanPathMoveitActionFeedback, 
            PlanPathMoveitGoal, 
            PlanPathMoveitResult, 
            PlanPathMoveitFeedback>
    {
        public Pose targetPose;
        public bool newResult;

        public PlanPathActionClient(string actionName, RosSocket rosSocket)
        {
            targetPose = new RosSharp.RosBridgeClient.MessageTypes.Geometry.Pose();
            this.actionName = actionName;
            this.rosSocket = rosSocket;
            action = new PlanPathMoveitAction();
            goalStatus = new RosSharp.RosBridgeClient.MessageTypes.Actionlib.GoalStatus();
        }

        protected override PlanPathMoveitActionGoal GetActionGoal()
        {
            action.action_goal.goal.targetPose = this.targetPose;
            return action.action_goal;
        }

        protected override void OnFeedbackReceived()
        {
            // Do nothing - no feedback is sent
        }

        protected override void OnResultReceived()
        {
            // Set flag for new result
            this.newResult = true;
        }

        protected override void OnStatusUpdated()
        {
            
        }

        // We return a bool nullable type to indicate the
        // result has not been received yet
        public bool? GetResult()
        {
            if (newResult)
            {
                // New result has been processed
                newResult = false;
                return action.action_result.result.pathResult;
            } else
            {
                return null;
            }
        }
    }
}