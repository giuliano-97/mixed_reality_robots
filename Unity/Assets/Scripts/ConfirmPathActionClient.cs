using System;
using RosSharp.RosBridgeClient;
using RosSharp.RosBridgeClient.Actionlib;
using RosSharp.RosBridgeClient.MessageTypes.Actionlib;
using RosSharp.RosBridgeClient.MessageTypes.Geometry;
using RosSharp.RosBridgeClient.MessageTypes.PandaUnitySimulation;


namespace MixedRealityRobot.ActionClients
{
    public class ConfirmPathActionClient :
        ActionClient<
            ConfirmPathMoveitAction,
            ConfirmPathMoveitActionGoal,
            ConfirmPathMoveitActionResult,
            ConfirmPathMoveitActionFeedback,
            ConfirmPathMoveitGoal,
            ConfirmPathMoveitResult,
            ConfirmPathMoveitFeedback>
    {

        public bool confirmation;

        public ConfirmPathActionClient(string actionName, RosSocket rosSocket)
        {
            this.actionName = actionName;
            this.rosSocket = rosSocket;
            action = new ConfirmPathMoveitAction();
            goalStatus = new RosSharp.RosBridgeClient.MessageTypes.Actionlib.GoalStatus();
        }
        protected override ConfirmPathMoveitActionGoal GetActionGoal()
        {
            action.action_goal.goal.confirmation = this.confirmation;
            return action.action_goal;
        }

        protected override void OnFeedbackReceived()
        {
            // Do nothing - no feedback is sent for this action
        }

        protected override void OnResultReceived()
        {
            // Do nothing - we don't care about the result
        }

        protected override void OnStatusUpdated()
        {
            // Do nothing - don't care about the status
        }
    }

}