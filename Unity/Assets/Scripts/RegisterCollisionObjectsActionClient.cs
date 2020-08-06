using System.Collections.Generic;
using RosSharp.RosBridgeClient;
using RosSharp.RosBridgeClient.Actionlib;
using RosSharp.RosBridgeClient.MessageTypes.PandaUnitySimulation;

namespace MixedRealityRobot.ActionClients
{
    public class RegisterCollisionObjectsActionClient : ActionClient
    <RegisterCollisionObjectsAction, RegisterCollisionObjectsActionGoal, RegisterCollisionObjectsActionResult,
     RegisterCollisionObjectsActionFeedback, RegisterCollisionObjectsGoal, RegisterCollisionObjectsResult, RegisterCollisionObjectsFeedback>
    {
        public List<SpatialMappingMesh> ssMeshes;
        public List<BoundingBox> bBoxes;

        public string status = "";
        public string feedback = "";
        public string result = "";

        public RegisterCollisionObjectsActionClient(string actionName, RosSocket rosSocket)
        {
            this.actionName = actionName;
            this.rosSocket = rosSocket;
            action = new RegisterCollisionObjectsAction();
            goalStatus = new RosSharp.RosBridgeClient.MessageTypes.Actionlib.GoalStatus();
        }

        protected override RegisterCollisionObjectsActionGoal GetActionGoal()
        {
            // Add bounding boxes
            if(bBoxes != null && bBoxes.Count > 0)
                action.action_goal.goal.boundingBoxes = bBoxes.ToArray();

            // Add meshes
            if(ssMeshes!= null && ssMeshes.Count > 0)
                action.action_goal.goal.spatialMappingMeshes = ssMeshes.ToArray();

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

        public string GetStatusString()
        {
            if (goalStatus != null)
            {
                return ((ActionStatus)(goalStatus.status)).ToString();
            }
            return "";
        }

        public string GetFeedbackString()
        {
            if (action != null)
                return action.action_feedback.feedback.log;
            return "";
        }

        public string GetResultString()
        {
            if (action != null)
                return action.action_result.result.log;
            return "";
        }
    }
}