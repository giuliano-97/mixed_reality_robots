using RosSharp.RosBridgeClient;
using RosSharp.RosBridgeClient.Actionlib;
using RosSharp.RosBridgeClient.MessageTypes.Actionlib;
using RosSharp.RosBridgeClient.MessageTypes.MoveBase;
using RosSharp.RosBridgeClient.MessageTypes.Geometry;

namespace MixedRealityRobot.ActionClients
{
    public class MoveBaseActionClient : 
        ActionClient<MoveBaseAction, 
                     MoveBaseActionGoal, 
                     MoveBaseActionResult, 
                     MoveBaseActionFeedback, 
                     MoveBaseGoal, 
                     MoveBaseResult, 
                     MoveBaseFeedback>
    {
        // Action status
        public string status;
        // Target position
        public Point target_position;
        // Target orientation
        public Quaternion target_orientation;
        // Target pose
        public PoseStamped target_base_pose;
        // Frame id
        public string frameID;
        // Set timer
        private static Timer timer = new Timer();

        public MoveBaseActionClient(string actionName, RosSocket rosSocket)
        {
            this.actionName = actionName;
            this.rosSocket = rosSocket;
            action = new MoveBaseAction();
            goalStatus = new GoalStatus();
            target_position = new Point();
            target_orientation = new Quaternion();
            target_base_pose = new PoseStamped();
            frameID = "base_link";
        }

        public string GetFeedbackString()
        {
            if (action != null)
            {
                // Extract pose from feedback of type PoseStamped
                Pose fb_pose = action.action_feedback.feedback.base_position.pose;
                // Return stringified pose
                return fb_pose.ToString();
            }
            return "";
        }

        public string GetResultString()
        {
            // Return empty string - the move_base action message has no result field
            return "";
        }

        public string GetStatusString()
        {
            if(goalStatus != null)
                return ((ActionStatus)goalStatus.status).ToString();
            return "";
        }

        // This function is called by SendGoal when the goal is dispatched
        protected override MoveBaseActionGoal GetActionGoal()
        {
            // Set frame id
            target_base_pose.header.frame_id = frameID;
            // Set time stamp
            timer.Now(target_base_pose.header.stamp);
            // Set new target position and orientation
            target_base_pose.pose.position = target_position;
            target_base_pose.pose.orientation = target_orientation;
            // Update goal
            action.action_goal.goal.target_pose = target_base_pose;
            
            return action.action_goal;
        }
        
        // Feedback callback
        protected override void OnFeedbackReceived()
        {
            // Not implemented - the corresponding text box in the UI is updated
            // when "Update" is called at each timestep
        }
        protected override void OnResultReceived()
        {
            // Not implemented - the action has no result field
        }
        protected override void OnStatusUpdated()
        {
            // Not implemented - the corresponding text box in the UI is updated
            // when "Update" is called at each timestep
        }
    }
}