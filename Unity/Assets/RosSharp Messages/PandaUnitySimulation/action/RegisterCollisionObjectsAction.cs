/* 
 * This message is auto generated by ROS#. Please DO NOT modify.
 * Note:
 * - Comments from the original code will be written in their own line 
 * - Variable sized arrays will be initialized to array of size 0 
 * Please report any issues at 
 * <https://github.com/siemens/ros-sharp> 
 */

using Newtonsoft.Json;

namespace RosSharp.RosBridgeClient.MessageTypes.PandaUnitySimulation
{
    public class RegisterCollisionObjectsAction : Action<RegisterCollisionObjectsActionGoal, RegisterCollisionObjectsActionResult, RegisterCollisionObjectsActionFeedback, RegisterCollisionObjectsGoal, RegisterCollisionObjectsResult, RegisterCollisionObjectsFeedback>
    {
        [JsonIgnore]
        public const string RosMessageName = "panda_unity_simulation/RegisterCollisionObjectsAction";

        public RegisterCollisionObjectsAction() : base()
        {
            this.action_goal = new RegisterCollisionObjectsActionGoal();
            this.action_result = new RegisterCollisionObjectsActionResult();
            this.action_feedback = new RegisterCollisionObjectsActionFeedback();
        }

    }
}
