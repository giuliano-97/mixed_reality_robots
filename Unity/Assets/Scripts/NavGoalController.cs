using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RosSharp.RosBridgeClient;

namespace MixedRealityRobot
{
    public class NavGoalController : MonoBehaviour
    {
        // 
        private const string actionName = "jackal/move_base";
        // RosConnector object to use RosBridge
        private RosConnector rosConnector;
        // Proxy to dispatch goals through RosBridge
        public ActionClients.MoveBaseActionClient moveBaseActionClient;
        public string status = "";
        public string feedback = "";
        public string frameID = "base_link";
        // Base link transform
        public Transform ReferenceFrame;
        // Navigation marker
        public GameObject navigationMarker;
        // Target position

        public Renderer TargetActive;
        public GameObject NavButton;

        // Start is called before the first frame update
        void Start()
        {
            // Get ros connector
            rosConnector = GetComponent<RosConnector>();
            // Create action client - attach to current web socket
            moveBaseActionClient =
                new ActionClients.MoveBaseActionClient(actionName, rosConnector.RosSocket);
            // Initialize action client
            moveBaseActionClient.Initialize();
            TargetActive = navigationMarker.GetComponent<Renderer>();
            NavButton.active = false;
        }

        // Update is called once per frame
        void Update()
        {
            // Continuously update goal status and feedback
            status = moveBaseActionClient.GetStatusString();
            feedback = moveBaseActionClient.GetFeedbackString();
            if (TargetActive.enabled == true)
            {
                NavButton.active = true;
            }
        }

        // Register new goal
        public void RegisterGoal(Vector3 newPos)
        {
            // Convert to ros convention and change to local coordinates
            newPos = RosSharp.TransformExtensions.Unity2Ros(ReferenceFrame.InverseTransformPoint(newPos));
    
            // Set new goal in the map coordinates frame
            moveBaseActionClient.frameID = frameID;
            moveBaseActionClient.target_position.x = newPos.x;
            moveBaseActionClient.target_position.y = newPos.y;
            moveBaseActionClient.target_position.z = 0.0;
            moveBaseActionClient.target_orientation.w = 1;

            Debug.Log("Sending new navigation goal in the odom frame!");

            // Call SendGoal explicitly
            moveBaseActionClient.SendGoal();
        }

        public void navigationplan()
        {
            RegisterGoal(navigationMarker.transform.position);
        }
    }
}
