using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;
using UnityEngine.Rendering;

namespace MixedRealityRobot.ActionClients
{
    [RequireComponent(typeof(RosSharp.RosBridgeClient.RosConnector))]
    public class ManipulationController : MonoBehaviour
    {
        // Instatiate ros connector
        private RosSharp.RosBridgeClient.RosConnector rosConnector;

        // Instantiate action client
        private PlanPathActionClient planPathActionClient;
        private ConfirmPathActionClient confirmPathActionClient;

        public GameObject manipulationMarker;
        public Transform referenceFrame;
        private Vector3 previousMarkerPosition;

        public bool planflag;
        public Renderer planwarning;
        public GameObject PlannerButton;
        public GameObject ExecutionButton;
        public bool exeflag = false;

        public string planPathActionName = "plan_path_moveit";
        public string confirmPathActionName = "confirm_path_moveit";
        public string status = "";
        [HideInInspector]
        public bool pathFound_bool = false;

        // Private constant pose
        public Vector3 targetPosition;
        private Quaternion targetOrientation;

        // Start is called before the first frame update
        void Start()
        {
            targetPosition = new Vector3(0.5f, 0.5f, 0.5f);
            targetOrientation = Quaternion.Euler(0,0,0);
            rosConnector = GetComponent<RosSharp.RosBridgeClient.RosConnector>();
            planPathActionClient = new PlanPathActionClient(planPathActionName, rosConnector.RosSocket);
            planPathActionClient.Initialize();
            confirmPathActionClient = new ConfirmPathActionClient(confirmPathActionName, rosConnector.RosSocket);
            confirmPathActionClient.Initialize();
            previousMarkerPosition = manipulationMarker.transform.position;
            PlannerButton.active = true;
            ExecutionButton.active = false;
        }

        // Update is called once per frame
        void Update()
        {
            bool? planPathResult = planPathActionClient.GetResult();
            if (planPathResult != null)
            {
                Debug.Log("A new result has been received!");
                if (planPathResult == true)
                {
                    // A path was found
                    status = "PATH FOUND";
                    pathFound_bool = true;
                    ExecutionButton.active = true;
                    planwarning.enabled = false;
                }
                else
                {
                    // No path found
                    status = "PATH NOT FOUND";
                    pathFound_bool = false;
                    ExecutionButton.active = false;
                    planwarning.enabled = true;
                }
            }
            if(exeflag == true)
            {
                ConfirmPathPlan();
                ExecutionButton.active = false;
                exeflag = false;
            }
        }

        public void RegisterPathPlanGoal()
        {
            SetTargetPosition(RosSharp.TransformExtensions.Unity2Ros(targetPosition));
            SetTargetOrientation(RosSharp.TransformExtensions.Unity2Ros(targetOrientation));
            planPathActionClient.SendGoal();
        }

        public void ConfirmPathPlan()
        {
            confirmPathActionClient.confirmation = true;
            confirmPathActionClient.SendGoal();
        }

        private void SetTargetPosition(Vector3 targetPos)
        {
            planPathActionClient.targetPose.position.x = targetPos.x;
            planPathActionClient.targetPose.position.y = targetPos.y;
            planPathActionClient.targetPose.position.z = targetPos.z;
        }

        private void SetTargetOrientation(Quaternion targetOrientation)
        {
            planPathActionClient.targetPose.orientation.x = targetOrientation.x;
            planPathActionClient.targetPose.orientation.y = targetOrientation.y;
            planPathActionClient.targetPose.orientation.z = targetOrientation.z;
            planPathActionClient.targetPose.orientation.w = targetOrientation.w;
        }
        public void executecommand()
        {
            exeflag = true;
        }
        public void plancommand()
        {
            Vector3 offset = new Vector3(0, -0.1f, 0);
            targetPosition = referenceFrame.InverseTransformPoint(manipulationMarker.transform.position) + offset;
            status = "SENDING NEW PATH GOAL";
            RegisterPathPlanGoal();
        }
    }
#if UNITY_EDITOR
    [CustomEditor(typeof(ManipulationController), true)]
    public class ManipulationControllerEditor : Editor
    {
        public override void OnInspectorGUI()
        {
            DrawDefaultInspector();

            ManipulationController manipulationController = (ManipulationController)target;
            if (manipulationController.pathFound_bool)
            {
                if (GUILayout.Button("Execute"))
                {
                    manipulationController.ConfirmPathPlan();
                }
            }
        }
    }
#endif

}
