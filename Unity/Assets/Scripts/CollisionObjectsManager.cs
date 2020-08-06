using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RosSharp.RosBridgeClient;
using RosSharp.RosBridgeClient.MessageTypes.PandaUnitySimulation;
using UnityEditor;

namespace MixedRealityRobot.ActionClients
{
    [RequireComponent(typeof(RosConnector))]
    public class CollisionObjectsManager : MonoBehaviour
    {
        private RosConnector rosConnector;

        public CollisionObjectsReader collisionObjectsReader;
        // TODO: implement service to clear the current set of obstacles

        // Collision objects action client and request status vars
        private RegisterCollisionObjectsActionClient registerCollisionObjectsActionClient;
        private readonly string registerCollisionObjectsActionName = "RegisterCollisionObjects";
        public string status = "";
        public string feedback = "";
        public string result = "";

        // MR interface obstacle avoidance buttons stuff
        public GameObject ObstaclesButton;
        private bool reloadObst;

        // Scene objects reader

        // Start is called before the first frame update
        void Start()
        {
            // Initialize the action client
            rosConnector = GetComponent<RosConnector>();
            registerCollisionObjectsActionClient = new RegisterCollisionObjectsActionClient(registerCollisionObjectsActionName, rosConnector.RosSocket);
            registerCollisionObjectsActionClient.Initialize();
            ObstaclesButton.SetActive(true);
        }

        void Update()
        {
            // Update the progress status
            status = registerCollisionObjectsActionClient.GetStatusString();
            feedback = registerCollisionObjectsActionClient.GetFeedbackString();
            result = registerCollisionObjectsActionClient.GetResultString();
        }

        public void RegisterGoal()
        {
            // TODO: update the array of meshes in the ROS# action client
            //      - Add option to register the spatial mesh

            // Get obstacles bounding boxes
            List<BoundingBox> boundingBoxes = collisionObjectsReader.GetObstaclesBoundingBoxes();

            // Set obstacles bounding boxes
            registerCollisionObjectsActionClient.bBoxes = boundingBoxes;

            // Get obstacles meshes
            //List<SpatialMappingMesh> spatialMappingMeshes = collisionObjectsReader.GetObstaclesMeshes();

            // Set obstacles meshes
            //registerCollisionObjectsActionClient.ssMeshes = spatialMappingMeshes;
            
            // Send the goal to the action server
            registerCollisionObjectsActionClient.SendGoal();
        }
    }

#if UNITY_EDITOR
    [CustomEditor(typeof(CollisionObjectsManager), true)]
    public class CollisionObjectsManagerEditor : Editor
    {
        public override void OnInspectorGUI()
        {
            DrawDefaultInspector();

            CollisionObjectsManager collisionObjectsManager = (CollisionObjectsManager)target;

            if (GUILayout.Button("Execute"))
            {
                collisionObjectsManager.RegisterGoal();
            }

        }
    }
#endif
}
