using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RosSharp;
using RosSharp.RosBridgeClient;
using RosSharp.RosBridgeClient.MessageTypes.Geometry;
using System;
using UnityEditor.Experimental.GraphView;
using RosSharp.RosBridgeClient.MessageTypes.FollowObject;
using RosSharp.RosBridgeClient.Actionlib;

namespace RosSharp.RosBridgeClient.Actionlib
{
    [RequireComponent(typeof(RosConnector))]
    public class UnityFollowObjectActionClient : MonoBehaviour
    {
        // rosconnector def
        private RosConnector rosConnector;
        // action client
        public FollowObjectActionClient followObjectActionClient;
        // action name can be specified in Unity
        public string actionName;

        // called whenever we play the scene 
        private void Start()
        {
            rosConnector = GetComponent<RosConnector>(); // connect the RosConnector
            followObjectActionClient = new FollowObjectActionClient(actionName, rosConnector.RosSocket); // defined in PoseActionClient
            followObjectActionClient.Initialize();
        }

        private void Update()
        {
        }

        public void RegisterGoal(UnityEngine.Vector3 newPosition, UnityEngine.Quaternion newOrientation)
        {
          
            followObjectActionClient.targetPosition.x = newPosition.x;
            followObjectActionClient.targetPosition.y = newPosition.y;
            followObjectActionClient.targetPosition.z = newPosition.z;
            followObjectActionClient.targetOrientation.x = newOrientation.x;
            followObjectActionClient.targetOrientation.y = newOrientation.y;
            followObjectActionClient.targetOrientation.z = newOrientation.z;
            followObjectActionClient.targetOrientation.w = newOrientation.w;
            Debug.Log("DEBUG SENDGOAL");
            followObjectActionClient.SendGoal(); // so that is done automatically

        }

    }

}
