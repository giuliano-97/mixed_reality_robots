using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RosSharp;
using RosSharp.RosBridgeClient;
using RosSharp.RosBridgeClient.MessageTypes.Geometry;
using UnityEngine.Timeline;
using MixedRealityRobot.ActionClients;
using RosSharp.RosBridgeClient.MessageTypes.FollowObject;
using RosSharp.RosBridgeClient.Actionlib;
using UnityEditor.Experimental.GraphView;
using JetBrains.Annotations;

// This script acts as a controller for the follow object manipulation by reading
// the updated marker position and adequately converting it to ROS coordinate.
// It then sends the ROS coordinate to the related action clinet in the ROS counterpart,
// using the action follow_object_moveit

namespace RosSharp.RosBridgeClient
{
    public class FollowObjectController : MonoBehaviour
    {
        // Robot with respect to compute the relative marker position
        public GameObject robot;
        // Marker that acts as a pose goal
        public GameObject marker;
        
        private UnityFollowObjectActionClient followObjectActionClient;

        // Variables initialisation
        private UnityEngine.Vector3 marker_Position;
        private UnityEngine.Quaternion marker_Orientation;
        private UnityEngine.Vector3 robot_position; 
        private UnityEngine.Vector3 endPose_position_ROS;
        private UnityEngine.Quaternion endPose_orientation_ROS;
        private UnityEngine.Vector3 endPose_position;
        private UnityEngine.Quaternion endPose_orientation;

        // Button used to trigger the function
        public GameObject followMeButton;
        private bool followMeHandler;

        // Flag used to implement the start/stop function
        private bool flag = false;

        // Start is called before the first frame update
        void Start()
        {
            // The ad-hoc button is activated once this script is activated too
            followMeButton.active = true;
            followObjectActionClient = GetComponent<UnityFollowObjectActionClient>(); 
        }

        // Update is called once per frame
        void Update()
        {
            // Enter the condition if either the start/stop button is pressed or the flag is on, meaning the follow object function is operating
            if (followMeHandler == true | flag == true) 
            {
                // When the button is pressed a second time the follow object function has to be interrupted, therefore flag = false
                if (followMeHandler == true & flag == true) 
                {
                    flag = false;
                }
                else
                {
                    flag = true;

                    // Initialisation
                    marker_Position = marker.transform.position;
                    marker_Orientation = marker.transform.rotation;
                    robot_position = robot.transform.position;

                    // Marker position transformation to robot coordinate system with the addition of an offset
                    UnityEngine.Vector3 offsetGripper = new UnityEngine.Vector3(0, -0.1f, 0);
                    endPose_position = robot.transform.InverseTransformPoint(marker_Position) + offsetGripper;

                    // Standard values for the orientation since are not required in ROS
                    endPose_orientation.x = 0;
                    endPose_orientation.y = 0;
                    endPose_orientation.z = 0;
                    endPose_orientation.w = 1;

                    // Coordinate transformation from Unity coordinate system to ROS coordinate system
                    endPose_position_ROS = RosSharp.TransformExtensions.Unity2Ros(endPose_position);
                    endPose_orientation_ROS = RosSharp.TransformExtensions.Unity2Ros(endPose_orientation); // since (0,0,0,1) no changes is done, here for future change

                    // Standard values in ROS
                    endPose_orientation_ROS.x = 0;
                    endPose_orientation_ROS.y = 0;
                    endPose_orientation_ROS.z = 0;
                    endPose_orientation_ROS.w = 1;

                    // The received pose, after being converted is sent to the server on the ROS side, through ROS#
                    followObjectActionClient.RegisterGoal(endPose_position_ROS, endPose_orientation_ROS);

                }

                followMeHandler = false;
            }
        }

        public void followflag()
        {
            followMeHandler = !followMeHandler;
        }
    }

}

// NOTES: the marker position could change over time and this is not tollerate by Moveit -> solution: accept 
// incoming data only every tot secs. The server implements this filtering capability by publing on a related topic
