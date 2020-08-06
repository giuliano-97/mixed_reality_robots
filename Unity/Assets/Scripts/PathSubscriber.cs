using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RosSharp.RosBridgeClient;
using System;
using MixedRealityRobot.Writers;

namespace MixedRealityRobot.Subscribers
{
    public class PathSubscriber : UnitySubscriber<RosSharp.RosBridgeClient.MessageTypes.Nav.Path>
    {
        public Transform ReferenceFrame;
        public LineWriter lineWriter;
        public int pathLength;

        protected override void Start()
        {
            // Initialize subscriber
            base.Start();
        }

        protected override void ReceiveMessage(
            RosSharp.RosBridgeClient.MessageTypes.Nav.Path message)
        {
            //Unpack Values
            pathLength = message.poses.Length;
            List<Vector3> points = new List<Vector3>();
            for (int i = 0; i < message.poses.Length; i++)
            {
                Vector3 next_waypoint_ros = new Vector3(
                   (float)message.poses[i].pose.position.x,
                   (float)message.poses[i].pose.position.y,
                   (float)message.poses[i].pose.position.z);
                points.Add(RosSharp.TransformExtensions.Ros2Unity(next_waypoint_ros));
            }
            lineWriter.Write(points);
        }
    }
}
