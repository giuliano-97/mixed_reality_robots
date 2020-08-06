using System.Collections;
using System.Collections.Generic;
using UnityEngine;


using RosSharp.RosBridgeClient;

using MixedRealityRobot.Writers;

namespace MixedRealityRobot.Subscribers
{
    public class DiffDriveController : UnitySubscriber<RosSharp.RosBridgeClient.MessageTypes.Geometry.Twist>
    {
        // Differential drive parameters - mirror the configuration of a
        // diff_drive_controller in ROS. Reference:
        // http://wiki.ros.org/diff_drive_controller

        public float WheelSeparation = 0.5f;   // Distance between the wheel centers
        public float WheelSeparationMultiplier = 1.0f; // Wheel separation multiplier
        // FIXME: For now let's assume that all the wheels have the same radius
        public float WheelRadius = 0.1f;   // Wheel radius 
        public float WheelRadiusMultiplier = 1.0f; // Wheel radius multiplier

        public float CommandTimeout = 2.5f; // Time after which command stops being valid
        public float MaxLinearVelocity = 2.0f; // Max allowed velocity
        public float MaxLinearAcceleration = 20.0f;  // Max allowed acceleration
        public float MaxAngularVelocity = 4.0f; // Max allowed velocity
        public float MaxAngularAcceleration = 25.0f; // Max allowed acceleration
        public DiffDriveWriter[] diffDriveWriters;

        protected override void Start()
        {
            // Initialize subscriber
            base.Start();
        }

        protected override void ReceiveMessage(
            RosSharp.RosBridgeClient.MessageTypes.Geometry.Twist message)
        {
            //Debug.Log("DiffDriveController: New message received on /cmd_vel");

            // Unpack velocity components
            // NOTE: the cast to float should not create problems because velocities
            // are unlikely to be larger than what a float can store. However:
            // TODO: limit velocities
            float vx = (float) - message.linear.x;
            float wz = (float) - message.angular.z;

            // Compute wheel speeds using differential drive kinematic formula
            // References:
            // https://github.com/ros-controls/ros_controllers/blob/melodic-devel/diff_drive_controller/src/diff_drive_controller.cpp
            // https://globaljournals.org/GJRE_Volume14/1-Kinematics-Localization-and-Control.pdf
            float leftWheelSpeed = (10 * vx -  50 * wz * WheelSeparation / 2) / WheelRadius;
            float rightWheelSpeed = (10 * vx + 50 * wz * WheelSeparation / 2) / WheelRadius;

            // Send velocities to writers
            foreach(DiffDriveWriter d in diffDriveWriters)
            {
                if (d.WheelSide == DiffDriveWriter.Side.Left)
                    d.Write(leftWheelSpeed);
                else
                    d.Write(rightWheelSpeed);
            }
        }
    }

}

