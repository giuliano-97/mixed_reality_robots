using System.Collections;
using System.Collections.Generic;
using UnityEngine;


namespace MixedRealityRobot.Writers
{
    [RequireComponent(typeof(HingeJoint))]
    public class DiffDriveWriter : MonoBehaviour
    {
        public enum Side
        {
            Right,
            Left
        }

        public Side WheelSide = Side.Left;
        public float MaxVelocity;

        private HingeJoint _hingeJoint;
        private JointMotor jointMotor;
        private float targetVelocity;
        private bool isMessageReceived;

        // Start is called before the first frame update
        void Start() 
        {
            // Try to determine whether the wheel is on the left/right 
            // side of the robot from the name of the link
            string lowercaseName = name.ToLower();
            if (lowercaseName.Contains("left"))
                WheelSide = Side.Left;
            else if (lowercaseName.Contains("right"))
                WheelSide = Side.Right;

            _hingeJoint = GetComponent<HingeJoint>();
            _hingeJoint.useMotor = true;
        }

        // Update is called once per frame
        void Update()
        {
            if (isMessageReceived)
                ProcessMessage();
        }

        private void ProcessMessage()
        {
            jointMotor = _hingeJoint.motor;
            jointMotor.targetVelocity = targetVelocity;
            _hingeJoint.motor = jointMotor;
            isMessageReceived = false;
        }

        public void Write(float input)
        {
            targetVelocity = input;
            isMessageReceived = true;
        }
    }

}
