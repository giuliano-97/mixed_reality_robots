using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RosSharp.RosBridgeClient;
using RosSharp.RosBridgeClient.MessageTypes.Std;
using RosSharp.RosBridgeClient.MessageTypes.Nav;


namespace MixedRealityRobot.Publishers
{
    [RequireComponent(typeof(OdomTimestampSync))]
    [RequireComponent(typeof(OdomTfPublisher))]
    public class OdometryPublisher : UnityPublisher<Odometry>
    {

        // Instead of derivating the base link transform, let's just use the
        // velocity of the rigid body to which the base link frame is attached
        // to get the velocity
        public GameObject InitialPoseHolder;
        private Vector3 relativePosition;
        private Quaternion relativeOrientation;
        public Transform BaseLinkTransform;
        public Rigidbody BaseLinkRigidBody;   
        public string FrameId = "odom";
        public string ChildFrameId = "base_link";
        private OdomTimestampSync odomTimestampSync;        

        // Constant Pose covariance matrix
        private static readonly double[] _poseCovariance =
        {
            0.001, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.001, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 1000000.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 1000000.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 1000000.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.03
        };

        private static readonly double[] _twistCovariance =
        {
            0.001, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.001, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.001, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 1000000.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 1000000.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.03
        };

        Odometry message;


        // Start is called before the first frame update
        protected override void Start()
        {
            base.Start();
            // Syncronize Tf publisher and odometry publisher
            odomTimestampSync = GetComponent<OdomTimestampSync>();
            // Add tf_prefix to frame id if necessary (?)
            string tf_prefix = GetComponent<OdomTfPublisher>().TF_prefix;
            if (tf_prefix.Length > 0)
            {
                ChildFrameId = tf_prefix + "/" + ChildFrameId;
            }
            // Create new Gameobject to hold initial transform
            InitialPoseHolder.transform.position = BaseLinkTransform.position;
            InitialPoseHolder.transform.rotation = BaseLinkTransform.rotation;
            // Initialize odometry message
            InitializeMessage();
        }

        // Update is called once per frame
        void FixedUpdate()
        {
            UpdateMessage();
        }


        void InitializeMessage()
        {
            // Initialize message: we are goint to assume that the covariance matrix
            // of the "measured" pose and twist are constant since this is a simulation
            // TODO: maybe add some Gaussian noise to make it more realistic (?)
            message = new Odometry
            {
                child_frame_id = ChildFrameId,
                header = new Header()
                {
                    frame_id = FrameId
                },
                pose = new RosSharp.RosBridgeClient.MessageTypes.Geometry.PoseWithCovariance()
                {
                    covariance = _poseCovariance
                },
                twist = new RosSharp.RosBridgeClient.MessageTypes.Geometry.TwistWithCovariance()
                {
                    covariance = _twistCovariance
                }
            };
        }

        void UpdateMessage()
        {
            // Update header - manually to synchronize with OdomTfPublisher msgs
            message.header.seq++;
            odomTimestampSync.UpdateTimestamp(out message.header.stamp);
            // Compute relative position
            relativePosition = InitialPoseHolder.transform.InverseTransformPoint(BaseLinkTransform.position);
            UpdatePosition(RosSharp.TransformExtensions.Unity2Ros(relativePosition));
            // Update orientation
            // TODO: check if this is correct
            relativeOrientation = Quaternion.Inverse(InitialPoseHolder.transform.rotation) * BaseLinkTransform.rotation; 
            UpdateOrientation(RosSharp.TransformExtensions.Unity2Ros(relativeOrientation));
            // Update linear velocity
            UpdateLinearVelocity(RosSharp.TransformExtensions.Unity2Ros(BaseLinkRigidBody.velocity));
            // Update angular velocity
            UpdateAngularVelocity(RosSharp.TransformExtensions.Unity2Ros(BaseLinkRigidBody.angularVelocity));
            Publish(message);
        }

        void UpdatePosition(Vector3 newPosition)
        {
            message.pose.pose.position.x = newPosition.x;
            message.pose.pose.position.y = newPosition.y;
            //message.pose.pose.position.z = newPosition.z;
            message.pose.pose.position.z = 0;
        }

        void UpdateOrientation(Quaternion newOrientation)
        {
            message.pose.pose.orientation.x = newOrientation.x;
            message.pose.pose.orientation.y = newOrientation.y;
            message.pose.pose.orientation.z = newOrientation.z;
            message.pose.pose.orientation.w = newOrientation.w;
        }

        void UpdateLinearVelocity(Vector3 newLinear)
        {
            message.twist.twist.linear.x = newLinear.x;
            message.twist.twist.linear.y = newLinear.y;
            //message.twist.twist.linear.z = newLinear.z;
            message.twist.twist.linear.z = 0;
        }

        void UpdateAngularVelocity(Vector3 newAngular)
        {
            message.twist.twist.angular.x = newAngular.x;
            message.twist.twist.angular.y = newAngular.y;
            message.twist.twist.angular.z = newAngular.z;
        }

    }

}
