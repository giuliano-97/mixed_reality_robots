using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using RosSharp;
using RosSharp.RosBridgeClient;
using RosSharp.RosBridgeClient.MessageTypes.Tf2;

namespace MixedRealityRobot.Publishers
{
    [RequireComponent(typeof(OdomTimestampSync))]
    public class OdomTfPublisher : 
        UnityPublisher<TFMessage>
    {
        public string TF_prefix = "jackal";
        public GameObject InitialPoseHolder;
        public Transform BaseOdomTransform;
        private OdomTimestampSync odomTimestampSync;
        private TFMessage message;
        private Vector3 relativePosition;
        private Quaternion relativeRotation;
        private readonly string frameId = "odom";
        private string childFrameId = "base_link";

        // Start is called before the first frame update
        protected override void Start()
        {
            base.Start();
            // Add tf_prefix to childFrameId (if there is one)
            if (TF_prefix.Length > 0)
            {
                childFrameId = TF_prefix + "/" + childFrameId;
            }
            // Get synchronizer instance
            odomTimestampSync = GetComponent<OdomTimestampSync>();
            // Initialize initial pose holder 
            InitialPoseHolder.transform.position = BaseOdomTransform.position;
            InitialPoseHolder.transform.rotation = BaseOdomTransform.rotation;
            // Initialize message
            RosSharp.RosBridgeClient.MessageTypes.Geometry.TransformStamped tf =
                new RosSharp.RosBridgeClient.MessageTypes.Geometry.TransformStamped()
            {
                header = new RosSharp.RosBridgeClient.MessageTypes.Std.Header
                {
                    frame_id = frameId
                },
                child_frame_id = childFrameId
            };
            RosSharp.RosBridgeClient.MessageTypes.Geometry.TransformStamped[] tf_array = { tf };
            message = new TFMessage(tf_array);
        }

        // Update is called once per frame
        void FixedUpdate()
        {
            UpdateMessage();
        }

        void UpdateMessage()
        {
            // Update timestamp - manually to synchronize with OdometryPublisher
            message.transforms[0].header.seq++;
            odomTimestampSync.UpdateTimestamp(out message.transforms[0].header.stamp);
            // Update translation
            relativePosition = InitialPoseHolder.transform.InverseTransformPoint(BaseOdomTransform.position);
            UpdateTranslation(TransformExtensions.Unity2Ros(BaseOdomTransform.position));
            // Update rotation
            relativeRotation = Quaternion.Inverse(InitialPoseHolder.transform.rotation) * BaseOdomTransform.rotation;
            UpdateRotation(TransformExtensions.Unity2Ros(BaseOdomTransform.rotation));
            // Publish message
            Publish(message);
        }

        void UpdateTranslation(Vector3 translation)
        {
            message.transforms[0].transform.translation.x = translation.x;
            message.transforms[0].transform.translation.y = translation.y;
            message.transforms[0].transform.translation.z = 0;
        }

        void UpdateRotation(Quaternion rotation)
        {
            message.transforms[0].transform.rotation.x = rotation.x;
            message.transforms[0].transform.rotation.y = rotation.y;
            message.transforms[0].transform.rotation.z = rotation.z;
            message.transforms[0].transform.rotation.w = rotation.w;
        }
    }
}
