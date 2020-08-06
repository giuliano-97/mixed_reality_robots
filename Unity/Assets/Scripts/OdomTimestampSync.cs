using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using RosSharp.RosBridgeClient;
namespace MixedRealityRobot
{
    public class OdomTimestampSync : MonoBehaviour
    {
        // Create a timer to give the same timestamp
        private static Timer timer = new Timer();
        private RosSharp.RosBridgeClient.MessageTypes.Std.Time odomTimestamp;

        // Start is called before the first frame update
        void Start()
        {
            odomTimestamp = timer.Now();
        }

        // Update is called once per frame
        void FixedUpdate()
        {
            odomTimestamp = timer.Now();
        }

        public void UpdateTimestamp(out RosSharp.RosBridgeClient.MessageTypes.Std.Time stamp)
        {
            stamp = odomTimestamp;
        }
       
    }
}
