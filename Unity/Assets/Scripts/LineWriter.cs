using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UIElements;

namespace MixedRealityRobot.Writers
{
    [RequireComponent(typeof(LineRenderer))]
    public class LineWriter : MonoBehaviour
    {
        private LineRenderer line;
        private List<Vector3> points;
        private bool done;
        public Transform ReferenceFrame;
        private bool isMsgReceived;

        public float xoff = 0f;
        public float yoff = 0f;
        public float zoff = 0f;


        // Start is called before the first frame update
        void Start()
        {
            line = GetComponent<LineRenderer>();
            done = true;
            isMsgReceived = false;
        }

        // Update is called once per frame
        void FixedUpdate()
        {
            Vector3 newPosition = new Vector3(ReferenceFrame.position.x + xoff, ReferenceFrame.position.y + yoff, ReferenceFrame.position.z + zoff);
            transform.position = newPosition;
            transform.rotation = ReferenceFrame.rotation;
            if (isMsgReceived)
                ProcessMessage();
        }

        private void ProcessMessage()
        {
            done = false;
            line.positionCount = points.Count;
            line.SetPositions(points.ToArray());
            done = true;
            isMsgReceived = false;
        }

        public void Write(List<Vector3> newPoints)
        {
            if(done)
            {
                points = newPoints;
                isMsgReceived = true;
            }
        }
    }

}
