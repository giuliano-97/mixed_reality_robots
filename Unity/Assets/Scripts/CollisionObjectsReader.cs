using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using UnityEngine;
using RosSharp.RosBridgeClient.MessageTypes.PandaUnitySimulation;

namespace MixedRealityRobot
{
    public class CollisionObjectsReader : MonoBehaviour
    {
        // This is to access the spatial mapping mesh
        [SerializeField] public SpatialMeshAccessManager spatialMeshAccessManager;
        // The reference frame wrt the obstacles are placed - in our case, the base of the manip
        public GameObject referenceFrame;
        private readonly bool _debug = false;

        public enum CoordinateSpace
        {
            Left,
            Right
        }

        // Nothing special
        void Start()
        {
        }

        // Nothing special, the meshes are retrieved only when it is necessary
        void Update()
        {
        }

        public List<BoundingBox> GetObstaclesBoundingBoxes()
        {
            // Get obstacles mesh filters
            MeshFilter[] obstMeshFilters = GetComponentsInChildren<MeshFilter>();

            // For each obstacle get the bounding box
            List<BoundingBox> boundingBoxes = new List<BoundingBox>();
            foreach(MeshFilter meshFilter in obstMeshFilters)
            {
                Matrix4x4 meshFilterLWMat = meshFilter.transform.localToWorldMatrix;
                Vector3 localDims = meshFilter.sharedMesh.bounds.size;
                Vector3 centerWorld = meshFilterLWMat.MultiplyPoint3x4(meshFilter.sharedMesh.bounds.center);
                Vector3 centerReference = RosSharp.TransformExtensions.Unity2Ros(referenceFrame.transform.InverseTransformPoint(centerWorld));
                Quaternion orientationWorld = meshFilter.transform.rotation;
                Quaternion orientationReference =
                    RosSharp.TransformExtensions.Unity2Ros(Quaternion.Inverse(referenceFrame.transform.rotation) * orientationWorld);
                Vector3 dims = RosSharp.TransformExtensions.Unity2Ros(Vector3.Scale(meshFilter.transform.localToWorldMatrix.lossyScale, localDims));

                BoundingBox bbox = new BoundingBox();
                bbox.header.frame_id = referenceFrame.name;
                bbox.name = meshFilter.name;
                bbox.pose.position.x = centerReference.x;
                bbox.pose.position.y = centerReference.y;
                bbox.pose.position.z = centerReference.z;
                bbox.pose.orientation.x = orientationReference.x;
                bbox.pose.orientation.y = orientationReference.y;
                bbox.pose.orientation.z = orientationReference.z;
                bbox.pose.orientation.w = orientationReference.w;
                bbox.dims.x = Math.Abs(dims.x);
                bbox.dims.y = Math.Abs(dims.y);
                bbox.dims.z = Math.Abs(dims.z);

                boundingBoxes.Add(bbox);
            }
            return boundingBoxes;
        }

        public List<SpatialMappingMesh> GetObstaclesMeshes()
        {
            // Get obstacles mesh filters
            MeshFilter[] obstMeshFilters = GetComponentsInChildren<MeshFilter>();

            // Convert each mesh filter to binary stl and wrap it into a message
            List<SpatialMappingMesh> meshes = new List<SpatialMappingMesh>();
            foreach (MeshFilter meshFilter in obstMeshFilters)
            {
                SpatialMappingMesh next = new SpatialMappingMesh();
                next.header.frame_id = "panda_link0";
                next.name = meshFilter.name;

                // Get mesh position and orientation in world coordinates and convert to ROS convention
                Vector3 position = RosSharp.TransformExtensions.Unity2Ros(meshFilter.transform.position);
                Quaternion orientation = RosSharp.TransformExtensions.Unity2Ros(meshFilter.transform.rotation);

                // Set mesh pose
                next.pose.position.x = position.x;
                next.pose.position.y = position.y;
                next.pose.position.z = position.z;
                next.pose.orientation.x = orientation.x;
                next.pose.orientation.y = orientation.y;
                next.pose.orientation.z = orientation.z;
                next.pose.orientation.w = orientation.w;

                // Convert mesh to binary stl stream
                next.mesh = SendSTLByte(meshFilter);

                // Add message to the list
                meshes.Add(next);
            }

            return meshes;
        }

        public List<SpatialMappingMesh> GetSpatialMappingMeshes()
        {
            // Get mesh filters
            List<MeshFilter> meshFilters = spatialMeshAccessManager.GetSpatialMappingMesh();

            // Convert each mesh filter to binary stl and wrap it into a message
            List<SpatialMappingMesh> meshes = new List<SpatialMappingMesh>();
            foreach(MeshFilter meshFilter in meshFilters)
            {
                SpatialMappingMesh next = new SpatialMappingMesh();
                next.header.frame_id = "world";
                next.name = meshFilter.name;

                // Get mesh position and orientation in world coordinates and convert to ROS convention
                //Vector3 position = RosSharp.TransformExtensions.Unity2Ros(referenceFrame.InverseTransformPoint(meshFilter.transform.position));
                Vector3 position = RosSharp.TransformExtensions.Unity2Ros(meshFilter.transform.position);
                Quaternion orientation = RosSharp.TransformExtensions.Unity2Ros(meshFilter.transform.rotation);

                // Set mesh pose
                next.pose.position.x = position.x; 
                next.pose.position.y = position.y;
                next.pose.position.z = position.z;
                next.pose.orientation.x = orientation.x;
                next.pose.orientation.y = orientation.y;
                next.pose.orientation.z = orientation.z;
                next.pose.orientation.w = orientation.w;

                // Convert mesh to binary stl stream
                next.mesh = SendSTLByte(meshFilter);

                // Add message to the list
                meshes.Add(next);
            }
            
            return meshes;
        }

        private byte[] SendSTLByte(MeshFilter meshFilter, bool convertToRightHandedCoordinates = true)
        {
            try
            {
                MemoryStream stream = new MemoryStream();
                using (BinaryWriter writer = new BinaryWriter(stream, new ASCIIEncoding()))
                {
                    // 80 byte header
                    writer.Write(new byte[80]);
                    uint totalTriangleCount = (uint) meshFilter.sharedMesh.triangles.Length / 3;

                    // unsigned long facet count (4 bytes)
                    writer.Write(totalTriangleCount);

                    Vector3[] V = meshFilter.sharedMesh.vertices;
                    for (int i = 0, vLength = V.Length; i < vLength; i++)
                    {
                        V[i] = meshFilter.transform.TransformPoint(V[i]);
                    }
                    Vector3[] N = meshFilter.sharedMesh.normals;
                    for (int i = 0, nLength = N.Length; i < nLength; i++)
                    {
                        N[i] = meshFilter.transform.TransformDirection(N[i]);
                    }

                    if (convertToRightHandedCoordinates)
                    {
                        for (int i = 0, c = V.Length; i < c; i++)
                        {
                            V[i] = RosSharp.TransformExtensions.Unity2Ros(V[i]);
                            N[i] = RosSharp.TransformExtensions.Unity2Ros(N[i]);
                        }
                    }

                    int[] t = meshFilter.sharedMesh.triangles;
                    int triangleCount = t.Length;

                    if (convertToRightHandedCoordinates)
                        System.Array.Reverse(t);

                    for (int i = 0; i < triangleCount; i += 3)
                    {
                        int a = t[i], b = t[i + 1], c = t[i + 2];

                        Vector3 avg = AvgNrm(N[a], N[b], N[c]);

                        writer.Write(avg.x);
                        writer.Write(avg.y);
                        writer.Write(avg.z);

                        writer.Write(V[a].x);
                        writer.Write(V[a].y);
                        writer.Write(V[a].z);

                        writer.Write(V[b].x);
                        writer.Write(V[b].y);
                        writer.Write(V[b].z);

                        writer.Write(V[c].x);
                        writer.Write(V[c].y);
                        writer.Write(V[c].z);

                        // stl specification says attribute byte count should be set to 0.
                        writer.Write((ushort)0);
                    }
                }
                byte[] STLbyte = stream.ToArray();

                if(_debug)
                    ByteArrayToFile(meshFilter.name, STLbyte);

                return STLbyte;
            }
            catch (System.Exception e)
            {
                UnityEngine.Debug.LogError(e.ToString());
                return new byte[0];
            }
        }

        private static bool ByteArrayToFile(string fileName, byte[] byteArray)
        {
            try
            {
                using (var fs = new FileStream(Path.Combine(Application.persistentDataPath, string.Format("{0}.stl", fileName)), FileMode.Create, FileAccess.Write))
                {
                    fs.Write(byteArray, 0, byteArray.Length);
                    fs.Flush();
                    Debug.Log(string.Format("File {0}.stl is created.", fileName));
                    return true;
                }
            }
            catch (System.Exception ex)
            {
                Debug.Log("Exception caught in process: " + ex.ToString());
                return false;
            }
        }

        private static Vector3 AvgNrm(Vector3 a, Vector3 b, Vector3 c)
        {
            return new Vector3(
                (a.x + b.x + c.x) / 3f,
                (a.y + b.y + c.y) / 3f,
                (a.z + b.z + c.z) / 3f);
        }
    }
}