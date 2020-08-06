using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.IO;
using UnityEngine;
using System.Text;


public class SpatialMeshExportManager : MonoBehaviour
{
    [SerializeField] private SpatialMeshAccessManager spatialMeshAccessManager = null;
    public GameObject ref_gameObject;
    private static Transform ref_transform;

    public enum CoordinateSpace
    {
        Left,
        Right
    }

    // Start is called before the first frame update
    void Start()
    {
        ref_transform = ref_gameObject.transform;
    }

    // Update is called once per frame
    void Update()
    {

    }

    public byte[] ExportMesh()
    {
        List<MeshFilter> meshes = spatialMeshAccessManager.GetSpatialMappingMesh();

        return SendSTLByte(meshes);
    }

    private static byte[] SendSTLByte(IList<MeshFilter> meshFilters, bool convertToRightHandedCoordinates = true)
    {
        try
        {
            MemoryStream stream = new MemoryStream();
            using (BinaryWriter writer = new BinaryWriter(stream, new ASCIIEncoding()))
            {
                // 80 byte header
                writer.Write(new byte[80]);
                uint totalTriangleCount = (uint)(meshFilters.Sum(x => x.sharedMesh.triangles.Length) / 3);

                // unsigned long facet count (4 bytes)
                writer.Write(totalTriangleCount);

                foreach (MeshFilter meshFilter in meshFilters)
                {
                    Vector3[] V = meshFilter.sharedMesh.vertices;
                    for ( int i = 0, vLength = V.Length; i < vLength; i++)
                    {
                        V[i] = Quaternion.Inverse(ref_transform.rotation) * (meshFilter.transform.TransformPoint(V[i]) - ref_transform.position);
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
                            V[i] = ToCoordinateSpace(V[i], CoordinateSpace.Right);
                            N[i] = ToCoordinateSpace(N[i], CoordinateSpace.Right);
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

                        // specification says attribute byte count should be set to 0.
                        writer.Write((ushort)0);
                    }
                }
                byte[] STLbyte = stream.ToArray();
#if DEBUG
                ByteArrayToFile("MeshDebugDump", STLbyte);
#endif
                return STLbyte;
            }
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

    private static Vector3 ToCoordinateSpace(Vector3 point, CoordinateSpace space)
    {
        if (space == CoordinateSpace.Left)
            return new Vector3(-point.y, point.z, point.x);

        return new Vector3(point.z, -point.x, point.y);
    }

    private static Vector3 AvgNrm(Vector3 a, Vector3 b, Vector3 c)
    {
        return new Vector3(
            (a.x + b.x + c.x) / 3f,
            (a.y + b.y + c.y) / 3f,
            (a.z + b.z + c.z) / 3f);
    }
}