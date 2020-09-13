using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Linq;
using System.Security.Cryptography;
using System.Text;
using System.Threading.Tasks;
using Rhino.Geometry;

namespace Generate
{
    public static class TestData
    {
        private static Point3d[] RandomPoints(BoundingBox bounds, int numPoints)
        {
            Vector3d diagonal = bounds.Diagonal;
            Random rand = new Random(42);
            Point3d[] points = new Point3d[numPoints];
            for (int i = 0; i < numPoints; i++)
            {
                points[i].X = bounds.Min.X + rand.NextDouble() * diagonal.X;
                points[i].Y = bounds.Min.Y + rand.NextDouble() * diagonal.Y;
                points[i].Z = bounds.Min.Z + rand.NextDouble() * diagonal.Z;
            }
            return points;
        }

        private static void WriteMesh(string path, string name, Mesh mesh)
        {
            int estimate = (sizeof(uint) * 2 ) + (mesh.Vertices.Count * 3 * sizeof(float)) + (mesh.Faces.Count * 3 * sizeof(uint));
            byte[] bytes = new byte[estimate];
            int pos = 0;
            Array.Copy(BitConverter.GetBytes((uint)mesh.Vertices.Count), 0, bytes, pos, sizeof(uint));
            pos += sizeof(uint);
            Array.Copy(BitConverter.GetBytes((uint)mesh.Faces.Count), 0, bytes, pos, sizeof(uint));
            pos += sizeof(uint);
            foreach (var pt in mesh.Vertices)
            {
                Array.Copy(BitConverter.GetBytes(pt.X), 0, bytes, pos, sizeof(float));
                pos += sizeof(float);
                Array.Copy(BitConverter.GetBytes(pt.Y), 0, bytes, pos, sizeof(float));
                pos += sizeof(float);
                Array.Copy(BitConverter.GetBytes(pt.Z), 0, bytes, pos, sizeof(float));
                pos += sizeof(float);
            }
            foreach (var face in mesh.Faces)
            {
                Array.Copy(BitConverter.GetBytes((uint)face.A), 0, bytes, pos, sizeof(uint));
                pos += sizeof(uint);
                Array.Copy(BitConverter.GetBytes((uint)face.B), 0, bytes, pos, sizeof(uint));
                pos += sizeof(uint);
                Array.Copy(BitConverter.GetBytes((uint)face.C), 0, bytes, pos, sizeof(uint));
                pos += sizeof(uint);
            }

            File.WriteAllBytes(Path.Combine(path, name + "_mesh.dat"), bytes);
        }

        private static void WritePoints(string path, string name, Point3d[] set1, Point3d[] set2)
        {
            if (set1.Length != set2.Length) throw new ArgumentException("Unequal sets");
            var bytes = new byte[sizeof(uint) + (set1.Length + set2.Length) * 3 * sizeof(float)];
            int pos = 0;
            Array.Copy(BitConverter.GetBytes((uint)set1.Length), 0, bytes, pos, sizeof(uint));
            pos += sizeof(uint);
            foreach (var pt in set1.Concat(set2))
            {
                Array.Copy(BitConverter.GetBytes((float)pt.X), 0, bytes, pos, sizeof(float));
                pos += sizeof(float);
                Array.Copy(BitConverter.GetBytes((float)pt.Y), 0, bytes, pos, sizeof(float));
                pos += sizeof(float);
                Array.Copy(BitConverter.GetBytes((float)pt.Z), 0, bytes, pos, sizeof(float));
                pos += sizeof(float);
            }

            File.WriteAllBytes(Path.Combine(path, name + "_points.dat"), bytes);
        }

        public static void WriteTestData(string path, string name, Mesh mesh, int numPoints)
        {
            mesh.Faces.ConvertQuadsToTriangles();
            BoundingBox box = mesh.GetBoundingBox(true);
            box.Inflate(box.Diagonal.Length / 8);
            Point3d[] set1 = RandomPoints(box, numPoints);
            Point3d[] set2 = set1.Select(p => mesh.ClosestPoint(p)).ToArray();
            WriteMesh(path, name, mesh);
            WritePoints(path, name, set1, set2);
        }

        public static void ReadTestPoints(string path, string name, out List<Point3d> samples, out List<Point3d> expectedResults)
        {
            var bytes = File.ReadAllBytes(Path.Combine(path, name + "_points.dat"));
            int pos = 0;
            uint nPoints = BitConverter.ToUInt32(bytes, pos);
            pos += sizeof(uint);
            samples = new List<Point3d>();
            expectedResults = new List<Point3d>();
            for (int i = 0; i < nPoints; i++)
            {
                float x = BitConverter.ToSingle(bytes, pos);
                pos += sizeof(float);
                float y = BitConverter.ToSingle(bytes, pos);
                pos += sizeof(float);
                float z = BitConverter.ToSingle(bytes, pos);
                pos += sizeof(float);
                samples.Add(new Point3d(x, y, z));
            }

            for (int i = 0; i < nPoints; i++)
            {
                float x = BitConverter.ToSingle(bytes, pos);
                pos += sizeof(float);
                float y = BitConverter.ToSingle(bytes, pos);
                pos += sizeof(float);
                float z = BitConverter.ToSingle(bytes, pos);
                pos += sizeof(float);
                expectedResults.Add(new Point3d(x, y, z));
            }
        }

        public static void ReadResults(string path, string name, out List<Point3d> results)
        {
            var bytes = File.ReadAllBytes(Path.Combine(path, name + "_results.dat"));
            int pos = 0;
            uint nPoints = BitConverter.ToUInt32(bytes, pos);
            pos += sizeof(uint);
            results = new List<Point3d>();

            for (int i = 0; i < nPoints; i++)
            {
                float x = BitConverter.ToSingle(bytes, pos);
                pos += sizeof(float);
                float y = BitConverter.ToSingle(bytes, pos);
                pos += sizeof(float);
                float z = BitConverter.ToSingle(bytes, pos);
                pos += sizeof(float);
                results.Add(new Point3d(x, y, z));
            }
        }
    }
}
