using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using OpenCVForUnity.CoreModule;
using TMPro;
using UnityEngine;
using Vector3 = UnityEngine.Vector3;

namespace Script
{
    public class MarkerArea : MonoBehaviour
    {
        private Mesh _mesh;
        private MeshRenderer _meshRenderer;
        private static readonly int[] Tris = { 0, 1, 2, 1, 3, 2 };
        private Camera _camera;

        private static readonly int[] CWise02 = { 0, 1, 2 };
        private static readonly int[] CcWise02 = { 0, 2, 1 };
        private static readonly int[] CWise13 = { 0, 2, 3 };
        private static readonly int[] CcWise13 = { 0, 3, 2 };

        /// <summary>
        /// Start is called before the first frame update
        /// </summary>
        void Start()
        {
            _mesh = GetComponent<MeshFilter>().mesh;
            _meshRenderer = GetComponent<MeshRenderer>();
            _camera = Camera.main;
            HideArea();
        }

        /// <summary>
        /// Creates a quad 
        /// </summary>
        /// <param name="pts">4 Points representing the quad</param>
        public void DefineNewArea(List<Vector3> pts)
        {
            if (pts.Count != 4) return;
            ShowArea();
            _mesh.vertices = pts.ToArray();
            _mesh.triangles = OrderTriangles(pts);
            _mesh.RecalculateBounds();
            _mesh.RecalculateNormals();
        }

        private int[] OrderTriangles(List<Vector3> pts)
        {
            var first3 = IsClockwiseTris(pts[0], pts[1], pts[2]) ? CWise02 : CcWise02;
            var last3 = IsClockwiseTris(pts[0], pts[2], pts[3]) ? CWise13 : CcWise13;
            return first3.JoinArray(last3);
        }

        private bool IsClockwiseTris(Vector3 a, Vector3 b, Vector3 c)
        {
            // Determine the two vectors that define triangle
            var ab = b - a;
            var bc = c - b;
            // Calculate normal vector of triangle using cross product
            var norm = Vector3.Cross(ab, bc);
            // Determine triangle facing using dot product with camera 
            return Vector3.Dot(_camera.transform.forward, norm) < 0;
        }

        public void HideArea() => _meshRenderer.enabled = false;
        public void ShowArea() => _meshRenderer.enabled = true;
    }
}