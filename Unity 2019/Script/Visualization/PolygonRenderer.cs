using System;
using System.Collections.Generic;
using UnityEngine;

namespace Script.Visualization
{
    
    public class PolygonRenderer : MonoBehaviour
    {
    
        [SerializeField] [Tooltip("Point prefab used polygon rendering")]
        private GameObject pointPrefab;

        private LineRenderer lr;
        private List<Vector3> points;
        

        private void Awake()
        {
            lr = GetComponent<LineRenderer>();
        }
        
    }
}

