using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Pathfindinder_API
{
    public class PathfindingNode
    {
        public Dictionary<int, float> Gcost = new Dictionary<int, float>();
        public Dictionary<int, float> Fcost = new Dictionary<int, float>();
        public Dictionary<int, float> Hcost = new Dictionary<int, float>();
        public Dictionary<int, bool> isVisted = new Dictionary<int, bool>();
        public Dictionary<int, PathfindingNode> parent = new Dictionary<int, PathfindingNode>();
        public Dictionary<int, int> version = new Dictionary<int, int>();
        public Vector2Int GridPosition;
        public Vector3 WorldPosition;
    }
}
