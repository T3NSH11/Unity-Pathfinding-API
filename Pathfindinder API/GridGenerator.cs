using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;


namespace Pathfindinder_API
{

    public class GridGenerator : MonoBehaviour
    {
        public float CellSize;
        public Vector2Int GridSize;
        public PathfindingNode[,] nodes;
        public bool ShowGridCells;
        public bool ShowGridLines;
        float Offsety;
        float Offsetx;

        private void Awake()
        {
            nodes = new PathfindingNode[GridSize.x, GridSize.y];

            for (int y = 0; y < GridSize.y; y++)
            {
                for (int x = 0; x < GridSize.x; x++)
                {
                    nodes[x, y] = new PathfindingNode();
                }
            }

            CreateGrid();
        }

        void CreateGrid()
        {
            for (int y = 0; y < GridSize.y; y++)
            {
                Offsety = CellSize * y;

                for (int x = 0; x < GridSize.x; x++)
                {
                    Offsetx = CellSize * x;

                    nodes[x, y].WorldPosition = new Vector3(
                        transform.position.x + Offsetx,
                        transform.position.y,
                        transform.position.z + Offsety);

                    nodes[x, y].GridPosition = new Vector2Int(x, y);
                }
            }
        }

        private void OnDrawGizmos()
        {
            if (ShowGridCells)
            {
                DrawGridCells();
            }

            if (ShowGridLines)
            {
                DrawGridLines();
            }
        }

        void DrawGridCells()
        {
            Gizmos.color = new Color(0.5f, 0.5f, 0.5f, 0.5f);


            for (int x = 0; x < GridSize.x; x++)
            {
                for (int z = 0; z < GridSize.y; z++)
                {
                    Gizmos.DrawCube(
                        new Vector3((x * CellSize + CellSize / 2f) + transform.position.x, transform.position.y, (z * CellSize + CellSize / 2f) + transform.position.z),
                        new Vector3(CellSize, 0.05f, CellSize)
                    );
                }
            }
        }

        void DrawGridLines()
        {
            #region Draw lines to represent grid
            Gizmos.color = Color.red;

            for (int x = 0; x <= GridSize.x; x++)
            {
                Gizmos.DrawLine(
                    new Vector3(x * CellSize + transform.position.x, transform.position.y, transform.position.z),
                    new Vector3(x * CellSize + transform.position.x, transform.position.y, GridSize.x * CellSize + transform.position.z));
            }

            for (int z = 0; z <= GridSize.y; z++)
            {
                Gizmos.DrawLine(
                    new Vector3(transform.position.x, transform.position.y, z * CellSize + transform.position.z),
                    new Vector3(GridSize.y * CellSize + transform.position.x, transform.position.y, z * CellSize + transform.position.z));
            }
            #endregion
        }
    }
}
