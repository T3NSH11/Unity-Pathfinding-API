using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace Pathfindinder_API
{
    public class PathFinder
    {
        public GridGenerator grid; // the grid that the pathfinder will use
        int version; // version used for tracking changes to nodes

        PathfindingNode originNode; // starting PathfindingNode of the pathfinding
        PathfindingNode targetNode; // target PathfindingNode of the pathfinding

        #region MoveObjectvariables
        private PathfindingNode lastTargetNode; // the last target PathfindingNode used in MoveObjectToPosition method
        private Stack<Vector3> currentPath; // the current path of nodes that the object needs to follow
        #endregion

        // the main method to find a path
        public Stack<Vector3> FindPath(Vector3 originPosition, Vector3 targetPosition, int ID)
        {
            originNode = GetNodeFromWorldPosition(originPosition);
            targetNode = GetNodeFromWorldPosition(targetPosition);

            PathfindingNode currentNode = originNode;
            List<PathfindingNode> openNodes = new List<PathfindingNode>();

            version++; // increase version for tracking changes

            // keep searching until the target PathfindingNode is reached
            while (currentNode.GridPosition != targetNode.GridPosition)
            {
                currentNode.isVisted[ID] = true; // mark the current PathfindingNode as visited

                if (openNodes.Contains(currentNode))
                {
                    openNodes.Remove(currentNode); // remove current PathfindingNode from open nodes if it exists
                }

                List<PathfindingNode> neighbors = FindNeigbours(currentNode, grid, ID); // find the neighboring nodes
                foreach (PathfindingNode neighbor in neighbors)
                {
                    if (neighbor.GridPosition == targetNode.GridPosition) // if the target PathfindingNode is found
                    {
                        neighbor.parent[ID] = currentNode;
                        currentNode = neighbor;
                        openNodes.Clear(); // clear the open nodes list
                        return (GeneratePath(currentNode, originNode, ID)); // generate path and return it
                    }

                    // update neighbor PathfindingNode if it's not visited and has a better G cost
                    if (neighbor.Gcost[ID] > GetGcost(neighbor, currentNode, ID) && neighbor.isVisted[ID] == false && neighbor.version[ID] == version)
                    {
                        neighbor.Gcost[ID] = GetGcost(neighbor, currentNode, ID);
                        neighbor.Hcost[ID] = GetHcost(neighbor, targetNode);
                        neighbor.Fcost[ID] = GetFcost(neighbor, ID);
                        neighbor.parent[ID] = currentNode;
                        openNodes.Add(neighbor);
                    }
                    // update neighbor PathfindingNode if it's not visited and has a new version
                    else if (neighbor.version[ID] < version)
                    {
                        neighbor.Gcost[ID] = GetGcost(neighbor, currentNode, ID);
                        neighbor.Hcost[ID] = GetHcost(neighbor, targetNode);
                        neighbor.Fcost[ID] = GetFcost(neighbor, ID);
                        neighbor.parent[ID] = currentNode;
                        neighbor.isVisted[ID] = false;
                        neighbor.version[ID] = version;
                        openNodes.Add(neighbor);
                    }
                }

                currentNode = FindLowestFcost(openNodes, ID); // find the PathfindingNode with the lowest F cost in the open nodes list
            }

            openNodes.Clear(); // clear the open nodes list
            return (GeneratePath(currentNode, originNode, ID)); // generate path and return it
        }

        // This function takes in a PathfindingNode and grid as input parameters, and returns a list of neighbouring nodes.
        List<PathfindingNode> FindNeigbours(PathfindingNode PathfindingNode, GridGenerator GridGenerator, int ID)
        {
            // Create an empty list to store neighbouring nodes.
            List<PathfindingNode> neighbours = new List<PathfindingNode>();

            // Loop through all adjacent positions around the current PathfindingNode.
            for (int x = -1; x <= 1; x++)
            {
                for (int y = -1; y <= 1; y++)
                {
                    // Skip the current PathfindingNode itself.
                    if (x == 0 && y == 0) continue;

                    // Calculate the grid position of the adjacent PathfindingNode to check.
                    int checkX = PathfindingNode.GridPosition.x + x;
                    int checkY = PathfindingNode.GridPosition.y + y;

                    // Check if the adjacent PathfindingNode is within the boundaries of the grid.
                    if (checkX >= 0 && checkX < GridGenerator.GridSize.x && checkY >= 0 && checkY < GridGenerator.GridSize.y)
                    {
                        // Get the neighbouring PathfindingNode from the grid and add it to the list of neighbours.
                        PathfindingNode neighbour = GridGenerator.nodes[checkX, checkY];
                        neighbours.Add(neighbour);
                    }
                }
            }
            // Return the list of neighbouring nodes.
            return neighbours;
        }


        // Calculates the G cost of a given PathfindingNode, which is the cost to move from the start PathfindingNode to the current PathfindingNode.
        float GetGcost(PathfindingNode PathfindingNode, PathfindingNode currentNode, int ID)
        {
            // If the PathfindingNode has a parent, add the distance from the parent to the current PathfindingNode's G cost.
            if (PathfindingNode.parent[ID] != null)
            {
                return (PathfindingNode.parent[ID].Gcost[ID] + Vector3.Distance(currentNode.WorldPosition, PathfindingNode.WorldPosition));
            }
            // If the PathfindingNode does not have a parent, return 0 as the G cost.
            else
                return 0f;
        }

        // Calculates the H cost of a given PathfindingNode, which is the heuristic cost to move from the current PathfindingNode to the target PathfindingNode.
        float GetHcost(PathfindingNode PathfindingNode, PathfindingNode targetNode)
        {
            // The Manhattan distance formula is used to calculate the heuristic cost.
            return Mathf.Sqrt(Mathf.Pow((PathfindingNode.WorldPosition.x - targetNode.WorldPosition.x), 2) + Mathf.Pow((PathfindingNode.WorldPosition.z - targetNode.WorldPosition.z), 2));
        }

        // Calculates the F cost of a given PathfindingNode, which is the sum of the G and H costs.
        float GetFcost(PathfindingNode PathfindingNode, int ID)
        {
            return (PathfindingNode.Hcost[ID] + PathfindingNode.Gcost[ID]);
        }


        // Find the PathfindingNode with the lowest F cost from a list of open nodes
        PathfindingNode FindLowestFcost(List<PathfindingNode> openNodes, int ID)
        {
            // Set initial values for the best PathfindingNode
            PathfindingNode BestNode = new PathfindingNode();
            BestNode.Fcost[ID] = Mathf.Infinity;

            // Iterate through each PathfindingNode in the open nodes list
            foreach (PathfindingNode PathfindingNode in openNodes)
            {
                // If the PathfindingNode's F cost is lower than the best PathfindingNode's F cost, update the best PathfindingNode
                if (PathfindingNode.Fcost[ID] < BestNode.Fcost[ID])
                {
                    BestNode = PathfindingNode;
                }
            }

            // Return the PathfindingNode with the lowest F cost
            return BestNode;
        }

        // Get the PathfindingNode in a grid from a world position
        public PathfindingNode GetNodeFromWorldPosition(Vector3 worldPosition)
        {
            // Calculate the x and y coordinates of the PathfindingNode based on the world position and grid properties
            int x = Mathf.FloorToInt((worldPosition.x - grid.transform.position.x) / grid.CellSize);
            int y = Mathf.FloorToInt((worldPosition.z - grid.transform.position.z) / grid.CellSize);

            // Check if the calculated x and y values are within the grid's boundaries
            if (x >= 0 && x < grid.nodes.GetLength(0) && y >= 0 && y < grid.nodes.GetLength(1))
            {
                // Return the PathfindingNode from the grid at the calculated x and y coordinates
                return grid.nodes[x, y];
            }
            else
            {
                // If the calculated x and y values are outside the grid's boundaries, return null
                return null;
            }
        }

        // Generate a path from a PathfindingNode to the origin PathfindingNode
        Stack<Vector3> GeneratePath(PathfindingNode PathfindingNode, PathfindingNode originNode, int ID)
        {
            // Create a stack to store the path
            Stack<Vector3> path = new Stack<Vector3>();

            // Traverse the path from the target PathfindingNode back to the origin PathfindingNode and add each PathfindingNode's position to the path stack
            while (PathfindingNode != originNode)
            {
                path.Push(PathfindingNode.WorldPosition);
                PathfindingNode = PathfindingNode.parent[ID];
            }

            // Add the origin PathfindingNode's position to the path stack
            if (PathfindingNode == originNode)
            {
                path.Push(PathfindingNode.WorldPosition);
            }

            // Return the path stack
            return path;
        }


        public void MoveObjectToPosition(GameObject gameObject, Vector3 targetPosition, float speed, int enemyID)
        {
            // Get the target PathfindingNode based on the target position
            PathfindingNode targetNode = GetNodeFromWorldPosition(targetPosition);

            // Get the rigidbody component of the game object
            Rigidbody rb = gameObject.GetComponent<Rigidbody>();

            // Check if the game object has a rigidbody component
            if (rb == null)
            {
                Debug.LogError("No Rigidbody on provided GameObject");
                return;
            }

            // Check if the target PathfindingNode has changed or if there is no current path
            if (targetNode != lastTargetNode || currentPath == null)
            {
                lastTargetNode = targetNode;
                currentPath = FindPath(gameObject.transform.position, targetPosition, enemyID);
                currentPath.Reverse();
            }

            // Check if there is a current path and if it is not empty
            if (currentPath != null && currentPath.Count > 0)
            {
                // Get the next waypoint on the path
                Vector3 nextWaypoint = currentPath.Peek();

                // Keep popping the path until the game object is at least one grid cell away from the waypoint
                while (Vector3.Distance(gameObject.transform.position, nextWaypoint) <= grid.CellSize)
                {
                    currentPath.Pop();

                    // If the path is empty, return from the function
                    if (currentPath.Count == 0)
                    {
                        return;
                    }

                    // Get the next waypoint on the path
                    nextWaypoint = currentPath.Peek();
                }

                // Get the movement direction towards the next waypoint and rotate the game object towards that direction
                Vector3 movementDirection = (new Vector3(nextWaypoint.x, gameObject.transform.position.y, nextWaypoint.z) - gameObject.transform.position).normalized;
                Quaternion targetRotation = Quaternion.LookRotation(movementDirection);
                gameObject.transform.rotation = Quaternion.Slerp(gameObject.transform.rotation, targetRotation, speed * Time.deltaTime);

                // Move the game object towards the next waypoint based on the speed and delta time
                rb.MovePosition(gameObject.transform.position + movementDirection * speed * Time.deltaTime);
            }
        }
    }
}
