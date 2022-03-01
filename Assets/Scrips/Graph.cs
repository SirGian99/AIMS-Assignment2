using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;



public class Graph{
    public int i_size;
    public int j_size;
    public float x_low;
    public float x_high;
    public float z_low;
    public float z_high;
    public float x_unit;
    public float z_unit;
    public Node start_node;
    public Node goal_node;
    public List<Node> path;
    public int walkable_nodes = 0;
    public int non_walkable_nodes = 0;
    public Vector3 centre;

    public int[,] graphTraversabilityMatrix;
        
    public Node[,] nodes;

    ////STC variables
    //public int VerticesCount;
    //public int EdgesCount;
    //public List<Edge> EdgeList;
    //public Node[] VertexArray;


    public Graph(int i_size, int j_size, float x_low, float x_high, float z_low, float z_high)
    {
        this.i_size = i_size;
        this.j_size = j_size;
        this.x_low = x_low;
        this.x_high = x_high;
        this.z_low = z_low;
        this.z_high = z_high;
        this.x_unit = (x_high - x_low) / i_size;
        this.z_unit = (z_high - z_low) / j_size;
        this.nodes = new Node[i_size, j_size];
        this.graphTraversabilityMatrix = new int[i_size, j_size];
        centre = new Vector3((x_high - x_low) / 2 + x_low, 0, (z_high - z_low) / 2 + z_low);
    }


    public void printTraversability()
    {
        string traversability_string = "";
        for (int i = 0; i < i_size; i++)
        {
            for (int j = 0; j < j_size; j++)
            {
                traversability_string += (graphTraversabilityMatrix[i, j] + " ");
            }
            traversability_string += "\n";
        }
        Debug.Log(traversability_string);
    }

    public static Graph CreateGraph(TerrainInfo terrainInfo, int x_N, int z_N)
    {
        if (terrainInfo == null)
        {
            //Debug.Log("terrain_manager is null");
            return null;
        }

        if (x_N <= 0 || z_N <= 0)
        {
            //Debug.Log("x_N or z_N is less than or equal to 0");
            return null;
        }

        //Debug.Log("myFunction");
        Graph graph = new Graph(x_N, z_N, terrainInfo.x_low, terrainInfo.x_high, terrainInfo.z_low, terrainInfo.z_high);
        float x_len = terrainInfo.x_high - terrainInfo.x_low;
        float z_len = terrainInfo.z_high - terrainInfo.z_low;
        float x_unit = x_len / x_N;
        float z_unit = z_len / z_N;
        graph.start_node = new Node((int)(terrainInfo.start_pos[0] / x_unit), (int)(terrainInfo.start_pos[2] / z_unit), terrainInfo.start_pos[0], terrainInfo.start_pos[2]);
        graph.goal_node = new Node((int)(terrainInfo.goal_pos[0] / x_unit), (int)(terrainInfo.goal_pos[2] / z_unit), terrainInfo.goal_pos[0], terrainInfo.goal_pos[2]);

        for (int i = 0; i < x_N; i++)
        {
            for (int j = 0; j < z_N; j++)
            {
                float x_center = terrainInfo.x_low + x_unit * (i + 0.5f);
                float z_center = terrainInfo.z_low + z_unit * (j + 0.5f);
                Node node = new Node(i, j, x_center, z_center);
                graph.nodes[i, j] = node;


                int i_index = terrainInfo.get_i_index(x_center);
                int j_index = terrainInfo.get_j_index(z_center);
                Collider[] collision = Physics.OverlapSphere(new Vector3(x_center, 1, z_center), Math.Max(x_unit/2, z_unit/2));
                bool walkable = true;
                foreach (Collider c in collision)
                {
                    if (c.name == "Cube")
                    {
                        walkable = false;
                        break;
                    }
                }
                walkable = true; //TODO understand why if I implement this (hence, if i remove this line) no path is found on terrain B

                if (terrainInfo.traversability[i_index, j_index] == 0 && walkable)
                {
                    graph.walkable_nodes += 1;
                    //Gizmos.color = Color.blue;
                    graph.graphTraversabilityMatrix[i, j] = 0; //0 means that the node is traversable
                    if (i == x_N || j == z_N)
                        continue;
                    int next_i_index = terrainInfo.get_i_index(x_center + x_unit); //right
                    if (terrainInfo.traversability[next_i_index, j_index] == 0)
                    {
                        Node right_node = new Node(i + 1, j, x_center + x_unit, z_center);
                        graph.graphTraversabilityMatrix[i + 1, j] = 0;
                        //Debug.Log("RIGHT:" + "Adding edge from node (" + node.i + "," + node.j + ") to node (" + right_node.i + "," + right_node.j + ")");
                    }
                    else
                    {
                        //Debug.Log("RIGHT:" + "Not adding edge from node (" + i + "," + j + ") to node (" + (i + 1) + "," + j + ")");
                        graph.graphTraversabilityMatrix[i + 1, j] = 1;
                    }

                    int next_j_index = terrainInfo.get_j_index(z_center + z_unit); //up
                    if (terrainInfo.traversability[i_index, next_j_index] == 0)
                    {
                        Node up_node = new Node(i, j + 1, x_center, z_center + z_unit);
                        graph.graphTraversabilityMatrix[i, j + 1] = 0;
                        //Debug.Log("UP: Adding edge from node (" + node.i + "," + node.j + ") to node (" + up_node.i + "," + up_node.j + ")");

                    }
                    else
                    {
                        //Gizmos.color = Color.red;
                        //Debug.Log("RIGHT:" + "Not adding edge from node (" + i + "," + j + ") to node (" + (i) + "," + (j + 1) + ")");
                        graph.graphTraversabilityMatrix[i, j + 1] = 1;

                    }
                    //Gizmos.DrawCube(new Vector3(x_center, 0, z_center), new Vector3(x_len - 0.1f, 0.1f, z_len - 0.1f));



                    /*int next_i_index = terrain_manager.get_i_index(x_center - x_unit); //left
                    if (terrain_manager.traversability[next_i_index, j_index] == 0){
                        Node left_node = new Node(i-1, j, x_center - x_unit, z_center);
                        GraphEdge edge = new GraphEdge(node, left_node);
                    }
                    int next_j_index = terrain_manager.get_j_index(z_center + z_unit); //down
                    if (terrain_manager.traversability[i_index, next_j_index] == 0){
                        Node down_node = new Node(i, j-1, x_center, z_center - z_unit);
                        GraphEdge edge = new GraphEdge(node, down_node);
                    }
                    */

                }
                else
                {
                    node.walkable = false;
                    graph.non_walkable_nodes += 1;
                    graph.graphTraversabilityMatrix[i, j] = 1; //1 means that the node is not traversable
                }

            }
        }

        foreach(Node node in graph.nodes)
        {
            node.neighbours = graph.getNeighbours(node);
        }

        ////// STC Code addition
        ////
        ////Create EdgesList and VertexArray
        //List<Node> VertexList = new List<Node>();
        //Edge newEdge = new Edge();
        //graph.EdgeList = new List<Edge>();
        //foreach (Node node in graph.nodes)
        //{
        //    foreach (Node neighbour in node.neighbours)
        //    {
        //        if (node.walkable == true && neighbour.walkable == true)
        //        {
        //            newEdge.Source = node;
        //            newEdge.Destination = neighbour;
        //            newEdge.Weight = 1;
        //            graph.EdgeList.Add(newEdge);
        //        }
        //    }
        //    if (node.walkable == true)
        //    {
        //        VertexList.Add(node);
        //    }
            
        //}
        //graph.EdgesCount = graph.EdgeList.Count;
        //graph.VerticesCount = VertexList.Count;
        //graph.VertexArray = new Node[graph.VerticesCount];
        //for (int v = 0; v < VertexList.Count; v++)
        //{
        //    graph.VertexArray[v] = VertexList[v];
        //}

        return graph;
    }

    public Node getNodeFromPoint(Vector3 position)
    {
        //Debug.Log(position.x);
        //Debug.Log(position.z);

        int i = (int)((position.x - x_low) / x_unit);
        int j = (int)((position.z - z_low)/ z_unit);
        //Debug.Log(i);
        //Debug.Log(j);
        //Debug.Log("Size i: " + nodes.GetLength(0) + " Size j: " + nodes.GetLength(1));
        return nodes[i, j];
    }

    public List<Node> getNeighbours(Node node, bool print=false, bool check_vehicle = false, bool supernodes = false)
    {
        List<Node> toReturn = new List<Node>();
        int supernode_addition = supernodes && node.is_supernode? 1:0; 
        for(int i = -1; i<2+supernode_addition; i++)
        {
            int current_i = node.i + i;
            if (current_i < 0 || current_i >= nodes.GetLength(0))
                continue;
            for(int j = -1; j<2+supernode_addition; j++)
            {
                int current_j = node.j + j;
                if (current_j < 0 || current_j >= nodes.GetLength(1) || i==0 && j==0 || nodes[current_i, current_j]==null)
                    continue;
                if (check_vehicle && node.assigned_veichle != nodes[current_i, current_j].assigned_veichle) //addition for Ass2.1
                    continue;
                if (nodes[current_i, current_j] == node || toReturn.Contains(nodes[current_i, current_j]))
                    continue;
                if (supernodes && node.is_supernode && current_i != node.i && current_j != node.j && nodes[current_i, current_j] != null && (nodes[current_i, current_j].is_supernode)) 
                {
                    Debug.Log("Nodo [" + node.i + "," + node.j + "] skippo il supernodo ["  + current_i + "," + current_j);
                    continue;
                }

                if(supernodes && node.is_supernode)
                {
                    if (i == -1 && j == -1)
                        continue;
                    if (i == -1 && j == 2)
                        continue;
                    if (i == 2 && j == -1)
                        continue;
                    if (i == 2 && j == 2)
                        continue;
                }

                toReturn.Add(nodes[current_i, current_j]);


                if (!nodes[current_i, current_j].walkable)
                {
                    if (node.i == current_i)
                    {
                        node.wallClosenessCost += 12 * x_unit + 0.01f; //it means that the wall is above or behind, better go in the other direction
                    }
                    else if (node.j == current_j)
                        node.wallClosenessCost += 12 * z_unit + 0.01f; //it means that the wall is next or before, better go in the other direction
                    else
                        node.wallClosenessCost += 2*(float)Math.Sqrt(x_unit * x_unit + z_unit * z_unit); //it means the wall is diagonally placed, better go in the other direction
                    if (print)
                        Debug.Log("Current node penalty: " + node.wallClosenessCost);
                }
            }
        }
        return toReturn;
    }






    public static Graph CreateSubGraph(Graph old_graph, int car_index, TerrainInfo terrainInfo, int x_N, int z_N)
    {
        if (terrainInfo == null)
        {
            //Debug.Log("terrain_manager is null");
            return null;
        }

        if (x_N <= 0 || z_N <= 0)
        {
            //Debug.Log("x_N or z_N is less than or equal to 0");
            return null;
        }

        //Debug.Log("myFunction");
        Graph graph = new Graph(x_N, z_N, terrainInfo.x_low, terrainInfo.x_high, terrainInfo.z_low, terrainInfo.z_high);
        float x_len = terrainInfo.x_high - terrainInfo.x_low;
        float z_len = terrainInfo.z_high - terrainInfo.z_low;
        float x_unit = x_len / x_N;
        float z_unit = z_len / z_N;
        graph.start_node = new Node((int)(terrainInfo.start_pos[0] / x_unit), (int)(terrainInfo.start_pos[2] / z_unit), terrainInfo.start_pos[0], terrainInfo.start_pos[2]);
        graph.goal_node = new Node((int)(terrainInfo.goal_pos[0] / x_unit), (int)(terrainInfo.goal_pos[2] / z_unit), terrainInfo.goal_pos[0], terrainInfo.goal_pos[2]);

        foreach(Node node in old_graph.nodes)
        {
            if(node.assigned_veichle == car_index)
            {
                graph.nodes[node.i, node.j] = node;

            }
        }
        bool has_merged = false;
        for(int i = 0; i<graph.nodes.GetLength(0)-1; i++)
        {
            //has_merged = false;

            for (int j = 0; j < graph.nodes.GetLength(1)-1; j++)
            {
                //has_merged = false;
                Node node = graph.nodes[i, j];
                if (node != null && !node.is_supernode && node.walkable)
                {
                    if(graph.nodes[i + 1, j]!=null && graph.nodes[i, j + 1] != null && graph.nodes[i + 1, j + 1] != null &&
                        graph.nodes[i+1, j].walkable && graph.nodes[i, j+1].walkable && graph.nodes[i + 1, j+1].walkable &&
                        !(graph.nodes[i + 1, j].is_supernode && graph.nodes[i, j + 1].is_supernode && graph.nodes[i + 1, j + 1].is_supernode))
                    {
                        //MERGING NODES
                        node.is_supernode = true;
                        node.worldPosition = new Vector3(node.worldPosition.x + graph.x_unit / 2, 0, node.worldPosition.z + graph.z_unit / 2);
                        node.merged_nodes = new List<Node>();
                        node.merged_nodes.Add(node);
                        node.merged_nodes.Add(old_graph.nodes[i + 1, j]);
                        node.merged_nodes.Add(old_graph.nodes[i, j + 1]);
                        node.merged_nodes.Add(old_graph.nodes[i + 1, j + 1]);
                        graph.nodes[i + 1, j] = node;
                        graph.nodes[i, j + 1] = node;
                        graph.nodes[i + 1, j + 1] = node;
                        old_graph.nodes[i + 1, j] = node;
                        old_graph.nodes[i, j + 1] = node;
                        old_graph.nodes[i + 1, j + 1] = node;
                        node.neighbours = null;
                        //has_merged = true;
                        


                        /*
                        graph.nodes[i + 1, j].is_supernode = true;
                        graph.nodes[i, j + 1].is_supernode = true;
                        graph.nodes[i + 1, j + 1].is_supernode = true;
                        graph.nodes[i + 1, j].worldPosition = node.worldPosition;
                        graph.nodes[i, j + 1].worldPosition = node.worldPosition;
                        graph.nodes[i + 1, j + 1].worldPosition = node.worldPosition;
                        */

                    }
                    node.neighbours = null; //ADDED TODO REMOVE


                }

                if (has_merged)
                    j++;
            }
            if (has_merged)
                i++;
        }

        foreach (Node node in graph.nodes)
        {
            if (node != null)
            {
                if (node.neighbours == null)
                {
                    node.neighbours = graph.getNeighbours(node, check_vehicle: true, supernodes:node.is_supernode);
                }
            }
        }

        foreach(Node n in graph.nodes)
        {
            if(n!=null && n.is_supernode && n.assigned_veichle == car_index)
            {
                graph.nodes[n.i + 1, n.j] = null;
                graph.nodes[n.i, n.j + 1] = null;
                graph.nodes[n.i + 1, n.j + 1] = null;
                old_graph.nodes[n.i + 1, n.j] = null;
                old_graph.nodes[n.i, n.j + 1] = null;
                old_graph.nodes[n.i + 1, n.j + 1] = null;
            }

            if (n != null)
            {
                for (int k=0; k < n.neighbours.Count; k++)
                {
                    Node neigh = n.neighbours[k];
                    if (neigh != null && !neigh.neighbours.Contains(n))
                    {
                        Debug.Log("REMOVING NODE [" + neigh.i + "," + neigh.j + "FROM NODE [" + n.i + "," + n.j);
                        n.neighbours.Remove(neigh);
                    }
                }
            }
        }

        return graph;
    }

    public static Graph MergeSubGraph(Graph graph, int car_index, List<Node> VertexList)
    {
        foreach (Node node in VertexList)
        {
            if (node.assigned_veichle == car_index)
            {
                graph.nodes[node.i, node.j] = node;

            }
        }

        foreach (Node node in graph.nodes)
        {
            if (node != null)
            {
                node.neighbours = graph.getNeighbours(node, check_vehicle: true);
            }
        }

        return graph;
    }





}