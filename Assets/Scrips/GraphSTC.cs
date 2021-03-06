using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public struct Edge
{
    public Node Source;
    public Node Destination;
    public float Weight;
}

public class GraphSTC
{
    public Vector3 start_pos;
    public Node starting_node;

    // STC variables
    public int VerticesCount;
    public int EdgesCount;
    public List<Edge> EdgeList;
    public Node[] VertexArray;
    public Graph original_graph;
    public float[,] adj_matrix;

    public GraphSTC(Graph graph, Vector3 start_pos)
    {
        this.start_pos = start_pos;
        original_graph = graph;
        this.starting_node = graph.getNodeFromPoint(start_pos);
        this.EdgeList = new List<Edge>();

        //Create EdgesList and VertexArray
        List<Node> VertexList = new List<Node>();
        Edge newEdge = new Edge();
        foreach (Node node in graph.nodes)
        {
            if (node != null)
            {
                foreach (Node neighbour in node.neighbours)
                {
                    if (node.walkable == true && neighbour.walkable == true)
                    {
                        newEdge.Source = node;
                        newEdge.Destination = neighbour;
                        newEdge.Weight = ManhattenDistance(node.worldPosition, neighbour.worldPosition);
                        if (Mathf.Abs(node.i - neighbour.i) + Mathf.Abs(node.j - neighbour.j) >= 1.9f && Mathf.Abs(node.i - neighbour.i) + Mathf.Abs(node.j - neighbour.j) <= 2.5f)
                        {
                            newEdge.Weight *= 100;
                            Debug.Log("QUI Current node: " + node.worldPosition + "neigh node: " + neighbour.worldPosition);
                        }else if (newEdge.Source.worldPosition.x != newEdge.Destination.worldPosition.x && newEdge.Source.worldPosition.z != newEdge.Destination.worldPosition.z)
                        {
                            newEdge.Weight *= 50;
                        }
                        if(newEdge.Source.parent != null && ( (newEdge.Destination.worldPosition.x == newEdge.Source.parent.worldPosition.x) || (newEdge.Destination.worldPosition.z == newEdge.Source.parent.worldPosition.z)))
                        {
                            newEdge.Weight *= 100; //means you are zigzagging, we don't want that. Nevers enters here
                        }
                        if(newEdge.Source.is_supernode && newEdge.Destination.is_supernode)
                        {
                            newEdge.Weight /= 50;
                        }
                        this.EdgeList.Add(newEdge);
                    }
                }
                if (node.walkable == true)
                {
                    VertexList.Add(node);
                }

            }
        }
        this.EdgesCount = this.EdgeList.Count;
        this.VerticesCount = VertexList.Count;
        this.VertexArray = new Node[this.VerticesCount];
        for (int v = 0; v < VertexList.Count; v++)
        {
            this.VertexArray[v] = VertexList[v];
        }

        //Create adjency matrix
        this.adj_matrix = new float[this.VerticesCount, this.VerticesCount];

        for(int i = 0; i<VerticesCount; i++)
        {
            for(int j = 0; j<VerticesCount; j++)
            {
                adj_matrix[i, j] = -1;
            }
        }


        foreach(Edge edge in EdgeList)
        {
            int u = System.Array.IndexOf(this.VertexArray, edge.Source);
            int v = System.Array.IndexOf(this.VertexArray, edge.Destination);
            this.adj_matrix[u, v] = edge.Weight;
            if (this.adj_matrix[v, u] == -1)
                this.adj_matrix[v, u] = edge.Weight;

        }
    }

    public float ManhattenDistance(Vector3 a, Vector3 b)
    {
        float dist = Mathf.Abs(a.x - b.x) + Mathf.Abs(a.y - b.y) + Mathf.Abs(a.z - b.z);
        return dist;
    }

    public Node get_closest_node(Vector3 position)
    {
        Node closest = VertexArray[0];
        bool found = false;
        while (!found)
        {
            foreach (Node node in closest.neighbours)
            {
                if (Vector3.Distance(position, node.worldPosition) < Vector3.Distance(position, closest.worldPosition))
                {
                    closest = node;
                    break;
                }
                found = true;
            }
        }
        return closest;
    }
}