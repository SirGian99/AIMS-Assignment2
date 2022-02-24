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
    
    // STC variables
    public int VerticesCount;
    public int EdgesCount;
    public List<Edge> EdgeList;
    public Node[] VertexArray;

    public GraphSTC(Graph graph, Vector3 start_pos)
    {
        this.start_pos = start_pos;
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
                        newEdge.Weight = Vector3.Distance(neighbour.worldPosition, start_pos);
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
    }
}
