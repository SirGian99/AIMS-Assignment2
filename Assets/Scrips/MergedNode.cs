using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class MergedNode
{
    //Down-left index of the merged node
    public int dl_i;
    public int dl_j;
    //Up-right index of the merged node
    public int ur_i;
    public int ur_j;

    public float x_unit;
    public float z_unit;
    public float x_low;
    public float z_low;

    public bool seen = false;

    public Vector3 position
    {
        get
        {
            return new Vector3(x_low +(dl_i + x_len/2f) * x_unit, 0, z_low + (dl_j + z_len/2f) * z_unit);
        }
    }

    public Vector3[,] vertices
    {
        get
        {
            Vector3[,] vert = new Vector3[2, 2];
            vert[0, 0] = new Vector3(position.x - (x_len / 2f) * x_unit, 0, position.z - (z_len / 2f) * z_unit);
            vert[0, 1] = new Vector3(position.x - (x_len / 2f) * x_unit, 0, position.z + (z_len / 2f) * z_unit);
            vert[1, 0] = new Vector3(position.x + (x_len / 2f) * x_unit, 0, position.z - (z_len / 2f) * z_unit);
            vert[1, 1] = new Vector3(position.x + (x_len / 2f) * x_unit, 0, position.z + (z_len / 2f) * z_unit);
            return vert;
        }
    }

    public int x_len
    {
        get
        {
            return ur_i - dl_i + 1;
        }
    }

    public int z_len
    {
        get
        {
            return ur_j - dl_j + 1;
        }
    }

    public List<MergedNode> neighbours = new List<MergedNode>();

    public MergedNode(int dl_i, int dl_j, int ur_i, int ur_j, float x_unit, float z_unit, float x_low, float z_low)
    {
        this.dl_i = dl_i;
        this.dl_j = dl_j;
        this.ur_i = ur_i;
        this.ur_j = ur_j;
        this.x_unit = x_unit;
        this.z_unit = z_unit;
        this.x_low = x_low;
        this.z_low = z_low;
    }

    public void seeNeigh(Vector3 position)
    {
        foreach(MergedNode neigh in neighbours)
        {
            if (!neigh.seen)
            {
                bool seen = true;
                foreach(Vector3 vertex in neigh.vertices)
                {
                    if (seen)
                    {
                        RaycastHit hit;
                        Vector3 direction = vertex - position;
                        float distance = direction.magnitude * 0.98f;
                        seen = seen && (!Physics.Raycast(position, direction.normalized, out hit, distance) || hit.collider.gameObject.name != "Cube");
                    }
                }
                neigh.seen = seen;
                ///TODO DEBUG
                if (seen)
                {
                    Debug.Log("Node" + neigh + "visible from position" + position + "in node " + this);

                }
                else
                {
                    Debug.Log("Node" + neigh + "not visible from position" + position + "in node " + this);
                }
            }
        }
    }

    public MergedNode getClosestUnseen(Vector3 position)
    {
        seeNeigh(position);
        MergedNode closest = this;
        float min_distace = float.MaxValue;
        foreach(MergedNode neigh in neighbours)
        {
            if (!neigh.seen)
            {
                float distance = (neigh.position - position).magnitude;
                if (distance < min_distace)
                {
                    min_distace = distance;
                    closest = neigh;
                }
            }
        }
        if(closest == this && neighbours.Count > 0)
        {//CAN HAPPEN THAT ALL OF THEM ARE VISITED AND STILL THE MAP IS NOT COMPLETE
            foreach(MergedNode starting_neigh in neighbours)
                foreach (MergedNode neigh in starting_neigh.neighbours)
                {
                    if (!neigh.seen)
                    {
                        float distance = (neigh.position - position).magnitude;
                        if (distance < min_distace)
                        {
                            min_distace = distance;
                            closest = neigh;
                        }
                    }
                }
        }
        
        return closest;
    }

    public Vector3 getClosestPoint(Vector3 position)
    {
        Vector3 closest = this.position;
        float min_distance = (position - this.position).magnitude;
        foreach(Vector3 vertex in vertices)
        {
            Vector3 moved_vertex = Vector3.MoveTowards(vertex, this.position, (vertex - this.position).magnitude * 0.15f);
            float distance = (moved_vertex - position).magnitude*1.4f;
            if (distance < min_distance)
            {
                min_distance = distance;
                closest = moved_vertex;
            }
        }
        return closest;
    }
    public override string ToString()
    {

        return "[(" + dl_i + "," + dl_j +");("+ur_i+"," +ur_j+ ")]";
    }
}

public class MergedGraph
{
    //at each position we find the merged node which covers that position in the original graph
    //if there is an obstacle, there we have nulls
    public MergedNode[,] nodes;
    public int i_size { get { return nodes.GetLength(0); } }
    public int j_size { get { return nodes.GetLength(1); } }

    public MergedGraph(int i, int j)
    {
        this.nodes = new MergedNode[i, j];
    }

    public static MergedGraph fromGraph(Graph graph)
    {
        MergedGraph merged_graph = new MergedGraph(graph.i_size, graph.j_size);

        foreach(Node n in graph.nodes)
        {
            if (n != null)
            {
                MergedNode new_node;
                if (n.is_supernode)
                {
                    new_node = new MergedNode(n.i, n.j, n.i + 1, n.j + 1, graph.x_unit, graph.z_unit, graph.x_low, graph.z_low);
                }
                else{
                    new_node = new MergedNode(n.i, n.j, n.i, n.j, graph.x_unit, graph.z_unit, graph.x_low, graph.z_low);
                }

                for(int i = new_node.dl_i; i <= new_node.ur_i; i++)
                {
                    for (int j = new_node.dl_j; j <= new_node.ur_j; j++)
                    {
                        merged_graph.nodes[i, j] = new_node;
                    }
                }

            }
        }

        return merged_graph;
    }
    public bool mergeNodes()
    {
        bool has_merged = false;

        //merging two subsequent nodes horizontally
        for (int j = 0; j < j_size; j++)
        {
            for (int i = 0; i < i_size; i++)
            {
                //prendi il nodo corrente. Basati sulla sua lunghezza e prendi il successivo sommando alla i la sua lunghezza. Controlla che matchino le altezze. nel caso, mergia
                //controlla che siano allineati
                MergedNode currentNode = nodes[i, j];
                if (currentNode != null && currentNode.dl_i == i && currentNode.dl_j == j)
                {
                    MergedNode next_node = nodes[i + currentNode.x_len, j];
                    //we check the two nodes are correctly alligned.
                    if (next_node != null && next_node.dl_j == j && currentNode.z_len == next_node.z_len)
                    {
                        MergedNode mergedNode = new MergedNode(currentNode.dl_i, currentNode.dl_j, next_node.ur_i, next_node.ur_j, currentNode.x_unit, currentNode.z_unit,currentNode.x_low, currentNode.z_low);
                        has_merged = true;

                        //update the original graph
                        for (int ii = mergedNode.dl_i; ii <= mergedNode.ur_i; ii++)
                        {
                            for (int jj = mergedNode.dl_j; jj <= mergedNode.ur_j; jj++)
                            {
                                nodes[ii, jj] = mergedNode;
                            }
                        }
                        i = mergedNode.ur_i;
                    }
                }
            }
        }



        //merging two subsequent nodes vertically
        for (int i = 0; i < i_size; i++)
        {
            for (int j = 0; j < j_size; j++)
            {
                //prendi il nodo corrente. Basati sulla sua altezza e prendi il successivo sommando alla j la sua altezza. Controlla che matchino le lunghezze. nel caso, mergia
                //controlla che siano allineati
                MergedNode currentNode = nodes[i, j];
                if (currentNode != null && currentNode.dl_i == i && currentNode.dl_j == j)
                {
                    MergedNode next_node = nodes[i, j + currentNode.z_len];
                    //we check the two nodes are correctly alligned.
                    if (next_node != null && next_node.dl_i == i && currentNode.x_len == next_node.x_len)
                    {
                        MergedNode mergedNode = new MergedNode(currentNode.dl_i, currentNode.dl_j, next_node.ur_i, next_node.ur_j, currentNode.x_unit, currentNode.z_unit, currentNode.x_low, currentNode.z_low);
                        has_merged = true;

                        //update the original graph
                        for (int ii = mergedNode.dl_i; ii <= mergedNode.ur_i; ii++)
                        {
                            for (int jj = mergedNode.dl_j; jj <= mergedNode.ur_j; jj++)
                            {
                                nodes[ii, jj] = mergedNode;
                            }
                        }
                        i = mergedNode.ur_i;
                    }
                }
            }
        }

        return has_merged;
    }

    public bool mergeNodesHorizontally()
    {
        bool has_merged = false;

        //merging two subsequent nodes horizontally
        for (int j = 0; j < j_size; j++)
        {
            for (int i = 0; i < i_size; i++)
            {
                //prendi il nodo corrente. Basati sulla sua lunghezza e prendi il successivo sommando alla i la sua lunghezza. Controlla che matchino le altezze. nel caso, mergia
                //controlla che siano allineati
                MergedNode currentNode = nodes[i, j];
                if (currentNode != null && currentNode.dl_i == i && currentNode.dl_j == j)
                {
                    MergedNode next_node = nodes[i + currentNode.x_len, j];
                    //we check the two nodes are correctly alligned.
                    if (next_node != null && next_node.dl_j == j && currentNode.z_len == next_node.z_len)
                    {
                        MergedNode mergedNode = new MergedNode(currentNode.dl_i, currentNode.dl_j, next_node.ur_i, next_node.ur_j, currentNode.x_unit, currentNode.z_unit, currentNode.x_low, currentNode.z_low);
                        has_merged = true;

                        //update the original graph
                        for (int ii = mergedNode.dl_i; ii <= mergedNode.ur_i; ii++)
                        {
                            for (int jj = mergedNode.dl_j; jj <= mergedNode.ur_j; jj++)
                            {
                                nodes[ii, jj] = mergedNode;
                            }
                        }
                        i = mergedNode.ur_i;
                    }
                }
            }
        }

        return has_merged;
    }

    public bool mergeNodesVertically()
    {
        bool has_merged = false;

        //merging two subsequent nodes vertically
        for (int i = 0; i < i_size; i++)
        {
            for (int j = 0; j < j_size; j++)
            {
                //prendi il nodo corrente. Basati sulla sua altezza e prendi il successivo sommando alla j la sua altezza. Controlla che matchino le lunghezze. nel caso, mergia
                //controlla che siano allineati
                MergedNode currentNode = nodes[i, j];
                if (currentNode != null && currentNode.dl_i == i && currentNode.dl_j == j)
                {
                    MergedNode next_node = nodes[i, j + currentNode.z_len];
                    //we check the two nodes are correctly alligned.
                    if (next_node != null && next_node.dl_i == i && currentNode.x_len == next_node.x_len)
                    {
                        MergedNode mergedNode = new MergedNode(currentNode.dl_i, currentNode.dl_j, next_node.ur_i, next_node.ur_j, currentNode.x_unit, currentNode.z_unit, currentNode.x_low, currentNode.z_low);
                        has_merged = true;

                        //update the original graph
                        for (int ii = mergedNode.dl_i; ii <= mergedNode.ur_i; ii++)
                        {
                            for (int jj = mergedNode.dl_j; jj <= mergedNode.ur_j; jj++)
                            {
                                nodes[ii, jj] = mergedNode;
                            }
                        }
                        i = mergedNode.ur_i;
                    }
                }
            }
        }

        return has_merged;
    }

    public void compute_neighbours()
    {
        foreach(MergedNode node in nodes)
        {
            if(node != null)
            {
                if(node.neighbours.Count == 0)
                {
                    for(int i = Mathf.Max(0, node.dl_i-1); i<=Mathf.Min(node.ur_i+1,nodes.GetLength(0)-1); i++)
                    {
                        for (int j = Mathf.Max(0, node.dl_j - 1); j <= Mathf.Min(node.ur_j + 1, nodes.GetLength(1)); j++)
                        {
                            if ((i == node.dl_i - 1 || i == node.ur_i + 1) && (j == node.dl_j - 1 || j == node.ur_j + 1))
                                continue;//skip diagonals
                            MergedNode neigh = nodes[i, j];
                            if (neigh != null && neigh != node && !node.neighbours.Contains(neigh))
                                node.neighbours.Add(neigh);
                        }

                    }
                }
            }
        }

    }

    public MergedNode get_closest_unseen_globally(MergedNode node)
    {
        for(int k = 1; k<Mathf.Max(i_size, j_size) ; k++)
        {
            for (int i = Mathf.Max(0, node.dl_i - k); i <= Mathf.Min(node.ur_i + k, nodes.GetLength(0)-1); i++)
            {
                for (int j = Mathf.Max(0, node.dl_j - k); j <= Mathf.Min(node.ur_j + k, nodes.GetLength(1)-1); j++)
                {
                    MergedNode neigh = nodes[i, j];
                    if (neigh != null && neigh != node && !neigh.seen)
                         return neigh;
                }

            }

        }
        return null;

    }
}