using System.Collections;
using System.Collections.Generic;
using UnityEngine;


// Data Stuture: Subset for kurskal's
public struct Subset
{
    public Node Parent { get; set; }
    public int Rank { get; set; }
}




public class DARP_controller
{
    public int n_agents;
    public Vector3[] initial_positions;
    public float[,,] evaluation_matrix;
    public int[,] assignment_matrix;
    public int[] assigned_cells;
    public float[] m;
    public Graph graph;
    public float fair_share;
    public int[] components;
    public float  h1, h2, best_h, best_h1;
    public int n_iter, subcomponents;
    public List<List<Vector3>> all_paths;
    public GraphSTC[] subgraphs;
    public float split_angle;



    public DARP_controller(TerrainInfo terrainInfo, int n_agents, Vector3[] initial_positions, Graph graph, float update_rate, float update_tolerance)
    {
        this.n_agents = n_agents;
        this.initial_positions = initial_positions;
        this.graph = graph;
        evaluation_matrix = new float[n_agents, graph.i_size, graph.j_size];
        assignment_matrix = new int[graph.i_size, graph.j_size];
        m = new float[n_agents];
        assigned_cells = new int[n_agents];
        fair_share = graph.walkable_nodes / n_agents;
        //create_evaluation(initial_positions);
        //update_assignment();
        //update_evaluation(update_rate, update_tolerance);

        this.components = new int[n_agents];
        this.h1 = 1f;
        this.h2 = 0.5f;
        this.best_h = 10f * n_agents;
        this.best_h1 = n_agents;
        this.n_iter = 10;
        this.split_angle = 0f;

        // Find ideal graph
        for (int i = 0; i < n_iter; i++)
        {
            // 1. assign to sub-graphs
            naive_assignment(split_angle); 
           
            int x1 = 0;
            int x2 = 0;
            float x = 0;
            for (int agent = 1; agent <= n_agents; agent++)
            {
                // 2. create sub-graphs 
                Graph sub = Graph.CreateSubGraph(graph, agent, terrainInfo);
                GraphSTC subSTC = new GraphSTC(sub, initial_positions[agent - 1]);

                // 3. heuristic: Sub-graphs has minimum disconnected components
                x1 = x1 + GraphComponents(subSTC);

                // 4. heuristic: minimum distance to grid from agents starting point
                x2 = x2 + 1; //MUST EDIT!!!
            }

            x = h1 * x1 + h2 * x2;
            // 5. Heuristic score evalulate
            if(x < best_h)//MUST EDIT!!!
            {
                best_h = x;
                subcomponents = x1;
                // store final assignments - //EDIT AFTER QUERY

                //store final commponent count
                for (int agent = 1; agent <= n_agents; agent++)
                {
                    Graph sub = Graph.CreateSubGraph(graph, agent, terrainInfo);
                    GraphSTC subSTC = new GraphSTC(sub, initial_positions[agent - 1]);
                    components[agent -1] = GraphComponents(subSTC);

                }
            }
            split_angle = split_angle + 10f;
        }

        

        //Merge and Split graphs if necessary
        while (subcomponents > n_agents)
        {
            //check all graphs
            for (int agent = 1; agent <= n_agents; agent++)
            {
                Graph sub = Graph.CreateSubGraph(graph, agent, terrainInfo);
                GraphSTC subSTC = new GraphSTC(sub, initial_positions[agent - 1]);

                //graph is disconnected
                if (GraphComponents(subSTC)!=1)
                {
                    //Get sub-graph
                    List<Node> VertexList = SpiltGraph(subSTC, true);

                    //Search neighbour index
                    int new_index = 0; //MUST EDIT!!!

                    //Re-assign indices
                    foreach(Node node in VertexList)
                    {
                        if(node != null)
                        {
                            assignment_matrix[node.i, node.j] = new_index;
                            node.assigned_veichle = new_index;
                        }
                    }
                    //
                    subcomponents = subcomponents - 1;
                }

            }
        }

        //Get paths for all agents
        for (int agent = 1; agent <= n_agents; agent++)
        {
            Graph sub = Graph.CreateSubGraph(graph, agent, terrainInfo);
            GraphSTC subSTC = new GraphSTC(sub, initial_positions[agent - 1]);
            subgraphs[agent - 1] = subSTC;
            Edge[] MinSTC = STC(subSTC);
            List<Vector3> path = new List<Vector3>();
            path = ComputePath(subSTC, MinSTC, subSTC.start_pos);
            all_paths[agent - 1] = path;
        }

        smooth_areas();
    }

    public void create_evaluation(Vector3[] positions)
    {
        if (positions.Length != n_agents)
        {
            Debug.LogError("Number of agent positions different from the number of agents");
        }
        for (int k = 0; k < positions.Length; k++)
        {
            m[k] = 1;
            //updating the evaluation matrix for agent k
            positions[k].y = 0;

            for (int i = 0; i < evaluation_matrix.GetLength(1); i++)
            {
                for (int j = 0; j < evaluation_matrix.GetLength(2); j++)
                {

                    Node current_node = graph.nodes[i, j];
                    //Debug.Log("the current node is [" + i + "," + j  + "] is walkable: " + current_node.walkable);
                    if (current_node.walkable)
                    {
                        //OLD HEURISTIC evaluation_matrix[k, i, j] = Vector3.Distance(positions[k], current_node.worldPosition);
                        //NEW: Manhattan
                        float multiplier = 1f;
                        bool hit = Physics.Raycast(positions[k], (current_node.worldPosition - positions[k]).normalized, Vector3.Distance(positions[k], current_node.worldPosition));
                        if (hit)
                        {
                            Debug.DrawLine(positions[k], current_node.worldPosition);
                            multiplier = 1.5f;
                        }
                        evaluation_matrix[k, i, j] = Mathf.Abs(positions[k].x - current_node.x_pos) + Mathf.Abs(positions[k].z - current_node.z_pos) * multiplier;
                    }
                    else
                        evaluation_matrix[k, i, j] = -1;
                }
            }
        }
    }

    public void update_assignment()
    {
        //Debug.Log("Agents = " + n_agents + "Eval mat dim : (" + evaluation_matrix.GetLength(1) + "," + evaluation_matrix.GetLength(2));

        assigned_cells = new int[n_agents];
        for (int i = 0; i < evaluation_matrix.GetLength(1); i++)
        {
            for (int j = 0; j < evaluation_matrix.GetLength(2); j++)
            {
                assignment_matrix[i, j] = 0;
                for (int k = 1; k < n_agents; k++)
                {
                    //better handle the number of assigned cells
                    if (evaluation_matrix[k, i, j] != -1)
                    {
                        if (assignment_matrix[i, j] != -1 && evaluation_matrix[k, i, j] < evaluation_matrix[assignment_matrix[i, j], i, j])
                        {
                            assignment_matrix[i, j] = k;
                        }
                    }
                    else
                    {
                        assignment_matrix[i, j] = -1;
                    }
                }
                if (assignment_matrix[i, j] != -1)
                    assigned_cells[assignment_matrix[i, j]]++;
                //Debug.Log("Ass mat [" + i + "," + j + "]: " + assignment_matrix[i, j]);
            }
        }
        for(int k = 0; k<n_agents; k++)
        {
            Debug.Log("Ass to " + k + ": " + assigned_cells[k]);
        }
    }

    public void update_evaluation(float rate, float tolerance, int safe_exit = 1)
    {
        float change = tolerance + 1;
        int exit = 0;
        while (change>tolerance) {
            change = 0;
            for (int k = 0; k < n_agents; k++)
            {
                float old_m = m[k];
                m[k] = m[k] + rate * (assigned_cells[k] - fair_share);
                Debug.Log("New m[" + k + "] = " + m[k]);
                change += (float)Mathf.Abs(m[k] - old_m);
                for (int i = 0; i < evaluation_matrix.GetLength(1); i++)
                {
                    for (int j = 0; j < evaluation_matrix.GetLength(2); j++)
                    {
                        float old_value = evaluation_matrix[k, i, j];
                        if(old_value != -1)
                            evaluation_matrix[k, i, j] *= m[k];
                    }
                }
            }
            exit++;
            update_assignment();
            if (exit >= safe_exit)
            {
                Debug.Log("Safe exit triggered, current change: " + change + " current tolerance: " + tolerance);
                break;
            }
           
        }

    }

    public void naive_assignment(float input_angle, Vector3? initial_position = null)
    {
        if (!initial_position.HasValue)
            initial_position = graph.centre;
        for (int i = 0; i < evaluation_matrix.GetLength(1); i++)
        {
            for (int j = 0; j < evaluation_matrix.GetLength(2); j++)
            {
                Node current = graph.nodes[i, j];
                if (current!=null && current.walkable)
                {
                    Vector3 direction = (current.worldPosition - graph.centre).normalized;
                    if (direction.y != 0)
                        direction = (new Vector3(current.worldPosition.x - initial_position.Value.x, 0, current.worldPosition.z - initial_position.Value.z)).normalized;

                    float angle = get_angle(direction.x, direction.z) + (input_angle * Mathf.PI / 180f);
                    // cap within 0-180
                    while (angle < 0 )
                        angle = 2 * Mathf.PI + angle;
                    while (angle > 2 * Mathf.PI)
                        angle = angle - 2 * Mathf.PI;

                  //Debug.Log("Node: [" + i + "," + j + "] direction: " + direction + " angle: " + (angle * 180 / Mathf.PI) + "assigned to " + (int)Mathf.Floor(angle * n_agents / (2 * Mathf.PI)));
                   assignment_matrix[i, j] = 1 + (int)Mathf.Floor(angle * n_agents / (2 * Mathf.PI));
                }

            }
        }
    }

    public float get_angle(float cos, float sin)
    {
        float acos = Mathf.Acos(cos); //between 0 and 180
        if (sin < 0)
        {
            acos = 2 * Mathf.PI - acos;
        }
        return acos;
    }

    public void smooth_areas()
    {
        for (int i = 0; i < evaluation_matrix.GetLength(1); i++)
        {
            for (int j = 0; j < evaluation_matrix.GetLength(2); j++)
            {
                Node current = graph.nodes[i, j];
                if (current.walkable)
                {
                    int[] neigh_car = new int[n_agents];
                    bool found = false;
                    foreach (Node neigh in current.neighbours)
                    {
                        if (neigh.walkable)
                        {
                            /*if (found)
                                break;
                            if (assignment_matrix[current.i, current.j] == assignment_matrix[neigh.i, neigh.j])
                                found = true;
                            else
                                neigh_car[assignment_matrix[neigh.i, neigh.j] - 1]++;
                            */
                            neigh_car[assignment_matrix[neigh.i, neigh.j] - 1]++;
                        }
                    }

                    if (!found || true)
                    {
                        int highest = 0;
                        for (int k = 0; k < n_agents; k++)
                        {
                            if (neigh_car[k] > neigh_car[highest])
                                highest = k;
                        }
                        assignment_matrix[i, j] = highest + 1;
                        current.assigned_veichle = assignment_matrix[i, j];
                    }

                }
            }
        }
    }

    // Sub-Func: Compute path 
    private List<Vector3> ComputePath(GraphSTC graph, Edge[] EdgeArray, Vector3 starting_position)
    {
        List<Vector3> path = new List<Vector3>();
        Vector3 waypoint;
        Node current_node;
        if (graph.starting_node != null)
        {
            current_node = graph.starting_node;
        }
        else
        {
            current_node = graph.get_closest_node(starting_position);

        }
        path.Add(current_node.worldPosition);
        current_node.visited = true;
        for (int i = 0; i < graph.VerticesCount;)
        {
            Node right = current_node.right_child;
            Node left = current_node.left_child;
            if (right != null && !right.visited)
            {
                path.Add(right.worldPosition);
                right.visited = true;
                current_node = right;
                i++;
            }
            else if (left != null && !left.visited)
            {
                path.Add(left.worldPosition);
                left.visited = true;
                current_node = left;
                i++;
            }
            else if (current_node.parent != null)
            {
                path.Add(current_node.parent.worldPosition);
                current_node = current_node.parent;

            }
            else
            {
                break;
            }
        }
        /*for (int i = 0; i < EdgeArray.Length-3; i++)
        {

            waypoint = EdgeArray[i].Destination.worldPosition;
            //Debug.Log(i+ "waypoint " + waypoint + EdgeArray.Length);
            path.Add(waypoint);

        }
        */


        return path;
    }

    // Sub-Func: Find minimum obstacle free path to subgraph
    private Vector3 FindMinDistancePoint(Graph graph, Vector3 start_pos)
    {
        float minDist = Mathf.Infinity;
        Vector3 start_grid = start_pos;
        foreach (Node node in graph.nodes)
        {
            if (node != null && node.walkable==true)
            {
                //Find path using AStar
                ;//EDIT!!!

                // Find path length
                float path_length = 1f; //EDIT!!!

                //Check if min path and assign it as start point
                if(path_length < minDist)
                {
                    minDist = path_length;
                    start_grid = node.worldPosition;
                }

            }
        }
        return start_grid;
    }

    // Sub-Func: Kurskal Algorithm to find number of compoenets in graph
    public int GraphComponents(GraphSTC graph)
    {
        int verticesCount = graph.VerticesCount;
        Node[] VertexArray = graph.VertexArray;
        int k = 0;
        int e = 0;
        int subset_count = 0;

        // Sort edges by cost- all costs are same
        graph.EdgeList.Sort((e1, e2) => e1.Weight.CompareTo(e2.Weight));

        // Create each vertex as subsets
        Subset[] subsets = new Subset[verticesCount];
        Subset sub;
        for (int v = 0; v < verticesCount; ++v)
        {
            sub = new Subset();
            sub.Parent = VertexArray[v];
            sub.Rank = 0;
            subsets[v] = sub;
        }

        // build min tree
        while (e < verticesCount - 1)
        {
            Edge nextEdge = graph.EdgeList[k];
            Node x = Find(subsets, nextEdge.Source, System.Array.IndexOf(VertexArray, nextEdge.Source), VertexArray);
            Node y = Find(subsets, nextEdge.Destination, System.Array.IndexOf(VertexArray, nextEdge.Destination), VertexArray);

            if (x != y)
            {
                e++;
                //Debug.Log("Edge " + e + " S: " + nextEdge.Source.worldPosition + " to D: " + nextEdge.Destination.worldPosition);
                Union(subsets, x, y, VertexArray);
            }
            k++;

            if (k >= graph.EdgesCount)
            {
                //Debug.Log("Car " + CarNumber + " TotalEdges, k: " + graph.EdgesCount + " " + k + " MinEdges,e"+ (verticesCount-1) + " " + e  + " " );
                break;
            }
        }
        subset_count = verticesCount - e;

        return subset_count;
    }

    // Sub-Func: Kurskal Algorithm to sub-graph
    public List<Node> SpiltGraph(GraphSTC graph, bool smallest)
    {
        int verticesCount = graph.VerticesCount;
        List<Node> result = new List<Node>();
        Node[] VertexArray = graph.VertexArray;
        int k = 0;
        int e = 0;
        int subset_count = 0;

        // Sort edges by cost- all costs are same
        graph.EdgeList.Sort((e1, e2) => e1.Weight.CompareTo(e2.Weight));

        // Create each vertex as subsets
        Subset[] subsets = new Subset[verticesCount];
        Subset sub;
        for (int v = 0; v < verticesCount; ++v)
        {
            sub = new Subset();
            sub.Parent = VertexArray[v];
            sub.Rank = 0;
            subsets[v] = sub;
        }

        // build min tree
        while (e < verticesCount - 1)
        {
            Edge nextEdge = graph.EdgeList[k];
            Node x = Find(subsets, nextEdge.Source, System.Array.IndexOf(VertexArray, nextEdge.Source), VertexArray);
            Node y = Find(subsets, nextEdge.Destination, System.Array.IndexOf(VertexArray, nextEdge.Destination), VertexArray);

            if (x != y)
            {
                e++;
                Union(subsets, x, y, VertexArray);
            }
            k++;

            if (k >= graph.EdgesCount)
            {
                break;
            }
        }
        subset_count = verticesCount - e;


        if (subset_count == 2)
        {
            //CASE: Two Subsets are created
            int set1 = subsets[0].Rank;
            List<Node> Vertex_set1 = new List<Node>();
            List<Node> Vertex_set2 = new List<Node>();
            for (int r = 0; r < subsets.Length; r++)
            {
                if (subsets[r].Rank == set1)
                {
                    Vertex_set1.Add(subsets[r].Parent);
                }
                else
                {
                    Vertex_set2.Add(subsets[r].Parent);
                }
            }

            if (smallest == true)
            {
                result = Vertex_set1.Count < Vertex_set2.Count ? Vertex_set1 : Vertex_set2;
            }
            else
            {
                result = Vertex_set1.Count > Vertex_set2.Count ? Vertex_set1 : Vertex_set2;
            }
        }


        return result;
    }

    // MAIN FUNC: Kurskal Algorithm to find minimum spanning tree
    public Edge[] STC(GraphSTC graph)
    {
        int verticesCount = graph.VerticesCount;
        Edge[] result = new Edge[verticesCount];
        Node[] VertexArray = graph.VertexArray;
        int k = 0;
        int e = 0;

        // Sort edges by cost- all costs are same
        graph.EdgeList.Sort((e1, e2) => e1.Weight.CompareTo(e2.Weight));

        // Create each vertex as subsets
        Subset[] subsets = new Subset[verticesCount];
        Subset sub;
        for (int v = 0; v < verticesCount; ++v)
        {
            sub = new Subset();
            sub.Parent = VertexArray[v];
            sub.Rank = 0;
            subsets[v] = sub;
        }

        // build min tree
        while (e < verticesCount - 1)
        {
            Edge nextEdge = graph.EdgeList[k];
            Node x = Find(subsets, nextEdge.Source, System.Array.IndexOf(VertexArray, nextEdge.Source), VertexArray);
            Node y = Find(subsets, nextEdge.Destination, System.Array.IndexOf(VertexArray, nextEdge.Destination), VertexArray);

            if (x != y)
            {
                result[e++] = nextEdge;
                nextEdge.Destination.parent = nextEdge.Source;
                if (nextEdge.Source.parent == null || nextEdge.Source.parent.i == nextEdge.Destination.i || nextEdge.Source.parent.j == nextEdge.Destination.j)
                {
                    if (nextEdge.Source.right_child != null)
                    {
                        Debug.Log("ERROR RIGHT!!!");
                    }
                    else
                    {
                        nextEdge.Source.right_child = nextEdge.Destination;
                    }
                }
                else
                {
                    if (nextEdge.Source.left_child != null)
                    {
                        Debug.Log("ERROR LEFT!!!");
                    }
                    else
                    {
                        nextEdge.Source.left_child = nextEdge.Destination;
                    }
                }
                //Debug.Log("Edge " + e + " S: " + nextEdge.Source.worldPosition + " to D: " + nextEdge.Destination.worldPosition);
                Union(subsets, x, y, VertexArray);
            }
            k++;
        }

        return result;
    }

    // Sub-Func: Identify parent of node a in Subset for kurskal's
    private Node Find(Subset[] subsets, Node vertex, int k, Node[] vertex_dict)
    {
        if (subsets[k].Parent != vertex)
        {
            subsets[k].Parent = Find(subsets, subsets[k].Parent,
                                System.Array.IndexOf(vertex_dict, subsets[k].Parent), vertex_dict);
        }

        return subsets[k].Parent;
    }

    // Sub-Func: Union of subsets for kurskal's
    private void Union(Subset[] subsets, Node x, Node y, Node[] vertex_dict)
    {
        Node xroot = Find(subsets, x, System.Array.IndexOf(vertex_dict, x), vertex_dict);
        Node yroot = Find(subsets, y, System.Array.IndexOf(vertex_dict, y), vertex_dict);

        if (subsets[System.Array.IndexOf(vertex_dict, xroot)].Rank < subsets[System.Array.IndexOf(vertex_dict, yroot)].Rank)
            subsets[System.Array.IndexOf(vertex_dict, xroot)].Parent = yroot;
        else if (subsets[System.Array.IndexOf(vertex_dict, xroot)].Rank > subsets[System.Array.IndexOf(vertex_dict, yroot)].Rank)
            subsets[System.Array.IndexOf(vertex_dict, yroot)].Parent = xroot;
        else
        {
            subsets[System.Array.IndexOf(vertex_dict, yroot)].Parent = xroot;
            ++subsets[System.Array.IndexOf(vertex_dict, xroot)].Rank;
        }
    }


}
