using System.Collections;
using System.Collections.Generic;
using UnityEngine;

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


    public DARP_controller(int n_agents, Vector3[] initial_positions, Graph graph, float update_rate, float update_tolerance)
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
        //naive_assignment(12, initial_positions[0]);
        greedy_assignment();
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

    public void naive_assignment(float input_angle = 0, Vector3? initial_position = null)
    {
        if (!initial_position.HasValue)
            initial_position = graph.centre;
        for (int i = 0; i < evaluation_matrix.GetLength(1); i++)
        {
            for (int j = 0; j < evaluation_matrix.GetLength(2); j++)
            {
                Node current = graph.nodes[i, j];
                if (current.walkable)
                {
                    Vector3 direction = (current.worldPosition - graph.centre).normalized;
                    if (direction.y != 0)
                        direction = (new Vector3(current.worldPosition.x - initial_position.Value.x, 0, current.worldPosition.z - initial_position.Value.z)).normalized;

                    float angle = get_angle(direction.x, direction.z) + (input_angle * Mathf.PI / 180f);
                    // cap within 0-180
                    while (angle < 0)
                        angle = 2 * Mathf.PI + angle;
                    while (angle > 2 * Mathf.PI)
                        angle = angle - 2 * Mathf.PI;

                    //Debug.Log("Node: [" + i + "," + j + "] direction: " + direction + " angle: " + (angle * 180 / Mathf.PI) + "assigned to " + (int)Mathf.Floor(angle * n_agents / (2 * Mathf.PI)));
                    assignment_matrix[i, j] = 1 + (int)Mathf.Floor(angle * n_agents / (2 * Mathf.PI));
                }

            }
        }
    }

    public void greedy_assignment()
    {
        List<Node> start_nodes = new List<Node>();
        int n = 0, top_shift = 0, bottom_shift=0, WalkableNodes = graph.walkable_nodes;
        while (n < n_agents)
        {
            int i, j;
            if (n == 0 && n_agents % 2 != 0)
            {
                i = graph.i_size / 2;
                j = graph.j_size / 2; // midpoint
                Debug.Log("0N " + n + " : " + i + " " + j);
            }
            else if(n%2==0)
            {
                i = graph.i_size - top_shift - 1;
                j = graph.j_size/2; // top
                Debug.Log("tN " + n + " : " + i + " " + j);
                top_shift++;
            }
            else
            {
                i = bottom_shift;
                j = graph.j_size / 2; // bottom
                Debug.Log("bN " + n + " : " + i + " " + j);
                bottom_shift++;
            }
            Node current = graph.nodes[i, j];
            if (current.walkable)
            {
                start_nodes.Add(current);
                assignment_matrix[i, j] = n + 1;
                n++;
                WalkableNodes--;
            }
        }
        /*
        while (WalkableNodes != 0)
        {
            foreach (Node node in graph.nodes)
            {
                for (int m = n_agents-1; m >=0; m--)
                {
                    if (node.walkable && assignment_matrix[node.i, node.j] == m + 1)
                    {
                        foreach (Node neighbour in node.neighbours)
                        {
                            if (node.walkable && neighbour.walkable
                            && assignment_matrix[node.i, node.j] == m + 1
                            && assignment_matrix[neighbour.i, neighbour.j] == 0)
                            {
                                assignment_matrix[neighbour.i, neighbour.j] = m + 1;
                                WalkableNodes--;
                            }
                        }
                        break;
                    }
                }
            }
        }
        */
        while (WalkableNodes != 0)
        {
            foreach (Node node in graph.nodes)
            {
                foreach (Node neighbour in node.neighbours)
                {
                    for (int m = n_agents - 1; m >= 0; m--)
                    {
                        if (node.walkable && neighbour.walkable
                            && assignment_matrix[node.i, node.j] == m + 1
                            && assignment_matrix[neighbour.i, neighbour.j] == 0)
                        {
                            assignment_matrix[neighbour.i, neighbour.j] = m + 1;
                            WalkableNodes--;
                            break;
                        }
                    }
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

    public void update_assigned_nodes(Graph graph)
    {
        foreach(Node n in graph.nodes)
        {
            if (n != null)
            {
                n.assigned_veichle = assignment_matrix[n.i, n.j];
            }
        }
    }
}
