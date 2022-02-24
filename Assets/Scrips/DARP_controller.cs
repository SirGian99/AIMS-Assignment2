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
        naive_assignment();
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

    public void naive_assignment()
    {
        for (int i = 0; i < evaluation_matrix.GetLength(1); i++)
        {
            for (int j = 0; j < evaluation_matrix.GetLength(2); j++)
            {
                Node current = graph.nodes[i, j];
                if (current.walkable)
                {
                    Vector3 direction = (current.worldPosition - graph.centre).normalized;
                    if (direction.y != 0)
                        direction = (new Vector3(current.worldPosition.x - graph.centre.x, 0, current.worldPosition.z - graph.centre.z)).normalized;

                    float angle = get_angle(direction.x, direction.z);
                    /*if (angle < 0 && direction.z<0)
                        angle = 2 * Mathf.PI + angle;
                    if (direction.x <= 0 && direction.z<=0)
                        angle += Mathf.PI;
                    */
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
}
