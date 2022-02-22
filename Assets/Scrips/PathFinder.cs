using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;

public class PathFinder : MonoBehaviour
{
    public static float[,] headings = new float[,] { { 135f, 90f, 45f }, { 180f, 0f, 0f }, { 225f, 270f, 315f } };

    public static void findPath(Graph graph, Vector3 start_position, Vector3 goal_position, float start_heading, bool drone = false)
    {
        Node start_node = graph.getNodeFromPoint(start_position);
        start_node.heading = start_heading;
        Node goal_node = graph.getNodeFromPoint(goal_position);
        Debug.Log("Start i: " + start_node.i + "Start j:" + start_node.j);
        Debug.Log("Goal i: " + goal_node.i + "Goal j:" + goal_node.j);

        List<Node> open_set = new List<Node>();
        HashSet<Node> closed_set = new HashSet<Node>();
        open_set.Add(start_node);
        bool k = true;

        while (open_set.Count > 0)
        {

            Node current = open_set[0];
            if (k)
            {
                Debug.Log("Nodo fuori penalty: " + current.wallClosenessCost);
                Debug.Log("Totale: " + current.fCost);
            }
            for (int i=1; i<open_set.Count; i++)
            {
                if(open_set[i].fCost < current.fCost || open_set[i].fCost == current.fCost && open_set[i].hCost < current.hCost)
                {
                    current = open_set[i];
                }
            }
            open_set.Remove(current);
            closed_set.Add(current);
            if (current == goal_node)
            {
                List<Node> path = new List<Node>();
                Node previous_node = current;

                while (previous_node != start_node)
                {
                    //Debug.Log("Coordinates (" + previous_node.i + "," + previous_node.j + ")");

                    if(previous_node.parent != null)
                    {
                        Node back_1 = previous_node.parent;
                        if(back_1.parent!= null)
                        {
                            Node back_2 = back_1.parent;

                            if(back_2.parent != null)
                            {
                                Node back_3 = back_2.parent;
                                if (Math.Abs(Math.Abs(previous_node.x_pos - back_3.x_pos) - 3 * graph.x_unit)< 0.01f && previous_node.z_pos == back_3.z_pos &&  // |--|
                                    Math.Abs(Math.Abs(previous_node.z_pos - back_1.z_pos) - graph.z_unit) < 0.01f && // 
                                    Math.Abs(Math.Abs(back_1.x_pos - back_2.x_pos) - graph.x_unit) < 0.01f && back_1.z_pos == back_2.z_pos
                                    )
                                {
                                    String s = String.Format("Amazing. Node1:[{0},{1}] Node2:[{2},{3}] Node3[{4},{5}] Node4[{6},{7}]",
                                        previous_node.i, previous_node.j, back_1.i, back_1.j, back_2.i, back_2.j, back_3.i, back_3.j);
                                    //TODO COMMENT THIS!!!
                                    Debug.Log(s);
                                    Node first_interpol = graph.nodes[previous_node.i, back_1.j];
                                    first_interpol.parent = back_1;
                                    previous_node.parent = first_interpol;
                                    Node second_interpol = graph.nodes[back_3.i, back_2.j];
                                    second_interpol.parent = back_3;
                                    back_2.parent = second_interpol;
                                    
                                    
                                }
                                
                                /*
                                if (previous_node.z_pos == back_3.z_pos) {
                                    Debug.Log("Amazing 0 Distanza: " + Math.Abs(previous_node.x_pos - back_3.x_pos) + " 3*x_unit = " + 3 * graph.x_unit);
                                    if(Math.Abs(previous_node.x_pos - back_3.x_pos) - 3 * graph.x_unit < 0.01f) //due to float precision
                                    {
                                        Debug.Log("Arrivo qui wow amazing 1");
                                        if (Math.Abs(previous_node.z_pos - back_1.z_pos) - graph.z_unit < 0.01f)
                                        {
                                            Debug.Log("Amazing 2");
                                        }

                                        if (Math.Abs(back_1.x_pos - back_2.x_pos) - graph.x_unit <0.01f && back_1.z_pos == back_2.z_pos)
                                        {
                                            Debug.Log("Amazing 3");
                                        }
                                    }
                                }
                                */
                                    /*if(Math.Abs(previous_node.x_pos - back_3.x_pos) <  graph.z_unit && previous_node.z_pos == back_3.z_pos && previous_node.x_pos != back_1.x_pos && back_1.z_pos == back_2.z_pos) //Curva a U o U capovolta
                                    {
                                        Debug.Log("Arrivo qui wow amazing");
                                        /*Node first_interpol = graph.nodes[back_1.i, previous_node.j];
                                        first_interpol.parent = back_1;
                                        previous_node.parent = first_interpol;
                                        Node second_interpol = graph.nodes[back_2.i, back_3.j];
                                        second_interpol.parent = back_3;
                                        back_2.parent = second_interpol;

                                    }
                                    */


                                }
                            }

                    }
                    

                    path.Add(previous_node);
                    previous_node = previous_node.parent;
                }
                path.Add(previous_node);
                path.Reverse();
                graph.path = path;
                /*foreach(Node n in graph.path)
                {
                    //Debug.Log("Path, nodo (" + previous_node.i + "," + previous_node.j + ")");
                }*/
            }

            foreach(Node neighbour in current.neighbours)
            {
                if (!neighbour.walkable || closed_set.Contains(neighbour))
                    continue;
                if(neighbour.i != current.i && neighbour.j != current.j )
                {
                    if (current.i < graph.nodes.GetLength(0) && current.j < graph.nodes.GetLength(1) && neighbour.i < graph.nodes.GetLength(0) && neighbour.j < graph.nodes.GetLength(1))
                        if (!(graph.nodes[current.i, neighbour.j].walkable && graph.nodes[neighbour.i, current.j].walkable))
                            
                            continue;
                }
                float costToNeighb = current.gCost + getDistance(graph, current, neighbour);
                float neigh_heading = headings[-neighbour.j + current.j + 1, neighbour.i - current.i + 1];
                float additional_cost = Math.Abs(current.heading - neigh_heading) / 22.5f;

                if (drone)
                {
                    if ((int)neigh_heading / 45 % 2 == 1)
                    {
                        costToNeighb += 100;
                    }
                    if(current.heading != neigh_heading)
                    {
                        costToNeighb *= 2;
                    }
                    if (neighbour.wallClosenessCost > 0)
                    {

                        additional_cost += 1000;
                    }
                }
                if (current.parent != null && current.parent.parent != null && Math.Abs(current.parent.parent.heading - neigh_heading) >=90)
                {
                    additional_cost *= 1.5f;
                }
                if(additional_cost != 0)
                {
                    Debug.Log("Current h: " + current.heading + " Next h:" + neigh_heading + "Penalty: " + additional_cost);
                }
                additional_cost = additional_cost > 0 ? additional_cost : 1;

               
                

                if (costToNeighb * additional_cost < neighbour.gCost || !open_set.Contains(neighbour))
                {
                    neighbour.gCost = costToNeighb;
                    neighbour.hCost = getDistance(graph, neighbour, goal_node);
                    neighbour.parent = current;
                    neighbour.heading = neigh_heading;
                    neighbour.hybridAdditionalCost = additional_cost;
                    //String s = String.Format("Starting from [{0},{1}] to [{2},{3}] with angle {4}", current.i, current.j, neighbour.i, neighbour.j, neighbour.heading);
                    //Debug.Log(s);
                    /*switch (current.heading){
                        case Node.Turn.A:
                            if (neighbour.i > current.i)
                                if(neighbour.j > current.j)
                                {
                                    neighbour.turn_from_parent[0] = Node.Turn.A;
                                    neighbour.turn_from_parent[1] = Node.Turn.R;
                                    neighbour.heading = 

                                }

                                else if (neighbour.i < current.i)
                            else neighbour.turn_from_parent[0] = neighbour.turn_from_parent[1];

                                        break;
                        case Node.Turn.B:
                            break;
                        case Node.Turn.L:
                            break;
                        case Node.Turn.R:
                            break;

                    }*/


                    if (!open_set.Contains(neighbour))
                        open_set.Add(neighbour);
                    else
                    {
                        open_set.Remove(neighbour);
                        open_set.Add(neighbour);
                    }
                }

            }
        }

    }

    public static float getDistance(Graph graph, Node start, Node end, bool neighbour = false)
    {
        int x_distance = Math.Abs(start.i - end.i);
        int z_distance = Math.Abs(start.j - end.j);

        float diagonal_cost = (float)(Math.Sqrt(Math.Pow(graph.x_unit,2) + Math.Pow(graph.z_unit,2)));

        float addition = 1;

        if (neighbour)
        {
            if (start.i != end.i && start.j != end.j && start.j < graph.nodes.GetLength(1) && end.j < graph.nodes.GetLength(1) && start.i < graph.nodes.GetLength(0) && end.i < graph.nodes.GetLength(0))
            {
                if (!graph.nodes[start.i, end.j].walkable)
                {
                    addition = 2;
                    Debug.Log("ADDITION");
                    if (!graph.nodes[start.j, end.i].walkable)
                        addition += 100000;
                }

                if (!graph.nodes[start.j, end.i].walkable)
                {
                    addition = 2;
                    Debug.Log("ADDITION 2");
                    if (!graph.nodes[start.i, end.j].walkable)
                        addition += 100000;
                }
            }

        }

        return addition * (diagonal_cost * (x_distance < z_distance ? x_distance : z_distance) + (x_distance >= z_distance ? x_distance * graph.x_unit : z_distance * graph.z_unit));

    }

    public static List<Node> pathUpsampling(List<Node> original_path, int mul_fact) {
        if(original_path == null || original_path.Count==0)
        return null;
        List<Node> upsampled_path = new List<Node>();
        for(int i = 0; i<original_path.Count-1; i++)
        {
            Node curr = original_path[i];
            Node next = original_path[i + 1];
            for(int j = 1; j <= mul_fact; j++)
            {
                float new_x_pos = (next.x_pos - curr.x_pos)/mul_fact * j + curr.x_pos;
                float new_z_pos = (next.z_pos - curr.z_pos) / mul_fact * j + curr.z_pos;
                Node new_node = new Node(-1, -1, new_x_pos, new_z_pos);
                new_node.heading = curr.heading;
                upsampled_path.Add(new_node);
            }
        }
        
        return upsampled_path;
    }


    public static List<Node> pathSmoothing(List<Node> path, float weight_data= 0.5f, float weight_smooth= 0.1f, float tolerance= 0.000001f)
    {
        List<Node> smoothed = new List<Node>();

        foreach(Node n in path)
        {
            smoothed.Add(n.copy());
        }
        if (smoothed.Count != path.Count)
        {
            Debug.Log("ERROR!");
        }

        
        float change = tolerance;


        float x_i_x, y_i_x,y_prev_x, y_next_x, y_i_saved_x;
        float x_i_z, y_i_z, y_prev_z , y_next_z, y_i_saved_z;

        while (change >= tolerance)
        {
            change = 0;
            for(int i = 1; i < smoothed.Count - 1; i++)
            {
                x_i_x = path[i].x_pos;
                y_i_x = smoothed[i].x_pos;
                y_prev_x = smoothed[i - 1].x_pos;
                y_next_x = smoothed[i + 1].x_pos;
                y_i_saved_x = y_i_x;
                y_i_x += weight_data * (x_i_x - y_i_x) + weight_smooth * (y_next_x + y_prev_x - (2 * y_i_x));
                smoothed[i].x_pos = y_i_x;
                change += Math.Abs(y_i_x - y_i_saved_x);

                x_i_z = path[i].z_pos;
                y_i_z = smoothed[i].z_pos;
                y_prev_z = smoothed[i - 1].z_pos;
                y_next_z = smoothed[i + 1].z_pos;
                y_i_saved_z = y_i_z;
                y_i_z += weight_data * (x_i_z - y_i_z) + weight_smooth * (y_next_z + y_prev_z - (2 * y_i_z));
                smoothed[i].z_pos = y_i_z;
                change += Math.Abs(y_i_z - y_i_saved_z);
            }
        }

        foreach(Node n in smoothed)
        {
            n.worldPosition = new Vector3(n.x_pos, 0, n.z_pos);
        }

        return smoothed;
    }


    public static List<Node> bezierPath(List<Node> path, int multiplier)
    {
        Debug.Log("WHILE path size: " + path.Count);
        int final_size = path.Count * 2 * multiplier;
        float[] coordinates;
        float[] final_coord;
        List<Node> bezier_path = new List<Node>();
        int i = 0;
        int k = 0;
        while (k<path.Count)
        {
            //Debug.Log("WHILE i=" + i + " final_size = " + final_size);
            int steps = 20;
            steps *= 2;
            steps = (final_size - i) > steps ? steps : final_size - i;
            coordinates = new float[steps];
            final_coord = new float[steps*multiplier];
            for (int j = 0; j<steps; j+=2, i+=2, k++)
            {   Debug.Log("WHILE i=" + i + " final_size = " + final_size);
                Node current = path[k];
                Debug.Log("WHILE i:" + i + " j:" + j + " position(" + current.x_pos + "," + current.z_pos + ")");
                coordinates[j] = current.x_pos;
                coordinates[j + 1] = current.z_pos;
            }

            Curves.BezierCurve bezier = new Curves.BezierCurve();
            bezier.Bezier2D(coordinates, steps/2, final_coord);

            for (int j = 0; j < steps; j += 2)
            {
                bezier_path.Add(new Node(1, 1, final_coord[j], final_coord[j + 1]));
            }
            
        }
        
        Debug.Log("WHILE FINITO, len=" + bezier_path.Count);
        return bezier_path;
    }


    public static List<Node> downsample_path2(List<Node> path)
    {
        List<Node> to_return = new List<Node>();
        Node old = path[0];
        to_return.Add(old);
        for(int i = 1; i<path.Count-1; i++)
        {
            Debug.Log("Old: " + old.heading + " Current " + path[i].heading + " Next " + path[i + 1].heading);
            if(Math.Abs(old.heading - path[i].heading) <0.01  && Math.Abs(old.heading - path[i + 1].heading)<0.01 ||
                Math.Abs(old.x_pos - path[i].x_pos) < 0.01 && Math.Abs(old.x_pos - path[i + 1].x_pos) < 0.01 ||
                Math.Abs(old.z_pos - path[i].z_pos) < 0.01 && Math.Abs(old.z_pos - path[i + 1].z_pos) < 0.01
                )
            {
                Debug.Log("Skipping node " + i);
                continue;

            }
            old = path[i];
            to_return.Add(old);
        }
        to_return.Add(path[path.Count - 1]);

        return to_return;
    }

    public static List<Node> downsample_path(List<Node> path)
    {
        List<Node> to_return = new List<Node>();


        List<Node> path_ok = downsample_path2(path);

        Node old = path_ok[0];
        to_return.Add(old);
        for (int i = 1; i < path_ok.Count - 1; i++)
        {
            if (Math.Abs(path_ok[i].heading - path_ok[i + 1].heading) > 0.1)
                to_return.Add(path_ok[i]);

        }
        to_return.Add(path[path.Count - 1]);

        return to_return;
    }
}
