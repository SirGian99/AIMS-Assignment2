using System.Collections;
using System.Collections.Generic;
using System;
using UnityEngine;
using System.IO;

namespace UnityStandardAssets.Vehicles.Car
{
        [RequireComponent(typeof(CarController))]
    public class CarAI1 : MonoBehaviour
    {
        // Variables for Car
        private CarController m_Car; // the car controller we want to use
        Vector3 carSize = new Vector3(4.5f, 0.41f, 4.5f);
        private Rigidbody rigidbody;
        public int CarNumber;
        

        // Variables for Terrain
        public GameObject terrain_manager_game_object;
        TerrainManager terrain_manager;
        public Graph graph;
        public Graph original_graph;
        public GraphSTC map;
        public Graph subgraph;
        public DARP_controller darp;


        // Variables for Players and Turrets
        public GameObject[] friends;
        public GameObject[] enemies;
        private float gunRange = 10f;

        // Variables for path & driving
        private float acceleration, max_speed;
        private bool MazeComplete;
        private float time;
        private Vector3 start_pos, goal_pos;
        private int path_index;
        private List<Vector3> my_path;
        private Node starting_node;
        private List<Node> path_to_starting_node = new List<Node>();

        //Temp
        private Edge[] min_tree;


        private void Start()
        {
            // Initialize Variables
            Time.timeScale = 1;
            time = 0;
            max_speed = 20;
            acceleration = 1f;
            MazeComplete = false;
            path_index = 1;
            gunRange = 10f;


            // Initialize Car and Terrain
            m_Car = GetComponent<CarController>();
            terrain_manager = terrain_manager_game_object.GetComponent<TerrainManager>();
            start_pos = terrain_manager.myInfo.start_pos;
            rigidbody = GetComponent<Rigidbody>();

            // Construct Terrain Graph
            int x_scale = terrain_manager.myInfo.x_N;
            int z_scale = terrain_manager.myInfo.z_N;
            float x_len = (terrain_manager.myInfo.x_high - terrain_manager.myInfo.x_low);
            float z_len = (terrain_manager.myInfo.z_high - terrain_manager.myInfo.z_low);
            float x_unit = x_len / x_scale;
            x_unit = 1.4142f * gunRange;
            float z_unit = x_unit;
            //Debug.Log("x_scale: " + x_scale + " z_scale: " + z_scale);
            x_scale = x_scale * ((int)(x_len / x_unit) / x_scale);
            z_scale = z_scale * ((int)(z_len / z_unit) / z_scale);
            

            //i want x_unit and z_unit to be √2r, where r is the range of the gun.
            //but i also want the new scales them to be a multiple of the original x_scale and z_scale            
            graph = Graph.CreateGraph(terrain_manager.myInfo, x_scale, z_scale);
            original_graph = Graph.CreateGraph(terrain_manager.myInfo, x_scale, z_scale);
            //Debug.Log("Walkable nodes: " + graph.walkable_nodes);
            //Debug.Log("Non walk nodes: " + graph.non_walkable_nodes);


            // Get Array of Friends and Eniemies
            friends = GameObject.FindGameObjectsWithTag("Player");
             Vector3[] initial_positions = new Vector3[friends.Length];
            int i = 0;
            foreach(GameObject friend in friends)
            {
                if (friend.name == this.name)
                {
                    CarNumber = i+1;
                }
                //Debug.Log(friend + " position: " + friend.gameObject.transform.position);
                initial_positions[i] = friend.gameObject.transform.position;
                i++;

            }


            // Plan your path here
            darp = new DARP_controller(friends.Length, initial_positions, graph, 0.0004f, 100);
            subgraph = Graph.CreateSubGraph(graph, CarNumber, terrain_manager.myInfo, x_scale, z_scale);
            map = new GraphSTC(subgraph, start_pos);
            Debug.Log("Car " + CarNumber + " Compoents: " + GraphComponents(map));
            if(GraphComponents(map)!=1)
            {
                List<Node> VertexList = SpiltGraph(map, true);
                //Search neighbour index

                //Merge Graph

            }

            graph = subgraph;

            //TODO must update the assigned veichle value in each node in the original graph
            darp.update_assigned_nodes(original_graph);
            starting_node = PathFinder.get_starting_node(transform.position, CarNumber, original_graph, (360 - transform.eulerAngles.y + 90) % 360, ref path_to_starting_node);

            my_path = new List<Vector3>();
            if (path_to_starting_node.Count > 0)
            {
                int upsampling_factor = 4;
                path_to_starting_node = PathFinder.pathUpsampling(path_to_starting_node, upsampling_factor);
                path_to_starting_node = PathFinder.pathSmoothing(path_to_starting_node, 0.6f, 0.2f, 1E-09f); //Now the path is ready to be trasversed
            }



            my_path = CreateDronePath(map, transform.position, starting_node);
            min_tree = STC(map);

        }


        private void FixedUpdate()
        {
            /*
            enemies = GameObject.FindGameObjectsWithTag("Enemy");

            // this is how you access information about the terrain
            int i = terrain_manager.myInfo.get_i_index(transform.position.x);
            int j = terrain_manager.myInfo.get_j_index(transform.position.z);
            float grid_center_x = terrain_manager.myInfo.get_x_pos(i);
            float grid_center_z = terrain_manager.myInfo.get_z_pos(j);

            Debug.DrawLine(transform.position, new Vector3(grid_center_x, 0f, grid_center_z));

            // Drive Car
            if (!MazeComplete)
            {
                time += Time.deltaTime;
                path_index = DriveCar(my_path, m_Car, path_index);

                if (m_Car.transform.position == goal_pos)
                {
                    MazeComplete = true;
                }
            }
            else
            {
                m_Car.Move(0f, 0f, 0f, 1);
                Debug.Log("Path completed for Player " + time);

            }
            */

        }


        // MAIN FUNC: Divide and conquer
        public List<Vector3> CreateDronePath(GraphSTC graph, Vector3 car_position, Node starting_node)
        {
            Edge[] MinSTC = STC(graph);
            List<Vector3> path = null;

            //if(CarNumber==2)//TODO TODO TODO TODO TODO TODO USE THIS ONLY FOR DEBUGGING
            path = ComputePath(graph, MinSTC, car_position, starting_node);

            return path;
        }

        // Sub-Func: Compute path 
        private List<Vector3> ComputePath(GraphSTC graphSTC, Edge[] EdgeArray, Vector3 starting_position, Node starting_node, Orientation initial_orientation=Orientation.UU)
        {
            List<Vector3> path = new List<Vector3>();
            Node current_node = starting_node;
            Orientation arriving_orientation = initial_orientation;

            /*
             * Old code to compute the starting node, should not be needed anymore
             * 
            if (graphSTC.starting_node != null && false)
            {
                current_node = graphSTC.starting_node;
            }
            else
            {

                int i = terrain_manager.myInfo.get_i_index(starting_position.x);
                int j = terrain_manager.myInfo.get_j_index(starting_position.z);

                current_node = graph.nodes[i, j];
                if(current_node == null)
                {
                    bool found = false;
                    for(int ii = -1; ii<2 && !found; ii++)
                    {
                        for(int jj = -1; jj<2 && !found; jj++)
                        {
                            Node n = graph.nodes[i + ii, j + jj];
                            if(n!= null && n.is_supernode)
                            {
                                foreach(Node merged in n.merged_nodes)
                                {
                                    if (merged.i == i && merged.j == j)
                                    {
                                        current_node = n;
                                        found = true;
                                        break;
                                    }
                                }
                            }
                        }
                    }
                }

                //current_node = graphSTC.get_closest_node(starting_position);
               
            }

            */

            path.Add(current_node.worldPosition);
            Node next_node = PathFinder.get_next_node(initial_orientation, current_node);

            for (int i = 0; i <= graphSTC.VerticesCount;)
            {

                Direction direction = PathFinder.get_direction(arriving_orientation, current_node, next_node);
                if (next_node.is_supernode)
                {
                    switch (direction)
                    {
                        case Direction.F:
                            switch (initial_orientation)
                            {
                                case Orientation.UU:
                                    path.Add(new Vector3(current_node.x_pos + (graph.x_unit / 2), 0, current_node.z_pos + (graph.z_unit / 2)));
                                    path.Add(new Vector3(next_node.x_pos + graph.x_unit / 2, 0, next_node.z_pos - graph.z_unit / 2));
                                    break;
                                case Orientation.DD:
                                    path.Add(new Vector3(current_node.x_pos - graph.x_unit / 2, 0, current_node.z_pos - graph.z_unit / 2));
                                    path.Add(new Vector3(next_node.x_pos - graph.x_unit / 2, 0, next_node.z_pos + graph.z_unit / 2));
                                    break;
                                case Orientation.R:
                                    path.Add(new Vector3(current_node.x_pos + graph.x_unit / 2, 0, current_node.z_pos - graph.z_unit / 2));
                                    path.Add(new Vector3(next_node.x_pos - graph.x_unit / 2, 0, next_node.z_pos - graph.z_unit / 2));
                                    break;
                                case Orientation.L:
                                    path.Add(new Vector3(current_node.x_pos - graph.x_unit / 2, 0, current_node.z_pos + graph.z_unit / 2));
                                    path.Add(new Vector3(next_node.x_pos + graph.x_unit / 2, 0, next_node.z_pos + graph.z_unit / 2));
                                    break;
                            }
                            break;
                        case Direction.TL:
                            switch (initial_orientation)
                            {
                                case Orientation.UU:
                                    path.Add(new Vector3(current_node.x_pos + graph.x_unit / 2, 0, current_node.z_pos + graph.z_unit / 2));
                                    path.Add(new Vector3(current_node.x_pos - graph.x_unit / 2, 0, current_node.z_pos + graph.z_unit / 2));
                                    path.Add(new Vector3(next_node.x_pos + graph.x_unit / 2, 0, next_node.z_pos + graph.z_unit / 2));
                                    break;
                                case Orientation.DD:
                                    path.Add(new Vector3(current_node.x_pos + graph.x_unit / 2, 0, current_node.z_pos - graph.z_unit / 2));
                                    path.Add(new Vector3(current_node.x_pos - graph.x_unit / 2, 0, current_node.z_pos - graph.z_unit / 2));
                                    path.Add(new Vector3(next_node.x_pos - graph.x_unit / 2, 0, next_node.z_pos - graph.z_unit / 2));
                                    break;
                                case Orientation.R:
                                    path.Add(new Vector3(current_node.x_pos + graph.x_unit / 2, 0, current_node.z_pos - graph.z_unit / 2));
                                    path.Add(new Vector3(current_node.x_pos + graph.x_unit / 2, 0, current_node.z_pos + graph.z_unit / 2));
                                    path.Add(new Vector3(next_node.x_pos + graph.x_unit / 2, 0, next_node.z_pos - graph.z_unit / 2));
                                    break;
                                case Orientation.L:
                                    path.Add(new Vector3(current_node.x_pos - graph.x_unit / 2, 0, current_node.z_pos + graph.z_unit / 2));
                                    path.Add(new Vector3(current_node.x_pos - graph.x_unit / 2, 0, current_node.z_pos - graph.z_unit / 2));
                                    path.Add(new Vector3(next_node.x_pos + graph.x_unit / 2, 0, next_node.z_pos + graph.z_unit / 2));
                                    break;
                            }
                            break;
                        case Direction.TR:
                            switch (initial_orientation)
                            {
                                case Orientation.UU:
                                    path.Add(new Vector3(next_node.x_pos - graph.x_unit / 2, 0, next_node.z_pos - graph.z_unit / 2));
                                    break;
                                case Orientation.DD:
                                    path.Add(new Vector3(next_node.x_pos + graph.x_unit / 2, 0, next_node.z_pos + graph.z_unit / 2));
                                    break;
                                case Orientation.R:
                                    path.Add(new Vector3(next_node.x_pos - graph.x_unit / 2, 0, next_node.z_pos + graph.z_unit / 2));
                                    break;
                                case Orientation.L:
                                    path.Add(new Vector3(next_node.x_pos - graph.x_unit / 2, 0, next_node.z_pos - graph.z_unit / 2));
                                    break;
                            }
                            break;
                        default:
                            throw new Exception("GOT DIRECTION ERROR");
                    }

                }
                else
                {
                    path.Add(next_node.worldPosition);
                }
                if (next_node.visited == false || i == graphSTC.VerticesCount)
                {
                    i++;
                    next_node.visited = true;
                }


                arriving_orientation = PathFinder.getOrientation(path[path.Count-2], path[path.Count-1]);
                next_node = PathFinder.get_next_node(arriving_orientation, current_node);

                if (next_node == null) //Supernode dead branch: do two left turns and then go to the parent
                {
                    switch (arriving_orientation)
                    {
                        case Orientation.UU:
                            path.Add(new Vector3(current_node.x_pos + graph.x_unit / 2, 0, current_node.z_pos + graph.z_unit / 2));
                            path.Add(new Vector3(current_node.x_pos - graph.x_unit / 2, 0, current_node.z_pos + graph.z_unit / 2));
                            path.Add(new Vector3(current_node.x_pos - graph.x_unit / 2, 0, current_node.z_pos - graph.z_unit / 2));
                            break;
                        case Orientation.DD:
                            path.Add(new Vector3(current_node.x_pos + graph.x_unit / 2, 0, current_node.z_pos - graph.z_unit / 2));
                            path.Add(new Vector3(current_node.x_pos - graph.x_unit / 2, 0, current_node.z_pos - graph.z_unit / 2));
                            path.Add(new Vector3(current_node.x_pos + graph.x_unit / 2, 0, current_node.z_pos + graph.z_unit / 2));
                            break;
                        case Orientation.R:
                            path.Add(new Vector3(current_node.x_pos + graph.x_unit / 2, 0, current_node.z_pos - graph.z_unit / 2));
                            path.Add(new Vector3(current_node.x_pos + graph.x_unit / 2, 0, current_node.z_pos + graph.z_unit / 2));
                            path.Add(new Vector3(current_node.x_pos - graph.x_unit / 2, 0, current_node.z_pos + graph.z_unit / 2));
                            break;
                        case Orientation.L:
                            path.Add(new Vector3(current_node.x_pos - graph.x_unit / 2, 0, current_node.z_pos + graph.z_unit / 2));
                            path.Add(new Vector3(current_node.x_pos - graph.x_unit / 2, 0, current_node.z_pos - graph.z_unit / 2));
                            path.Add(new Vector3(current_node.x_pos + graph.x_unit / 2, 0, current_node.z_pos - graph.z_unit / 2));
                            break;
                    }

                    next_node = current_node.parent;
                }


            } 

            return path;
        }




        // MAIN FUNC: Separate terrain into different areas

        // Sub-Func: Kurskal Algorithm to find number of compoenets in graph
        public int GraphComponents(GraphSTC graph)
        {
            int verticesCount = graph.VerticesCount;
            Node[] VertexArray = graph.VertexArray;
            int k = 0;
            int e = 0;
            int subset_count = 0;

            // Sort edges by cost- all costs are same
            graph.EdgeList.Sort((e1,e2)=> e1.Weight.CompareTo(e2.Weight));

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
                
                //if (k >= graph.EdgesCount)
                //{
                    //Debug.Log("Car " + CarNumber + " TotalEdges, k: " + graph.EdgesCount + " " + k + " MinEdges,e"+ (verticesCount-1) + " " + e  + " " );
                  //  break;
                //}
            }
            subset_count = verticesCount - e;

            return subset_count;
        }

        // Sub-Func: Kurskal Algorithm to sub-graph
        public List<Node> SpiltGraph(GraphSTC graph, Boolean smallest)
        {
            int verticesCount = graph.VerticesCount;
            List<Node> result = new List<Node>();
            Node[] VertexArray = graph.VertexArray;
            int k = 0;
            int e = 0;
            int subset_count = 0;

            // Sort edges by cost- all costs are same
            graph.EdgeList.Sort((e1,e2)=> e1.Weight.CompareTo(e2.Weight));

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

        // Sub-Func: Find Closest neighbour




        // MAIN FUNC: Kurskal Algorithm to find minimum spanning tree
        public Edge[] STC(GraphSTC graph)
        {
            int verticesCount = graph.VerticesCount;
            Edge[] result = new Edge[verticesCount];
            Node[] VertexArray = graph.VertexArray;
            int k = 0;
            int e = 0;

            // Sort edges by cost- all costs are same
            graph.EdgeList.Sort((e1,e2)=> e1.Weight.CompareTo(e2.Weight));

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
                    if(nextEdge.Source.children == null)
                    {
                        nextEdge.Source.children = new List<Node>();
                    }
                    nextEdge.Source.children.Add(nextEdge.Destination);

                    /*
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
                    */

                    //Debug.Log("Edge " + e + " S: " + nextEdge.Source.worldPosition + " to D: " + nextEdge.Destination.worldPosition);
                    Union(subsets, x, y, VertexArray);
                }
                k++;
                //Debug.Log("Car " + CarNumber + " k,e: " + k + " " + e + " " + graph.EdgesCount);
                if (k >= graph.EdgesCount)
                {
                   Debug.Log("STILL ENTERING HERE: "  + CarNumber);
                    break;
                }
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

        // Data Stuture: Subset for kurskal's
        public struct Subset
        {
            public Node Parent { get; set; }
            public int Rank { get; set; }
        }


        // MAIN FUNC: Car Drive
        public int DriveCar(List<Vector3> player_path, CarController player_Car, int player_pathIndex)
        {
            Vector3 player_waypoint = player_path[player_pathIndex];
            int brake = 0;
            int handBrake = 0;
            float car_steer, car_acc;
            bool MazeComplete = false;

            //find steering needed to get to next point
            car_steer = Steer(player_Car.transform.position, player_Car.transform.eulerAngles.y, player_waypoint);

            // if turn is too small, don't steer
            if (Mathf.Abs(car_steer) < 0.2f)
            {
                car_steer = 0;
            }

            // if turn is too big, brake to turn easily & don't accelerate
            if (Mathf.Abs(car_steer) > 0.8f && player_Car.CurrentSpeed > max_speed / 10)
            {
                car_acc = 0;
                if (player_Car.CurrentSpeed > max_speed / 5)
                {
                    handBrake = 1;
                }
                else
                {
                    handBrake = 0;
                }
            }
            // apply acceleration to get to next point
            else
            {
                car_acc = Accelerate(player_Car.transform.position, player_Car.transform.eulerAngles.y, player_waypoint);
                handBrake = 0;
                // slow down towards end
                //if (path_index > (player_path.Count - 10))
                //{
                //    acceleration = acceleration * 0.2f;
                //}
            }
            // addtional checks
            if (player_pathIndex < player_path.Count - 1)
            {
                int check = Mathf.Min(3 + (int)(player_Car.CurrentSpeed * 1.6f * 1.6f * player_Car.CurrentSpeed / 500), player_path.Count - 1 - player_pathIndex);
                for (int i = 1; i <= check; ++i)
                {
                    float steer_check = Steer(player_Car.transform.position, player_Car.transform.eulerAngles.y, player_path[player_pathIndex + i]);
                    if (Mathf.Abs(steer_check) > 0.8f && (player_Car.CurrentSpeed * 1.6f * 1.6f * player_Car.CurrentSpeed)
                        >= Vector3.Distance(player_Car.transform.position, player_path[player_pathIndex + i]) * 250 * 0.8f)
                    {
                        car_acc = 0;
                        brake = 1;
                        break;
                    }
                }
            }
            // if current speed is max, then don't accelerate
            if (player_Car.CurrentSpeed >= max_speed)
            {
                car_acc = 0;
            }

            // if acceleration is reverse, apply backwards turns
            if (car_acc < 0)
            {
                player_Car.Move(-car_steer, brake, car_acc * acceleration, handBrake);
            }
            else
            {
                player_Car.Move(car_steer, car_acc * acceleration, -brake, handBrake);
            }

            if (Vector3.Distance(player_Car.transform.position, player_waypoint) <= 5 + player_Car.CurrentSpeed / 40)
            {
                player_pathIndex = Mathf.Min(player_pathIndex + 1, player_path.Count - 1);
            }

            return player_pathIndex;
        }

        // Sub-Func: To find steering angle for car
        private float Steer(Vector3 position, float theta, Vector3 target)
        {
            Vector3 direction = Quaternion.Euler(0, theta, 0) * Vector3.forward;
            Vector3 directionToTarget = target - position;
            float angle = Vector3.Angle(direction, directionToTarget) * Mathf.Sign(-direction.x * directionToTarget.z + direction.z * directionToTarget.x);
            return Mathf.Clamp(angle, -m_Car.m_MaximumSteerAngle, m_Car.m_MaximumSteerAngle) / m_Car.m_MaximumSteerAngle;
        }

        //Sub-Func: To find accleration for car
        private float Accelerate(Vector3 position, float theta, Vector3 target)
        {
            Vector3 direction = Quaternion.Euler(0, theta, 0) * Vector3.forward;
            Vector3 directionToTarget = target - position;
            return Mathf.Clamp(direction.x * directionToTarget.x + direction.z * directionToTarget.z, -1, 1);
        }



        //MAIN FUNC: Visualise
        void OnDrawGizmos() // draws grid on map and shows car
        {
            Color[] colors = { Color.red, Color.cyan, Color.yellow, Color.white, Color.black, Color.green };


            

            if (graph != null)
                {

                    Node currentNode = graph.getNodeFromPoint(transform.position);
                    //Debug.Log("CAR INITIAL NODE: [" + currentNode.i + "," + currentNode.j + "]");
                    Gizmos.color = Color.cyan; // position of car
                    //Debug.Log("Current car node: [" + currentNode.i + "," + currentNode.j + "]");
                    //Gizmos.DrawCube(currentNode.worldPosition, new Vector3(graph.x_unit * 0.8f, 0.5f, graph.z_unit * 0.8f));
                int z = 0;
                foreach (Node n in graph.nodes) // graph.path 
                {
                    if (n!= null &&(n.assigned_veichle == CarNumber || !n.walkable))
                    {
                        int index = darp.assignment_matrix[n.i, n.j];

                        Gizmos.color = colors[index];
                        if (graph.path != null && graph.path.Contains(n))
                            Gizmos.color = Color.white;

                        float scale_factor = n.is_supernode ? 1.8f : 0.9f;
                        Gizmos.DrawCube(n.worldPosition, new Vector3(graph.x_unit * scale_factor, 0.5f, graph.z_unit * scale_factor));

                        //This code below draws the neighbours of the bigger nodes
                        /*if(z <= 300 && n.is_supernode && n.assigned_veichle == CarNumber)
                        {
                            foreach(Node neigh in n.neighbours)
                            {
                                if (neigh.walkable)
                                {
                                    Gizmos.color = Color.black;
                                    scale_factor = neigh.is_supernode ? 1.8f : 0.9f;
                                    Gizmos.DrawCube(neigh.worldPosition, new Vector3(graph.x_unit * scale_factor, 0.5f, graph.z_unit * scale_factor));
                                }
                            }
                            z++;
                        }
                        */

                    }

                }
                }


            if (map != null)
            {
                foreach (Edge edge in map.EdgeList)
                {
                    //UNCOMMENT THIS TO SEE ALL THE EDGES
                    /*
                    Gizmos.color = Color.red;
                    Gizmos.DrawLine(edge.Source.worldPosition, edge.Destination.worldPosition);
                    Gizmos.DrawSphere(edge.Source.worldPosition, 2f);
                    Gizmos.color = Color.green;
                    Gizmos.DrawSphere(edge.Destination.worldPosition, 2f);
                    */
                }
            }


            //Show min span tree
            if (min_tree != null)
                {
                    Gizmos.color = colors[CarNumber];
                    //Gizmos.color = Color.red;
                    for (int i = 0; i < min_tree.Length - 1; ++i)
                    {

                    if (min_tree[i].Source != null && min_tree[i].Destination != null)
                        Gizmos.DrawLine(min_tree[i].Source.worldPosition + Vector3.up, min_tree[i].Destination.worldPosition + Vector3.up);
                        ;
                        //Gizmos.DrawSphere(min_tree[i].Source.worldPosition, 5f);
                        //Gizmos.DrawSphere(min_tree[i].Destination.worldPosition, 2f);

                    }
                }


                //Show the path to the goal
                if (my_path != null)
                {
                    Gizmos.color = Color.black;
                    for (int i = 0; i < my_path.Count - 1; ++i)
                    {
                        ;
                        //Gizmos.DrawLine(my_path[i], my_path[i + 1]);
                    }
                }


            

            
        }
    }
}
