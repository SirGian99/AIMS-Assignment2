using System.Collections;
using System.Collections.Generic;
using System;
using UnityEngine;
using System.IO;

namespace UnityStandardAssets.Vehicles.Car
{
        [RequireComponent(typeof(CarController))]
    public class CarAI1_Gian : MonoBehaviour
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
        private CarConfigSpace ObstacleSpace;


        // Variables for Players and Turrets
        public GameObject[] friends;
        public GameObject[] enemies;
        private float gunRange = 10f;

        // Variables for path & driving
        private float acceleration, max_speed;
        private bool MazeComplete, PlayerCrashed;
        private float mazeTimer, driveTimer, stuckTimer, recoveryTime;
        private float recoverySteer;
        private Vector3 start_pos, goal_pos, previous_pos;
        private int path_index;
        private List<Vector3> my_path;
        private Node starting_node;
        private List<Node> path_to_starting_node = new List<Node>();
        private List<Vector3> final_path = new List<Vector3>();
        private Edge[] min_tree;
        enum car_state {drive, front_crash, back_crash};
        private car_state car_status;

        enum car_detection_state {no_detection, frontal_detection, frontal_crash}
        private car_detection_state car_status_gian = car_detection_state.no_detection;

        private bool had_hit_backward = false;
        private float accelerationAmount = 0;
        private float footbrake = 0;
        private float steeringAmount = 0;
        private float handbrake = 0;
        private Vector3 targetPosition;

        private void Start()
        {
            // Initialize Variables
            Time.timeScale = 10;
            driveTimer = 0f;
            max_speed = 20;
            acceleration = 1f;
            MazeComplete = false;
            mazeTimer = 0f;
            path_index = 1;
            gunRange = 10f;
            car_status = car_state.drive;
            stuckTimer = 0f;
            PlayerCrashed = false;
            recoveryTime = 0.7f;
            recoverySteer = 0.45f;//45 degrees gives best result


            // Initialize Car and Terrain
            m_Car = GetComponent<CarController>();
            terrain_manager = terrain_manager_game_object.GetComponent<TerrainManager>();
            start_pos = terrain_manager.myInfo.start_pos;
            rigidbody = GetComponent<Rigidbody>();
            
            // Initialize ConfigSpace
            CreateObstacleSpace();

            // Construct Terrain Graph
            int x_scale = terrain_manager.myInfo.x_N;
            int z_scale = terrain_manager.myInfo.z_N;
            float x_len = (terrain_manager.myInfo.x_high - terrain_manager.myInfo.x_low);
            float z_len = (terrain_manager.myInfo.z_high - terrain_manager.myInfo.z_low);
            float x_unit = x_len / x_scale;
            x_unit = 1.4142f * gunRange;
            float z_unit = x_unit;
            x_scale = x_scale * ((int)(x_len / x_unit) / x_scale);
            z_scale = z_scale * ((int)(z_len / z_unit) / z_scale);
            //i want x_unit and z_unit to be √2r, where r is the range of the gun.
            //but i also want the new scales them to be a multiple of the original x_scale and z_scale            
            graph = Graph.CreateGraph(terrain_manager.myInfo, x_scale, z_scale);
            original_graph = Graph.CreateGraph(terrain_manager.myInfo, x_scale, z_scale);


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


            // DARP Algorithm
            // Get subgraph:
            darp = new DARP_controller(friends.Length, initial_positions, graph, 0.0004f, 100);
            subgraph = Graph.CreateSubGraph(graph, CarNumber, terrain_manager.myInfo, x_scale, z_scale);
            map = new GraphSTC(subgraph, start_pos);
            graph = subgraph;
            //TODO must update the assigned veichle value in each node in the original graph
            darp.update_assigned_nodes(original_graph);
            starting_node = PathFinder.get_starting_node(transform.position, CarNumber, original_graph, graph, (360 - transform.eulerAngles.y + 90) % 360, ref path_to_starting_node);

            // Get min tree:
            min_tree = Prim_STC(map, starting_node);

            // Get path from min_tree:
            int upsampling_factor = 4;
            my_path = new List<Vector3>();
            //my_path = CreateDronePath(map, transform.position, starting_node);
            my_path = ComputePath(map, min_tree, transform.position, starting_node);
            my_path = PathFinder.pathUpsampling(my_path, upsampling_factor);
            my_path = PathFinder.pathSmoothing(my_path, 0.6f, 0.2f, 1E-09f);

            // Get path to starting node
            if (path_to_starting_node.Count > 0)
            {
                path_to_starting_node = PathFinder.pathUpsampling(path_to_starting_node, upsampling_factor);
                path_to_starting_node = PathFinder.pathSmoothing(path_to_starting_node, 0.6f, 0.2f, 1E-09f); //Now the path is ready to be trasversed
            }



            my_path = CreateDronePath(map, transform.position, starting_node, get_initial_orientation(transform.position, starting_node.worldPosition));
            
            //min_tree = STC(map);
            min_tree = Prim_STC(map, starting_node);
            my_path = PathFinder.pathUpsampling(my_path, upsampling_factor);
            //my_path = PathFinder.pathSmoothing(my_path, 0.6f, 0.2f, 1E-09f);

            foreach(Node n in path_to_starting_node)
            {
                final_path.Add(n.worldPosition);
            }
            final_path.AddRange(my_path);

            // TODO : ADD CURVES to PATH

            my_path = final_path;

        }


        private void FixedUpdate()
        {
            mazeTimer += Time.deltaTime;
            enemies = GameObject.FindGameObjectsWithTag("Enemy");
            Debug.Log("Enemies remaining: " + enemies.Length + " Time Elapsed: " + mazeTimer);

            // Drive Car
            if (!MazeComplete) 
            {
                //path_index = DriveCar(my_path, m_Car, path_index);
                path_index = gianDriveCar(my_path, m_Car, path_index);
                //Check if all enemies killed for maze completion
                if (enemies.Length <= 0 || path_index >= my_path.Count)
                {
                    MazeComplete = true;
                    Debug.Log("Car " + CarNumber + "Path Completed in " + mazeTimer);
                }
            }
            else
            {
                // Do nothing for now - wait for others to finish
                // How to utilise this extra time???

            }

            // Exit Game logic
            if(enemies.Length <= 0)
            {
                Debug.Log("All enemies killed in " + mazeTimer + " Exiting Game...");
                UnityEditor.EditorApplication.isPlaying = false;
            }
        }


        // MAIN FUNC: Divide and conquer
        public List<Vector3> CreateDronePath(GraphSTC graph, Vector3 car_position, Node starting_node, Orientation initial_orientation = Orientation.UU)
        {
            Edge[] MinSTC = Prim_STC(graph, starting_node);
            List<Vector3> path = null;

            //if(CarNumber==2)//TODO TODO TODO TODO TODO TODO USE THIS ONLY FOR DEBUGGING
            path = ComputePath(graph, MinSTC, car_position, starting_node, initial_orientation);

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

            if (!current_node.is_supernode)
            {
                path.Add(current_node.worldPosition);
            }
            else
            {
                path.Add(new Vector3(current_node.x_pos + graph.x_unit / 2, 0, current_node.z_pos - graph.z_unit / 2));
            }
            Node next_node;// = PathFinder.get_next_node(initial_orientation, current_node);

            for (int i = 0; i <= graphSTC.VerticesCount;)
            {
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
                            path.Add(new Vector3(current_node.x_pos - graph.x_unit / 2, 0, current_node.z_pos - graph.z_unit / 2));
                            path.Add(new Vector3(current_node.x_pos + graph.x_unit / 2, 0, current_node.z_pos - graph.z_unit / 2));
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
                    arriving_orientation = PathFinder.getOrientation(path[path.Count - 2], path[path.Count - 1]);
                }

                Direction direction = PathFinder.get_direction(arriving_orientation, current_node, next_node);
                if (next_node.is_supernode)
                {
                    if (current_node.is_supernode)
                    {
                        switch (direction)
                        {
                            case Direction.F:
                                switch (arriving_orientation)
                                {
                                    case Orientation.UU:
                                        path.Add(new Vector3(current_node.x_pos + (graph.x_unit / 2f), 0, current_node.z_pos + (graph.z_unit / 2)));
                                        path.Add(new Vector3(next_node.x_pos + graph.x_unit / 2, 0, next_node.z_pos - graph.z_unit / 2));
                                        break;
                                    case Orientation.DD:
                                        path.Add(new Vector3(current_node.x_pos - (graph.x_unit / 2f), 0, current_node.z_pos - graph.z_unit / 2));
                                        path.Add(new Vector3(next_node.x_pos - graph.x_unit / 2, 0, next_node.z_pos + graph.z_unit / 2));
                                        break;
                                    case Orientation.R:
                                        path.Add(new Vector3(current_node.x_pos + (graph.x_unit / 2f), 0, current_node.z_pos - graph.z_unit / 2));
                                        path.Add(new Vector3(next_node.x_pos - graph.x_unit / 2, 0, next_node.z_pos - graph.z_unit / 2));
                                        break;
                                    case Orientation.L:
                                        path.Add(new Vector3(current_node.x_pos - (graph.x_unit / 2f), 0, current_node.z_pos + (graph.z_unit / 2f)));
                                        path.Add(new Vector3(next_node.x_pos + (graph.x_unit / 2f), 0, next_node.z_pos + graph.z_unit / 2));
                                        break;
                                }
                                break;
                            case Direction.TL:
                                switch (arriving_orientation)
                                {
                                    case Orientation.UU:
                                        path.Add(new Vector3(current_node.x_pos + (graph.x_unit / 2f), 0, current_node.z_pos + graph.z_unit / 2));
                                        path.Add(new Vector3(current_node.x_pos - (graph.x_unit / 2f), 0, current_node.z_pos + graph.z_unit / 2));
                                        path.Add(new Vector3(next_node.x_pos + (graph.x_unit / 2f), 0, next_node.z_pos + graph.z_unit / 2));
                                        break;
                                    case Orientation.DD:
                                        path.Add(new Vector3(current_node.x_pos - (graph.x_unit / 2f), 0, current_node.z_pos - graph.z_unit / 2));
                                        path.Add(new Vector3(current_node.x_pos + (graph.x_unit / 2f), 0, current_node.z_pos - graph.z_unit / 2));
                                        path.Add(new Vector3(next_node.x_pos - (graph.x_unit / 2f), 0, next_node.z_pos - graph.z_unit / 2));
                                        break;
                                    case Orientation.R:
                                        path.Add(new Vector3(current_node.x_pos + (graph.x_unit / 2f), 0, current_node.z_pos - graph.z_unit / 2));
                                        path.Add(new Vector3(current_node.x_pos + (graph.x_unit / 2f), 0, current_node.z_pos + graph.z_unit / 2));
                                        path.Add(new Vector3(next_node.x_pos + (graph.x_unit / 2f), 0, next_node.z_pos - graph.z_unit / 2));
                                        break;
                                    case Orientation.L:
                                        path.Add(new Vector3(current_node.x_pos - (graph.x_unit / 2f), 0, current_node.z_pos + graph.z_unit / 2));
                                        path.Add(new Vector3(current_node.x_pos - (graph.x_unit / 2f), 0, current_node.z_pos - graph.z_unit / 2));
                                        path.Add(new Vector3(next_node.x_pos - (graph.x_unit / 2f), 0, next_node.z_pos + graph.z_unit / 2));
                                        break;
                                }
                                break;
                            case Direction.TR:
                                switch (arriving_orientation)
                                {
                                    case Orientation.UU:
                                        path.Add(new Vector3(next_node.x_pos - graph.x_unit / 2f, 0, next_node.z_pos - graph.z_unit / 2f));
                                        break;
                                    case Orientation.DD:
                                        path.Add(new Vector3(next_node.x_pos + graph.x_unit / 2f, 0, next_node.z_pos + graph.z_unit / 2f));
                                        break;
                                    case Orientation.R:
                                        path.Add(new Vector3(next_node.x_pos - graph.x_unit / 2f, 0, next_node.z_pos + graph.z_unit / 2f));
                                        break;
                                    case Orientation.L:
                                        path.Add(new Vector3(next_node.x_pos + (graph.x_unit / 2f), 0, next_node.z_pos - graph.z_unit / 2f));
                                        break;
                                }
                                break;
                            default:
                                throw new Exception("GOT DIRECTION ERROR");
                        }
                        //arriving_orientation = PathFinder.getOrientation(path[path.Count - 2], path[path.Count - 1]);
                    }
                    else
                    {
                        path.Add(current_node.worldPosition);
                        switch (direction)
                        {
                            case Direction.F:
                                switch (arriving_orientation)
                                {
                                    case Orientation.UU:
                                        path.Add(new Vector3(next_node.x_pos + graph.x_unit / 2, 0, next_node.z_pos - graph.z_unit / 2));
                                        break;
                                    case Orientation.DD:
                                        path.Add(new Vector3(next_node.x_pos - graph.x_unit / 2, 0, next_node.z_pos + graph.z_unit / 2));
                                        break;
                                    case Orientation.R:
                                        path.Add(new Vector3(next_node.x_pos - graph.x_unit / 2, 0, next_node.z_pos - graph.z_unit / 2));
                                        break;
                                    case Orientation.L:
                                        path.Add(new Vector3(next_node.x_pos + graph.x_unit / 2, 0, next_node.z_pos + graph.z_unit / 2));
                                        break;
                                }
                                break;
                            case Direction.TL:
                                switch (arriving_orientation)
                                {
                                    case Orientation.UU:
                                        path.Add(new Vector3(next_node.x_pos + graph.x_unit / 2, 0, next_node.z_pos + graph.z_unit / 2));
                                        break;
                                    case Orientation.DD:
                                        path.Add(new Vector3(next_node.x_pos - graph.x_unit / 2, 0, next_node.z_pos - graph.z_unit / 2));//seems wrong at a first glance but i'm tired and may be wrong myself
                                        break;
                                    case Orientation.R:
                                        path.Add(new Vector3(next_node.x_pos + graph.x_unit / 2, 0, next_node.z_pos - graph.z_unit / 2));
                                        break;
                                    case Orientation.L:
                                        path.Add(new Vector3(next_node.x_pos + graph.x_unit / 2, 0, next_node.z_pos + graph.z_unit / 2));
                                        break;
                                }
                                break;
                            case Direction.TR:
                                switch (arriving_orientation)
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
                            case Direction.UTURN:
                                switch (arriving_orientation)
                                {
                                    case Orientation.UU:
                                        path.Add(new Vector3(next_node.x_pos - graph.x_unit / 2, 0, next_node.z_pos + graph.z_unit / 2));
                                        break;
                                    case Orientation.DD:
                                        path.Add(new Vector3(next_node.x_pos + graph.x_unit / 2, 0, next_node.z_pos - graph.z_unit / 2));
                                        break;
                                    case Orientation.R:
                                        path.Add(new Vector3(next_node.x_pos + graph.x_unit / 2, 0, next_node.z_pos + graph.z_unit / 2));
                                        break;
                                    case Orientation.L:
                                        path.Add(new Vector3(next_node.x_pos - graph.x_unit / 2, 0, next_node.z_pos - graph.z_unit / 2));
                                        break;
                                }
                                break;
                            default:
                                throw new Exception("GOT DIRECTION ERROR SMALL NODE");
                        }
                        //arriving_orientation = PathFinder.getOrientation(current_node, next_node);
                    }
                }
                else
                {
                    if (current_node.is_supernode)
                    {
                        switch (direction)
                        {
                            case Direction.F:
                                path.Add(next_node.worldPosition);
                                break;
                            case Direction.TL:
                                switch (arriving_orientation)
                                {
                                    case Orientation.UU:
                                        path.Add(new Vector3(current_node.x_pos + (graph.x_unit / 2f), 0, current_node.z_pos + graph.z_unit / 2));
                                        path.Add(new Vector3(current_node.x_pos - (graph.x_unit / 2f), 0, current_node.z_pos + graph.z_unit / 2));
                                        path.Add(next_node.worldPosition);
                                        break;
                                    case Orientation.DD:
                                        path.Add(new Vector3(current_node.x_pos - (graph.x_unit / 2f), 0, current_node.z_pos - graph.z_unit / 2));
                                        path.Add(new Vector3(current_node.x_pos + (graph.x_unit / 2f), 0, current_node.z_pos - graph.z_unit / 2));
                                        path.Add(next_node.worldPosition);
                                        break;
                                    case Orientation.R:
                                        path.Add(new Vector3(current_node.x_pos + (graph.x_unit / 2f), 0, current_node.z_pos - graph.z_unit / 2));
                                        path.Add(new Vector3(current_node.x_pos + (graph.x_unit / 2f), 0, current_node.z_pos + graph.z_unit / 2));
                                        path.Add(next_node.worldPosition);
                                        break;
                                    case Orientation.L:
                                        path.Add(new Vector3(current_node.x_pos - (graph.x_unit / 2f), 0, current_node.z_pos + graph.z_unit / 2));
                                        path.Add(new Vector3(current_node.x_pos - (graph.x_unit / 2f), 0, current_node.z_pos - graph.z_unit / 2));
                                        path.Add(next_node.worldPosition);
                                        break;
                                }
                                break;
                            case Direction.TR:
                                path.Add(next_node.worldPosition);
                                break;
                            default:
                                throw new Exception("GOT DIRECTION ERROR");
                        }
                    }
                    else
                    {
                        path.Add(next_node.worldPosition);
                    }
                    arriving_orientation = PathFinder.getOrientation(path[path.Count - 2], path[path.Count - 1]);
                }
                if (next_node.visited == false || i == graphSTC.VerticesCount)
                {
                    i++;
                    next_node.visited = true;
                }

                if (next_node == starting_node && next_node.children_to_visit ==0)
                    break;

                arriving_orientation = PathFinder.getOrientation(current_node, next_node);
                current_node = next_node;



            }

            return path;
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


        // MAIN FUNC: Prims Algorithm to find minimum spanning tree
        public Edge[] Prim_STC(GraphSTC graph, Node starting_node)
        {

            int start_node = Array.IndexOf(graph.VertexArray, starting_node);
            string adjacency_string = "DEBUGG\n";

            if (CarNumber == 1)
            {
                
                for(int z = 0; z< graph.VertexArray.Length; z++)
                {
                    Debug.Log("DEBUGG Node " + z + ": " + graph.VertexArray[z]);
                }

                for(int i = 0; i<graph.adj_matrix.GetLength(0); i++)
                {
                    for(int j= 0; j < graph.adj_matrix.GetLength(1); j++)
                    {
                        adjacency_string += "" + graph.adj_matrix[i, j] + ";";
                    }
                    adjacency_string += "\n";
                }
                Debug.Log(adjacency_string);

            }

            int verticesCount = graph.VerticesCount;
            int[] parent = new int[verticesCount];
            float[] key = new float[verticesCount];
            bool[] mstSet = new bool[verticesCount];

            for(int i = 0; i < verticesCount; i++)
            {
                mstSet[i] = false;
            }

            mstSet[start_node] = true;
            starting_node.parent = null;
            int min_cost_src = -1;
            int min_cost_dest = -1;
            float min_cost = float.MaxValue;

            Edge[] result = new Edge[verticesCount];

            for (int k = 0; k < verticesCount; k++)
            {
                for (int source_index = 0; source_index < verticesCount; source_index++)
                {
                    if (mstSet[source_index])
                    {
                        for (int dest_index = 0; dest_index < verticesCount; dest_index++)
                        {
                            if (!mstSet[dest_index] && graph.adj_matrix[source_index, dest_index] >= 0 && graph.adj_matrix[source_index, dest_index] < min_cost)
                            {
                                min_cost_src = source_index;
                                min_cost_dest = dest_index;
                                min_cost = graph.adj_matrix[source_index, dest_index];
                            }
                        }
                    }
                }

                if (CarNumber == 2)
                {
                    Debug.Log("Adding edge Source index: " + min_cost_src + " dest index:" + min_cost_dest + " wasSourceIn" + mstSet[min_cost_src] + "wasDestIn" + mstSet[min_cost_dest]);
                }
                mstSet[min_cost_dest] = true;
                Edge nextEdge = new Edge();
                nextEdge.Source = graph.VertexArray[min_cost_src];
                nextEdge.Destination = graph.VertexArray[min_cost_dest];
                nextEdge.Weight = min_cost;
                if(!nextEdge.Source.children.Contains(nextEdge.Destination))
                    nextEdge.Source.children.Add(nextEdge.Destination);
                nextEdge.Destination.parent = nextEdge.Source;
                result[k] = nextEdge;
                min_cost = float.MaxValue;
                if (CarNumber == 2)
                    Debug.Log("Adding edge from node " + nextEdge.Source + " to " + nextEdge.Destination + " k:" + k + "Total" + (verticesCount-1));
                //min_cost_dest = -1;
                //min_cost_src = -1;
                
            }
            /*


            for (int i = 0; i < verticesCount; ++i)
            {
                key[i] = int.MaxValue;
                mstSet[i] = false;
            }

            key[0] = 0f;
            parent[0] = -1;

            for(int count = 0; count < verticesCount -1; ++count)
            {
                int u = MinKey(key, mstSet, verticesCount);
                mstSet[u] = true;

                for(int v = 0; v < verticesCount; ++v)
                {
                    if(Convert.ToBoolean(graph.adj_matrix[u, v]) && mstSet[v] == false && graph.adj_matrix[u, v] < key[v])
                    {
                        parent[v] = u;
                        key[v] = graph.adj_matrix[u, v];
                    }
                }
            }

            // Construct Edges
            //Edge[] result = new Edge[verticesCount - 1];
            for (int i = 1; i < verticesCount; ++i)
            {
                Edge nextEdge = new Edge();
                nextEdge.Source = graph.VertexArray[parent[i]];
                nextEdge.Destination = graph.VertexArray[i];
                nextEdge.Weight = graph.adj_matrix[i, parent[i]];
                result[i - 1] = nextEdge;
                //Debug.Log(CarNumber + " NewEdge (" + nextEdge.Source.i + "," + nextEdge.Source.j + ") - (" + nextEdge.Destination.i + "," + nextEdge.Destination.j + ")");
                Debug.Log(CarNumber + " NewEdge "+ nextEdge.Source + " - " + nextEdge.Destination);
            }
            Debug.Log(CarNumber + " Vertices: " + verticesCount + " Edges: " + result.Length);
            */
            return result;
        }

        //Sub-Func: function to find minimum key
        private static int MinKey(float[] key, bool[] set, int verticesCount)
        {
            float min = float.MaxValue;
            int minIndex = 0;

            for (int v = 0; v < verticesCount; ++v)
            {
                if (set[v] == false && key[v] < min)
                {
                    min = key[v];
                    minIndex = v;
                }
            }

            return minIndex;
        }


        // MAIN FUNC: Kurskal Algorithm to find minimum spanning tree - NOT USED
        public Edge[] Kruskal_STC(GraphSTC graph)
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


        // MAIN FUNC: To create obstacle space
        private void CreateObstacleSpace()
        {
            //Obstacle Sapce
            Quaternion carRotation = m_Car.transform.rotation;
            m_Car.transform.rotation = Quaternion.identity;
            ObstacleSpace = new CarConfigSpace();
            BoxCollider carCollider = GameObject.Find("ColliderBottom").GetComponent<BoxCollider>();
            ObstacleSpace.BoxSize = carCollider.transform.TransformVector(carCollider.size);
            m_Car.transform.rotation = carRotation;
        }


        //GIAN DRIVE VERSION

        public int gianDriveCar(List<Vector3> player_path, CarController player_Car, int player_pathIndex) {


            handbrake = 0;
            Vector3 player_waypoint = player_path[player_pathIndex];

            float current_angle = (360 - transform.eulerAngles.y + 90) % 360;
            Debug.Log("Current angle: " + current_angle);

            float targetDistanceMargin = (float)Math.Sqrt(graph.x_unit * graph.x_unit * +graph.z_unit * graph.z_unit) / 2;
            Vector3 nextPosition = new Vector3(player_waypoint.x, transform.position.y, player_waypoint.z);
            targetDistanceMargin = Vector3.Distance(nextPosition, player_pathIndex==0 ? transform.position : new Vector3(player_path[player_pathIndex-1].x, transform.position.y, player_path[player_pathIndex - 1].z))/2;
            targetDistanceMargin = 5f;
            Debug.Log("Next position is " + nextPosition);
            targetPosition = nextPosition;
            //float distanceToTarget = Vector3.Distance(transform.position, targetPosition);

            if (player_pathIndex != player_path.Count) {
                //get_closest_node(transform.position, final_path, nodeNumber) <= nodeNumber + 1 && distanceToTarget > targetDistanceMargin && !in_the_same_cell(transform.position, targetPosition, graph) && stop == 50)
                
                    SetAccelerationSteering(heading_steps: 0);
                    avoid_obstacles(range: graph.x_unit/2);
                    Debug.Log("Acceleration is set to " + accelerationAmount);
                    Debug.Log("Steering is set to " + steeringAmount);
                    Debug.Log("Footbrake is set to:" + footbrake);

                Debug.Log("Speed:" + m_Car.CurrentSpeed);

                m_Car.Move(steeringAmount, accelerationAmount, footbrake, handbrake);

                }
            if (Vector3.Distance(player_Car.transform.position, targetPosition) <= targetDistanceMargin)
            {
                player_pathIndex = Mathf.Min(player_pathIndex + 1, player_path.Count - 1);
            }

            return player_pathIndex;
        }

        public void SetAccelerationSteering(float current_heading = 0, float lookahead_heading = 0, int heading_steps = 0)
        {
            float max_speed = 65;
            Vector3 directionToMove = (this.targetPosition - transform.position).normalized;
            float dot = Vector3.Dot(transform.forward, directionToMove);
            float steeringAngle = Vector3.SignedAngle(transform.forward, directionToMove, Vector3.up);
            this.steeringAmount = steeringAngle / m_Car.m_MaximumSteerAngle;
            float safe_steering = Math.Abs(this.steeringAmount) > 0.5 ? 0.7f : 1;

            max_speed *= (float)Math.Exp(-heading_steps / 2);


            if (dot >= 0)
            {

                this.accelerationAmount = (max_speed - m_Car.CurrentSpeed) / max_speed * safe_steering;
                this.footbrake = 0;
                if (m_Car.CurrentSpeed >= max_speed)
                    this.footbrake = (m_Car.CurrentSpeed - max_speed) / 10;
            }
            else
            {
                this.accelerationAmount = 0f; 
                this.footbrake = -1f;
            }

        }



        // MAIN FUNC: Car Drive
        public int DriveCar(List<Vector3> player_path, CarController player_Car, int player_pathIndex)
        {
            Vector3 player_waypoint = player_path[player_pathIndex];
            float steeringAmount, accelerationAmount, car_steer_next;

            //find steering needed to get to next point
            steeringAmount = Steer(player_Car.transform.position, player_Car.transform.eulerAngles.y, player_waypoint);
            accelerationAmount = Accelerate(player_Car.transform.position, player_Car.transform.eulerAngles.y, player_waypoint);
            car_steer_next = Steer(player_Car.transform.position, player_Car.transform.eulerAngles.y, player_path[(player_pathIndex + 1) % player_path.Count]);
            // Do opposite turn if next next steer is in opposte
            if (Math.Abs(car_steer_next) > 0.3f && Math.Abs(steeringAmount) < 0.3f)
            {
                steeringAmount = -car_steer_next;
            }

            // DRIVE BLOCK
            if (!PlayerCrashed)
            {
                driveTimer += Time.deltaTime;

                //Check if car stuck and not at starting pointing
                if (driveTimer >= stuckTimer && Vector3.Distance(start_pos, player_Car.transform.position) > 0 && path_index > 10)
                {
                    driveTimer = 0;
                    //Debug.Log("Crash pos: " + Vector3.Distance(previous_pos, player_Car.transform.position));
                    //check: car not moved very much from previous position
                    if (Vector3.Distance(previous_pos, player_Car.transform.position) < 0.05f)
                    {
                        PlayerCrashed = true;
                        // check if obstacle in front of car
                        if (player_Car.CurrentSpeed<0.5f && Physics.BoxCast(player_Car.transform.position,
                            new Vector3(ObstacleSpace.BoxSize.x / 2, ObstacleSpace.BoxSize.y / 2, 0.5f),
                            player_Car.transform.forward,
                            Quaternion.LookRotation(player_Car.transform.forward),
                            ObstacleSpace.BoxSize.z / 2))
                        {
                            car_status = car_state.front_crash;
                        }
                        else if(player_Car.CurrentSpeed < 0.5f)
                        {
                            car_status = car_state.back_crash;
                        }
                        /*
                        else if(Physics.BoxCast(player_Car.transform.position,
                            new Vector3(ObstacleSpace.BoxSize.x / 2, ObstacleSpace.BoxSize.y / 2, 0.5f),
                            Quaternion.AngleAxis(0, Vector3.down) * player_Car.transform.forward,
                            Quaternion.LookRotation(Quaternion.AngleAxis(0, Vector3.down) * player_Car.transform.forward),
                            ObstacleSpace.BoxSize.z / 2))
                        {
                            car_status = car_state.back_crash;
                        }
                        else
                        {
                            PlayerCrashed = false;
                            car_status = car_state.drive;
                        }*/
                    }
                    else
                    {
                        // update previous car position
                        previous_pos = player_Car.transform.position;
                    }
                }
                float old_steering = steeringAmount;
                avoid_obstacles();
                if (old_steering != steeringAmount)
                {
                    Debug.Log("Crashed, old steering: " + old_steering + " new steering: " + steeringAmount);
                }
                // if current speed is max, then don't accelerate
                if (player_Car.CurrentSpeed >= max_speed)
                {
                    accelerationAmount = 0;
                }
                // if acceleration is reverse, apply backwards turns
                if (accelerationAmount < 0)
                {
                    player_Car.Move(-steeringAmount, footbrake, accelerationAmount * acceleration, 0);
                }
                else
                {
                    player_Car.Move(steeringAmount, accelerationAmount * acceleration, 0, 0);
                }

            }
            // CRASH RECOVERY BLOCK
            else
            {
                stuckTimer += Time.deltaTime;

                // immediately drive away from crash position
                if (stuckTimer <= recoveryTime && Math.Abs(steeringAmount) > recoverySteer)
                {      
                    //reverse car if crashed in front
                    if(car_status == car_state.front_crash)
                    {
                        //Debug.Log("Front Crash recovery logic " + "Steer: " + car_steer + "Time: " + stuckTimer);
                        player_Car.Move(-steeringAmount, 0, -1, 0);
                        // switch up crash direction if recovery delayed
                        if(stuckTimer > recoveryTime/2 && PlayerCrashed)
                        {
                            car_status = car_state.back_crash;
                            //Debug.Log("Back Crash recovery logic " + "Steer: " + car_steer + "Time: " + stuckTimer);
                            player_Car.Move(steeringAmount, 1, 0, 0);
                        }
                    }
                    //drive forward if crashed while reverse drive
                    else
                    {
                        //Debug.Log("Back Crash recovery logic " + "Steer: " + car_steer + "Time: " + stuckTimer);
                        player_Car.Move(steeringAmount, 1, 0, 0);
                    }
                }
                else
                {
                    // reset to drive mode from recovery
                    stuckTimer = 0;
                    car_status = car_state.drive;
                    PlayerCrashed = false;
                }
            }
            
            //update to new path index
            if (Vector3.Distance(player_Car.transform.position, player_waypoint) <= 5f)
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


        private List<float> DoDistanceSensing(float maxRange, float angular_spread = Mathf.PI / 6, bool lasers = false, bool verboseMode = false)
        {
            // Currently using 8 range finders: forward, backward, Right and Left
            // Angular spread defines the angle in radian from the forward/backward axis
            // The transform.(forward and right) directions can sometimes contain an unwanted y component
            var trueForward = new Vector3(transform.forward.x, 0, transform.forward.z);
            var trueRight = new Vector3(transform.right.x, 0, transform.right.z);
            var newDirection1 = Vector3.RotateTowards(trueForward, trueRight, angular_spread, 0f);
            var newDirection2 = Vector3.RotateTowards(trueForward, trueRight, -angular_spread, 0f);

            Vector3[] lookDirections = new Vector3[]
            {
                trueForward, -1 * trueForward,
                trueRight, -1 * trueRight,
                newDirection1, -1 * newDirection1,
                newDirection2, -1 * newDirection2
            };

            List<float> rangeResults = new List<float>();

            foreach (var laser_vect in lookDirections)
            {
                RaycastHit hit;
                if (Physics.Raycast(transform.position, laser_vect, out hit, maxRange) && hit.collider.gameObject.name == "Cube")
                {
                    Vector3 closestObstacleInFront = laser_vect * (hit.distance - 1f);
                    if (lasers)
                    {
                        Debug.DrawRay(transform.position, closestObstacleInFront, Color.yellow);
                    }

                    // Add range reading to the list of results
                    rangeResults.Add(hit.distance);
                }
                else
                {
                    // Add negative (out of range) reading to results
                    rangeResults.Add(-1f);
                }
            }

            if (verboseMode)
            {
                Debug.Log("Forward: " + rangeResults[0] + " Left: " + rangeResults[3] + " Angled: " + rangeResults[4]);
            }

            return rangeResults;
        }


        private void avoid_obstacles_FSM(float maxRange)
        {
            List<float> hit = DoDistanceSensing(maxRange, lasers: true);
            bool turning = Mathf.Abs(steeringAmount) >= 0.2f;
            bool turning_right = steeringAmount >= 0.2f;
            bool turning_left = steeringAmount <= -0.2f;
            bool driving_forward = Vector3.Dot(new Vector3(transform.forward.x, 0, transform.forward.z), rigidbody.velocity)>0;

            if(driving_forward && !turning)
            {
                switch (car_status_gian) //update status
                {
                    case car_detection_state.no_detection:
                        if (hit[0] != -1 || hit[4] != -1 || hit[6] != -1)
                            car_status_gian = car_detection_state.frontal_detection;
                        break;
                    case car_detection_state.frontal_detection:
                        if (hit[0] == -1 && hit[4] == -1 && hit[6] == -1)
                            car_status_gian = car_detection_state.no_detection;
                        else if (m_Car.CurrentSpeed < 0.5f)
                        {
                            car_status_gian = car_detection_state.frontal_crash;
                            steeringAmount *= -1;
                        }
                        break;
                    case car_detection_state.frontal_crash:
                        if (hit[0] == -1 && hit[4] == -1 && hit[6] == -1)
                            car_status_gian = car_detection_state.no_detection;
                        break;
                }

                switch (car_status_gian)
                {
                    case car_detection_state.frontal_detection:
                        if (hit[0] != -1)
                        {
                            accelerationAmount *= 0.5f;
                            footbrake += 0.3f;
                        }
                        if (hit[4] != -1)
                        {
                            accelerationAmount *= 0.7f;
                            footbrake += 0.15f;
                            steeringAmount -= 0.4f;
                        }
                        if (hit[6] != -1)
                        {
                            accelerationAmount *= 0.7f;
                            footbrake += 0.15f;
                            steeringAmount += 0.4f;
                        }

                        break;
                    case car_detection_state.frontal_crash:
                        accelerationAmount = 0;
                        footbrake = -1; //drive backward, steering angle set when the collision is detected
                        break;
                }

            }

            if(driving_forward && turning_right)
            {
                switch (car_status_gian) //update status
                {
                    case car_detection_state.no_detection:
                        if (hit[2] != -1 && hit[4] != -1)
                            car_status_gian = car_detection_state.frontal_detection;
                        break;
                    case car_detection_state.frontal_detection:
                        if (hit[0] != -1 && hit[4] != -1 && hit[6] != -1)
                            car_status_gian = car_detection_state.no_detection;
                        else if (m_Car.CurrentSpeed < 0.5f)
                        {
                            car_status_gian = car_detection_state.frontal_crash;
                            steeringAmount *= -1;
                        }
                        break;
                    case car_detection_state.frontal_crash:
                        if (hit[0] != -1 && hit[4] != -1 && hit[6] != -1)
                            car_status_gian = car_detection_state.no_detection;
                        break;
                }

                switch (car_status_gian)
                {
                    case car_detection_state.frontal_detection:
                        if (hit[0] != -1)
                        {
                            accelerationAmount *= 0.5f;
                            footbrake += 0.3f;
                        }
                        if (hit[4] != -1)
                        {
                            accelerationAmount *= 0.7f;
                            footbrake += 0.15f;
                            steeringAmount -= 0.4f;
                        }
                        if (hit[6] != -1)
                        {
                            accelerationAmount *= 0.7f;
                            footbrake += 0.15f;
                            steeringAmount += 0.4f;
                        }

                        break;
                    case car_detection_state.frontal_crash:
                        accelerationAmount = 0;
                        footbrake = -1; //drive backward, steering angle set when the collision is detected
                        break;
                }
            }




        }


        private void avoid_obstacles(bool curve_approaching = false, float range=5f)
        {
            RaycastHit hit;
            Vector3 maxRange = new Vector3(range, 0, range);
            bool had_hit = false;
            bool had_hit_frontally = false;


            if (!had_hit_backward && Physics.Raycast(transform.position + transform.up, transform.TransformDirection(Vector3.forward), out hit, maxRange.z) && hit.collider.gameObject.name == "Cube")
            {
                Vector3 closestObstacleInFront = transform.TransformDirection(Vector3.forward) * hit.distance;
                Debug.DrawRay(transform.position, closestObstacleInFront, Color.yellow);
                this.accelerationAmount *= 0.5f;
                this.footbrake = this.footbrake < 0.1f ? 0.5f : this.footbrake * 2;
                Debug.Log("Frontal collision, distance: " + hit.distance);
                had_hit = true;
                had_hit_frontally = true;

                if (hit.distance < 5 && m_Car.CurrentSpeed<1) //recovery from frontal hit
                {
                    Debug.Log("Collision STOP");
                    this.accelerationAmount = 0;
                    this.footbrake = -1;
                    this.steeringAmount *= -1;
                    this.handbrake = 0;
                }
            }

            /*if (Physics.Raycast(transform.position + transform.up, transform.TransformDirection(Vector3.back), out hit, maxRange.z))
            {
                Vector3 closestObstacleInFront = transform.TransformDirection(Vector3.forward) * hit.distance;
                Debug.DrawRay(transform.position, closestObstacleInFront, Color.yellow);
                this.accelerationAmount = 1;
                this.footbrake = 0;
                Debug.Log("Back collision");
                had_hit = true;
            }*/

            if (!had_hit_frontally && Physics.Raycast(transform.position + transform.right, transform.TransformDirection(Vector3.right), out hit, maxRange.x) && hit.collider.gameObject.name == "Cube")
                //Physics.Raycast(transform.position + transform.right, transform.TransformDirection(Vector3.up + Vector3.right), out hit, Mathf.Sqrt(maxRange.x * maxRange.x + maxRange.y * maxRange.y)))
            {
                
                Vector3 closestObstacleInFront = transform.TransformDirection(Vector3.forward) * hit.distance;
                Debug.DrawRay(transform.position, closestObstacleInFront, Color.yellow);
                this.accelerationAmount *= 0.7f;
                this.footbrake = this.footbrake < 0.1f ? 0.3f : this.footbrake * 1.5f;
                this.steeringAmount += -0.5f;
                Debug.Log("Right collision");
                had_hit = true;

                
            }

            if (!had_hit_frontally && Physics.Raycast(transform.position+transform.right + transform.up, transform.TransformDirection(new Vector3(1,0,1)), out hit, Mathf.Sqrt(maxRange.x * maxRange.x + maxRange.z * maxRange.z)/3f) && hit.collider.gameObject.name == "Cube")
            {
                

                Vector3 closestObstacleInFront = transform.TransformDirection(new Vector3(1, 0, 1)) * hit.distance;
                Debug.DrawRay(transform.position, closestObstacleInFront, Color.red);
                this.accelerationAmount *= 0.7f;
                this.footbrake = this.footbrake < 0.1f ? 0.3f : this.footbrake * 1.5f;
                this.steeringAmount += -0.5f;
                Debug.Log("Right-up collision " + hit.collider.gameObject.name);
                had_hit = true;
                had_hit_frontally = true;



            }

            if (!had_hit_frontally && Physics.Raycast(transform.position + transform.right + transform.up, transform.TransformDirection(new Vector3(-1, 0, 1)), out hit, Mathf.Sqrt(maxRange.x * maxRange.x + maxRange.z * maxRange.z) / 3f) && hit.collider.gameObject.name == "Cube")
            {


                Vector3 closestObstacleInFront = transform.TransformDirection(new Vector3(-1, 0, 1)) * hit.distance;
                Debug.DrawRay(transform.position, closestObstacleInFront, Color.red);
                this.accelerationAmount *= 0.7f;
                this.footbrake = this.footbrake < 0.1f ? 0.3f : this.footbrake * 1.5f;
                this.steeringAmount += 0.5f;
                Debug.Log("left-up collision " + hit.collider.gameObject.name);
                had_hit = true;
                had_hit_frontally = true;



            }

            if (!had_hit_frontally && Physics.Raycast(transform.position + transform.right, transform.TransformDirection(Vector3.left), out hit, maxRange.x) && hit.collider.gameObject.name == "Cube")
                //Physics.Raycast(transform.position + transform.right, transform.TransformDirection((Vector3.up + Vector3.left).normalized), out hit, Mathf.Sqrt(maxRange.x * maxRange.x + maxRange.y* maxRange.y)))
            {
                Vector3 closestObstacleInFront = transform.TransformDirection(Vector3.forward) * hit.distance;
                Debug.DrawRay(transform.position, closestObstacleInFront, Color.yellow);
                this.accelerationAmount *= 0.7f;
                this.footbrake = this.footbrake < 0.1f ? 0.3f : this.footbrake * 1.5f;
                this.steeringAmount += 0.5f;
                Debug.Log("Left collision");

                had_hit = true;
            }

            if (!had_hit && !curve_approaching)
            {
                this.accelerationAmount *= 1.25f;
                Debug.Log("Not hit speed");
            }

            if (!had_hit && m_Car.CurrentSpeed < 1f || had_hit_backward)
            {
                had_hit_backward = true;
                this.accelerationAmount = 1;
                this.footbrake = 0;
                this.handbrake = 0;
                this.steeringAmount *= 1;
                if (m_Car.CurrentSpeed > 10f)
                    had_hit_backward = false;
            }


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
                for (int i = path_index; i < my_path.Count - 1; ++i)
                {
                    Gizmos.color = colors[i/2 % colors.Length];
                    Gizmos.DrawLine(my_path[i], my_path[i + 1]);
                }
            }


            

            
        }
        private Orientation get_initial_orientation(Vector3 initial_pos, Vector3 arriving_pos)
        {
            Vector3 direction = (arriving_pos - initial_pos).normalized;

            float acos = Mathf.Acos(direction.x); //between 0 and 180
            if (direction.z < 0)
            {
                acos = 2 * Mathf.PI - acos;
            }

            acos = Mathf.Rad2Deg * acos;

            int direction_angle = (int)acos / 45;
            Debug.Log("Angle: " + acos + "Direction: " + direction_angle);


            switch (direction_angle)
            {
                case 0:
                case 7:
                    return Orientation.R;
                case 1:
                case 2:
                    return Orientation.UU;
                case 3:
                case 4:
                    return Orientation.L;
                case 5:
                case 6:
                    return Orientation.DD;
                default:
                    return Orientation.UU;
            }

        }


    }
}
