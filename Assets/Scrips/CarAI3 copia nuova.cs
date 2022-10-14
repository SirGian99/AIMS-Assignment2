/*using System.Collections;
using System.Collections.Generic;
using System;
using UnityEngine;
using System.IO;

namespace UnityStandardAssets.Vehicles.Car
{
    [RequireComponent(typeof(CarController))]
    public class CarAI3 : MonoBehaviour
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
        public float gunRange = 10f;

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
        enum car_state { drive, front_crash, back_crash };
        private car_state car_status;
        private List<Vector3> enemy_positions = new List<Vector3>();

        //Path matrix from one target to the other. The starting position is the last one.
        List<Node>[,] paths;
        int[,] costs;


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
            foreach (GameObject friend in friends)
            {
                if (friend.name == this.name)
                {
                    CarNumber = i + 1;
                }
                //Debug.Log(friend + " position: " + friend.gameObject.transform.position);
                initial_positions[i] = friend.gameObject.transform.position;
                i++;

            }
            enemies = GameObject.FindGameObjectsWithTag("Enemy");
            foreach (GameObject obj in enemies)
            {
                enemy_positions.Add(obj.gameObject.transform.position);
            }


            // DARP Algorithm
            // Get subgraph:
            darp = new DARP_controller(friends.Length, initial_positions, graph, 0.0004f, 100);
            subgraph = Graph.CreateSubGraph(graph, CarNumber, terrain_manager.myInfo, x_scale, z_scale);
            map = new GraphSTC(subgraph, start_pos, enemy_positions);
            graph = subgraph;
            darp.update_assigned_nodes(original_graph);

            List<Node> myEnemies = new List<Node>();

            foreach(Vector3 enemy in enemy_positions)
            {
                Node enemy_node = graph.getNodeFromPoint(enemy);
                if (enemy_node != null && enemy_node.assigned_veichle == CarNumber)
                {
                    enemy_node.worldPosition = enemy;
                    myEnemies.Add(enemy_node);
                }
            }
            int n_enemies = myEnemies.Count;
            paths = new List<Node>[n_enemies + 1, n_enemies + 1];
            costs = new int[n_enemies + 1, n_enemies + 1];
            starting_node = PathFinder.get_starting_node(transform.position, CarNumber, original_graph, graph, (360 - transform.eulerAngles.y + 90) % 360, ref path_to_starting_node);
            myEnemies.Add(starting_node);
            for(int e = 0; e<n_enemies; e++)
            {
                for (int k = e+1; k<n_enemies+1; k++)
                {
                    PathFinder.findPath(graph, myEnemies[e].worldPosition, myEnemies[k].worldPosition, get_angle(myEnemies[e].worldPosition, myEnemies[k].worldPosition));
                    paths[e, k] = graph.path;
                    paths[k, e] = new List<Node>(graph.path);
                    paths[k, e].Reverse();
                    costs[e, k] = paths[e, k].Count;
                    costs[k, e] = paths[k, e].Count;
                }
            }
            //getting the starting enemy to reach
            //fai prim e trova lo shortest path
            int closest = n_enemies;
            for(int e = 0; e < n_enemies; e++)
            {
                if (costs[n_enemies, e] != 0 && costs[n_enemies, e] < costs[n_enemies, closest])
                    closest = e;
            }
            // Get min tree:
            min_tree = Prim_STC(map, starting_node);
            min_tree = PruneTree(min_tree, enemy_positions);

            // Get path from min_tree:
            int upsampling_factor = 4;
            my_path = new List<Vector3>();
            my_path = ComputePath(map, min_tree, transform.position, starting_node, get_initial_orientation(transform.position, starting_node.worldPosition));

            //findPath(original_graph, start_position, current.worldPosition, heading);
            my_path = PathFinder.pathUpsampling(my_path, upsampling_factor);
            my_path = PathFinder.pathSmoothing(my_path, 0.6f, 0.2f, 1E-09f);

            // Get path to starting node
            if (path_to_starting_node.Count > 0)
            {
                path_to_starting_node = PathFinder.pathUpsampling(path_to_starting_node, upsampling_factor);
                path_to_starting_node = PathFinder.pathSmoothing(path_to_starting_node, 0.6f, 0.2f, 1E-09f); //Now the path is ready to be trasversed
            }

            // Combine both paths
            foreach (Node n in path_to_starting_node)
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
                path_index = DriveCar(my_path, m_Car, path_index);

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
            if (enemies.Length <= 0)
            {
                Debug.Log("All enemies killed in " + mazeTimer + " Exiting Game...");
                UnityEditor.EditorApplication.isPlaying = false;
            }
        }


        // MAIN FUNC: Compute path 
        private List<Vector3> ComputePath(GraphSTC graphSTC, Edge[] EdgeArray, Vector3 starting_position, Node starting_node, Orientation initial_orientation = Orientation.UU)
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

/* ////TODO REMOVE FOR COMMENT
 * 
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

                if (next_node == starting_node && next_node.children_to_visit == 0)
                    break;

                arriving_orientation = PathFinder.getOrientation(current_node, next_node);
                current_node = next_node;



            }

            return path;
        }


        //private List<Vector3> ComputeMinTreePath()

        // MAIN FUNC: Prims Algorithm to find minimum spanning tree
        public Edge[] Prim_STC(GraphSTC graph, Node starting_node)
        {

            int start_node = Array.IndexOf(graph.VertexArray, starting_node);
            string adjacency_string = "DEBUGG\n";

            if (CarNumber == 1)
            {

                for (int z = 0; z < graph.VertexArray.Length; z++)
                {
                    Debug.Log("DEBUGG Node " + z + ": " + graph.VertexArray[z]);
                }

                for (int i = 0; i < graph.adj_matrix.GetLength(0); i++)
                {
                    for (int j = 0; j < graph.adj_matrix.GetLength(1); j++)
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

            for (int i = 0; i < verticesCount; i++)
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
                if (!nextEdge.Source.children.Contains(nextEdge.Destination))
                    nextEdge.Source.children.Add(nextEdge.Destination);
                nextEdge.Destination.parent = nextEdge.Source;
                result[k] = nextEdge;
                min_cost = float.MaxValue;
                if (CarNumber == 2)
                    Debug.Log("Adding edge from node " + nextEdge.Source + " to " + nextEdge.Destination + " k:" + k + "Total" + (verticesCount - 1));
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

/* ////TODO REMOVE FOR COMMENT
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


        // MAIN FUNC: Prune tree branch if no enemy
        public static Edge[] PruneTree(Edge[] EdgeArray, List<Vector3> enemy_positions)
        {
            bool leaf_removed = true;
            // remove all leaves with no enemy node
            while (leaf_removed)
            {
                leaf_removed = false;
                for (int i = 0; i < EdgeArray.Length; i++)
                {
                    Edge leafEdge = EdgeArray[i];
                    // find if edge is leaf or not
                    bool isLeaf = true;
                    foreach (Edge edge in EdgeArray)
                    {
                        if (leafEdge.Destination == edge.Source)
                        {
                            isLeaf = false;
                        }
                    }
                    // find if edge has enemny node
                    bool isEnemyLeaf = false;
                    foreach (Vector3 enemy in enemy_positions)
                    {
                        if (leafEdge.Destination.is_supernode)
                        {
                            if (Vector3.Distance(enemy, leafEdge.Destination.worldPosition) <= 25f)
                            {
                                isEnemyLeaf = true;
                            }
                            //if (Vector3.Distance(enemy, leafEdge.Source.worldPosition) <= 25f)
                            //{
                            //    isEnemyLeaf = true;
                            //}
                        }
                        else
                        {
                            if (Vector3.Distance(enemy, leafEdge.Destination.worldPosition) <= 12f)
                            {
                                isEnemyLeaf = true;
                            }
                            if (Vector3.Distance(enemy, leafEdge.Source.worldPosition) <= 12f)
                            {
                                isEnemyLeaf = true;
                            }
                        }
                        // Need to account for supernode??
                    }

                    // remove if edge is not enemy node
                    if (isLeaf && !isEnemyLeaf)
                    {
                        leaf_removed = true;
                        // remove from array
                        Node destination = leafEdge.Destination;
                        Node source = leafEdge.Source;
                        if(destination.parent == source)
                        {
                            source.children.Remove(destination);
                            destination.parent = destination;
                            foreach(Node child in destination.children)
                            {
                                Debug.Log("PRUNING"+destination + " has children: " + child);
                            }
                        }
                        else if(source.parent == destination)
                        {
                            Debug.Log("PRUNING Source is the child of destination");
                            destination.children.Remove(source);
                            source.parent = source;
                            foreach (Node child in source.children)
                            {
                                Debug.Log("PRUNING"+source + " has children: " + child);
                            }
                        }
                        else
                        {
                            Debug.Log("ERROR WHILE PRUNING. Source:" + source + "\nSource parent:" + source.parent + "\nDestination:" + destination + "Dest parent:"+destination.parent);
                            foreach (Node child in source.children)
                            {
                                Debug.Log("PRUNING" + source + " has children: " + child);
                            }

                            foreach (Node child in destination.children)
                            {
                                Debug.Log("PRUNING" + destination + " has children: " + child);
                            }
                        }
                        RemoveAt(ref EdgeArray, i);
                    }
                }
            }
            return EdgeArray;
        }

        // Sub-func for removing Array element
        public static void RemoveAt<T>(ref T[] arr, int index)
        {
            for (int a = index; a < arr.Length - 1; a++)
            {
                // moving elements downwards, to fill the gap at [index]
                arr[a] = arr[a + 1];
            }
            // finally, let's decrement Array's size by one
            Array.Resize(ref arr, arr.Length - 1);
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


        // MAIN FUNC: Car Drive
        public int DriveCar(List<Vector3> player_path, CarController player_Car, int player_pathIndex)
        {
            Vector3 player_waypoint = player_path[player_pathIndex];
            float car_steer, car_acc, car_steer_next;

            //find steering needed to get to next point
            car_steer = Steer(player_Car.transform.position, player_Car.transform.eulerAngles.y, player_waypoint);
            car_acc = Accelerate(player_Car.transform.position, player_Car.transform.eulerAngles.y, player_waypoint);
            car_steer_next = Steer(player_Car.transform.position, player_Car.transform.eulerAngles.y, player_path[(player_pathIndex + 1) % player_path.Count]);
            // Do opposite turn if next next steer is in opposte
            if (Math.Abs(car_steer_next) > 0.3f && Math.Abs(car_steer) < 0.3f)
            {
                car_steer = -car_steer_next;
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
                        if (Physics.BoxCast(player_Car.transform.position,
                            new Vector3(ObstacleSpace.BoxSize.x / 2, ObstacleSpace.BoxSize.y / 2, 0.5f),
                            player_Car.transform.forward,
                            Quaternion.LookRotation(player_Car.transform.forward),
                            ObstacleSpace.BoxSize.z / 2))
                        {
                            car_status = car_state.front_crash;
                        }
                        else
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

/* ////TODO REMOVE FOR COMMENT

                    }
                    else
                    {
                        // update previous car position
                        previous_pos = player_Car.transform.position;
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
                    player_Car.Move(-car_steer, 0, car_acc * acceleration, 0);
                }
                else
                {
                    player_Car.Move(car_steer, car_acc * acceleration, 0, 0);
                }

            }
            // CRASH RECOVERY BLOCK
            else
            {
                stuckTimer += Time.deltaTime;

                // immediately drive away from crash position
                if (stuckTimer <= recoveryTime && Math.Abs(car_steer) > recoverySteer)
                {
                    //reverse car if crashed in front
                    if (car_status == car_state.front_crash)
                    {
                        //Debug.Log("Front Crash recovery logic " + "Steer: " + car_steer + "Time: " + stuckTimer);
                        player_Car.Move(-car_steer, 0, -acceleration, 0);
                        // switch up crash direction if recovery delayed
                        if (stuckTimer > recoveryTime / 2 && PlayerCrashed)
                        {
                            car_status = car_state.back_crash;
                            //Debug.Log("Back Crash recovery logic " + "Steer: " + car_steer + "Time: " + stuckTimer);
                            player_Car.Move(car_steer, acceleration, 0, 0);
                        }
                    }
                    //drive forward if crashed while reverse drive
                    else
                    {
                        //Debug.Log("Back Crash recovery logic " + "Steer: " + car_steer + "Time: " + stuckTimer);
                        player_Car.Move(car_steer, car_acc * acceleration, 0, 0);
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
                    if (n != null && (n.assigned_veichle == CarNumber || !n.walkable))
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
/* ////TODO REMOVE FOR COMMENT


                    }

                }
            }


            //Show min span tree
            if (min_tree != null)
            {
                Gizmos.color = colors[CarNumber];
                //Gizmos.color = Color.red;
                for (int i = 0; i < min_tree.Length - 1; ++i)
                {
                    Gizmos.color = colors[CarNumber];
                    if (min_tree[i].Source != null && min_tree[i].Destination != null)
                        Gizmos.DrawLine(min_tree[i].Source.worldPosition + Vector3.up, min_tree[i].Destination.worldPosition + Vector3.up);
                    ;

                }
            }


            //Show the path to the goal
            if (my_path != null)
            {
                Gizmos.color = Color.black;
                for (int i = path_index; i < my_path.Count - 1; ++i)
                {
                    Gizmos.color = colors[i / 2 % colors.Length];
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


        private float get_angle(Vector3 initial_pos, Vector3 arriving_pos)
        {
            Vector3 direction = (arriving_pos - initial_pos).normalized;

            float acos = Mathf.Acos(direction.x); //between 0 and 180
            if (direction.z < 0)
            {
                acos = 2 * Mathf.PI - acos;
            }

            acos = Mathf.Rad2Deg * acos;

            int direction_angle = ((int)acos / 45)*45;
            return direction_angle;

        }

    }
}

*////TODO REMOVE FOR COMMENT
