using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;


namespace UnityStandardAssets.Vehicles.Car
{
    [RequireComponent(typeof(CarController))]
    public class CarAI2 : MonoBehaviour
    {
        private CarController m_Car; // the car controller we want to use

        public GameObject terrain_manager_game_object;
        TerrainManager terrain_manager;

        public GameObject[] friends;
        public GameObject[] enemies;
        public int CarNumber;


        private MergedNode[,] merged_nodes_v;
        private MergedNode[,] merged_nodes_h;
        private MergedNode[,] merged_nodes_final;

        // Variables for Terrain
        public Graph graph;
        public Graph original_graph;
        public Graph subgraph;
        public DARP_controller darp;

        //path variables
        List<Vector3> path = new List<Vector3>();
        List<MergedNode> node_path = new List<MergedNode>();
        private Node starting_node;
        private List<Node> path_to_starting_node = new List<Node>();
        private List<Node> astar_path = new List<Node>();
        private List<Vector3> astar_path_points = new List<Vector3>();

        //driving variables
        private float mazeTimer=0;
        private bool MazeComplete = false;
        private int path_index = 0;
        private bool backward_crash = false;
        private bool frontal_crash = false;

        private float accelerationAmount = 0;
        private float footbrake = 0;
        private float steeringAmount = 0;
        private float handbrake = 0;
        private Vector3 targetPosition;

        private void Start()
        {
            Time.timeScale = 5;
            int gunRange = 10;
            // get the car controller
            m_Car = GetComponent<CarController>();
            terrain_manager = terrain_manager_game_object.GetComponent<TerrainManager>();


            // note that both arrays will have holes when objects are destroyed
            // but for initial planning they should work
            friends = GameObject.FindGameObjectsWithTag("Player");
            // Note that you are not allowed to check the positions of the turrets in this problem

            // Get Array of Friends and Eniemies
            friends = GameObject.FindGameObjectsWithTag("Player");
            Vector3[] initial_positions = new Vector3[friends.Length];
            int i = 0;
            foreach (GameObject friend in friends)
            {
                if (friend.name == this.name)
                {
                    CarNumber = i + 1;
                    if (CarNumber == 1)
                        CarNumber = 3;
                    else if (CarNumber == 3)
                        CarNumber = 1;

                }
                //if (CarNumber == 1) Debug.Log(friend + " position: " + friend.gameObject.transform.position);
                initial_positions[i] = friend.gameObject.transform.position;
                i++;

            }
            


            int x_scale = terrain_manager.myInfo.x_N;
            int z_scale = terrain_manager.myInfo.z_N;
            float x_len = (terrain_manager.myInfo.x_high - terrain_manager.myInfo.x_low);
            float z_len = (terrain_manager.myInfo.z_high - terrain_manager.myInfo.z_low);
            float x_unit = 1.4142f * gunRange;
            float z_unit = x_unit;
            x_scale = x_scale * ((int)(x_len / x_unit) / x_scale);
            z_scale = z_scale * ((int)(z_len / z_unit) / z_scale);
            //i want x_unit and z_unit to be √2r, where r is the range of the gun.
            //but i also want the new scales them to be a multiple of the original x_scale and z_scale            
            graph = Graph.CreateGraph(terrain_manager.myInfo, x_scale, z_scale);
            original_graph = Graph.CreateGraph(terrain_manager.myInfo, x_scale, z_scale);
            darp = new DARP_controller(friends.Length, initial_positions, graph, 0.0004f, 100);
            subgraph = Graph.CreateSubGraph(graph, CarNumber, terrain_manager.myInfo, x_scale, z_scale);
            darp.update_assigned_nodes(original_graph);
            starting_node = PathFinder.get_starting_node(transform.position, CarNumber, original_graph, subgraph, (360 - transform.eulerAngles.y + 90) % 360, ref path_to_starting_node);

            int upsampling_factor = 4;
            if (path_to_starting_node.Count > 0)
            {
                path_to_starting_node = PathFinder.pathUpsampling(path_to_starting_node, upsampling_factor);
                path_to_starting_node = PathFinder.pathSmoothing(path_to_starting_node, 0.6f, 0.2f, 1E-09f); //Now the path is ready to be trasversed
            }

            /*subgraph = Graph.CreateSubGraph(graph, CarNumber, terrain_manager.myInfo, x_scale, z_scale, false);

            MergedGraph merged_graph = MergedGraph.fromGraph(subgraph);
            bool has_merged = merged_graph.mergeNodesVertically();
            while (has_merged) 
            {
                has_merged = merged_graph.mergeNodesVertically();
            }
            has_merged = merged_graph.mergeNodesHorizontally();
            while (has_merged)
            {
                has_merged = merged_graph.mergeNodesHorizontally();
            }
            merged_graph.compute_neighbours();
            merged_nodes_v = merged_graph.nodes;


            merged_graph = MergedGraph.fromGraph(subgraph);
            has_merged = merged_graph.mergeNodesHorizontally();
            while (has_merged)
            {
                has_merged = merged_graph.mergeNodesHorizontally();
            }

            has_merged = merged_graph.mergeNodesVertically();
            while (has_merged)
            {
                has_merged = merged_graph.mergeNodesVertically();
            }
            merged_graph.compute_neighbours();

            merged_nodes_h = merged_graph.nodes;
            */

            MergedGraph merged_graph = MergedGraph.fromGraph(subgraph);
            bool has_merged = merged_graph.mergeNodes();
            while (has_merged)
            {
                has_merged = merged_graph.mergeNodes();
            }
            merged_nodes_final = merged_graph.nodes;
            merged_graph.compute_neighbours();


            Vector3 current_position = initial_positions[CarNumber-1];
            int curr_i = terrain_manager.myInfo.get_i_index(current_position.x);
            int curr_j = terrain_manager.myInfo.get_j_index(current_position.z);
            MergedNode current = merged_nodes_final[starting_node.i, starting_node.j];
            //MergedNode current = merged_nodes_final[curr_i, curr_j];

            MergedNode next;
            int safe_exit = 1000;
            while (current!=null && safe_exit>0)
            {
                safe_exit--;
                path.Add(current_position);
                node_path.Add(current);
                current.seen = true;

                next = current.getClosestUnseen(current_position);
                if(next == current)
                {
                    next = merged_graph.get_closest_unseen_globally(current);
                }
                if (next != null)
                {
                    current_position = next.getClosestPoint(current_position);
                }
                current = next;
            }

            //at this point, in path, we have a list of points. Retrieve the node from each point and compute the A* path between them
            //compute a more precise graph for driving
            original_graph = Graph.CreateGraph(terrain_manager.myInfo, x_scale*2, z_scale*2);
            current_position = initial_positions[CarNumber - 1];
            float heading = (360 - transform.eulerAngles.y + 90) % 360;
            foreach (Vector3 next_node in path)
            {
                //Node astar_next = graph.getNodeFromPoint(next_node);
                List<Node> path_to_next = PathFinder.findAstarPath(original_graph, current_position, next_node, heading);
                heading = path_to_next[path_to_next.Count - 1].heading;
                current_position = next_node;
                astar_path.AddRange(path_to_next);
            }

            astar_path = PathFinder.pathUpsampling(astar_path, 4);
            astar_path = PathFinder.pathSmoothing(astar_path, 0.6f, 0.2f, 1E-09f);

            foreach (Node node in astar_path)
                astar_path_points.Add(node.worldPosition);
        }


        private void FixedUpdate()
        {
            mazeTimer += Time.deltaTime;
            enemies = GameObject.FindGameObjectsWithTag("Enemy");
            if (CarNumber == 1) Debug.Log("Enemies remaining: " + enemies.Length + " Time Elapsed: " + mazeTimer);

            // Drive Car
            if (!MazeComplete)
            {
                //path_index = DriveCar(my_path, m_Car, path_index);
                path_index = gianDriveCar(astar_path_points, m_Car, path_index);
                //Check if all enemies killed for maze completion
                if (enemies.Length <= 0 || path_index >= astar_path.Count)
                {
                    MazeComplete = true;
                    if (CarNumber == 1) Debug.Log("Car " + CarNumber + "Path Completed in " + mazeTimer);
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
                if (CarNumber == 1) Debug.Log("All enemies killed in " + mazeTimer + " Exiting Game...");
                UnityEditor.EditorApplication.isPlaying = false;
            }


        }

        public int gianDriveCar(List<Vector3> player_path, CarController player_Car, int player_pathIndex)
        {


            handbrake = 0;
            Vector3 player_waypoint = player_path[player_pathIndex];

            float current_angle = (360 - transform.eulerAngles.y + 90) % 360;
            //if (CarNumber == 1) Debug.Log("Current angle: " + current_angle);

            float targetDistanceMargin = (float)Math.Sqrt(graph.x_unit * graph.x_unit * +graph.z_unit * graph.z_unit) / 2;
            Vector3 nextPosition = new Vector3(player_waypoint.x, transform.position.y, player_waypoint.z);
            targetDistanceMargin = Vector3.Distance(nextPosition, player_pathIndex == 0 ? transform.position : new Vector3(player_path[player_pathIndex - 1].x, transform.position.y, player_path[player_pathIndex - 1].z)) / 2;
            targetDistanceMargin = graph.x_unit / 2.5f;
            //if (CarNumber == 1) { Debug.Log("Next position is " + nextPosition + "Path index: " + player_pathIndex + " Path count: " + player_path.Count);}
            targetPosition = nextPosition;
            //float distanceToTarget = Vector3.Distance(transform.position, targetPosition);

            if (player_pathIndex != player_path.Count)
            {
                //get_closest_node(transform.position, final_path, nodeNumber) <= nodeNumber + 1 && distanceToTarget > targetDistanceMargin && !in_the_same_cell(transform.position, targetPosition, graph) && stop == 50)

                SetAccelerationSteering(heading_steps: 0);
                avoid_obstacles(range: graph.x_unit / 3);
                if (CarNumber == 1)
                {
                    Debug.Log("Acceleration is set to " + accelerationAmount);
                    Debug.Log("Steering is set to " + steeringAmount);
                    Debug.Log("Footbrake is set to:" + footbrake);
                    Debug.Log("Speed:" + m_Car.CurrentSpeed);
                }

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
            float max_speed = 500;
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

        private void avoid_obstacles(bool curve_approaching = false, float range = 5f)
        {
            RaycastHit hit;
            Vector3 maxRange = new Vector3(range, 0, range / 1.5f);
            bool had_hit = false;
            bool had_hit_frontally = false;

            if (CarNumber == 1)
            {
                Debug.Log("Backward crash: " + backward_crash);

                Debug.Log("Frontal  crash: " + frontal_crash);
            }
            if (frontal_crash && Physics.Raycast(transform.position + transform.up, transform.TransformDirection(Vector3.forward), out hit, maxRange.z) && hit.collider.gameObject.name == "Cube")
            {
                this.accelerationAmount = 0;
                this.footbrake = -1;
                this.steeringAmount *= -1;

                return;
            }
            else frontal_crash = false; //stays in the frontal crash state driving backward until there is no obstacle in front of the car

            if (backward_crash && m_Car.CurrentSpeed < 5f)
            {
                this.accelerationAmount = 1;
                this.footbrake = 0;
                return;
            }
            else backward_crash = false;

            if (m_Car.CurrentSpeed < 5f)
            {
                this.accelerationAmount += 0.5f;
            }

            if (Physics.Raycast(transform.position + transform.up, transform.TransformDirection(Vector3.forward), out hit, maxRange.z) && hit.collider.gameObject.name == "Cube")
            {
                Vector3 closestObstacleInFront = transform.TransformDirection(Vector3.forward) * hit.distance;
                Debug.DrawRay(transform.position, closestObstacleInFront, Color.yellow);
                this.accelerationAmount *= 0.5f;
                this.footbrake = this.footbrake < 0.2f ? 0.5f : this.footbrake * 2;
                if (CarNumber == 1) Debug.Log("Frontal collision, distance: " + hit.distance);
                had_hit = true;
                had_hit_frontally = true;

                if (hit.distance < 5 && m_Car.CurrentSpeed < 1) //recovery from frontal hit
                {
                    frontal_crash = true;
                    if (CarNumber == 1) Debug.Log("Collision STOP");
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
                if (CarNumber == 1) Debug.Log("Back collision");
                had_hit = true;
            }*/

            if (Physics.Raycast(transform.position + transform.right, transform.TransformDirection(Vector3.right), out hit, maxRange.x) && hit.collider.gameObject.name == "Cube")
            //Physics.Raycast(transform.position + transform.right, transform.TransformDirection(Vector3.up + Vector3.right), out hit, Mathf.Sqrt(maxRange.x * maxRange.x + maxRange.y * maxRange.y)))
            {

                Vector3 closestObstacleInFront = transform.TransformDirection(Vector3.forward) * hit.distance;
                Debug.DrawRay(transform.position, closestObstacleInFront, Color.yellow);
                this.accelerationAmount *= 0.7f;
                this.footbrake = this.footbrake < 0.1f ? 0.3f : this.footbrake * 1.5f;
                this.steeringAmount += -0.5f;
                if (CarNumber == 2)
                    if (CarNumber == 1) Debug.Log("Right collision");
                had_hit = true;


            }

            if (Physics.Raycast(transform.position + transform.right + transform.up, transform.TransformDirection(new Vector3(1, 0, 1)), out hit, Mathf.Sqrt(maxRange.x * maxRange.x + maxRange.z * maxRange.z) / 3f) && hit.collider.gameObject.name == "Cube")
            {
                Vector3 closestObstacleInFront = transform.TransformDirection(new Vector3(1, 0, 1)) * hit.distance;
                Debug.DrawRay(transform.position, closestObstacleInFront, Color.red);
                this.accelerationAmount *= 0.7f;
                this.footbrake = this.footbrake < 0.1f ? 0.3f : this.footbrake * 1.5f;
                this.steeringAmount += -0.5f;
                if (CarNumber == 2)
                    if (CarNumber == 1) Debug.Log("Right-up collision " + hit.collider.gameObject.name);
                had_hit = true;
                had_hit_frontally = true;



            }

            if (Physics.Raycast(transform.position + transform.right + transform.up, transform.TransformDirection(new Vector3(-1, 0, 1)), out hit, Mathf.Sqrt(maxRange.x * maxRange.x + maxRange.z * maxRange.z) / 3f) && hit.collider.gameObject.name == "Cube")
            {


                Vector3 closestObstacleInFront = transform.TransformDirection(new Vector3(-1, 0, 1)) * hit.distance;
                Debug.DrawRay(transform.position, closestObstacleInFront, Color.red);
                this.accelerationAmount *= 0.7f;
                this.footbrake = this.footbrake < 0.1f ? 0.3f : this.footbrake * 1.5f;
                this.steeringAmount += 0.5f;
                if (CarNumber == 2)
                    if (CarNumber == 1) Debug.Log("left-up collision " + hit.collider.gameObject.name);
                had_hit = true;
                had_hit_frontally = true;



            }

            if (Physics.Raycast(transform.position + transform.right, transform.TransformDirection(Vector3.left), out hit, maxRange.x) && hit.collider.gameObject.name == "Cube")
            //Physics.Raycast(transform.position + transform.right, transform.TransformDirection((Vector3.up + Vector3.left).normalized), out hit, Mathf.Sqrt(maxRange.x * maxRange.x + maxRange.y* maxRange.y)))
            {
                Vector3 closestObstacleInFront = transform.TransformDirection(Vector3.forward) * hit.distance;
                Debug.DrawRay(transform.position, closestObstacleInFront, Color.yellow);
                this.accelerationAmount *= 0.7f;
                this.footbrake = this.footbrake < 0.1f ? 0.3f : this.footbrake * 1.5f;
                this.steeringAmount += 0.5f;
                if (CarNumber == 2)
                    if (CarNumber == 1) Debug.Log("Left collision");

                had_hit = true;
            }

            if (!had_hit && !curve_approaching)
            {
                this.accelerationAmount *= 1.25f;
                if (CarNumber == 2)
                    if (CarNumber == 1) Debug.Log("Not hit speed");
            }

            if (!had_hit && m_Car.CurrentSpeed < 1f)
            {
                if (CarNumber == 1) Debug.Log("Had hit backward");
                backward_crash = true;
                this.accelerationAmount = 1;
                this.footbrake = 0;
                this.handbrake = 0;
                this.steeringAmount *= 1;
            }

            if (had_hit && m_Car.CurrentSpeed < 1f)
            {
                frontal_crash = true;
                this.accelerationAmount = 0;
                this.footbrake = -1;
                this.handbrake = 0;
                this.steeringAmount *= -1;

            }


        }


        void OnDrawGizmos() // draws grid on map and shows car
        {
            Color[] colors = { Color.red, Color.cyan, Color.yellow, Color.white, Color.black, Color.green };
            Color color = colors[CarNumber];
            Gizmos.color = color;
            for(int i = 0; i< merged_nodes_final.GetLength(0); i++)
            {
                for (int j = 0; j < merged_nodes_final.GetLength(1); j++)
                {
                    //MergedNode node = merged_nodes_v[i, j];
                    MergedNode node = merged_nodes_final[i, j];
                    if (node != null && i == node.dl_i && j==node.dl_j)
                    {
                        Gizmos.color = color;
                        //Gizmos.DrawWireCube(node.position, new Vector3(node.x_len * node.x_unit, 5, node.z_len * node.z_unit));
                        Gizmos.DrawCube(node.position, new Vector3(node.x_len * node.x_unit, 5, node.z_len * node.z_unit)*0.98f);

                        /* CODE TO PRINT NEIGHBOURS
                         * if (CarNumber == 2)
                        {
                            foreach(MergedNode n in merged_nodes[5, 9].neighbours)
                            {
                                Gizmos.color = Color.blue;
                                Gizmos.DrawCube(n.position, new Vector3(n.x_len * n.x_unit, 5, n.z_len * n.z_unit) * 0.9f);
                            }
                        }
                        */

                    }
                    /*
                    node = merged_nodes_h[i, j];
                    if (node != null && i == node.dl_i && j == node.dl_j)
                    {
                        Gizmos.color = colors[CarNumber-1];
                        //Gizmos.DrawWireCube(node.position, new Vector3(node.x_len * node.x_unit, 5, node.z_len * node.z_unit));
                        Gizmos.DrawWireCube(node.position, new Vector3(node.x_len * node.x_unit, 5, node.z_len * node.z_unit) * 0.98f);
                        Gizmos.DrawWireCube(node.position, new Vector3(node.x_len * node.x_unit, 5, node.z_len * node.z_unit) * 0.97f);
                        Gizmos.DrawWireCube(node.position, new Vector3(node.x_len * node.x_unit, 5, node.z_len * node.z_unit) * 0.96f);
                        Gizmos.DrawWireCube(node.position, new Vector3(node.x_len * node.x_unit, 5, node.z_len * node.z_unit) * 0.95f);

                        /* CODE TO PRINT NEIGHBOURS
                         * if (CarNumber == 2)
                        {
                            foreach(MergedNode n in merged_nodes[5, 9].neighbours)
                            {
                                Gizmos.color = Color.blue;
                                Gizmos.DrawCube(n.position, new Vector3(n.x_len * n.x_unit, 5, n.z_len * n.z_unit) * 0.9f);
                            }
                        }
                        */
                    /*
                    }
                    */
                



                }
            }

            Gizmos.color = Color.black;
            for(int i = 0; i<path.Count-1; i++)
            {
                //Gizmos.DrawLine(path[i], path[i + 1]);
            }
            color = colors[CarNumber-1];
            Gizmos.color = color;
            int col = CarNumber-1;
            int k = 0;
            Gizmos.color = Color.red;
            for(int p = 0; p<astar_path.Count - 1; p++)
            {
                Gizmos.DrawLine(astar_path[p].worldPosition, astar_path[p + 1].worldPosition);
            }
                //foreach(List<Node> path in astar_paths)
                //{
                
                    /*foreach(Node node in astar_path)
                    {
                        Gizmos.color = colors[col % colors.Length];
                        Gizmos.DrawCube(node.worldPosition, new Vector3(original_graph.x_unit, 0, original_graph.z_unit) * 0.9f);
                        Gizmos.color = colors[col % colors.Length];
                    Gizmos.color = colors[col+2 % colors.Length];

                    if (k < astar_path.Count - 1)
                        Gizmos.DrawLine(astar_path[k].worldPosition, astar_path[k + 1].worldPosition);
                    k++;
                    }
                    */
                    //col++;
                    //if (col == 12)
                    //    break;
                    //if (col % colors.Length == CarNumber)
                    //    col++;

                }


        }
        

    }


