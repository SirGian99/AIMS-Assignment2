﻿using System.Collections;
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

        private List<Node> final_node_path = new List<Node>();


        private bool backward_crash = false;
        private bool frontal_crash = false;

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
            

            for (int e = 0; e<enemy_positions.Count; e++)
            {

                if (original_graph.getNodeFromPoint(enemy_positions[e]).assigned_veichle != CarNumber) {
                    enemy_positions.Remove(enemy_positions[e]);
                    e--;
                }
            }

            starting_node = PathFinder.get_starting_node(transform.position, CarNumber, original_graph, graph, (360 - transform.eulerAngles.y + 90) % 360, ref path_to_starting_node);
            original_graph = Graph.CreateGraph(terrain_manager.myInfo, x_scale * 2, z_scale * 2);

            
            foreach (Node n in path_to_starting_node)
            {
                final_path.Add(n.worldPosition);
                final_node_path.Add(n);
            }


            Vector3 current_position = starting_node.worldPosition;
            float heading = (360 - transform.eulerAngles.y + 90) % 360;
            //compute closest turret
            int safe_exit = 10;
            while (enemy_positions.Count > 0)
            {
                Vector3 closest_point = new Vector3(0, 0, 0);
                int min_distance = int.MaxValue;
                int turret_index=0;
                List<Node> path_to_closest = new List<Node>();
                for (int e = 0; e<enemy_positions.Count; e++)
                {
                    Vector3 turret = enemy_positions[e];
                    List<Node> path_to_current = PathFinder.findAstarPath(original_graph, current_position, turret, heading);
                    if (path_to_current.Count < min_distance)
                    {
                        min_distance = path_to_current.Count;
                        path_to_closest = path_to_current;
                        turret_index = e;
                        closest_point = turret;
                    }
                }
                if(CarNumber==2)Debug.Log("Iteration " + (10 - safe_exit) + "closest enemy: " + closest_point);
                current_position = closest_point;
                heading = path_to_closest[path_to_closest.Count - 1].heading;
                final_node_path.AddRange(path_to_closest);
                enemy_positions.RemoveAt(turret_index);
                safe_exit--;
            }

            int upsampling_factor = 4;
            if (final_node_path.Count > 0)
            {
                final_node_path = PathFinder.pathUpsampling(final_node_path, upsampling_factor);
                final_node_path = PathFinder.pathSmoothing(final_node_path, 0.6f, 0.2f, 1E-09f); //Now the path is ready to be trasversed
            }

            my_path = new List<Vector3>();
            foreach(Node n in final_node_path)
            {
                my_path.Add(n.worldPosition);
            }

        }


        private void FixedUpdate()
        {
            mazeTimer += Time.deltaTime;
            enemies = GameObject.FindGameObjectsWithTag("Enemy");
            Debug.Log("Enemies remaining: " + enemies.Length + " Time Elapsed: " + mazeTimer);

            // Drive Car
            if (!MazeComplete)
            {
                path_index = gianDriveCar(my_path, m_Car, path_index);

                //path_index = DriveCar(my_path, m_Car, path_index);

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


        // MAIN FUNC: Compute path 
        private List<Vector3> ComputePath(GraphSTC graphSTC, Edge[] EdgeArray, Vector3 starting_position, Node starting_node, Orientation initial_orientation = Orientation.UU)
        {
            List<Vector3> path = new List<Vector3>();
            Node current_node = starting_node;
            Orientation arriving_orientation = initial_orientation;

            if (!current_node.is_supernode)
            {
                path.Add(current_node.worldPosition);
            }
            else
            {
                path.Add(new Vector3(current_node.x_pos + graph.x_unit / 1.7f, 0, current_node.z_pos - graph.z_unit / 1.7f));
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
                            path.Add(new Vector3(current_node.x_pos + graph.x_unit / 1.7f, 0, current_node.z_pos + graph.z_unit / 1.7f));
                            path.Add(new Vector3(current_node.x_pos - graph.x_unit / 1.7f, 0, current_node.z_pos + graph.z_unit / 1.7f));
                            path.Add(new Vector3(current_node.x_pos - graph.x_unit / 1.7f, 0, current_node.z_pos - graph.z_unit / 1.7f));
                            break;
                        case Orientation.DD:
                            path.Add(new Vector3(current_node.x_pos - graph.x_unit / 1.7f, 0, current_node.z_pos - graph.z_unit / 1.7f));
                            path.Add(new Vector3(current_node.x_pos + graph.x_unit / 1.7f, 0, current_node.z_pos - graph.z_unit / 1.7f));
                            path.Add(new Vector3(current_node.x_pos + graph.x_unit / 1.7f, 0, current_node.z_pos + graph.z_unit / 1.7f));
                            break;
                        case Orientation.R:
                            path.Add(new Vector3(current_node.x_pos + graph.x_unit / 1.7f, 0, current_node.z_pos - graph.z_unit / 1.7f));
                            path.Add(new Vector3(current_node.x_pos + graph.x_unit / 1.7f, 0, current_node.z_pos + graph.z_unit / 1.7f));
                            path.Add(new Vector3(current_node.x_pos - graph.x_unit / 1.7f, 0, current_node.z_pos + graph.z_unit / 1.7f));
                            break;
                        case Orientation.L:
                            path.Add(new Vector3(current_node.x_pos - graph.x_unit / 1.7f, 0, current_node.z_pos + graph.z_unit / 1.7f));
                            path.Add(new Vector3(current_node.x_pos - graph.x_unit / 1.7f, 0, current_node.z_pos - graph.z_unit / 1.7f));
                            path.Add(new Vector3(current_node.x_pos + graph.x_unit / 1.7f, 0, current_node.z_pos - graph.z_unit / 1.7f));
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
                                        path.Add(new Vector3(current_node.x_pos + (graph.x_unit / 1.7f), 0, current_node.z_pos + (graph.z_unit / 1.7f)));
                                        path.Add(new Vector3(next_node.x_pos + graph.x_unit / 1.7f, 0, next_node.z_pos - graph.z_unit / 1.7f));
                                        break;
                                    case Orientation.DD:
                                        path.Add(new Vector3(current_node.x_pos - (graph.x_unit / 1.7f), 0, current_node.z_pos - graph.z_unit / 1.7f));
                                        path.Add(new Vector3(next_node.x_pos - graph.x_unit / 1.7f, 0, next_node.z_pos + graph.z_unit / 1.7f));
                                        break;
                                    case Orientation.R:
                                        path.Add(new Vector3(current_node.x_pos + (graph.x_unit / 1.7f), 0, current_node.z_pos - graph.z_unit / 1.7f));
                                        path.Add(new Vector3(next_node.x_pos - graph.x_unit / 1.7f, 0, next_node.z_pos - graph.z_unit / 1.7f));
                                        break;
                                    case Orientation.L:
                                        path.Add(new Vector3(current_node.x_pos - (graph.x_unit / 1.7f), 0, current_node.z_pos + (graph.z_unit / 2f)));
                                        path.Add(new Vector3(next_node.x_pos + (graph.x_unit / 1.7f), 0, next_node.z_pos + graph.z_unit / 1.7f));
                                        break;
                                }
                                break;
                            case Direction.TL:
                                switch (arriving_orientation)
                                {
                                    case Orientation.UU:
                                        path.Add(new Vector3(current_node.x_pos + (graph.x_unit / 1.7f), 0, current_node.z_pos + graph.z_unit / 1.7f));
                                        path.Add(new Vector3(current_node.x_pos - (graph.x_unit / 1.7f), 0, current_node.z_pos + graph.z_unit / 1.7f));
                                        path.Add(new Vector3(next_node.x_pos + (graph.x_unit / 1.7f), 0, next_node.z_pos + graph.z_unit / 1.7f));
                                        break;
                                    case Orientation.DD:
                                        path.Add(new Vector3(current_node.x_pos - (graph.x_unit / 1.7f), 0, current_node.z_pos - graph.z_unit / 1.7f));
                                        path.Add(new Vector3(current_node.x_pos + (graph.x_unit / 1.7f), 0, current_node.z_pos - graph.z_unit / 1.7f));
                                        path.Add(new Vector3(next_node.x_pos - (graph.x_unit / 1.7f), 0, next_node.z_pos - graph.z_unit / 1.7f));
                                        break;
                                    case Orientation.R:
                                        path.Add(new Vector3(current_node.x_pos + (graph.x_unit / 1.7f), 0, current_node.z_pos - graph.z_unit / 1.7f));
                                        path.Add(new Vector3(current_node.x_pos + (graph.x_unit / 1.7f), 0, current_node.z_pos + graph.z_unit / 1.7f));
                                        path.Add(new Vector3(next_node.x_pos + (graph.x_unit / 1.7f), 0, next_node.z_pos - graph.z_unit / 1.7f));
                                        break;
                                    case Orientation.L:
                                        path.Add(new Vector3(current_node.x_pos - (graph.x_unit / 1.7f), 0, current_node.z_pos + graph.z_unit / 1.7f));
                                        path.Add(new Vector3(current_node.x_pos - (graph.x_unit / 1.7f), 0, current_node.z_pos - graph.z_unit / 1.7f));
                                        path.Add(new Vector3(next_node.x_pos - (graph.x_unit / 1.7f), 0, next_node.z_pos + graph.z_unit / 1.7f));
                                        break;
                                }
                                break;
                            case Direction.TR:
                                switch (arriving_orientation)
                                {
                                    case Orientation.UU:
                                        path.Add(new Vector3(next_node.x_pos - graph.x_unit / 1.7f, 0, next_node.z_pos - graph.z_unit / 1.7f));
                                        break;
                                    case Orientation.DD:
                                        path.Add(new Vector3(next_node.x_pos + graph.x_unit / 1.7f, 0, next_node.z_pos + graph.z_unit / 1.7f));
                                        break;
                                    case Orientation.R:
                                        path.Add(new Vector3(next_node.x_pos - graph.x_unit / 1.7f, 0, next_node.z_pos + graph.z_unit / 1.7f));
                                        break;
                                    case Orientation.L:
                                        path.Add(new Vector3(next_node.x_pos + (graph.x_unit / 1.7f), 0, next_node.z_pos - graph.z_unit / 1.7f));
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
                                        path.Add(new Vector3(next_node.x_pos + graph.x_unit / 1.7f, 0, next_node.z_pos - graph.z_unit / 1.7f));
                                        break;
                                    case Orientation.DD:
                                        path.Add(new Vector3(next_node.x_pos - graph.x_unit / 1.7f, 0, next_node.z_pos + graph.z_unit / 1.7f));
                                        break;
                                    case Orientation.R:
                                        path.Add(new Vector3(next_node.x_pos - graph.x_unit / 1.7f, 0, next_node.z_pos - graph.z_unit / 1.7f));
                                        break;
                                    case Orientation.L:
                                        path.Add(new Vector3(next_node.x_pos + graph.x_unit / 1.7f, 0, next_node.z_pos + graph.z_unit / 1.7f));
                                        break;
                                }
                                break;
                            case Direction.TL:
                                switch (arriving_orientation)
                                {
                                    case Orientation.UU:
                                        path.Add(new Vector3(next_node.x_pos + graph.x_unit / 1.7f, 0, next_node.z_pos + graph.z_unit / 1.7f));
                                        break;
                                    case Orientation.DD:
                                        path.Add(new Vector3(next_node.x_pos - graph.x_unit / 1.7f, 0, next_node.z_pos - graph.z_unit / 1.7f));//seems wrong at a first glance but i'm tired and may be wrong myself
                                        break;
                                    case Orientation.R:
                                        path.Add(new Vector3(next_node.x_pos + graph.x_unit / 1.7f, 0, next_node.z_pos - graph.z_unit / 1.7f));
                                        break;
                                    case Orientation.L:
                                        path.Add(new Vector3(next_node.x_pos + graph.x_unit / 1.7f, 0, next_node.z_pos + graph.z_unit / 1.7f));
                                        break;
                                }
                                break;
                            case Direction.TR:
                                switch (arriving_orientation)
                                {
                                    case Orientation.UU:
                                        path.Add(new Vector3(next_node.x_pos - graph.x_unit / 1.7f, 0, next_node.z_pos - graph.z_unit / 1.7f));
                                        break;
                                    case Orientation.DD:
                                        path.Add(new Vector3(next_node.x_pos + graph.x_unit / 1.7f, 0, next_node.z_pos + graph.z_unit / 1.7f));
                                        break;
                                    case Orientation.R:
                                        path.Add(new Vector3(next_node.x_pos - graph.x_unit / 1.7f, 0, next_node.z_pos + graph.z_unit / 1.7f));
                                        break;
                                    case Orientation.L:
                                        path.Add(new Vector3(next_node.x_pos - graph.x_unit / 1.7f, 0, next_node.z_pos - graph.z_unit / 1.7f));
                                        break;
                                }
                                break;
                            case Direction.UTURN:
                                switch (arriving_orientation)
                                {
                                    case Orientation.UU:
                                        path.Add(new Vector3(next_node.x_pos - graph.x_unit / 1.7f, 0, next_node.z_pos + graph.z_unit / 1.7f));
                                        break;
                                    case Orientation.DD:
                                        path.Add(new Vector3(next_node.x_pos + graph.x_unit / 1.7f, 0, next_node.z_pos - graph.z_unit / 1.7f));
                                        break;
                                    case Orientation.R:
                                        path.Add(new Vector3(next_node.x_pos + graph.x_unit / 1.7f, 0, next_node.z_pos + graph.z_unit / 1.7f));
                                        break;
                                    case Orientation.L:
                                        path.Add(new Vector3(next_node.x_pos - graph.x_unit / 1.7f, 0, next_node.z_pos - graph.z_unit / 1.7f));
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
                                        path.Add(new Vector3(current_node.x_pos + (graph.x_unit / 1.7f), 0, current_node.z_pos + graph.z_unit / 1.7f));
                                        path.Add(new Vector3(current_node.x_pos - (graph.x_unit / 1.7f), 0, current_node.z_pos + graph.z_unit / 1.7f));
                                        path.Add(next_node.worldPosition);
                                        break;
                                    case Orientation.DD:
                                        path.Add(new Vector3(current_node.x_pos - (graph.x_unit / 1.7f), 0, current_node.z_pos - graph.z_unit / 1.7f));
                                        path.Add(new Vector3(current_node.x_pos + (graph.x_unit / 1.7f), 0, current_node.z_pos - graph.z_unit / 1.7f));
                                        path.Add(next_node.worldPosition);
                                        break;
                                    case Orientation.R:
                                        path.Add(new Vector3(current_node.x_pos + (graph.x_unit / 1.7f), 0, current_node.z_pos - graph.z_unit / 1.7f));
                                        path.Add(new Vector3(current_node.x_pos + (graph.x_unit / 1.7f), 0, current_node.z_pos + graph.z_unit / 1.7f));
                                        path.Add(next_node.worldPosition);
                                        break;
                                    case Orientation.L:
                                        path.Add(new Vector3(current_node.x_pos - (graph.x_unit / 1.7f), 0, current_node.z_pos + graph.z_unit / 1.7f));
                                        path.Add(new Vector3(current_node.x_pos - (graph.x_unit / 1.7f), 0, current_node.z_pos - graph.z_unit / 1.7f));
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
                }

                if (next_node == starting_node && next_node.children_to_visit == 0)
                    break;

                arriving_orientation = PathFinder.getOrientation(current_node, next_node);
                current_node = next_node;



            }

            return path;
        }



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
                        if (destination.parent == source)
                        {
                            source.children.Remove(destination);
                            destination.parent = destination;
                            foreach (Node child in destination.children)
                            {
                                Debug.Log("PRUNING" + destination + " has children: " + child);
                            }
                        }
                        else if (source.parent == destination)
                        {
                            Debug.Log("PRUNING Source is the child of destination");
                            destination.children.Remove(source);
                            source.parent = source;
                            foreach (Node child in source.children)
                            {
                                Debug.Log("PRUNING" + source + " has children: " + child);
                            }
                        }
                        else
                        {
                            Debug.Log("ERROR WHILE PRUNING. Source:" + source + "\nSource parent:" + source.parent + "\nDestination:" + destination + "Dest parent:" + destination.parent);
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


            int col = CarNumber - 1;
            int k = 0;
            //foreach(List<Node> path in astar_paths)
            //{

            foreach (Node node in final_node_path)
            {
                Gizmos.color = colors[col % colors.Length];
                Gizmos.DrawCube(node.worldPosition, new Vector3(original_graph.x_unit, 0, original_graph.z_unit) * 0.9f);
                Gizmos.color = colors[col % colors.Length];
                Gizmos.color = colors[col + 2 % colors.Length];

                if (k < final_node_path.Count - 1)
                    Gizmos.DrawLine(final_node_path[k].worldPosition, final_node_path[k + 1].worldPosition);
                k++;
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

        private List<Vector3> path_deobstacle(List<Vector3> path, Graph graph)
        {
            for (int i = 0; i < path.Count; i++)
            {
                Vector3 waypoint = path[i];
                Node node = graph.getNodeFromPoint(waypoint);
                while (!node.walkable)
                {
                    Vector3 direction = (waypoint - node.worldPosition) / 1.9f;
                    waypoint = waypoint + direction;
                    path[i] = waypoint;
                    node = graph.getNodeFromPoint(waypoint);
                }
            }
            return path;
        }

    }

}
