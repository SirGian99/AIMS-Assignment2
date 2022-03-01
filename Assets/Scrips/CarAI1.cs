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
        private Edge[] my_tree;


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
            //Debug.Log("x_unit: " + x_unit + " z_unit: " + z_unit);
            //Debug.Log("x_scale: " + x_scale + " z_scale: " + z_scale);
            //Debug.Log("x_len: " + x_len + " z_len: " + z_len);

            /*

            float z_unit = z_len / z_scale;
            float ratio = x_unit / z_unit;


            if (x_unit < carSize.x * 2)
            {
                Debug.Log("Block width is too low");
                x_scale /= 2;
                x_unit *= 2;
            }
            if (z_unit < carSize.z)
            {
                Debug.Log("Block height is too low");
                z_scale /= 2;
                z_unit *= 2;
            }
            */

            //i want x_unit and z_unit to be √2r, where r is the range of the gun.
            //but i also want the new scales them to be a multiple of the original x_scale and z_scale            
            graph = Graph.CreateGraph(terrain_manager.myInfo, x_scale, z_scale);
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
            darp = new DARP_controller(terrain_manager.myInfo, friends.Length, initial_positions, graph, 0.0004f, 100);
            my_path = new List<Vector3>();
            my_path = darp.all_paths[CarNumber - 1];
            my_tree = darp.STC(darp.subgraphs[CarNumber - 1]);

        }


        private void FixedUpdate()
        {
            
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


            //Show min span tree
            if (my_tree != null)
                {
                    Gizmos.color = colors[CarNumber];
                    //Gizmos.color = Color.red;
                    for (int i = 0; i < my_tree.Length - 1; ++i)
                    {

                    if (my_tree[i].Source != null && my_tree[i].Destination != null)
                        Gizmos.DrawLine(my_tree[i].Source.worldPosition + Vector3.up, my_tree[i].Destination.worldPosition + Vector3.up);
                        ;
                        //Gizmos.DrawSphere(my_tree[i].Source.worldPosition, 5f);
                        //Gizmos.DrawSphere(my_tree[i].Destination.worldPosition, 2f);

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
