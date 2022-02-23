using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;


namespace UnityStandardAssets.Vehicles.Car
{
    [RequireComponent(typeof(CarController))]
    public class CarAI1 : MonoBehaviour
    {
        // Variables for Car
        private CarController m_Car; // the car controller we want to use
        Vector3 carSize = new Vector3(4.5f, 0.41f, 4.5f);
        private Rigidbody rigidbody;

        // Variables for Terrain
        public GameObject terrain_manager_game_object;
        TerrainManager terrain_manager;
        public Graph graph;

        // Variables for Players and Turrets
        public GameObject[] friends;
        public GameObject[] enemies;
        private float gunRange;

        // Variables for path & driving
        private float acceleration, max_speed;
        private bool MazeComplete;
        private float time;
        private Vector3 start_pos, goal_pos;
        private int path_index;
        private List<Vector3> my_path;


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
            goal_pos = terrain_manager.myInfo.goal_pos;
            rigidbody = GetComponent<Rigidbody>();

            // Construct Terrain Graph
            int x_scale = terrain_manager.myInfo.x_N;
            int z_scale = terrain_manager.myInfo.z_N;
            float x_len = (terrain_manager.myInfo.x_high - terrain_manager.myInfo.x_low);
            float z_len = (terrain_manager.myInfo.z_high - terrain_manager.myInfo.z_low);
            float x_unit = x_len / x_scale;
            x_unit = 1.4142f * gunRange;
            float z_unit = x_unit;
            Debug.Log("x_scale: " + x_scale + " z_scale: " + z_scale);
            x_scale = x_scale * ((int)(x_len / x_unit) / x_scale);
            z_scale = z_scale * ((int)(z_len / z_unit) / z_scale);
            Debug.Log("x_unit: " + x_unit + " z_unit: " + z_unit);
            Debug.Log("x_scale: " + x_scale + " z_scale: " + z_scale);
            Debug.Log("x_len: " + x_len + " z_len: " + z_len);

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


            // Get Array of Friends and Eniemies
            friends = GameObject.FindGameObjectsWithTag("Player");


            // Plan your path here
            // ...


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
            if (graph != null)
            {
                foreach (Node n in graph.nodes) // graph.path 
                {
                    Gizmos.color = (n.walkable) ? Color.blue : Color.red;
                    if (graph.path != null && graph.path.Contains(n))
                        Gizmos.color = Color.white;
                    Gizmos.DrawCube(n.worldPosition, new Vector3(graph.x_unit * 0.8f, 0.5f, graph.z_unit * 0.8f));
                }


                Node currentNode = graph.getNodeFromPoint(transform.position);
                //Debug.Log("CAR INITIAL NODE: [" + currentNode.i + "," + currentNode.j + "]");
                Gizmos.color = Color.cyan; // position of car
                Gizmos.DrawCube(currentNode.worldPosition, new Vector3(graph.x_unit * 0.8f, 0.5f, graph.z_unit * 0.8f));
            }
        }

    }
}
