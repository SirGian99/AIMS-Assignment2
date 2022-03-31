using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;


namespace UnityStandardAssets.Vehicles.Car
{
    [RequireComponent(typeof(CarController))]
    public class CarAI5 : MonoBehaviour
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
        private CarConfigSpace ObstacleSpace;


        // Variables for Players and Turrets
        public GameObject[] friends, cars;
        public GameObject[] enemies;
        private List<Vector3> enemy_positions = new List<Vector3>();
        private int enemy_count;
        public Destructable Health;
        public List<float> HealthVec = new List<float>();
        public float TotalHealth;

        // Variables for path & driving
        private float acceleration, max_speed;
        private bool MazeComplete, PlayerCrashed;
        private float mazeTimer, driveTimer, stuckTimer, recoveryTime;
        private float recoverySteer;
        private Vector3 previous_pos;
        private int path_index;
        private List<Vector3> my_path;
        private Node peeking_node, firing_node;
        enum car_state { drive, front_crash, back_crash };
        private car_state car_status;

        enum car_detection_state { no_detection, frontal_detection, frontal_crash }
        private car_detection_state car_status_gian = car_detection_state.no_detection;

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
            Time.timeScale = 2;
            driveTimer = 0f;
            max_speed = 18;
            acceleration = 1f;
            MazeComplete = false;
            mazeTimer = 0f;
            car_status = car_state.drive;
            stuckTimer = 0f;
            PlayerCrashed = false;
            recoveryTime = 0.7f;
            recoverySteer = 0.45f;//45 degrees gives best result


            // Initialize Car and Terrain
            m_Car = GetComponent<CarController>();
            terrain_manager = terrain_manager_game_object.GetComponent<TerrainManager>();
            rigidbody = GetComponent<Rigidbody>();
            Health = GetComponent<Destructable>();

            // Initialize ConfigSpace
            CreateObstacleSpace();

            // Construct Terrain Graph
            int x_scale = terrain_manager.myInfo.x_N*3;
            int z_scale = terrain_manager.myInfo.z_N*3;
            
            //i want x_unit and z_unit to be √2r, where r is the range of the gun.
            //but i also want the new scales them to be a multiple of the original x_scale and z_scale            
            graph = Graph.CreateGraph(terrain_manager.myInfo, x_scale, z_scale);


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
                //if (CarNumber == 1) Debug.Log(friend + " position: " + friend.gameObject.transform.position);
                initial_positions[i] = friend.gameObject.transform.position;
                i++;

            }
            enemies = GameObject.FindGameObjectsWithTag("Enemy");
            enemy_count = enemies.Length;
            foreach (GameObject obj in enemies)
            {
                Debug.DrawLine(transform.position, obj.transform.position, Color.black, 10f);
                enemy_positions.Add(obj.gameObject.transform.position);

            }
            DangerMap();
            peeking_node = peekNode();
            Debug.Log("Car " + CarNumber + " PeekingNode " + peeking_node + " " + peeking_node.worldPosition);
            firing_node = fireNode();
            my_path = ComputePath();
            path_index = 0;

        }



        private void FixedUpdate()
        {
            
            mazeTimer += Time.deltaTime;
            enemies = GameObject.FindGameObjectsWithTag("Enemy");
            cars = GameObject.FindGameObjectsWithTag("Player");
            // Health info
            HealthVec.Clear();
            TotalHealth = 0;
            for (int i = 0; i < cars.Length; i++)
            {
                HealthVec.Add(cars[i].GetComponent<Destructable>().health);
                TotalHealth = TotalHealth + cars[i].GetComponent<Destructable>().health;
            }
            
            // Exit Game logic
            if (enemies.Length <= 0)
            {
                if (CarNumber == 1) Debug.Log("All enemies killed in " + mazeTimer + " Remaining Health"+ TotalHealth +" Exiting Game...");
                UnityEditor.EditorApplication.isPlaying = false;
            }

            // Re-evaulate DangerLevel if enemy killed
            if (enemies.Length != enemy_count && enemies.Length > 0)
            {
                Debug.Log("Enemy killed ");
                DangerMap();
                peeking_node = peekNode();
                firing_node = fireNode();
                my_path = ComputePath();
                path_index = 0;
                enemy_count = enemies.Length;

            }
            
            if (CarNumber == 1) Debug.Log("Remaining Health" + TotalHealth + " Enemies remaining: " + enemies.Length + " Time Elapsed: " + mazeTimer);

            // Drive Car
            if (!MazeComplete)
            {
                path_index = DriveCar(my_path, m_Car, path_index);

                //path_index = gianDriveCar(my_path, m_Car, path_index);
                //Check if all enemies killed for maze completion
                if (enemies.Length <= 0)
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

            


        }

        // MAIN FUNC: Assign dangerlevel to each node
        private void DangerMap()
        {
            // Find enemy positions
            enemy_positions = new List<Vector3>();
            foreach (GameObject obj in enemies)
            {
                if (obj.gameObject.transform.position != null)
                {
                    enemy_positions.Add(obj.gameObject.transform.position);
                }
            }

            bool obstacle = false;
            foreach (Node node in graph.nodes)
            {
                if (node.walkable)
                {
                    node.dangerLevel = 0;
                    int i = 0;
                    foreach (Vector3 enemy in enemy_positions)
                    {
                        obstacle = Physics.Raycast(enemy, node.worldPosition - enemy, Vector3.Magnitude(enemy - node.worldPosition), 1 << 9);
                        if (!obstacle)
                        {
                            node.dangerLevel++;
                            if (node.dangerLevel == 1)
                            {
                                node.assigned_enemy = i;
                            }
                            i++;
                        }
                    }
                }
            }
        }

        // MAIN FUNC: Find closest shooting node
        private Node peekNode()
        {
            List<Node> peekNodes = new List<Node>();
            Node bestNode = graph.nodes[0, 0];
            foreach (Node node in graph.nodes)
            {
                if(node.dangerLevel==0)
                {
                    foreach(Node neighbour in node.neighbours)
                    {
                        if(neighbour.dangerLevel == 1)
                        {
                            peekNodes.Add(node);
                            if (Vector3.Distance(friends[0].transform.position, node.worldPosition) < Vector3.Distance(friends[0].transform.position, bestNode.worldPosition))
                            {
                                bestNode = node;
                                break;
                            }

                        }
                    }
                }
            }
            //Debug.Log("Car " + CarNumber + " bestNode " + bestNode);

            List<Node> bestNodes = new List<Node>();
            bestNodes.Add(bestNode);
            int c = 1;
            while (bestNodes.Count < friends.Length)
            {
                foreach (Node node in peekNodes)
                {
                    if (node.assigned_enemy == bestNode.assigned_enemy 
                        && Vector3.Distance(bestNode.worldPosition, node.worldPosition) <= (c * 2 * graph.x_unit)
                        && !bestNodes.Contains(node))
                    {
                        //Debug.Log("c: " +c+ " NEW bestNode found " + node + " bestNodes " + bestNodes.Count);
                        bestNodes.Add(node);

                    }
                    if (bestNodes.Count >= friends.Length)
                    {
                        break;
                    }
                }
                c++;
            }
            return bestNodes[CarNumber-1];
        }

        // MAIN FUNC: Find firing node bases on closet shooting node
        private Node fireNode()
        {
            List<Node> peekNodes = new List<Node>();
            Node bestNode = graph.nodes[0, 0];
            foreach (Node node in graph.nodes)
            {
                if (node.dangerLevel == 1)
                {
                    foreach (Node neighbour in node.neighbours)
                    {
                        if (neighbour.dangerLevel == 0)
                        {
                            peekNodes.Add(node);
                            if (Vector3.Distance(friends[0].transform.position, node.worldPosition) < Vector3.Distance(friends[0].transform.position, bestNode.worldPosition))
                            {
                                bestNode = node;
                                break;
                            }

                        }
                    }
                }
            }
            //Debug.Log("Car " + CarNumber + " bestNode " + bestNode);

            List<Node> bestNodes = new List<Node>();
            bestNodes.Add(bestNode);
            int c = 1;
            while (bestNodes.Count < friends.Length)
            {
                foreach (Node node in peekNodes)
                {
                    if (node.assigned_enemy == bestNode.assigned_enemy
                        && Vector3.Distance(bestNode.worldPosition, node.worldPosition) <= (c * 2 * graph.x_unit)
                        && !bestNodes.Contains(node))
                    {
                        //Debug.Log("c: " +c+ " NEW bestNode found " + node + " bestNodes " + bestNodes.Count);
                        bestNodes.Add(node);

                    }
                    if (bestNodes.Count >= friends.Length)
                    {
                        break;
                    }
                }
                c++;
            }
            return bestNodes[CarNumber - 1];
        }

        // MAIN FUNC: Find path
        private List<Vector3> ComputePath()
        {
            List<Vector3> path = new List<Vector3>();
            List<Node> path_to_starting_node = PathFinder.findSafePath(graph, m_Car.transform.position, peeking_node.worldPosition, (360 - transform.eulerAngles.y + 90) % 360);
            Debug.Log("Car " + CarNumber + " Start: " + m_Car.transform.position + " peek: " + peeking_node.worldPosition + " path " + path_to_starting_node.Count);
            foreach (Node node in path_to_starting_node)
            {
                path.Add(node.worldPosition);
            }
            path.Add(firing_node.worldPosition);
            return path;
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
                if (driveTimer >= stuckTimer && path_index > 10)
                {
                    driveTimer = 0;
                    //if (CarNumber == 1) Debug.Log("Crash pos: " + Vector3.Distance(previous_pos, player_Car.transform.position));
                    //check: car not moved very much from previous position
                    if (Vector3.Distance(previous_pos, player_Car.transform.position) < 0.05f)
                    {
                        PlayerCrashed = true;
                        // check if obstacle in front of car
                        if (player_Car.CurrentSpeed < 0.5f && Physics.BoxCast(player_Car.transform.position,
                            new Vector3(ObstacleSpace.BoxSize.x / 2, ObstacleSpace.BoxSize.y / 2, 0.5f),
                            player_Car.transform.forward,
                            Quaternion.LookRotation(player_Car.transform.forward),
                            ObstacleSpace.BoxSize.z / 1.7f))
                        {
                            car_status = car_state.front_crash;
                        }
                        else if (player_Car.CurrentSpeed < 0.5f)
                        {
                            car_status = car_state.back_crash;
                        }
                        /*
                        else if(Physics.BoxCast(player_Car.transform.position,
                            new Vector3(ObstacleSpace.BoxSize.x / 2, ObstacleSpace.BoxSize.y / 2, 0.5f),
                            Quaternion.AngleAxis(0, Vector3.down) * player_Car.transform.forward,
                            Quaternion.LookRotation(Quaternion.AngleAxis(0, Vector3.down) * player_Car.transform.forward),
                            ObstacleSpace.BoxSize.z / 1.7f))
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
                    if (CarNumber == 1) Debug.Log("Crashed, old steering: " + old_steering + " new steering: " + steeringAmount);
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
                    if (car_status == car_state.front_crash)
                    {
                        //if (CarNumber == 1) Debug.Log("Front Crash recovery logic " + "Steer: " + car_steer + "Time: " + stuckTimer);
                        player_Car.Move(-steeringAmount, 0, -1, 0);
                        // switch up crash direction if recovery delayed
                        if (stuckTimer > recoveryTime / 2 && PlayerCrashed)
                        {
                            car_status = car_state.back_crash;
                            //if (CarNumber == 1) Debug.Log("Back Crash recovery logic " + "Steer: " + car_steer + "Time: " + stuckTimer);
                            player_Car.Move(steeringAmount, 1, 0, 0);
                        }
                    }
                    //drive forward if crashed while reverse drive
                    else
                    {
                        //if (CarNumber == 1) Debug.Log("Back Crash recovery logic " + "Steer: " + car_steer + "Time: " + stuckTimer);
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

        // Sub-Func: To find accleration for car
        private float Accelerate(Vector3 position, float theta, Vector3 target)
        {
            Vector3 direction = Quaternion.Euler(0, theta, 0) * Vector3.forward;
            Vector3 directionToTarget = target - position;
            return Mathf.Clamp(direction.x * directionToTarget.x + direction.z * directionToTarget.z, -1, 1);
        }

        // Sub-Func: To avoid obstacles
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

        //MAIN FUNC: Visualise
        void OnDrawGizmos() // draws grid on map and shows car
        {
                       
            if (graph != null)
            {
                // Show Danger Levels on terrain 
                Color[] colors = { new Color(0f, 1f, 0f, 0.2f), new Color(1f, 0.9f, 0.016f, 0.2f), new Color(1f, 0.5f, 0f, 0.2f), new Color(1f, 0.25f, 0f, 0.2f), new Color(1f, 0f, 0f, 0.2f) };
                foreach (Node n in graph.nodes) // graph.path 
                {
                    if (n != null && n.walkable)
                    { 
                        Gizmos.color = colors[n.dangerLevel];
                        //if (graph.path != null && graph.path.Contains(n))
                           // Gizmos.color = Color.white;

                        if(n == peeking_node)
                        {
                            Gizmos.color = new Color(0f, 0f, 0.9f, 1f);
                        }
                        if(n == firing_node)
                        {
                            Gizmos.color = Color.red;
                        }
                        float scale_factor = n.is_supernode ? 1.8f : 0.9f;
                        Gizmos.DrawCube(n.worldPosition, new Vector3(graph.x_unit * scale_factor, 0.5f, graph.z_unit * scale_factor));

                    }

                }

                // Show each car in different color
                Color[] car_colors = { Color.grey, new Color(0.933f, 0.509f, 0.933f, 1f), new Color(0.541f, 0.168f, 0.886f, 1f), Color.magenta };
                Gizmos.color = car_colors[CarNumber];
                //Gizmos.DrawSphere(m_Car.transform.position, graph.x_unit);

                //Show the path to the goal
                if (my_path != null)
                {
                    for (int i = path_index; i < my_path.Count - 1; ++i)
                    {
                        Gizmos.color = car_colors[CarNumber];
                        Gizmos.DrawLine(my_path[i], my_path[i + 1]);
                    }
                }
            }

            





        }
    }
}
