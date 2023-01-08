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
        private List<Vector3> enemy_positions, enemy_list;
        private int enemy_count;
        public Destructable Health;
        public List<float> HealthVec = new List<float>();
        public float TotalHealth;
        private bool friendsReady, IamReady, waiting, f1, i1;
        private Vector3 targetEnemy;

        // Variables for path & driving
        private float acceleration, max_speed, capped_speed;
        private bool MazeComplete, PlayerCrashed, waitSync;
        private float mazeTimer, driveTimer, stuckTimer, recoveryTime, waitTimer;
        private float recoverySteer;
        private Vector3 previous_pos;
        private int path_index;
        private List<Vector3> my_path;
        private Node peeking_node, firing_node, enemy_node;
        private List<Node> peekingNodes, firingNodes, enemyNodes, previousFire;
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
            Time.timeScale = 5;
            driveTimer = 0f;
            capped_speed = 10f;
            max_speed = capped_speed;
            acceleration = 1f;
            MazeComplete = false;
            mazeTimer = 0f;
            car_status = car_state.drive;
            stuckTimer = 0f;
            PlayerCrashed = false;
            recoveryTime = 0.7f;
            recoverySteer = 0.45f;//45 degrees gives best result
            waitTimer = 0;


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
            graph = Graph.CreateGraph(terrain_manager.myInfo, x_scale, z_scale);

            // Get Array of Friends and Eniemies
            friends = GameObject.FindGameObjectsWithTag("Player");
            cars = GameObject.FindGameObjectsWithTag("Player");
            int i = 0;
            foreach (GameObject friend in friends)
            {
                if (friend.name == this.name)
                {
                    CarNumber = i + 1;
                }
                i++;

            }
            enemies = GameObject.FindGameObjectsWithTag("Enemy");

            // Compute Danger Map & Find peek and fire nodes
            NextTarget();

        }



        private void FixedUpdate()
        {
            mazeTimer += Time.deltaTime;

            // Health info of all cars
            HealthVec.Clear();
            TotalHealth = 0;
            cars = GameObject.FindGameObjectsWithTag("Player");
            for (int i = 0; i < cars.Length; i++)
            {
                HealthVec.Add(cars[i].GetComponent<Destructable>().health);
                TotalHealth = TotalHealth + cars[i].GetComponent<Destructable>().health;
            }
            if (CarNumber == 1) Debug.Log("Health: " + TotalHealth + " Enemies left: " + enemies.Length + " Time: " + mazeTimer);

            enemies = GameObject.FindGameObjectsWithTag("Enemy");
            // Exit Game logic
            if (enemies.Length <= 0)
            {
                Debug.Log(" Final Health: " + TotalHealth + " Kill Time: " + mazeTimer + " Exiting Game...");
                UnityEditor.EditorApplication.isPlaying = false;
            }

            // Check if friends are in peeking node
            friendsReady = false;
            IamReady = false;
            for (int i = 0; i < cars.Length; i++)
            {
                if(cars[i]!=null && i!=(CarNumber-1))
                {
                    if(Vector3.Distance(cars[i].transform.position, peekingNodes[i].worldPosition) < (graph.x_unit*0.75f) )
                    {
                        friendsReady = true;
                    }
                }
                if (cars[i] != null && i == (CarNumber - 1))
                {
                    if (Vector3.Distance(cars[i].transform.position, peekingNodes[i].worldPosition) < (graph.x_unit*0.75f))
                    {
                        IamReady = true;
                    }
                }
                if(cars.Length<friends.Length)
                {
                    //friendsReady = true;
                    ;
                }
            }

            // Run waitTimer when atleast 2 are ready
            if (!waiting && (friendsReady && IamReady && (!f1 || !i1)))
            //    if (!waiting && !f1 && friendsReady && !i1 && IamReady)
            {
                waiting = true;
                waitTimer = 15f;
                
            }
            if(waiting)
            {
                waitTimer -= Time.deltaTime;
                Debug.Log("waiting - Car " + CarNumber);
            }
            if(waitTimer<=0)
            {
                waitSync = true;
                waiting = false;
            }
            f1 = friendsReady;
            i1 = IamReady;

            // Re-evaulate DangerLevel if enemy killed
            if (enemies.Length != enemy_count && enemies.Length > 0)
            {
                // Compute Danger Map & Find peek and fire nodes
                NextTarget();
                waitSync = false;
            }

            // Execute until maze complete
            if (!MazeComplete)
            {
                // Assist speeds based on path_index
                if(my_path.Count - path_index < 3)
                {
                    max_speed = capped_speed*0.1f;
                }
                if(my_path.Count - path_index < 6)
                {
                    max_speed = capped_speed * 0.3f;
                }
                else
                {
                    max_speed = capped_speed;
                }

                //if(CarNumber==1) Debug.Log("Speed " + max_speed);

                //Drive Car
                if (waiting || (!waiting && !waitSync && my_path.Count - 1 == path_index))
                {
                    m_Car.Move(0, 0, 1, 1);
                }
                else
                {
                    path_index = DriveCar(my_path, m_Car, path_index);
                }
                if (!waitSync && my_path.Count-1==path_index)
                {
                    path_index--;
                }

                
                //Drive Car
                /*
                path_index = DriveCar(my_path, m_Car, path_index);

                // Add fire node to path
                if(my_path[path_index]==peeking_node.worldPosition && waitTimer<=0)
                {
                    Debug.Log("Car " + CarNumber + " moving to fire node");
                    my_path.Add(firing_node.worldPosition);
                    waitTimer = 5f;
                }
                */
                
                //Check if all enemies killed for maze completion
                if (enemies.Length <= 0)
                {
                    MazeComplete = true;
                }
            }
            else
            {
                // Do nothing for now - wait for others to finish
            }
        }

        // MAIN FUNC:
        private void NextTarget()
        {
            
            Debug.Log("Computing Danger Map and fire points");
            DangerMap();
            firingNodes = fireNodesClosest();
            firing_node = firingNodes[CarNumber - 1];
            //peekingNodes = peekNodes();
            peekingNodes = FirePeekNodes(firingNodes);
            peeking_node = peekingNodes[CarNumber - 1];
            my_path = ComputePath();
            my_path.Add(firing_node.worldPosition);
            path_index = 0;
            enemy_count = enemies.Length;
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
                    enemy_positions.Add(obj.transform.position);
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
                            
                        }
                        i++;
                    }
                }
            }
        }


        // MAIN FUNC: Find closest shooting node
        private List<Node> peekNodes()
        {
            List<Node> peekNodes = new List<Node>();
            Node bestNode = graph.nodes[0, 0];
            foreach (Node node in graph.nodes)
            {
                if (node.dangerLevel == 0)
                {
                    foreach (Node neighbour in node.neighbours)
                    {
                        if (neighbour.dangerLevel == 1)
                        {
                            peekNodes.Add(node);
                            if (Vector3.Distance(cars[0].transform.position, node.worldPosition) < Vector3.Distance(cars[0].transform.position, bestNode.worldPosition))
                            {
                                bestNode = node;
                                break;
                            }

                        }
                    }
                }
            }
            //Debug.Log("Car " + CarNumber + " bestNode " + bestNode.assigned_enemy);

            
            List<Node> bestNodes = new List<Node>();
            bestNodes.Add(bestNode);
            int c = 1;
            while (bestNodes.Count < cars.Length)
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
                    if (bestNodes.Count >= cars.Length)
                    {
                        break;
                    }
                }
                c++;
            }
           

            /*
            List<Node> bestNodes = new List<Node>();
            bestNodes.Add(bestNode);
            int c = 1;
            while (bestNodes.Count < friends.Length)
            {
                float maxDifference = 5;
                float interestAngle = Vector3.Angle(enemy_node.worldPosition, bestNode.worldPosition);

                foreach (Node node in peekNodes)
                {
                    float newAngle = Vector3.Angle(enemy_node.worldPosition, node.worldPosition);
                    float calculatedDifference = Mathf.Abs(Mathf.DeltaAngle(interestAngle, newAngle));

                    if (node.assigned_enemy == bestNode.assigned_enemy
                        && Vector3.Distance(bestNode.worldPosition, node.worldPosition) <= (c * 2 * graph.x_unit)
                        && !bestNodes.Contains(node)
                        && calculatedDifference <= maxDifference)
                    {
                        //Debug.Log("c: " +c+ " NEW bestNode found " + node + " bestNodes " + bestNodes.Count);
                        Debug.DrawLine(bestNode.worldPosition, enemy_node.worldPosition, Color.white);
                        bestNodes.Add(node);

                    }
                    if (bestNodes.Count >= friends.Length)
                    {
                        break;
                    }
                }
                c++;
            }
            */
            return bestNodes;
        }

        // MAIN FUNC: Find firing node bases on closet shooting node
        private List<Node> fireNodes()
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
                            if (Vector3.Distance(cars[0].transform.position, node.worldPosition) < Vector3.Distance(cars[0].transform.position, bestNode.worldPosition))
                            {
                                bestNode = node;
                                break;
                            }

                        }
                    }
                }
            }
            targetEnemy = enemy_positions[bestNode.assigned_enemy];

            List<Node> bestNodes = new List<Node>();
            bestNodes.Add(bestNode);
            int c = 1;
            while (bestNodes.Count < cars.Length)
            {
                foreach (Node node in peekNodes)
                {
                    if (node.assigned_enemy == bestNode.assigned_enemy
                        && Vector3.Distance(bestNode.worldPosition, node.worldPosition) <= (c * 2 * graph.x_unit)
                        && !bestNodes.Contains(node)
                        && DistanceLineSegmentPoint(bestNode.worldPosition, targetEnemy, node.worldPosition) <= graph.x_unit)
                    {
                        bestNodes.Add(node);
                    }
                    if (bestNodes.Count >= cars.Length)
                    {
                        break;
                    }
                }
                c++;
            }
            return bestNodes;
        }

        // MAIN FUNC: Find firing node bases on closet shooting node
        private List<Node> fireNodesbasedTargetAngle()
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
                            if (Vector3.Distance(cars[0].transform.position, node.worldPosition) < Vector3.Distance(cars[0].transform.position, bestNode.worldPosition))
                            {
                                bestNode = node;
                                break;
                            }

                        }
                    }
                }
            }
            targetEnemy = enemy_positions[bestNode.assigned_enemy];

            // Nodes having same angle
            List<Node> bestNodes = new List<Node>();
            bestNodes.Add(bestNode);
            int c = 1;
            while (bestNodes.Count < cars.Length)
            {
                float maxDifference = 5;
                float interestAngle = Vector3.Angle(targetEnemy, bestNode.worldPosition);

                foreach (Node node in peekNodes)
                {
                    float newAngle = Vector3.Angle(targetEnemy, node.worldPosition);
                    float calculatedDifference = Mathf.Abs(Mathf.DeltaAngle(interestAngle, newAngle));

                    if (node.assigned_enemy == bestNode.assigned_enemy
                        && Vector3.Distance(bestNode.worldPosition, node.worldPosition) <= (c * 2 * graph.x_unit)
                        && !bestNodes.Contains(node)
                        && calculatedDifference <= maxDifference)
                    {
                        bestNodes.Add(node);

                    }
                    if (bestNodes.Count >= cars.Length)
                    {
                        break;
                    }
                }
                c++;
            }
            return bestNodes;
        }

        // MAIN FUNC: Find peeknode next to firing nodes
        private List<Node> FirePeekNodes(List<Node> firePoints)
        {
            // neigbhour closest to car
            List<Node> peekPoints = new List<Node>();
            foreach (Node firenode in firePoints) //3
            {
                Node bestneighbour = firenode.neighbours[0];
                foreach (Node neighbour in firenode.neighbours) //8
                {
                    if (neighbour.dangerLevel == 0 
                        && !peekPoints.Contains(neighbour)
                        && Vector3.Distance(cars[peekPoints.Count].transform.position, neighbour.worldPosition) < Vector3.Distance(cars[peekPoints.Count].transform.position, bestneighbour.worldPosition))
                    {
                        bestneighbour = neighbour;
                    }

                }
                peekPoints.Add(bestneighbour);
            }


            // Neighbour of neighbour has more green
            /*
            List<Node> peekPoints = new List<Node>();
            foreach (Node firenode in firePoints) //3
            {
                Node bestneighbour = firenode.neighbours[0];
                int best = 8;
                foreach (Node neighbour in firenode.neighbours) //8
                {
                    int cnt = 8;
                    if (neighbour.dangerLevel == 0 && !peekPoints.Contains(neighbour))
                    {
                        foreach (Node nofn in neighbour.neighbours) //8
                        {
                            if(nofn.dangerLevel == 0)
                            {
                                cnt--;
                            }
                        }
                    }
                    if(cnt<best)
                    {
                        bestneighbour = neighbour;
                    }
                }
                peekPoints.Add(bestneighbour);
            }
            */

            //neighbour is perpendicular
            /*
            List<Node> peekPoints = new List<Node>();
            foreach (Node firenode in firePoints) //3
            {
                Vector3 refline = firenode.worldPosition - enemy_node.worldPosition;
                foreach (Node neighbour in firenode.neighbours) //8
                {
                   Vector3 nextnode = firenode.worldPosition - neighbour.worldPosition;
                    if (neighbour.dangerLevel == 0 
                        && !peekPoints.Contains(neighbour)
                        && Mathf.Abs(Vector3.Dot(refline, nextnode)) <0.5f)
                    {
                        peekPoints.Add(neighbour);
                        break;
                    }
                }
            }
            */



            return peekPoints;
        }

        // MAIN FUNC: Find firing node bases on closet shooting node
        private List<Node> fireNodesClosest()
        {
            List<Node> bestNodes = new List<Node>();
            Node bestNode= graph.nodes[0, 0]; ;
            while (bestNodes.Count < cars.Length)
            {
                bestNode = graph.nodes[0, 0];
                foreach (Node node in graph.nodes)
                {
                    if(bestNodes.Count>0)
                    {
                        if (node.dangerLevel == 1 && !bestNodes.Contains(node) && node.assigned_enemy==bestNodes[0].assigned_enemy)
                        {
                            foreach (Node neighbour in node.neighbours)
                            {
                                if (neighbour.dangerLevel == 0)
                                {
                                    if (Vector3.Distance(cars[bestNodes.Count].transform.position, node.worldPosition) < Vector3.Distance(cars[bestNodes.Count].transform.position, bestNode.worldPosition))
                                    {
                                        bestNode = node;
                                        break;
                                    }

                                }
                            }
                        }
                    }
                    else
                    {
                        if (node.dangerLevel == 1)
                        {
                            foreach (Node neighbour in node.neighbours)
                            {
                                if (neighbour.dangerLevel == 0)
                                {
                                    if (Vector3.Distance(cars[0].transform.position, node.worldPosition) < Vector3.Distance(cars[0].transform.position, bestNode.worldPosition))
                                    {
                                        bestNode = node;
                                        break;
                                    }

                                }
                            }
                        }
                    }
                }
                bestNodes.Add(bestNode);
            }
            targetEnemy = enemy_positions[bestNode.assigned_enemy];

            return bestNodes;
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
            return path;
        }

        // Sub-Func Distance to point (p) from line segment (end points a b)
        float DistanceLineSegmentPoint(Vector3 a, Vector3 b, Vector3 p)
        {
            // If a == b line segment is a point and will cause a divide by zero in the line segment test.
            // Instead return distance from a
            if (a == b)
                return Vector3.Distance(a, p);

            // Line segment to point distance equation
            Vector3 ba = b - a;
            Vector3 pa = a - p;
            return (pa - ba * (Vector3.Dot(pa, ba) / Vector3.Dot(ba, ba))).magnitude;
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
                    //if (CarNumber == 1) Debug.Log("Crashed, old steering: " + old_steering + " new steering: " + steeringAmount);
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


        // MAIN FUNC: Car Drive with just waypoints
        public int A4DriveCar(List<Vector3> player_path, CarController player_Car, int player_pathIndex)
        {
            Vector3 player_waypoint = player_path[player_pathIndex];
            Vector3 player_nextwaypoint = player_path[player_pathIndex+1];
            float car_steer, car_acc, car_steer_next;
            int brake = 0, handBrake = 0;

            //find steering needed to get to next point
            car_steer = Steer(player_Car.transform.position, player_Car.transform.eulerAngles.y, player_waypoint);
            car_acc = Accelerate(player_Car.transform.position, player_Car.transform.eulerAngles.y, player_waypoint);
            car_steer_next = Steer(player_Car.transform.position, player_Car.transform.eulerAngles.y, player_nextwaypoint);
            // Do opposite turn if next next steer is in opposte
            if (Math.Abs(car_steer_next) > 0.3f && Math.Abs(car_steer) < 0.3f)
            {
                car_steer = -car_steer_next;
            }
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
            // DRIVE BLOCK
            if (!PlayerCrashed)
            {
                driveTimer += Time.deltaTime;

                //Check if car stuck and not at starting pointing
                if (driveTimer >= stuckTimer &&  mazeTimer >= 5)
                {
                    driveTimer = 0;
                    //Debug.Log("Crash pos: " + Vector3.Distance(previous_pos, player_Car.transform.position));
                    //check: car not moved very much from previous position
                    if (Vector3.Distance(previous_pos, player_Car.transform.position) < 0.05f)
                    {
                        PlayerCrashed = true; ///// SUPPRESS CRASH RECOVERY FOR NOW
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
                    player_Car.Move(-car_steer, brake, car_acc * acceleration, handBrake);
                }
                else
                {
                    player_Car.Move(car_steer, car_acc * acceleration, -brake, handBrake);
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
            if(player_pathIndex==player_path.Count)
            {
                player_pathIndex--;
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

            //if (CarNumber == 1) Debug.Log("Backward crash: " + backward_crash + "\n Frontal  crash: " + frontal_crash);
            
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
                //if (CarNumber == 1) Debug.Log("Frontal collision, distance: " + hit.distance);
                had_hit = true;
                had_hit_frontally = true;

                if (hit.distance < 5 && m_Car.CurrentSpeed < 1) //recovery from frontal hit
                {
                    frontal_crash = true;
                    //if (CarNumber == 1) Debug.Log("Collision STOP");
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
                //if (CarNumber == 2)
                    //if (CarNumber == 1) Debug.Log("Right collision");
                had_hit = true;


            }

            if (Physics.Raycast(transform.position + transform.right + transform.up, transform.TransformDirection(new Vector3(1, 0, 1)), out hit, Mathf.Sqrt(maxRange.x * maxRange.x + maxRange.z * maxRange.z) / 3f) && hit.collider.gameObject.name == "Cube")
            {
                Vector3 closestObstacleInFront = transform.TransformDirection(new Vector3(1, 0, 1)) * hit.distance;
                Debug.DrawRay(transform.position, closestObstacleInFront, Color.red);
                this.accelerationAmount *= 0.7f;
                this.footbrake = this.footbrake < 0.1f ? 0.3f : this.footbrake * 1.5f;
                this.steeringAmount += -0.5f;
                //if (CarNumber == 2)
                    //if (CarNumber == 1) Debug.Log("Right-up collision " + hit.collider.gameObject.name);
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
                //if (CarNumber == 2)
                    //if (CarNumber == 1) Debug.Log("left-up collision " + hit.collider.gameObject.name);
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
                //if (CarNumber == 2)
                    //if (CarNumber == 1) Debug.Log("Left collision");

                had_hit = true;
            }

            if (!had_hit && !curve_approaching)
            {
                this.accelerationAmount *= 1.25f;
                //if (CarNumber == 2)
                    //if (CarNumber == 1) Debug.Log("Not hit speed");
            }

            if (!had_hit && m_Car.CurrentSpeed < 1f)
            {
                //if (CarNumber == 1) Debug.Log("Had hit backward");
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
                        /*
                        if(n.dangerLevel==1)
                        {
                            Color[] colors1 = {Color.cyan, Color.yellow, Color.white, Color.black, Color.green, Color.magenta};
                            Gizmos.color = colors1[n.assigned_enemy];
                        }
                        */

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

                // show current enemy
                Gizmos.color = Color.black;
                Gizmos.DrawSphere(targetEnemy, graph.x_unit);

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
