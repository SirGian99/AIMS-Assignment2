using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;



namespace UnityStandardAssets.Vehicles.Car
{
    [RequireComponent(typeof(CarController))]
    public class CarAI_RRT : MonoBehaviour
    {
        private CarController m_Car; // the car controller we want to use

        public GameObject terrain_manager_game_object;
        TerrainManager terrain_manager;

        // Players and Turrets
        public GameObject[] friends;
        public GameObject[] enemies;



        // Generic Variables
        private float time_step;
        private Vector3 start_pos, goal_pos;
        int alive_players; 

        // Variables for Car
        private float car_length;
        private float acceleration;
        private float max_speed;
        private float start_theta, start_speed;
        

        // Variables for RRT
        public int n;
        private float grid_size;
        private int Xgrid, Zgrid;
        private float map_width, map_height;
        private int map_size;
        private List<CarTreeNode> my_path;
        private List<CarTreeNode>[,] map;
        private CarConfigSpace ObstacleSpace;
        public bool Visualize = true;

        // Variables for path
        private List<List<Vector3>> all_paths;

        // Variables for driving
        private bool MazeComplete;
        private float time;
        private int path_index;


        private void Start()
        {
            Debug.Log("START");

            // get the car controller & terrain info
            //Debug.Log("Getting car and terrain info");
            m_Car = GetComponent<CarController>();
            terrain_manager = terrain_manager_game_object.GetComponent<TerrainManager>();
            start_pos = terrain_manager.myInfo.start_pos;
            goal_pos = terrain_manager.myInfo.goal_pos;
            start_theta = m_Car.transform.eulerAngles.y;
            start_speed = m_Car.CurrentSpeed;


            // Initialize Variables
            //Debug.Log("Initializing variables");
            Time.timeScale = 1;
            time_step = 0.05f;
            max_speed = 20;
            acceleration = 1f;
            n = 5500000;
            time = 0;
            MazeComplete = false;
            path_index = 1;



            // note that both arrays will have holes when objects are destroyed
            // but for initial planning they should work
            friends = GameObject.FindGameObjectsWithTag("Player");
            // Note that you are not allowed to check the positions of the turrets in this problem


            // Initialize ConfigSpace
            //Debug.Log("Initialize ConfigSpace");
            CreateObstacleSpace();

            // Run RRT
            //Debug.Log("Search for paths");
            my_path = new List<CarTreeNode>();
            my_path = RRT();


            if (my_path == null)
            {
                Debug.Log("No path found!");
            }



            // choose path based on starting pos

        }


        private void FixedUpdate()
        {
            enemies = GameObject.FindGameObjectsWithTag("Enemy");

            // Drive Car
            if (!MazeComplete)
            {
                time += Time.deltaTime;
                //path_index = ExecutePath(my_path, m_Car, path_index);

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

        /// FUNCTIONS

        // MAIN FUNC: To create obstacle space
        private void CreateObstacleSpace()
        {
            // Find Car Size
            //Debug.Log("Finding car size");
            WheelCollider[] wheels = FindObjectsOfType<WheelCollider>();
            float maxval = 0;
            car_length = 0;
            for (int i = 1; i < wheels.Length; i++)
            {
                if ((wheels[0].transform.position - wheels[i].transform.position).magnitude > maxval)
                {
                    car_length = maxval;
                    maxval = (wheels[0].transform.position - wheels[i].transform.position).magnitude;
                }
            }
            //car_length = Mathf.Max(m_Car.renderer.bounds.size.x, m_Car.renderer.bounds.size.z);

            //Obstacle Sapce
            Quaternion carRotation = m_Car.transform.rotation;
            m_Car.transform.rotation = Quaternion.identity;
            ObstacleSpace = new CarConfigSpace();
            BoxCollider carCollider = GameObject.Find("ColliderBottom").GetComponent<BoxCollider>();
            ObstacleSpace.BoxSize = carCollider.transform.TransformVector(carCollider.size);
            m_Car.transform.rotation = carRotation;

            // Terrain space
            //Debug.Log("Finding terrain grid");
            map_width = (terrain_manager.myInfo.x_high - terrain_manager.myInfo.x_low);
            map_height = (terrain_manager.myInfo.z_high - terrain_manager.myInfo.z_low);
            grid_size = Mathf.Min(map_width / terrain_manager.myInfo.x_N, map_height / terrain_manager.myInfo.z_N);
            Xgrid = (int)(map_width / grid_size);
            Zgrid = (int)(map_height / grid_size);
            CarTreeNode start_node = new CarTreeNode(start_pos, m_Car.transform.eulerAngles.y, 0, 0);
            map = new List<CarTreeNode>[Xgrid, Zgrid];
            for (int i = 0; i < Xgrid; i++)
            {
                for (int j = 0; j < Zgrid; j++)
                {
                    if (NoObstacle(i, j))
                    {
                        map[i, j] = new List<CarTreeNode>();
                    }
                }
            }
            map[NeighbourIndicesX(start_node.position.x), NeighbourIndicesZ(start_node.position.z)].Add(start_node);
            map_size = 1;

        }

        //Sub-func: Check if any obstacle in map
        private bool NoObstacle(int i, int j)
        {
            float x = grid_size * i + grid_size / 2 + terrain_manager.myInfo.x_low;
            float z = grid_size * j + grid_size / 2 + terrain_manager.myInfo.z_low;

            // check traversablity of the position
            if (terrain_manager.myInfo.traversability[terrain_manager.myInfo.get_i_index(x), terrain_manager.myInfo.get_j_index(z)] > 0.5f)
            {
                return false;
            }
            else
            {
                return true;
            }
        }

        // MAIN FUNC: perform RRT
        private List<CarTreeNode> RRT()
        {
            bool ReachedGoal = false;
            int goal_count = 0;
            CarTreeNode finalwaypoint = null;
            CarTreeNode b, c;
            float TowardsGoal = Vector3.Distance(start_pos, goal_pos);
            for (int i = 0; i < n; i++)
            {
                // pick a random point
                //Debug.Log(i + "Pick random point");
                Vector3 a;
                do
                {
                    if ((ReachedGoal ? 1 : UnityEngine.Random.Range(0f, 1f)) > 0.1f)
                    {
                        // Choose point within goal distance if goal found already
                        Vector2 p = UnityEngine.Random.insideUnitCircle * TowardsGoal;
                        a = new Vector3(p.x, 0, p.y);
                    }
                    else
                    {
                        // choose new point on grid
                        a = new Vector3(UnityEngine.Random.Range(terrain_manager.myInfo.x_low, terrain_manager.myInfo.x_high),
                                            0, UnityEngine.Random.Range(terrain_manager.myInfo.z_low, terrain_manager.myInfo.z_high));
                    }
                } while (ObstacleSpace.Collision(a.x, a.z, 0));

                // find nearest node b to a
                //Debug.Log(i + "Find nearest node");
                List<CarTreeNode> b_list = NearestNeighbours(a, 1);
                b = b_list.Count > 0 ? b_list[0] : null; ;
                if (b == null)
                {
                    continue;
                }

                // find inputs u to go from b to a & apply inputs u to reach c
                //Debug.Log(i + "Apply inputs u");
                c = DriveCar(b, a, 1);

                // Check if new point exists
                if (c != null)
                {
                    // Check if new point has no obstacle
                    //Debug.Log(i + "Check if "+ c + "has no obstacle");
                    int x_i = NeighbourIndicesX(c.position.x);
                    int z_j = NeighbourIndicesZ(c.position.z);
                    if (!NoObstacle(x_i, z_j))
                    {
                        continue;
                    }

                    //// RRT
                    // Add node to tree
                    //Debug.Log(n + "Add node to tree");
                    //map[x_i, z_j].Add(c);
                    //c.parent = b;
                    //b.children.Add(c);

                    //// RRT*
                    //find k-neighbours of c
                    //Debug.Log(i + "Find k neighbours");
                    List<CarTreeNode> c_list = NearestNeighbours(c.position, Mathf.Min(5 + map_size / 1000, map_size));
                    map[x_i, z_j].Add(c);
                    map_size++;

                    //make closest obstacle free point p the parent to c
                    CarTreeNode min_b = b;
                    float min_cost = c.cost;
                    foreach (CarTreeNode node in c_list)
                    {
                        CarTreeNode p = DriveCar(node, c.position, 3);
                        if (p != null)
                        {
                            if (Vector3.Distance(p.position, c.position) <= 0.1f && p.cost < min_cost)
                            {
                                min_cost = p.cost;
                                min_b = node;
                            }
                        }
                    }

                    //make k-neighbours children of parent p if they are close
                    //Debug.Log(i + " c = " + c.position + " goal = " + goal_pos);
                    c.parent = min_b;
                    min_b.children.Add(c);

                    //Go towards goal
                    if (Vector3.Distance(c.position, goal_pos) < TowardsGoal)
                    {
                        //Debug.Log(i + "Update goal distance");
                        TowardsGoal = Vector3.Distance(c.position, goal_pos);
                    }


                    // Check if goal found
                    if (Vector3.Distance(c.position, goal_pos) <= 5)
                    {
                        //Debug.Log("Goal is close");
                        ReachedGoal = true;
                        goal_count++;

                        if (finalwaypoint == null || c.cost < finalwaypoint.cost)
                        {
                            finalwaypoint = c;
                        }

                        if (goal_count >= 10)
                        {
                            Debug.Log("Reached Goals" + goal_count + "on Tree");
                            break;
                        }
                    }
                }
            }

            // Return path if goal reached
            if (ReachedGoal == true)
            {
                Debug.Log("Goal found - Computing final path");
                List<CarTreeNode> finalpath = ComputePath(finalwaypoint);
                return finalpath;
            }
            else
            {
                Debug.Log("No path found");
                return null;
            }

        }

        // Sub-func: Get the nearest node in tree to point a - NOT USED!!!!!
        public CarTreeNode NearestNode(Vector3 a, List<CarTreeNode> tree)
        {
            float shortest = 0;
            float distance;
            CarTreeNode nearest = null;
            foreach (CarTreeNode node in tree)
            {
                distance = Vector3.Distance(node.position, a);
                if (distance < shortest)
                {
                    shortest = distance;
                    nearest = node;
                }
            }
            return nearest;
        }

        // Sub-func: Find and apply steering & acceleration controls get to a from b
        public CarTreeNode DriveCar(CarTreeNode b, Vector3 target, int factor)
        {
            int step_size = 5;
            Vector3 position = b.position;
            float theta = b.theta;
            float speed = b.speed;
            float cost = b.cost;
            for (int i = 0; i < step_size * factor; i++)
            {
                float delta_phi = m_Car.m_MaximumSteerAngle * Steer(position, theta, target);
                float brake = 1;

                // if acceleration is negative - invert steeing angle --> backwards driving
                if (Accelerate(position, theta, target) < 0)
                {
                    delta_phi = -delta_phi;
                }
                if (Mathf.Abs(delta_phi) <= 0.4f * m_Car.m_MaximumSteerAngle)
                {
                    delta_phi = 0;
                }
                // steering is too high
                else if (Mathf.Abs(delta_phi) > 0.8f * m_Car.m_MaximumSteerAngle)
                {
                    if (Mathf.Abs(speed) > max_speed / 10)
                    {
                        brake = -1;
                    }
                }

                // Calculate motion model values according to kinematic car model
                // x is forward, Z is sideways??
                // xDiff = v sin theta
                // zDiff = v cos theta
                // thetaDiff = v/L tan phi
                //float xDiff = speed * Mathf.Sin(Mathf.Deg2Rad * theta);
                //float zDiff = speed * Mathf.Cos(Mathf.Deg2Rad * theta);
                //float thetaDiff = (speed / car_length) * Mathf.Tan(Mathf.Deg2Rad * delta_phi) * Mathf.Rad2Deg;

                // Find next position & orientation using Euler's method
                //float new_theta = theta + thetaDiff * time_step;
                //Vector3 new_pos = new Vector3((position.x + xDiff * time_step), 0, (position.z + zDiff * time_step));
                //float new_cost = b.cost + Vector3.Distance(position, new_pos);
                //position = new_pos;
                //theta = new_theta;

                // Find next position & orientation using RK4 method
                float[] next_move = car_motion(position.x, position.z, theta, delta_phi, speed, time_step);
                Vector3 new_pos = new Vector3(next_move[0], next_move[1]);
                position.x = next_move[0];
                position.z = next_move[1];
                theta = next_move[2];
                cost = b.cost + Vector3.Distance(position, new_pos);

                //if collision, return null
                if (ObstacleSpace.Collision(position.x, position.z, theta))
                {
                    return null;
                }

                //if close to target, stop interating
                if (Vector3.Distance(position, target) <= 0.1f)
                {
                    break;
                }

                //Update speed
                speed = speed + Mathf.Clamp(Accelerate(position, theta, target) * acceleration * time_step * brake, -max_speed, max_speed);
            }
            CarTreeNode c = new CarTreeNode(position, theta, speed, cost);
            return c;
        }

        // Sub-Func: Car motion using RK4
        private float[] car_motion(float x, float z, float theta, float delta_phi, float speed, float step)
        {
            float[] k1 = car_model(speed, theta, delta_phi);
            float[] k2 = car_model(speed, theta + k1[2] * (step / 2), delta_phi);
            float[] k3 = car_model(speed, theta + k2[2] * (step / 2), delta_phi);
            float[] k4 = car_model(speed, theta + k3[2] * (step / 2), delta_phi);

            float[] new_move = new float[] { 0, 0, 0 };
            float[] old_move = new float[] { x, z, theta };
            for (int i = 0; i < 3; i++)
            {
                new_move[i] = old_move[i] + (step / 6) * (k1[i] + k2[i] + k3[i] + k4[i]);
            }
            return new_move;

        }

        // Sub-Func: Kinematic Motion model
        private float[] car_model(float speed, float theta, float delta_phi)
        {
            float xDiff = speed * Mathf.Sin(Mathf.Deg2Rad * theta);
            float zDiff = speed * Mathf.Cos(Mathf.Deg2Rad * theta);
            float thetaDiff = (speed / car_length) * Mathf.Tan(Mathf.Deg2Rad * delta_phi) * Mathf.Rad2Deg;
            float[] diff = new float[] { xDiff, zDiff, thetaDiff };
            return diff;
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

        // Sub func: To find indexes of nearest neighbour - i
        public int NeighbourIndicesX(float x)
        {
            int i = (int)Mathf.Floor(Xgrid * (x - terrain_manager.myInfo.x_low) / map_width);
            if (i < 0)
            {
                i = 0;
            }
            else if (i > Xgrid - 1)
            {
                i = Xgrid - 1;
            }
            return i;
        }

        // Sub func: To find indexes of nearest neighbour - j
        public int NeighbourIndicesZ(float z)
        {
            int j = (int)Mathf.Floor(Zgrid * (z - terrain_manager.myInfo.z_low) / map_height);
            if (j < 0)
            {
                j = 0;
            }
            else if (j > Zgrid - 1)
            {
                j = Zgrid - 1;
            }

            return j;
        }

        // Sub-func: Find Neighbours of a point
        public List<CarTreeNode> NearestNeighbours(Vector3 a, int k)
        {
            List<float> closest = new List<float>();
            List<CarTreeNode> knn = new List<CarTreeNode>();

            int x_i = NeighbourIndicesX(a.x);
            int z_j = NeighbourIndicesZ(a.z);

            //Debug.Log("Start neighbour queue/search");
            Queue<int[]> queue = new Queue<int[]>();
            bool[,] visited = new bool[Xgrid, Zgrid];
            int[] startCell = { x_i, z_j };
            int[] stopBlock = { -1, -1 };

            queue.Enqueue(startCell);
            queue.Enqueue(stopBlock);

            int blocks = 1;
            int current = 0;

            while (queue.Count > 0)
            {
                int[] cell = queue.Dequeue();

                //Stop block
                if (cell[0] == -1 && cell[1] == -1)
                {
                    current = 0;
                    blocks = queue.Count - 1;

                    //If we have found a point on this level, then return
                    if (knn.Count == k)
                    {
                        return knn;
                    }
                }
                else if (!visited[cell[0], cell[1]])
                {
                    visited[cell[0], cell[1]] = true;

                    //No need to check if obstacle
                    if (!NoObstacle(cell[0], cell[1]))
                    {
                        current++;
                        if (current == blocks)
                        {
                            queue.Enqueue(stopBlock);
                        }
                    }
                    else
                    {
                        foreach (CarTreeNode node in map[cell[0], cell[1]])
                        {
                            float distance = Vector3.Distance(node.position, a);
                            if (knn.Count < k)
                            {
                                knn.Add(node);
                                closest.Add(distance);
                            }
                            else
                            {
                                int maxIndex = 0;
                                float maxDist = 0;
                                for (int i = 0; i < k; ++i)
                                {
                                    if (closest[i] > maxDist)
                                    {
                                        maxIndex = i;
                                        maxDist = closest[i];
                                    }
                                }
                                if (distance < closest[maxIndex])
                                {
                                    closest[maxIndex] = distance;
                                    knn[maxIndex] = node;
                                }
                            }

                        }

                        //increase queue sixe if needed
                        if (knn.Count < k)
                        {
                            current++;
                            for (int i = -1; i <= 1; ++i)
                            {
                                for (int j = -1; j <= 1; ++j)
                                {
                                    if (cell[0] + i >= 0 && cell[0] + i < Xgrid && cell[1] + j >= 0 && cell[1] + j < Zgrid)
                                    {
                                        queue.Enqueue(new int[] { cell[0] + i, cell[1] + j });
                                    }
                                }
                            }
                            if (current == blocks)
                            {
                                queue.Enqueue(stopBlock);
                            }
                        }
                        else
                        {
                            current++;
                        }
                    }
                }
                else
                {
                    current++;
                    if (current == blocks)
                    {
                        queue.Enqueue(stopBlock);
                    }
                }
            }
            //Debug.Log("Return knn");
            return knn;
        }

        // Sub-Func: Compute path 
        private List<CarTreeNode> ComputePath(CarTreeNode node)
        {
            List<CarTreeNode> FinalPath = new List<CarTreeNode>();
            CarTreeNode goal_node = new CarTreeNode(goal_pos, 0f, 0f, 0f);
            while (node.parent != null)
            {
                FinalPath.Insert(0, node.parent);
                node = node.parent;
            }
            FinalPath.Add(goal_node);
            return FinalPath;
        }



        // MAIN FUNC: Visualize paths
        private void OnDrawGizmos()
        {
            if (!Application.isPlaying)
            {
                return;
            }

            // Show trees
            if (Visualize == true)
            {
                Gizmos.color = Color.blue;
                for (int i = 0; i < Xgrid; ++i)
                {
                    for (int j = 0; j < Zgrid; ++j)
                    {
                        if (NoObstacle(i, j))
                        {
                            for (int k = 0; k < map[i, j].Count; ++k)
                            {
                                foreach (CarTreeNode child in map[i, j][k].children)
                                {
                                    Gizmos.DrawLine(map[i, j][k].position + Vector3.up * 0.75f, child.position + Vector3.up * 0.75f);
                                }
                            }
                        }
                    }
                }
            }

            //Show the path to the goal
            if (my_path != null)
            {
                Gizmos.color = Color.white;
                for (int i = 0; i < my_path.Count - 1; ++i)
                {
                    Gizmos.DrawLine(my_path[i].position + Vector3.up, my_path[i + 1].position + Vector3.up);
                }
            }
        }
    }
}
