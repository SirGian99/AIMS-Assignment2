using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;


namespace UnityStandardAssets.Vehicles.Car
{
    [RequireComponent(typeof(CarController))]
    public class CarAI4 : MonoBehaviour
    {
        // Variables for Car
        private CarController m_Car; // the car controller we want to use
        public int CarNumber;

        // Variables for Terrain
        public GameObject terrain_manager_game_object;
        TerrainManager terrain_manager;
        private CarConfigSpace ObstacleSpace;

        // Variables for Players and Turrets
        public GameObject[] friends;
        public GameObject[] enemies;

        // Variables for path & driving
        private float acceleration, max_speed;
        private bool MazeComplete, PlayerCrashed;
        private float mazeTimer, driveTimer, stuckTimer, recoveryTime;
        private float recoverySteer;
        private Vector3 start_pos, previous_pos;
        enum car_state { drive, front_crash, back_crash };
        private car_state car_status;
        private Vector3 waypoint, nextwaypoint, new_pos = Vector3.zero, x1, intermediate;
        private float angle, spacing, checkRadius, speed=0f, v2, v1=0f;
        private int path_index, checkDensity;
        private List<Vector3> my_path, leader_path;
        private List<bool> pathWidth;
        private bool narrowpath = false;

        private int delay = 20;
        private void Start()
        {
            // Initialize Variables
            Time.timeScale = 1;
            driveTimer = 0f;
            max_speed = 130f;
            acceleration = 1f;
            MazeComplete = false;
            mazeTimer = 0f;
            car_status = car_state.drive;
            stuckTimer = 0f;
            PlayerCrashed = false;
            recoveryTime = 0.7f;
            recoverySteer = 0.45f;//45 degrees gives best result
            nextwaypoint = Vector3.up;
            angle = 90;
            spacing = 40;
            checkDensity = 2;
            checkRadius = 1f;
            x1 = GameObject.FindWithTag("Leader").transform.position;


            // Initialize Car and Terrain
            m_Car = GetComponent<CarController>();
            terrain_manager = terrain_manager_game_object.GetComponent<TerrainManager>();
            start_pos = terrain_manager.myInfo.start_pos;
            // Initialize ConfigSpace
            CreateObstacleSpace();

            // Get Array of Friends and Eniemies
            friends = GameObject.FindGameObjectsWithTag("Player");
            int i = 0;
            foreach (GameObject friend in friends)
            {
                if (friend.name == this.name)
                {
                    CarNumber = i + 1;
                    break;
                }
                i++;
            }
            enemies = GameObject.FindGameObjectsWithTag("Enemy");

            // path
            my_path = new List<Vector3>();
            my_path.Add(start_pos);
            leader_path = new List<Vector3>();
            pathWidth = new List<bool>();
        }


        private void FixedUpdate()
        {
            mazeTimer += Time.deltaTime;
            enemies = GameObject.FindGameObjectsWithTag("Enemy");
            if(CarNumber == 1)
            {
                Debug.Log("Enemies remaining: " + enemies.Length + " Time Elapsed: " + mazeTimer);
            }

            if (delay > 0)
            {
                delay--;
                return;
            }
            // Drive Car
            if (!MazeComplete)
            {
                // Find new waypoints
                waypoint = nextwaypoint;
                nextwaypoint = FormationPoint();
                nextwaypoint = adjustWaypoint(nextwaypoint);

                my_path.Add(nextwaypoint);
                leader_path.Add(GameObject.FindWithTag("Leader").transform.position);
                pathWidth.Add(narrowpath);

                // Calculate required acceleration:
                acceleration = ((v2 - v1) / Time.deltaTime);
                acceleration *= Vector3.Distance(m_Car.transform.position, waypoint) * max_speed * 10f;

                // Execute car control
                DriveCar(m_Car, waypoint, nextwaypoint);
                //ImprovedDriveCar(m_Car, waypoint, nextwaypoint);
                //path_index = DriveCarwithCompetePath(my_path, m_Car, path_index);

                //Check if all enemies killed for maze completion or if leader stopped
                if (enemies.Length <= 0 || Vector3.Distance(leader_path[leader_path.Count - 1], leader_path[leader_path.Count - 10]) == 0)
                {
                    MazeComplete = true;
                }
            }
            else
            {
                // Do nothing for now - wait for others to finish

            }

            // Exit Game logic
            if (enemies.Length <= 0)
            {
                Debug.Log("Leader stopped or All enemies killed in " + mazeTimer + " Exiting Game...");
                UnityEditor.EditorApplication.isPlaying = false;
            }
        }

        private Vector3 adjustWaypoint(Vector3 waypoint)
        {
            RaycastHit hit;
            float range = 100;

            // Cast a sphere wrapping character controller 10 meters forward
            // to see if it is about to hit anything.
            Collider[] hitColliders = Physics.OverlapSphere(waypoint, range);
            Vector3 to_return = waypoint;
            float distance = float.MaxValue;
            foreach (var hitCollider in hitColliders)
            {
                if(hitCollider.name == "Cube")
                {
                    if (Physics.Raycast(waypoint, (hitCollider.transform.position - waypoint).normalized, out hit, range) && (hit.collider.name == "Cube"|| hit.collider.name.ToLower().Contains("car")))
                    {
                        Vector3 hitpoint = hit.point;
                        if (hit.distance < distance)
                        {
                            distance = hit.distance;
                            to_return = Vector3.MoveTowards(hitpoint, waypoint, distance * 1.2f);
                        }
                    }
                }

            }
            return to_return;
            
        }


        // MAIN FUNC: Follow the leader with spacing
        private Vector3 FormationPoint()
        {
            Transform leader = GameObject.FindWithTag("Leader").transform;

            float totalSpacing = spacing, currentSpacing, leftSpacing, rightSpacing;
            int levels = 10;
            bool left, right;
            List<bool> levelIntensity = new List<bool>(new bool[levels]);
            float scale;

            //Calculate speed of leader car:
            v1 = speed;
            speed = Vector3.Distance(leader.position, x1) / Time.deltaTime;
            x1 = leader.position;
            v2 = speed;

            // Check if path in front of leader is narrow or not and assign spacing
            narrowpath = false;
            left = false;
            right = false;
            for (int i = -checkDensity; i <= checkDensity; ++i)
            {
                for (int j = -checkDensity; j <= checkDensity; ++j)
                {
                    for (float refAngle = -180; refAngle <= 180; refAngle = refAngle + 10)
                    {
                        float ellipse = .75f;
                        if (refAngle >= -35 && refAngle <= 35)
                        {
                            ellipse = 2.5f;
                        }
                        Vector3 refline = Quaternion.AngleAxis(refAngle, leader.up) * leader.forward * ellipse * totalSpacing;
                        for (float k = 0.0f; k <= checkRadius; k = k + 0.1f)
                        {
                            float x = i + (leader.position + k * refline).x;
                            float z = j + (leader.position + k * refline).z;
                            narrowpath = narrowpath || Obstacle(x, z);
                            //Debug.DrawLine(leader.position, new Vector3(x, 0, z));
                            if (Obstacle(x, z))
                            {
                                levelIntensity[(int)(k * levels)] = true;
                                if (refAngle >= -180 && refAngle < 0)
                                {
                                    left = true;

                                }
                                else
                                {
                                    right = true;
                                }
                                //Debug.Log("Level: " + (k * levels) + " k " + k);
                            }
                        }
                    }
                }
            }

            //Debug.Log("Narrow Path dectected  = " + narrowpath);
            // Do sparse spacing
            currentSpacing = totalSpacing;
            leftSpacing = currentSpacing;
            rightSpacing = currentSpacing;
            scale = 2f;
            // Adjust position based on collision status
            if (narrowpath)
            {
                //Debug.Log("L1: " + L1 + " L2: " + L2 + " L3: " + L3 + " L4: " + L4);
                // Do tight spacing
                for (int i = levels - 1; i >= 0; i--)
                {
                    if (levelIntensity[i] == true)
                    {
                        currentSpacing = totalSpacing / ((levels - i + 1) * 3);
                        // Debug.Log("Level: " + i + " spacing " + (levels - i + 1) * 2);
                        scale = speed / ((levels - i + 1) * 4);
                        //rightSpacing = totalSpacing / 2;
                        //leftSpacing = totalSpacing / 2;
                        if (left)
                        {
                            leftSpacing = currentSpacing;
                        }
                        if (right)
                        {
                            rightSpacing = currentSpacing;
                        }

                    }
                }
                //Debug.Log("Current: " + currentSpacing + " scale " + scale);
            }


            // Find new positions
            switch (CarNumber)
            {
                case 1:
                    intermediate = Quaternion.AngleAxis(angle, leader.up) * -leader.forward * leftSpacing;
                    break;
                case 2:
                    intermediate = Quaternion.AngleAxis(angle / 2, leader.up) * -leader.forward * leftSpacing / 2;
                    break;
                case 3:
                    intermediate = Quaternion.AngleAxis(-angle, leader.up) * -leader.forward * rightSpacing;
                    break;
                case 4:
                    intermediate = Quaternion.AngleAxis(-angle / 2, leader.up) * -leader.forward * rightSpacing / 2;
                    break;
                default:
                    intermediate = Quaternion.AngleAxis(-180, leader.up) * -leader.forward * currentSpacing;
                    break;
            }

            new_pos = Vector3.Lerp(new_pos, intermediate, Time.deltaTime / scale);

            return leader.position + new_pos;
        }
        //Sub-func: Check if any obstacle
        private bool Obstacle(float x, float z)
        {
            // check traversablity of the position
            if (terrain_manager.myInfo.traversability[terrain_manager.myInfo.get_i_index(x), terrain_manager.myInfo.get_j_index(z)] > 0.5f)
            {
                return true;
            }
            else
            {
                return false;
            }
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


        // MAIN FUNC: Car Drive with just waypoints
        public void DriveCar(CarController player_Car, Vector3 player_waypoint, Vector3 player_nextwaypoint)
        {
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
                if (driveTimer >= stuckTimer && Vector3.Distance(start_pos, player_Car.transform.position) > 0 && mazeTimer>=5)
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

        }

        // MAIN FUNC: Car Drive with complete path
        public int DriveCarwithCompetePath(List<Vector3> player_path, CarController player_Car, int player_pathIndex)
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

        // Sub-Func: To find accleration for car
        private float Accelerate(Vector3 position, float theta, Vector3 target)
        {
            Vector3 direction = Quaternion.Euler(0, theta, 0) * Vector3.forward;
            Vector3 directionToTarget = target - position;
            return Mathf.Clamp(direction.x * directionToTarget.x + direction.z * directionToTarget.z, -1, 1);
        }


        //MAIN FUNC: Visualise
        private void OnDrawGizmos()
        {
            if (!Application.isPlaying)
            {
                return;
            }

            Gizmos.color = Color.blue;

            Transform leader = GameObject.FindWithTag("Leader").transform;
            Gizmos.DrawSphere(leader.position + new_pos, 1f);
            //Gizmos.color = Color.red;
            //Gizmos.DrawSphere(leader.position + intermediate, 1f);

            //Show the path to the goal
            if (my_path != null)
            {
                Color[] colors = { Color.cyan, Color.white, Color.black, Color.green };
                Gizmos.color = colors[CarNumber - 1];
                for (int i = 0; i < my_path.Count - 1; ++i)
                {
                    Gizmos.DrawLine(my_path[i], my_path[i + 1]);
                    ;
                }
            }
            //Show the path to the goal
            if (leader_path != null && pathWidth!=null)
            {
                for (int i = 0; i < leader_path.Count - 1; ++i)
                {
                    Gizmos.color = Color.yellow;
                    if (pathWidth[i])
                    {
                        Gizmos.color = Color.red;
                    }
                    Gizmos.DrawLine(leader_path[i], leader_path[i + 1]);
                    ;
                }
            }
        }




    }
}
