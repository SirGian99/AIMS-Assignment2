﻿using System.Collections;
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
        Vector3 carSize = new Vector3(4.5f, 0.41f, 4.5f);
        private Rigidbody rigidbody;
        public int CarNumber;


        // Variables for Terrain
        public GameObject terrain_manager_game_object;
        TerrainManager terrain_manager;
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
        enum car_state { drive, front_crash, back_crash };
        private car_state car_status;
        private Vector3 waypoint, nextwaypoint, new_pos = Vector3.zero, x1;
        float angle, spacing, speed=0f, v2, v1=0f;
        private int path_index;
        private List<Vector3> my_path;

        private void Start()
        {
            // Initialize Variables
            Time.timeScale = 1;
            driveTimer = 0f;
            max_speed = 150f;
            acceleration = 1f;
            MazeComplete = false;
            mazeTimer = 0f;
            gunRange = 10f;
            car_status = car_state.drive;
            stuckTimer = 0f;
            PlayerCrashed = false;
            recoveryTime = 0.7f;
            recoverySteer = 0.45f;//45 degrees gives best result
            nextwaypoint = Vector3.up;
            angle = 90;
            spacing = 40;
            x1 = GameObject.FindWithTag("Leader").transform.position;


            // Initialize Car and Terrain
            m_Car = GetComponent<CarController>();
            terrain_manager = terrain_manager_game_object.GetComponent<TerrainManager>();
            start_pos = terrain_manager.myInfo.start_pos;
            rigidbody = GetComponent<Rigidbody>();
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
        }


        private void FixedUpdate()
        {
            mazeTimer += Time.deltaTime;
            enemies = GameObject.FindGameObjectsWithTag("Enemy");
            //Debug.Log("Enemies remaining: " + enemies.Length + " Time Elapsed: " + mazeTimer);


            // Drive Car
            if (!MazeComplete)
            {
                // Find new waypoints
                waypoint = nextwaypoint;
                nextwaypoint = FormationPoint();
                my_path.Add(nextwaypoint);

                // Calculate required acceleration:
                acceleration = (v2 - v1)/ Time.deltaTime;
                acceleration *= 10f;
                if (Vector3.Distance(m_Car.transform.position, waypoint) > 1f)
                {
                    acceleration *= 5f;
                }
                Debug.Log("Car: " + CarNumber + " speed- " + v2 + " acc- " + acceleration);

                // Execute car control
                DriveCar(m_Car, waypoint, nextwaypoint);
                //path_index = DriveCarwithCompetePath(my_path, m_Car, path_index);

                //Check if all enemies killed for maze completion
                if (enemies.Length <= 0)
                {
                    MazeComplete = true;
                    Debug.Log("Car " + CarNumber + "Path Completed in " + mazeTimer);
                }
            }
            else
            {
                // Do nothing for now - wait for others to finish

            }

            // Exit Game logic
            if (enemies.Length <= 0)
            {
                Debug.Log("All enemies killed in " + mazeTimer + " Exiting Game...");
                UnityEditor.EditorApplication.isPlaying = false;
            }
        }


        // MAIN FUNC: Follow the leader with spacing
        private Vector3 FormationPoint()
        {
            Transform leader = GameObject.FindWithTag("Leader").transform;

            float totalSpacing = spacing;
            int checkDistance = 4;

            //Calculate speed of leader car:
            v1 = speed;
            speed = Vector3.Distance(leader.position, x1) / Time.deltaTime;
            x1 = leader.position;
            v2 = speed;

            // Check if path in front of leader is narrow or not and assign spacing
            bool narrowpath = false;
            for (int i = -checkDistance; i <= checkDistance; ++i)
            {
                for (int j = -checkDistance; j <= checkDistance; ++j)
                {
                    narrowpath = narrowpath || terrain_manager.myInfo.traversability[terrain_manager.myInfo.get_i_index(i + (leader.position + Quaternion.AngleAxis(-angle, leader.up) * -leader.forward * 3 * totalSpacing).x), terrain_manager.myInfo.get_j_index(j + (leader.position + Quaternion.AngleAxis(-angle, leader.up) * -leader.forward * 3 * totalSpacing).z)] > 0.5f;
                    narrowpath = narrowpath || terrain_manager.myInfo.traversability[terrain_manager.myInfo.get_i_index(i + (leader.position + Quaternion.AngleAxis(-angle, leader.up) * -leader.forward * 2 * totalSpacing).x), terrain_manager.myInfo.get_j_index(j + (leader.position + Quaternion.AngleAxis(-angle, leader.up) * -leader.forward * 2 * totalSpacing).z)] > 0.5f;

                }
            }
            Debug.Log("Narrow Path dectected  = " + narrowpath);
            // Adjust position based on collision status
            if (narrowpath)
            {
                // Do tight spacing
                switch (CarNumber)
                {
                    case 1:
                        new_pos = Vector3.Lerp(new_pos, Quaternion.AngleAxis(angle, leader.up) * -leader.forward * (totalSpacing / 2), Time.deltaTime / speed);
                        break;
                    case 2:
                        new_pos = Vector3.Lerp(new_pos, Quaternion.AngleAxis(angle / 2, leader.up) * -leader.forward * (totalSpacing / 2), Time.deltaTime / speed);
                        break;
                    case 3:
                        new_pos = Vector3.Lerp(new_pos, Quaternion.AngleAxis(-angle, leader.up) * -leader.forward * (totalSpacing / 2), Time.deltaTime / speed);
                        break;
                    case 4:
                        new_pos = Vector3.Lerp(new_pos, Quaternion.AngleAxis(-angle / 2, leader.up) * -leader.forward * (totalSpacing / 2), Time.deltaTime / speed);
                        break;
                    default:
                        new_pos = Vector3.Lerp(new_pos, Quaternion.AngleAxis(-180, leader.up) * -leader.forward, Time.deltaTime / speed);
                        break;
                }
            }
            else
            {
                // Do sparse spacing
                switch (CarNumber)
                {
                    case 1:
                        new_pos = Vector3.Lerp(new_pos, Quaternion.AngleAxis(angle, leader.up) * -leader.forward * totalSpacing, Time.deltaTime / speed);
                        break;
                    case 2:
                        new_pos = Vector3.Lerp(new_pos, Quaternion.AngleAxis(angle / 2, leader.up) * -leader.forward * (totalSpacing / 2), Time.deltaTime / speed);
                        break;
                    case 3:
                        new_pos = Vector3.Lerp(new_pos, Quaternion.AngleAxis(-angle, leader.up) * -leader.forward * totalSpacing, Time.deltaTime / speed);
                        break;
                    case 4:
                        new_pos = Vector3.Lerp(new_pos, Quaternion.AngleAxis(-angle / 2, leader.up) * -leader.forward * (totalSpacing / 2), Time.deltaTime / speed);
                        break;
                    default:
                        new_pos = Vector3.Lerp(new_pos, Quaternion.AngleAxis(-180, leader.up) * -leader.forward, Time.deltaTime / speed);
                        break;
                }
            }


            return leader.position + new_pos;
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
        public void DriveCar(CarController player_Car, Vector3 player_waypoint, Vector3 player_nextwaypoint)
        {
            float car_steer, car_acc, car_steer_next;

            //find steering needed to get to next point
            car_steer = Steer(player_Car.transform.position, player_Car.transform.eulerAngles.y, player_waypoint);
            car_acc = Accelerate(player_Car.transform.position, player_Car.transform.eulerAngles.y, player_waypoint);
            car_steer_next = Steer(player_Car.transform.position, player_Car.transform.eulerAngles.y, player_nextwaypoint);
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
                if (driveTimer >= stuckTimer && Vector3.Distance(start_pos, player_Car.transform.position) > 0 && mazeTimer>=5)
                {
                    driveTimer = 0;
                    //Debug.Log("Crash pos: " + Vector3.Distance(previous_pos, player_Car.transform.position));
                    //check: car not moved very much from previous position
                    if (Vector3.Distance(previous_pos, player_Car.transform.position) < 0.05f)
                    {
                        //PlayerCrashed = true; ///// SUPPRESS CRASH RECOVERY FOR NOW
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

        }

        // MAIN FUNC: Car Drive
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

        //Sub-Func: To find accleration for car
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
            //float car_length = 4.47f, car_width = 2.43f, car_high = 2f;
            //float scale = 1f;
            //Vector3 cube_size = new Vector3(car_width * scale, car_high * scale, car_length * scale);

            Gizmos.color = Color.blue;

            Transform leader = GameObject.FindWithTag("Leader").transform;
            Gizmos.DrawSphere(leader.position + new_pos, 1f);


            //Show the path to the goal
            if (my_path != null)
            {
                Color[] colors = { Color.red, Color.cyan, Color.yellow, Color.white, Color.black, Color.green };
                Gizmos.color = colors[CarNumber - 1];
                for (int i = 0; i < my_path.Count - 1; ++i)
                {
                    Gizmos.DrawLine(my_path[i], my_path[i + 1]);
                }
            }
        }




    }
}
