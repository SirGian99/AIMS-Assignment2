using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;


namespace UnityStandardAssets.Vehicles.Car
{
    [RequireComponent(typeof(CarController))]
    public class CarAI4_1: MonoBehaviour
    {
        private CarController m_Car; // the car controller we want to use

        public GameObject terrain_manager_game_object;
        TerrainManager terrain_manager;

        public GameObject[] friends;
        public GameObject[] enemies;

        public float maxVelocity;
        public float acceleration;
        private float timeStep;
        private int numberOfSteps;
        private float time;

        private float steerDirection;
        private float accelerationDirection;
        private float brake;
        private float handBrake;

        private bool crashed;
        private float crashTime;
        private float crashCheckTime;
        private float crashDirection;
        private Vector3 previousPosition;
        private CarConfigSpace ObstacleSpace;

        Vector3 followPoint;
        Vector3 previousPoint;
        int carNumber;

        float angle;
        float spacing;
        Vector3 offset = Vector3.zero;

        bool circularPattern;

        private void Start()
        {
            Time.timeScale = 1;
            maxVelocity = 150;
            acceleration = 1f;

            timeStep = 0.05f;
            numberOfSteps = 5;
            time = 0;

            steerDirection = 0;
            accelerationDirection = 0;
            brake = 0;
            handBrake = 0;
            crashed = false;
            crashTime = 0;
            crashCheckTime = 1f;
            crashDirection = 0;
            previousPosition = Vector3.up;

            terrain_manager = terrain_manager_game_object.GetComponent<TerrainManager>();
            m_Car = GetComponent<CarController>();

            InitializeCSpace();

            circularPattern = true;

            angle = 90;
            spacing = circularPattern ? 25 : 20;


            // note that both arrays will have holes when objects are destroyed
            // but for initial planning they should work
            friends = GameObject.FindGameObjectsWithTag("Player");
            enemies = GameObject.FindGameObjectsWithTag("Enemy");

            carNumber = 0;
            for (int i = 0; i < friends.Length; ++i)
            {
                if (friends[i].name == this.name)
                {
                    carNumber = i;
                    if (circularPattern)
                    {
                        angle = carNumber * 90 - 45;
                    }
                    break;
                }
            }
        }

        private void FixedUpdate()
        {
            previousPoint = followPoint;
            followPoint = FindFollowPoint();
            float pointVelocity = Vector3.Distance(previousPoint, followPoint) / Time.deltaTime;
            if (circularPattern)
            {
                angle += Time.deltaTime * 10;
            }

            if (!crashed)
            {
                time += Time.deltaTime;
                if (time >= crashCheckTime && Vector3.Distance(m_Car.transform.position, terrain_manager.myInfo.start_pos) > 5)
                {
                    time = 0;
                    if (Vector3.Distance(previousPosition, m_Car.transform.position) < 0.1f)
                    {
                        crashed = true;
                        if (Physics.BoxCast(
                            m_Car.transform.position,
                            new Vector3(ObstacleSpace.BoxSize.x / 2, ObstacleSpace.BoxSize.y / 2, 0.5f),
                            m_Car.transform.forward,
                            Quaternion.LookRotation(m_Car.transform.forward),
                            ObstacleSpace.BoxSize.z / 2
                        ))
                        {
                            crashDirection = -1;
                        }
                        else
                        {
                            crashDirection = 1;
                        }

                    }
                    else
                    {
                        previousPosition = m_Car.transform.position;
                    }
                }
                steerDirection = SteerInput(m_Car.transform.position, m_Car.transform.eulerAngles.y, followPoint);
                accelerationDirection = 1;//AccelerationInput(m_Car.transform.position, m_Car.transform.eulerAngles.y, followPoint);

                if (m_Car.CurrentSpeed >= pointVelocity + Vector3.Distance(followPoint, m_Car.transform.position))
                {
                    accelerationDirection = 0;
                }
                else if (m_Car.CurrentSpeed >= maxVelocity)
                {
                    accelerationDirection = 0;
                }

                m_Car.Move(steerDirection, accelerationDirection * acceleration, -brake, handBrake);
            }
            else
            {
                crashTime += Time.deltaTime;
                if (crashTime <= 1f)
                {
                    steerDirection = SteerInput(m_Car.transform.position, m_Car.transform.eulerAngles.y, followPoint);
                    if (crashDirection > 0)
                    {
                        m_Car.Move(steerDirection, acceleration, 0, 0);
                    }
                    else
                    {
                        m_Car.Move(-steerDirection, 0, -acceleration, 0);
                    }
                }
                else
                {
                    crashTime = 0;
                    crashed = false;
                }
            }
        }

        private bool CheckSpacingLeft(float actualSpacingLeft)
        {
            Transform leader = GameObject.FindWithTag("Leader").transform;
            bool collision = false;
            int radius = 1;
            for (int i = -radius; i <= radius; ++i)
            {
                for (int j = -radius; j <= radius; ++j)
                {
                    collision = collision || terrain_manager.myInfo.traversability[terrain_manager.myInfo.get_i_index(i + (leader.position + Quaternion.AngleAxis(angle, leader.up) * -leader.forward * 3 * actualSpacingLeft).x), terrain_manager.myInfo.get_j_index(j + (leader.position + Quaternion.AngleAxis(angle, leader.up) * -leader.forward * 3 * actualSpacingLeft).z)] > 0.5f;
                    collision = collision || terrain_manager.myInfo.traversability[terrain_manager.myInfo.get_i_index(i + (leader.position + Quaternion.AngleAxis(angle, leader.up) * -leader.forward * 2 * actualSpacingLeft).x), terrain_manager.myInfo.get_j_index(j + (leader.position + Quaternion.AngleAxis(angle, leader.up) * -leader.forward * 2 * actualSpacingLeft).z)] > 0.5f;
                }
            }
            return collision;
        }

        private bool CheckSpacingRight(float actualSpacingRight)
        {
            Transform leader = GameObject.FindWithTag("Leader").transform;
            bool collision = false;
            int radius = 1;
            for (int i = -radius; i <= radius; ++i)
            {
                for (int j = -radius; j <= radius; ++j)
                {
                    collision = collision || terrain_manager.myInfo.traversability[terrain_manager.myInfo.get_i_index(i + (leader.position + Quaternion.AngleAxis(-angle, leader.up) * -leader.forward * 3 * actualSpacingRight).x), terrain_manager.myInfo.get_j_index(j + (leader.position + Quaternion.AngleAxis(-angle, leader.up) * -leader.forward * 3 * actualSpacingRight).z)] > 0.5f;
                    collision = collision || terrain_manager.myInfo.traversability[terrain_manager.myInfo.get_i_index(i + (leader.position + Quaternion.AngleAxis(-angle, leader.up) * -leader.forward * 2 * actualSpacingRight).x), terrain_manager.myInfo.get_j_index(j + (leader.position + Quaternion.AngleAxis(-angle, leader.up) * -leader.forward * 2 * actualSpacingRight).z)] > 0.5f;

                }
            }
            return collision;
        }

        private bool CheckSpacingCircle(float spacingCircle)
        {
            Transform leader = GameObject.FindWithTag("Leader").transform;
            bool collision = false;
            int radius = 1;
            int x = (int)Mathf.Round(Mathf.Sin(Mathf.Deg2Rad * leader.eulerAngles.y));
            int z = (int)Mathf.Round(Mathf.Cos(Mathf.Deg2Rad * leader.eulerAngles.y));
            for (int i = -radius; i <= radius; ++i)
            {
                for (int j = -radius; j <= radius; ++j)
                {
                    collision = collision || terrain_manager.myInfo.traversability[terrain_manager.myInfo.get_i_index(i + (leader.position + leader.forward * 20 + Quaternion.AngleAxis(angle, leader.up) * -Vector3.forward * spacingCircle).x), terrain_manager.myInfo.get_j_index(j + (leader.position + leader.forward * 20 + Quaternion.AngleAxis(angle, leader.up) * -Vector3.forward * spacingCircle).z)] > 0.5f;
                    collision = collision || terrain_manager.myInfo.traversability[x + terrain_manager.myInfo.get_i_index(i + (leader.position + leader.forward * 20 + Quaternion.AngleAxis(angle, leader.up) * -Vector3.forward * spacingCircle).x), z + terrain_manager.myInfo.get_j_index(j + (leader.position + leader.forward * 20 + Quaternion.AngleAxis(angle, leader.up) * -Vector3.forward * spacingCircle).z)] > 0.5f;
                }
            }
            return collision;
        }

        private Vector3 FindFollowPoint()
        {

            Transform leader = GameObject.FindWithTag("Leader").transform;

            float invLerpSpeed = 5f;

            if (circularPattern)
            {
                float actualSpacingCircle = spacing;
                while (CheckSpacingCircle(actualSpacingCircle))
                {
                    actualSpacingCircle--;
                    if (actualSpacingCircle <= 0)
                    {
                        break;
                    }
                }
                offset = Vector3.Lerp(offset, Quaternion.AngleAxis(angle, leader.up) * -Vector3.forward * actualSpacingCircle, Time.deltaTime / invLerpSpeed);
            }
            else
            {
                float actualSpacingLeft = spacing;
                float actualSpacingRight = spacing;
                while (CheckSpacingLeft(actualSpacingLeft))
                {
                    actualSpacingLeft--;
                }
                while (CheckSpacingRight(actualSpacingRight))
                {
                    actualSpacingRight--;
                }

                switch (carNumber)
                {
                    case 0:
                        offset = Vector3.Lerp(offset, Quaternion.AngleAxis(angle, leader.up) * -leader.forward * actualSpacingLeft, Time.deltaTime / invLerpSpeed);
                        break;
                    case 1:
                        offset = Vector3.Lerp(offset, Quaternion.AngleAxis(angle, leader.up) * -leader.forward * 3 * actualSpacingLeft, Time.deltaTime / invLerpSpeed);
                        break;
                    case 2:
                        offset = Vector3.Lerp(offset, Quaternion.AngleAxis(-angle, leader.up) * -leader.forward * actualSpacingRight, Time.deltaTime / invLerpSpeed);
                        break;
                    default:
                        offset = Vector3.Lerp(offset, Quaternion.AngleAxis(-angle, leader.up) * -leader.forward * 3 * actualSpacingRight, Time.deltaTime / invLerpSpeed);
                        break;
                }
            }

            bool collision = false;
            while (terrain_manager.myInfo.traversability[terrain_manager.myInfo.get_i_index((leader.position + (circularPattern ? leader.forward * 20 : Vector3.zero) + offset).x), terrain_manager.myInfo.get_j_index((leader.position + (circularPattern ? leader.forward * 20 : Vector3.zero) + offset).z)] > 0.5f)
            {
                if (terrain_manager.myInfo.traversability[terrain_manager.myInfo.get_i_index((leader.position + (circularPattern ? leader.forward * 20 : Vector3.zero)).x), terrain_manager.myInfo.get_j_index((leader.position + (circularPattern ? leader.forward * 20 : Vector3.zero)).z)] > 0.5f)
                {
                    break;
                }
                offset *= 0.9f;
                collision = true;
            }
            if (collision)
            {
                offset *= 0.5f;
            }

            return leader.position + (circularPattern ? leader.forward * 20 : Vector3.zero) + offset;
        }

        //Determines steer angle for the car
        private float SteerInput(Vector3 position, float theta, Vector3 point)
        {
            Vector3 direction = Quaternion.Euler(0, theta, 0) * Vector3.forward;
            Vector3 directionToPoint = point - position;
            float angleBetweenDirections = Vector3.Angle(direction, directionToPoint) * Mathf.Sign(-direction.x * directionToPoint.z + direction.z * directionToPoint.x);
            float steerAngle = Mathf.Clamp(angleBetweenDirections, -m_Car.m_MaximumSteerAngle, m_Car.m_MaximumSteerAngle) / m_Car.m_MaximumSteerAngle;
            return steerAngle;
        }

        //Determines acceleration for the car
        private float AccelerationInput(Vector3 position, float theta, Vector3 point)
        {
            Vector3 direction = Quaternion.Euler(0, theta, 0) * Vector3.forward;
            Vector3 directionToPoint = point - position;
            return Mathf.Clamp(direction.x * directionToPoint.x + direction.z * directionToPoint.z, -1, 1);
        }

        //Get size of car collider to be used with C space
        private void InitializeCSpace()
        {
            Quaternion carRotation = m_Car.transform.rotation;
            m_Car.transform.rotation = Quaternion.identity;
            ObstacleSpace = new CarConfigSpace();
            BoxCollider carCollider = GameObject.Find("ColliderBottom").GetComponent<BoxCollider>();
            ObstacleSpace.BoxSize = carCollider.transform.TransformVector(carCollider.size);
            m_Car.transform.rotation = carRotation;
        }



        private void OnDrawGizmos()
        {
            if (!Application.isPlaying)
            {
                return;
            }
            float car_length = 4.47f, car_width = 2.43f, car_high = 2f;
            float scale = 1f;
            Vector3 cube_size = new Vector3(car_width * scale, car_high * scale, car_length * scale);

            Gizmos.color = Color.blue;

            Transform leader = GameObject.FindWithTag("Leader").transform;
            Gizmos.DrawSphere(leader.position + (circularPattern ? leader.forward * 20 : Vector3.zero) + offset, 1f);
        }
    }
}