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
        private CarController m_Car; // the car controller we want to use


        public GameObject terrain_manager_game_object;
        TerrainManager terrain_manager;
        public Graph graph;
        Vector3 carSize = new Vector3(4.5f, 0.41f, 4.5f);
        public Rigidbody rigidbody;
        public float gunRange = 10f;
        public DARP_controller darp;



        public GameObject[] friends;
        public GameObject[] enemies;

        private void Start()
        {
            // get the car controller
            m_Car = GetComponent<CarController>();
            terrain_manager = terrain_manager_game_object.GetComponent<TerrainManager>();
            rigidbody = GetComponent<Rigidbody>();
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

            Debug.Log("Walkable nodes: " + graph.walkable_nodes);
            Debug.Log("Non walk nodes: " + graph.non_walkable_nodes);
            // note that both arrays will have holes when objects are destroyed
            // but for initial planning they should work
            friends = GameObject.FindGameObjectsWithTag("Player");
            Vector3[] initial_positions = new Vector3[friends.Length];
            int i = 0;
            foreach(GameObject friend in friends)
            {
                Debug.Log(friend + " position: " + friend.gameObject.transform.position);
                initial_positions[i] = friend.gameObject.transform.position;
                i++;
            }

            // Note that you are not allowed to check the positions of the turrets in this problem




            // Plan your path here
            // ...

            /*Process otherProcess = new Process();
            Debug.Log("App path: " + Application.dataPath);
            otherProcess.StartInfo.FileName = Application.dataPath + "/prova.py";
            otherProcess.StartInfo.RedirectStandardInput = true;
            otherProcess.StartInfo.RedirectStandardOutput = true;
            otherProcess.Start();
            StreamReader reader = otherProcess.StandardOutput;
            string output = reader.ReadToEnd();
            Debug.Log(output);
            otherProcess.WaitForExit();
            */
            darp = new DARP_controller(friends.Length, initial_positions, graph, 0.0004f, 100);
        }


        private void FixedUpdate()
        {

            enemies = GameObject.FindGameObjectsWithTag("Enemy");

            // Execute your path here
            // ...

            Vector3 avg_pos = Vector3.zero;

            foreach (GameObject friend in friends)
            {
                avg_pos += friend.transform.position;
            }
            avg_pos = avg_pos / friends.Length;
            Vector3 direction = (avg_pos - transform.position).normalized;

            bool is_to_the_right = Vector3.Dot(direction, transform.right) > 0f;
            bool is_to_the_front = Vector3.Dot(direction, transform.forward) > 0f;

            float steering = 0f;
            float acceleration = 0;

            if (is_to_the_right && is_to_the_front)
            {
                steering = 1f;
                acceleration = 1f;
            }
            else if (is_to_the_right && !is_to_the_front)
            {
                steering = -1f;
                acceleration = -1f;
            }
            else if (!is_to_the_right && is_to_the_front)
            {
                steering = -1f;
                acceleration = 1f;
            }
            else if (!is_to_the_right && !is_to_the_front)
            {
                steering = 1f;
                acceleration = -1f;
            }

            // this is how you access information about the terrain
            int i = terrain_manager.myInfo.get_i_index(transform.position.x);
            int j = terrain_manager.myInfo.get_j_index(transform.position.z);
            float grid_center_x = terrain_manager.myInfo.get_x_pos(i);
            float grid_center_z = terrain_manager.myInfo.get_z_pos(j);

            Debug.DrawLine(transform.position, new Vector3(grid_center_x, 0f, grid_center_z));


            // this is how you control the car
            //Debug.Log("Steering:" + steering + " Acceleration:" + acceleration);
            m_Car.Move(steering, acceleration, acceleration, 0f);
            //m_Car.Move(0f, -1f, 1f, 0f);


        }

        void OnDrawGizmos() // draws grid on map and shows car
        {
            if (graph != null)
            {
                Node currentNode = graph.getNodeFromPoint(transform.position);
                //Debug.Log("CAR INITIAL NODE: [" + currentNode.i + "," + currentNode.j + "]");
                Gizmos.color = Color.cyan; // position of car
                //Debug.Log("Current car node: [" + currentNode.i + "," + currentNode.j + "]");
                Gizmos.DrawCube(currentNode.worldPosition, new Vector3(graph.x_unit * 0.8f, 0.5f, graph.z_unit * 0.8f));
                foreach (Node n in graph.nodes) // graph.path 
                {
                    Color[] colors = { Color.red, Color.cyan, Color.yellow, Color.white, Color.black, Color.green};
                    int index = darp.assignment_matrix[n.i, n.j];

                    Gizmos.color = colors[index];
                    if (graph.path != null && graph.path.Contains(n))
                        Gizmos.color = Color.white;

                    Gizmos.DrawCube(n.worldPosition, new Vector3(graph.x_unit * 0.8f, 0.5f, graph.z_unit * 0.8f));

                }


               }
        }

    }
}
