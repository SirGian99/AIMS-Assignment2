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
        public List<GraphSTC> drone_maps;

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
        public DARP_controller darp;

        //Temp
        private Edge[] min_tree;


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
            Debug.Log("Walkable nodes: " + graph.walkable_nodes);
            Debug.Log("Non walk nodes: " + graph.non_walkable_nodes);
            map = new GraphSTC(graph, start_pos);

            // Get Array of Friends and Eniemies
            friends = GameObject.FindGameObjectsWithTag("Player");
            Vector3[] initial_positions = new Vector3[friends.Length];
            int i = 0;
            foreach(GameObject friend in friends)
            {
                Debug.Log(friend + " position: " + friend.gameObject.transform.position);
                initial_positions[i] = friend.gameObject.transform.position;
                i++;
                if(friend.gameObject.transform.position == m_Car.transform.position)
                {
                    CarNumber = i;
                }
            }

            // Plan your path here
            my_path = new List<Vector3>();
            my_path = createDARP(map, start_pos);
            min_tree = STC(map);
            darp = new DARP_controller(friends.Length, initial_positions, graph, 0.0004f, 100);

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


        // MAIN FUNC: Divide and conquer
        public List<Vector3> createDARP(GraphSTC graph, Vector3 start_pos)
        {
            Edge[] MinSTC = STC(graph);
            List<Vector3> path = new List<Vector3>();
            path = ComputePath(MinSTC, start_pos);

            return path;
        }

        // Sub-Func: Compute path 
        private List<Vector3> ComputePath(Edge[] EdgeArray, Vector3 start_pos)
        {
            List<Vector3> path = new List<Vector3>();
            Vector3 waypoint;
            path.Add(start_pos);
            for (int i = 0; i < EdgeArray.Length-1; i++)
            {
                waypoint = EdgeArray[i].Destination.worldPosition;
                //Debug.Log(i+ "waypoint " + waypoint + EdgeArray.Length);
                path.Add(waypoint);
            }
            return path;
        }




        // MAIN FUNC: Separate terrain into different areas




        // MAIN FUNC: Kurskal Algorithm to find minimum spanning tree
        public Edge[] STC(GraphSTC graph)
        {
            int verticesCount = graph.VerticesCount;
            Edge[] result = new Edge[verticesCount];
            Node[] VertexArray = graph.VertexArray;
            int k = 0;
            int e = 0;

            // Sort edges by cost- all costs are same
            graph.EdgeList.Sort((e1,e2)=> e1.Weight.CompareTo(e2.Weight));

            // Create each vertex as subsets
            Subset[] subsets = new Subset[verticesCount];
            Subset sub;
            for (int v = 0; v < verticesCount; ++v)
            {
                sub = new Subset();
                sub.Parent = VertexArray[v];
                sub.Rank = 0;
                subsets[v] = sub;
            }

            // build min tree
            while (e < verticesCount - 1)
            {
                Edge nextEdge = graph.EdgeList[k];
                Node x = Find(subsets, nextEdge.Source, System.Array.IndexOf(VertexArray, nextEdge.Source), VertexArray);
                Node y = Find(subsets, nextEdge.Destination, System.Array.IndexOf(VertexArray, nextEdge.Destination), VertexArray);

                if (x != y)
                {
                    result[e++] = nextEdge;
                    //Debug.Log("Edge " + e + " S: " + nextEdge.Source.worldPosition + " to D: " + nextEdge.Destination.worldPosition);
                    Union(subsets, x, y, VertexArray);
                }
                k++;
            }

            return result;
        }

        // Sub-Func: Identify parent of node a in Subset for kurskal's
        private Node Find(Subset[] subsets, Node vertex, int k, Node[] vertex_dict)
        {
            if (subsets[k].Parent != vertex)
            {
                subsets[k].Parent = Find(subsets, subsets[k].Parent,
                                    System.Array.IndexOf(vertex_dict, subsets[k].Parent), vertex_dict);
            }

            return subsets[k].Parent;
        }

        // Sub-Func: Union of subsets for kurskal's
        private void Union(Subset[] subsets, Node x, Node y, Node[] vertex_dict)
        {
            Node xroot = Find(subsets, x, System.Array.IndexOf(vertex_dict, x), vertex_dict);
            Node yroot = Find(subsets, y, System.Array.IndexOf(vertex_dict, y), vertex_dict);

            if (subsets[System.Array.IndexOf(vertex_dict, xroot)].Rank < subsets[System.Array.IndexOf(vertex_dict, yroot)].Rank)
                subsets[System.Array.IndexOf(vertex_dict, xroot)].Parent = yroot;
            else if (subsets[System.Array.IndexOf(vertex_dict, xroot)].Rank > subsets[System.Array.IndexOf(vertex_dict, yroot)].Rank)
                subsets[System.Array.IndexOf(vertex_dict, yroot)].Parent = xroot;
            else
            {
                subsets[System.Array.IndexOf(vertex_dict, yroot)].Parent = xroot;
                ++subsets[System.Array.IndexOf(vertex_dict, xroot)].Rank;
            }
        }

        // Data Stuture: Subset for kurskal's
        public struct Subset
        {
            public Node Parent { get; set; }
            public int Rank { get; set; }
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
            // Show graph grids
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

            //Show min span tree
            if(min_tree != null)
            {
                Gizmos.color = Color.red;
                for (int i = 0; i < min_tree.Length - 1; ++i)
                {
                    Gizmos.DrawLine(min_tree[i].Source.worldPosition + Vector3.up, min_tree[i].Destination.worldPosition + Vector3.up);
                }
            }


            ////Show the path to the goal
            //if (my_path != null)
            //{
            //    Gizmos.color = Color.white;
            //    for (int i = 0; i < my_path.Count - 1; ++i)
            //    {
            //        Gizmos.DrawLine(my_path[i] + Vector3.up, my_path[i + 1] + Vector3.up);
            //}
            //}

        }
    }
}
