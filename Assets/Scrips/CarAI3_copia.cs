/* ////TODO REMOVE FOR COMMENT
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using static Node;
using static DubinsPath;
using static GenerateDrivingDirections;


namespace UnityStandardAssets.Vehicles.Car
{

    [RequireComponent(typeof(CarController))]



    public class CarAI : MonoBehaviour
    {

        private CarController m_Car; // the car controller we want to use

        public GameObject terrain_manager_game_object;
        TerrainManager terrain_manager;
        public Graph graph;
        GenerateDrivingDirections dubinsPathGenerator;
        private Vector3 targetPosition;
        float steeringAmount;
        float accelerationAmount;
        float handbrake;
        float footbrake;
        int nodeNumber;
        int stop = 50;
        Vector3 carSize = new Vector3(2.43f, 0.41f, 4.47f);
        float starting_time;
        int curves = 0;


        //driving helpers
        private bool u_curve = false;
        private float u_curve_final_heading = 0;
        bool isCurveVertical = true;
        bool had_hit_backward = false;

        private bool adjusting;
        Vector3? reference_position;


        List<Node> up_and_smooth;
        List<Node> bez_path;
        List<Node> upsampled_path;
        List<Node> final_path;
        Vector3 target_velocity;
        Vector3 target_position;
        Vector3 old_target_pos;
        public Rigidbody rigidbody;
        public float k_p = 1.2f;
        public float k_d = 0.26f;
        int node_index = 1;
        int max_stuck_safe_exit = 10000;
        int stuck_safe_exit = 10000;

        //Path matrix from one target to the other. The starting position is the last one.
        List<Node>[,] paths;


        private void Start()
        {
            // get the car controller
            m_Car = GetComponent<CarController>();
            terrain_manager = terrain_manager_game_object.GetComponent<TerrainManager>();

            rigidbody = GetComponent<Rigidbody>();

            // Plan your path here
            // Replace the code below that makes a random path
            // ...

            Vector3 start_pos = terrain_manager.myInfo.start_pos;
            Vector3 goal_pos = terrain_manager.myInfo.goal_pos;
            nodeNumber = 0; // We start at the beginning of the path

            List<Vector3> my_path = new List<Vector3>();

            my_path.Add(start_pos);

            for (int i = 0; i < 3; i++)
            {
                Vector3 waypoint = start_pos + new Vector3(UnityEngine.Random.Range(-50.0f, 50.0f), 0, UnityEngine.Random.Range(-30.0f, 30.0f));
                my_path.Add(waypoint);
            }
            my_path.Add(goal_pos);
            Debug.Log(terrain_manager.myInfo.traversability.GetLength(0));
            Debug.Log(terrain_manager.myInfo.traversability.GetLength(1));
            Debug.Log(terrain_manager.myInfo.traversability.Length);
            Debug.Log(terrain_manager.myInfo.traversability.Rank);
            string traversability_string = "";
            /*
            Vector3 carSize = new Vector3(2f, 0.81f, 2f)*2;
            Vector3 granularity = new Vector3((terrain_manager.myInfo.x_high - terrain_manager.myInfo.x_low) / carSize.x, 1, (terrain_manager.myInfo.z_high - terrain_manager.myInfo.z_low) / carSize.z);

            Vector3 cube_size = new Vector3((terrain_manager.myInfo.x_high - terrain_manager.myInfo.x_low)/ terrain_manager.myInfo.x_N, 0, (terrain_manager.myInfo.z_high - terrain_manager.myInfo.z_low) / terrain_manager.myInfo.z_N);
            granularity = cube_size / 2;
            Debug.Log("Granularity x: "+(int)(granularity.x));
            Debug.Log("Granularity z: "+(int)(granularity.z));

            graph = Graph.CreateGraph(terrain_manager.myInfo, (int)(granularity.x), (int)(granularity.z));

            */
/* ////TODO REMOVE FOR COMMENT

int x_scale = terrain_manager.myInfo.x_N * 2;
            int z_scale = terrain_manager.myInfo.z_N * 2;
            float x_len = (terrain_manager.myInfo.x_high - terrain_manager.myInfo.x_low);
            float z_len = (terrain_manager.myInfo.z_high - terrain_manager.myInfo.z_low);
            float x_unit = x_len / x_scale;
            float z_unit = z_len / z_scale;
            float ratio = x_unit / z_unit;


            Debug.Log("Variables. x_scale: " + x_scale + " z_scale: " + z_scale + " x_unit: " + x_unit + " z_unit: " + z_unit + " ratio: " + ratio);

            if (x_unit < carSize.x * 2)
            {
                Debug.Log("Block width is too low");
                x_scale /= 2;
                x_unit *= 2;
                /*
                x_unit = carSize.x * 3;
                x_scale = (int)(x_len / x_unit) * 2;
                */
/* ////TODO REMOVE FOR COMMENT

            }
            if (z_unit < carSize.z)
            {
                Debug.Log("Block height is too low");
                z_scale /= 2;
                z_unit *= 2;
                /*
                z_unit = carSize.z * 1.5f;
                z_scale = (int)(z_len / z_unit) * 2;
                */
/* ////TODO REMOVE FOR COMMENT

            }

            if (x_unit > carSize.x * 4)
            {
                x_scale *= 2;
                x_unit /= 2;

            }
            if (z_unit > carSize.z * 4)
            {
                z_scale *= 4;
                z_unit /= 4;
                /*
                z_unit = carSize.z * 1.5f;
                z_scale = (int)(z_len / z_unit) * 2;
                */
/* ////TODO REMOVE FOR COMMENT

            }

            Debug.Log("Variables After car rescaling. x_scale: " + x_scale + " z_scale: " + z_scale +  " x_unit: " + x_unit + " z_unit: " + z_unit + " ratio: " + ratio);

            /*
            if (z_unit < x_unit)
            {
                float old = z_scale; ///TODO Remove
                z_scale = (int)(z_len * x_scale / x_len);
                Debug.Log("Changing z_scale from " + old + " to " + z_scale);

            }
            else
            {
                float old = x_scale; ///TODO Remove
                x_scale = (int)(x_len * z_scale / z_len);
                Debug.Log("Changing x_scale from " + old + " to " + x_scale);

            }
            */
/*
ratio = x_unit / z_unit;

if (ratio < 1)
{
    ratio = 1 / ratio;
    z_scale *= (int)ratio;
    Debug.Log("Rescaling z of " + ratio + " going from " + terrain_manager.myInfo.z_N + " to " + z_scale);
}
else
{
    x_scale *= (int)ratio;
    Debug.Log("Rescaling x of " + ratio + " going from " + terrain_manager.myInfo.x_N + " to " + x_scale);
}
*/
/* ////TODO REMOVE FOR COMMENT


if (x_unit > 1.5 * z_unit)
            {
                x_scale *= 2;
            }
            else if (z_unit > 1.5 * x_unit)
            {
                z_scale *= 2;
            }

            

            x_unit = x_len / x_scale;
            z_unit = z_len / z_scale;


            Debug.Log("Variables after. x_scale: " + x_scale + " z_scale: " + z_scale + " x_len: " + x_len + " z_len: " + z_len + " x_unit: " + x_unit + " z_unit: " + z_unit + " ratio: " + ratio);

            Debug.Log("Final block size: [" + x_unit + "," + z_unit + "]");


            graph = Graph.CreateGraph(terrain_manager.myInfo, x_scale, z_scale);
            //graph = Graph.CreateGraph(terrain_manager.myInfo, terrain_manager.myInfo.x_N*2, terrain_manager.myInfo.z_N*2);
            Debug.Log("QUI: " + x_scale + " " + z_scale + " " + terrain_manager.myInfo.traversability.GetLength(0) + " " + terrain_manager.myInfo.traversability.GetLength(1) + " ");

            for (int i = 0; i < terrain_manager.myInfo.traversability.GetLength(0); i++)
            {
                for (int j = 0; j < terrain_manager.myInfo.traversability.GetLength(1); j++)
                {
                    traversability_string += (terrain_manager.myInfo.traversability[i, j] + " ");
                    if (terrain_manager.myInfo.traversability[i, j] == 0 && terrain_manager.myInfo.traversability[i, j + 1] == 0)
                    {
                        if (graph.AreConnected(new Node(i, j, 0, 0), new Node(i, j + 1, 0, 0)))
                        {
                            //Debug.Log("Connected");
                        }
                        else
                        {
                            //Debug.Log("ERROR!!!!!");
                        }

                    }
                }
                traversability_string += "\n";
            }
            //Debug.Log(traversability_string);
            //Debug.Log("HERE!");
            graph.printTraversability();



            // Plot your path to see if it makes sense
            // Note that path can only be seen in "Scene" window, not "Game" window
            Vector3 old_wp = start_pos;
            foreach (var wp in my_path)
            {
                //Debug.DrawLine(old_wp, wp, Color.red, 100f);
                old_wp = wp;
            }
            PathFinder.findPath(graph, start_pos, goal_pos, (360 - transform.eulerAngles.y + 90) % 360); // path is accessible through graph.path

            //bez_path = PathFinder.bezierPath(graph.path, 2);
            int upsampling_factor = 4;
            upsampled_path = PathFinder.pathUpsampling(graph.path, upsampling_factor);
            up_and_smooth = PathFinder.pathSmoothing(upsampled_path, 0.4f, 0.7f, 1f);
            //graph.path = PathFinder.pathSmoothing(graph.path);
            final_path = up_and_smooth;


            for (int i = upsampling_factor; i<final_path.Count-1; i++)
            {
                int debug_oldc = curves;
                curves += (int)(Math.Abs(final_path[i].heading - final_path[i + 1].heading) / 45);
                Debug.Log("INITIAL Current h:" + final_path[i].heading + " next h: " + final_path[i + 1].heading);

                if (debug_oldc != curves)
                {
                    ;
                }
            }
            Debug.Log("Number of curves: " + curves);

            Debug.Log("Percorsi. Up_n_sm: " + up_and_smooth + " Bez: " + bez_path + " normal: " + graph.path + " final: " + final_path);

            foreach (Node n in graph.path)
            {
                Debug.Log("Original heading: " + n.heading);
            }

            //graph.path = up_and_smooth;
            //dubinsPathGenerator = new GenerateDrivingDirections(m_Car);


            /*////TODO Path smoothr test
/* ////TODO REMOVE FOR COMMENT

List<Node> path_to_smooth = new List<Node>();

            path_to_smooth.Add(new Node(1, 1, 0, 0));
            path_to_smooth.Add(new Node(1, 1, 0, 1));
            path_to_smooth.Add(new Node(1, 1, 0, 2));
            path_to_smooth.Add(new Node(1, 1, 1, 2));
            path_to_smooth.Add(new Node(1, 1, 2, 2));
            path_to_smooth.Add(new Node(1, 1, 3, 2));
            path_to_smooth.Add(new Node(1, 1, 4, 2));
            path_to_smooth.Add(new Node(1, 1, 4, 3));
            path_to_smooth.Add(new Node(1, 1, 4, 4));

            List<Node> smoothed = PathFinder.pathSmoothing(path_to_smooth);

            foreach(Node n in smoothed)
            {
                Debug.Log("SMOOTH [" + n.x_pos + "," + n.z_pos + "]" );
            }
            */
/* ////TODO REMOVE FOR COMMENT

        }

        void OnDrawGizmos() // draws grid on map and shows car
        {
            if (graph != null)
            {
                ////TODO UNCOMMENT THIS LOOP TO SEE THE BLOCKS OF THE GRID
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

            if (final_path != null)
            {
                for (int i = 0; i < final_path.Count - 1; i++)
                {
                    Gizmos.color = Color.red;
                    Gizmos.DrawLine(final_path[i].worldPosition, final_path[i + 1].worldPosition);
                }

                /*
                List<Node> augmented_path = up_and_smooth;//PathFinder.pathUpsampling(graph.path, 8);

                for (int i = 0; up_and_smooth!=null && i < augmented_path.Count; i++)
                {
                    Gizmos.color = Color.black;
                    //Gizmos.DrawSphere(augmented_path[i].worldPosition, 0.5f);
                    Gizmos.DrawLine(augmented_path[i].worldPosition, augmented_path[i + 1].worldPosition);
                    //Gizmos.DrawLine(augmented_path[i].worldPosition, augmented_path[i + 1].worldPosition);
                }

                /*for (int i = 0; i < graph.path.Count; i++)
                {
                    Gizmos.color = Color.red;
                    //Gizmos.DrawSphere(graph.path[i].worldPosition, 0.5f);
                    Gizmos.DrawLine(graph.path[i].worldPosition, graph.path[i + 1].worldPosition);
                    //Gizmos.DrawLine(augmented_path[i].worldPosition, augmented_path[i + 1].worldPosition);
                }
                */

/*
if(bez_path!=null)
for (int i = 0; i < bez_path.Count-1; i++)
{
    Gizmos.color = Color.black;
    //Gizmos.DrawSphere(augmented_path[i].worldPosition, 0.5f);
    Gizmos.DrawLine(bez_path[i].worldPosition, bez_path[i + 1].worldPosition);
    //Gizmos.DrawLine(augmented_path[i].worldPosition, augmented_path[i + 1].worldPosition);
    Debug.Log( "i: " + i + " Current point: [" + bez_path[i].x_pos + "," + bez_path[i].z_pos + "]");
}
*/
/* ////TODO REMOVE FOR COMMENT

}


}


public void SetNextTarget(Vector3 targetPosition)
{
this.targetPosition = targetPosition;
}

public void setHandbrake()
{
this.handbrake = 0f;
}



public void SetAccelerationSteering(float current_heading=0, float lookahead_heading=0, int heading_steps=0)
{
float max_speed = 65;
Vector3 directionToMove = (this.targetPosition - transform.position).normalized;
float dot = Vector3.Dot(transform.forward, directionToMove);
float steeringAngle = Vector3.SignedAngle(transform.forward, directionToMove, Vector3.up);
this.steeringAmount = steeringAngle / m_Car.m_MaximumSteerAngle;
float safe_steering = Math.Abs(this.steeringAmount) > 0.5 ? 0.7f : 1;

float heading_difference = Mathf.Clamp((Math.Abs(current_heading - lookahead_heading) / 45f) * 2, 1, 100);
if (u_curve)
{
max_speed = 50;
if (m_Car.CurrentSpeed > max_speed*0.6)
{
    this.handbrake = 1;
}
}
else
max_speed *= (float)Math.Exp(-heading_steps / 2);


if (dot >= 0)
{

this.accelerationAmount = (max_speed - m_Car.CurrentSpeed) / max_speed * safe_steering;
this.footbrake = 0;
if (m_Car.CurrentSpeed >= max_speed)
    this.footbrake = (m_Car.CurrentSpeed - max_speed) / 10;
}
else
{
this.accelerationAmount = 0f; // this doesn't work because the acceleration is clamped to 0,1.
this.footbrake = -1f;
}

steeringAngle = Mathf.Clamp(steeringAngle, -25, 25);

}


private void OnDrawGizmosSelected()
{
Gizmos.color = Color.red;


Gizmos.DrawLine(transform.position, targetPosition);
Gizmos.DrawSphere(transform.position, 1f);
Gizmos.DrawSphere(targetPosition, 1f);


Gizmos.color = Color.yellow;
Vector3 nextNode = new Vector3(final_path[nodeNumber + 1].x_pos, transform.position.y, final_path[nodeNumber + 1].z_pos);
Gizmos.DrawLine(transform.position, nextNode);

}


private void FixedUpdate()
{

if (nodeNumber == 0)
starting_time = Time.time;
if (stop > 0)
{
setHandbrake();


Node n = final_path[nodeNumber];
Debug.DrawLine(transform.position, final_path[get_closest_node(transform.position, final_path, nodeNumber)].worldPosition);

int lookahead = (int)(curves / 2.5);
int u_curve_lookahead = (int)(lookahead * 1.7);
//TODO
//we could tune it based on the number of curves in the path and theyr tipe (like, ucurves and so on)
//idea: use 2 lookah., one for normal speed and another one to detect u curves.

Node lookahead_node;
int heading_steps = 0;
int tolerance;
float current_angle = (360 - transform.eulerAngles.y + 90) % 360;
Debug.Log("Current angle: " + current_angle);

if (u_curve)
{
    tolerance = isCurveVertical ? (int)(Math.Atan(graph.x_unit / graph.z_unit) * Mathf.Rad2Deg) : (int)(Math.Atan(graph.z_unit / graph.x_unit)* Mathf.Rad2Deg);
    Debug.Log("Current angle: " + transform.eulerAngles.y + " to_reach: " + u_curve_final_heading + "tolerance: " + tolerance);
    if (n.heading == u_curve_final_heading || Math.Abs(u_curve_final_heading - current_angle) < tolerance)
    {
        u_curve = false; //finished the u_curve
        Debug.Log("U curve end detected");
    }

}

if ((nodeNumber + 1) < final_path.Count && !u_curve)
{
    lookahead_node = final_path[nodeNumber + 1];

    heading_steps = n.heading != lookahead_node.heading  && !adjusting ? 1 : 0;
    int heading_difference;
    float intermediate_heading = heading_steps==1 ? lookahead_node.heading : -1;

    for (int j = 1; j < u_curve_lookahead && nodeNumber + 1 + j < final_path.Count; j++)
    {
        Debug.Log("Lookhaead node heading: " + lookahead_node.heading + " next_node_heading: " + final_path[nodeNumber + 1 + j].heading);
        if (lookahead_node.heading != final_path[nodeNumber + 1 + j].heading && j < lookahead)
        {
            if (heading_steps == 0)
                intermediate_heading = final_path[nodeNumber + 1 + j].heading;
            heading_steps++;
        }
        lookahead_node = final_path[nodeNumber + 1 + j];
        heading_difference = (int)(Math.Abs(n.heading - lookahead_node.heading));
        Debug.Log("J: " + j + " Nodenumber: " + nodeNumber + " Current h: " + n.heading + " Final h: " + final_path[nodeNumber + 1 + j].heading);
        if (heading_difference >= 180 && heading_steps == 2) // && (vertical && Vector3.Distance(n.worldPosition, final_path[nodeNumber + 1 + j].worldPosition) > graph.x_unit*2 || !vertical && Vector3.Distance(n.worldPosition, final_path[nodeNumber + 1 + j].worldPosition) > graph.z_unit * 2))
        { //the addition after the first && is probabily wrong
            u_curve = true;
            u_curve_final_heading = final_path[nodeNumber + 1 + j].heading;
            isCurveVertical = u_curve_final_heading == 90 || u_curve_final_heading == 270;
            Debug.Log("Is curve vertical? " + isCurveVertical);

            Debug.Log("U curve detected! Heading Steps: " + heading_steps);
            break;
        }
    }
    //heading_steps += heading_difference / 45;
}

Debug.Log("Heading steps: " + heading_steps);

float targetDistanceMargin = (float)Math.Sqrt(graph.x_unit * graph.x_unit * +graph.z_unit * graph.z_unit) / 2;
targetDistanceMargin = 5f;
Vector3 nextPosition = new Vector3(n.x_pos, transform.position.y, n.z_pos);
Debug.Log("Next position is " + nextPosition);
SetNextTarget(nextPosition);
float distanceToTarget = Vector3.Distance(transform.position, targetPosition);

if (get_closest_node(transform.position, final_path, nodeNumber) <= nodeNumber + 1 && distanceToTarget > targetDistanceMargin && !in_the_same_cell(transform.position, targetPosition, graph) && stop == 50)
{

    SetAccelerationSteering(heading_steps: heading_steps);
    avoid_obstacles(heading_steps>0);
    Debug.Log("Acceleration is set to " + accelerationAmount);
    Debug.Log("Steering is set to " + steeringAmount);
    Debug.Log("Speed:" + m_Car.CurrentSpeed);
    m_Car.Move(steeringAmount, accelerationAmount, footbrake, handbrake);

}
else //we reached the waypoint or end point
{
    if (inRange(targetPosition, terrain_manager.myInfo.goal_pos, (graph.x_unit + graph.z_unit) / 5 * 2)) // we made it to the end, stop the car
    {
        m_Car.Move(0f, 0f, -1f, 1f);
        stop--;
        if (starting_time >= 0)
        {
            Debug.Log("REACHED IN " + (Time.time - starting_time) + " seconds");
            starting_time = -1;
        }
    }
    else // we arrived at a waypoint node, move to the next one
    {
        bool was_adjusting = adjusting;
        final_path[nodeNumber + 1].worldPosition = move_next_point(final_path[nodeNumber + 1].heading, final_path[nodeNumber + 1].worldPosition);


        if (adjusting == false && was_adjusting == true && Math.Abs(n.heading - final_path[nodeNumber + 1].heading) < 0.1) //in pratica, non è più necessario. Facciamolo finché ci serve (rettilineo) con l'heading
        {
            if (reference_position == null)
                reference_position = nextPosition;
            final_path[nodeNumber + 1].worldPosition = move_next_point(final_path[nodeNumber + 1].heading, final_path[nodeNumber + 1].worldPosition, forcing_position: (Vector3)reference_position);
            Debug.Log("ADJUSTING");

        }
        else
            reference_position = null;
        final_path[nodeNumber + 1].x_pos = final_path[nodeNumber + 1].worldPosition.x;
        final_path[nodeNumber + 1].z_pos = final_path[nodeNumber + 1].worldPosition.z;

        nodeNumber += 1;

    }
}
}
else
{

m_Car.Move(0f, 0f, -1f, 1f);

}


}

public bool inRange(Vector3 current_pos, Vector3 target_pos, float range)
{
return (Math.Abs(current_pos.x - target_pos.x) + Math.Abs(current_pos.z - target_pos.z)) <= range;
}

public int get_closest_node(Vector3 position, List<Node> path, int current_index)
{
Debug.Log("Finding the closest node to " + position);
int closest = -1;
float distance = 1000000000;
position = new Vector3(position.x, 0, position.z);
int range = 10;
for (int i = (int)Mathf.Clamp(current_index - range, 0, path.Count); i < Math.Min(path.Count, current_index + 10); i++)
{
Vector3 to_compare = new Vector3(path[i].x_pos, 0, path[i].z_pos);
float new_distance = Vector3.Distance(position, to_compare);
if (new_distance <= distance)
{
    distance = new_distance;
    closest = i;
    Debug.Log("Closest node at index i:" + i + " distance: " + distance + " and position " + path[i].worldPosition);
}
}
return closest;
}


public bool in_the_same_cell(Vector3 pos1, Vector3 pos2, Graph graph)
{
return graph.getNodeFromPoint(pos1).Equals(graph.getNodeFromPoint(pos2));

}

public Vector3 move_next_point(float heading, Vector3 next_point, Vector3? forcing_position = null)
{
RaycastHit hit;
adjusting = false;
Vector3 to_return = new Vector3(next_point.x, next_point.y, next_point.z);
if(forcing_position != null)
{
next_point = (Vector3) forcing_position;
}


if (heading == 90 || heading == 270) //moving vertical
{
float maxRange = graph.x_unit;
if (Physics.Raycast(next_point, Vector3.left, out hit, maxRange))
{
    to_return.x += graph.x_unit/2;
    adjusting = true;

}
if (Physics.Raycast(next_point, Vector3.right, out hit, maxRange))
{
    to_return.x -= graph.x_unit/2;
    adjusting = true;
}
}

else if (heading == 0 || heading == 180)//moving horizontal
{
float maxRange = graph.z_unit;
if (Physics.Raycast(next_point, Vector3.up, out hit, maxRange))
{
    to_return.z += graph.z_unit/2;
    adjusting = true;

}
if (Physics.Raycast(next_point, Vector3.down, out hit, maxRange))
{
    to_return.z -= graph.z_unit/2;
    adjusting = true;

}
}

return to_return;
}

//this must be called when the first turn has already been done in the U curve
private bool is_safe_turn(float final_heading, float intermediate_heading, bool isVertical, int current_node_index) //returns true weather, given a heading direction, there is enough space
{
while(Math.Abs(final_path[current_node_index].heading - final_heading) > 0.1)
{
Node node = final_path[current_node_index];
float heading = node.heading;


if (isVertical)
{
    if (intermediate_heading == 180)//anti-clockwise turn
    {

    }
    else if (intermediate_heading == 0) //clockwise turn
    {

    }
    else return false;
}


if (heading == 90 || heading == 270) //moving vertical
{
    float maxRange = graph.x_unit;
    if (Physics.Raycast(node.worldPosition, Vector3.left, maxRange))
    {
        return false;

    }
    if (Physics.Raycast(node.worldPosition, Vector3.right, maxRange))
    {
        return false;
    }
}

else if (heading == 0 || heading == 180)//moving horizontal
{
    float maxRange = graph.z_unit;
    if (Physics.Raycast(node.worldPosition, Vector3.up, maxRange))
    {
        return false;

    }
    if (Physics.Raycast(node.worldPosition, Vector3.down, maxRange))
    {
        return false;

    }
}

current_node_index++;
}
return true;
}

private void avoid_obstacles(bool curve_approaching = false)
{
RaycastHit hit;
Vector3 maxRange = carSize*1.2f;
bool had_hit = false;


if (!had_hit_backward && Physics.Raycast(transform.position + transform.up, transform.TransformDirection(Vector3.forward), out hit, maxRange.z))
{
Vector3 closestObstacleInFront = transform.TransformDirection(Vector3.forward) * hit.distance;
Debug.DrawRay(transform.position, closestObstacleInFront, Color.yellow);
this.accelerationAmount *= 0.5f;
this.footbrake = this.footbrake < 0.1f ? 0.5f : this.footbrake * 2;
Debug.Log("Frontal collision, distance: " + hit.distance);
had_hit = true;

if(hit.distance < 5) //recovery from frontal hit
{
    Debug.Log("Collision STOP");
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
Debug.Log("Back collision");
had_hit = true;

}*/
/* ////TODO REMOVE FOR COMMENT

if (Physics.Raycast(transform.position + transform.right, transform.TransformDirection(Vector3.right), out hit, maxRange.x))
            {
                Vector3 closestObstacleInFront = transform.TransformDirection(Vector3.forward) * hit.distance;
                Debug.DrawRay(transform.position, closestObstacleInFront, Color.yellow);
                this.accelerationAmount *= 0.7f;
                this.footbrake = this.footbrake < 0.1f ? 0.3f : this.footbrake * 1.5f;
                this.steeringAmount += -0.5f;
                Debug.Log("Right collision");
                had_hit = true;
                

            }

            if (Physics.Raycast(transform.position + transform.right, transform.TransformDirection(Vector3.left), out hit, maxRange.x))
            {
                Vector3 closestObstacleInFront = transform.TransformDirection(Vector3.forward) * hit.distance;
                Debug.DrawRay(transform.position, closestObstacleInFront, Color.yellow);
                this.accelerationAmount *= 0.7f;
                this.footbrake = this.footbrake < 0.1f ? 0.3f : this.footbrake * 1.5f;
                this.steeringAmount += 0.5f;
                Debug.Log("Left collision");
                
                had_hit = true;
            }

            if (!had_hit && !curve_approaching)
            {
                this.accelerationAmount *= 1.25f;
                Debug.Log("Not hit speed");
            }

            if(!had_hit && m_Car.CurrentSpeed < 1f || had_hit_backward)
            {
                had_hit_backward = true;
                this.accelerationAmount = 1;
                this.footbrake = 0;
                this.handbrake = 0;
                this.steeringAmount *= 1;
                if (m_Car.CurrentSpeed > 10f)
                    had_hit_backward = false;
            }


        }

    }

}

 *////TODO REMOVE FOR COMMENT
