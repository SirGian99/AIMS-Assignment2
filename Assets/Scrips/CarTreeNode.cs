using System.Collections;
using System.Collections.Generic;
using UnityEngine;

// Data Structures
public class CarTreeNode
{
    // Each node on path tree stores the following data
    // position, theta, speed, RRT cost, parentnode, list of children
    // Full tree is just list of node

    public Vector3 position;
    public float theta;
    public float speed;
    public float cost;

    public CarTreeNode parent = null;
    public List<CarTreeNode> children;

    public CarTreeNode(Vector3 position, float theta, float speed, float cost)
    {
        this.position = position;
        this.theta = theta;
        this.speed = speed;
        this.cost = cost;
        children = new List<CarTreeNode>();
    }
}