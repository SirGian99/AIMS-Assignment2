using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Node
{
    public int i;
    public int j;
    public float x_pos;
    public float z_pos;
    public bool walkable;
    public Vector3 worldPosition;
    public float heading;


    public float gCost;
    public float hCost;
    public float hybridAdditionalCost = 1; //to be used while using hybrid A*
    public float wallClosenessCost = 0; 
    public float fCost {
        get
        {
            //Debug.Log("Node [" + i + "," + j + "] penalty " + wallClosenessCost +" fcost: " + (gCost + hCost + hybridAdditionalCost + wallClosenessCost));
            return gCost * hybridAdditionalCost + hCost + wallClosenessCost;
        }
    }
    public Node parent;
    public List<Node> neighbours;


    public Node(int i, int j, float x_pos, float z_pos, bool walkable=true)
    {
        this.i = i;
        this.j = j;
        this.x_pos = x_pos;
        this.z_pos = z_pos;
        this.walkable = walkable;
        this.worldPosition = new Vector3(x_pos, 0f, z_pos);
    }

    // override object.Equals
    public override bool Equals(object obj)
    {
        //
        // See the full list of guidelines at
        //   http://go.microsoft.com/fwlink/?LinkID=85237
        // and also the guidance for operator== at
        //   http://go.microsoft.com/fwlink/?LinkId=85238
        //

        if (obj == null || GetType() != obj.GetType())
        {
            return false;
        }


        return i == ((Node)obj).i && j == ((Node)obj).j; //////TODO add check also on positions
    }

    // override object.GetHashCode
    public override int GetHashCode()
    {

        return base.GetHashCode();
    }

    public Node copy()
    {
        Node copy = new Node(i, j, x_pos, z_pos, walkable);
        copy.neighbours = neighbours;
        copy.gCost = gCost;
        copy.hCost = hCost;
        copy.hybridAdditionalCost = hybridAdditionalCost;
        copy.wallClosenessCost = wallClosenessCost;
        copy.heading = heading;
        copy.parent = parent;
        return copy;
    }

}

