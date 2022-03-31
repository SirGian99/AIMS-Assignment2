using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;


namespace UnityStandardAssets.Vehicles.Car
{
	[RequireComponent(typeof(CarController))]
	public class CarAI5 : MonoBehaviour
	{
		private CarController m_Car; // the car controller we want to use
		public Destructable Health;
		List<Vector3> Path = new List<Vector3>();
		public List<Vector3> ShootingPoints = new List<Vector3>();
		public List<float> HealthVec = new List<float>();
		public GameObject terrain_manager_game_object;
		TerrainManager terrain_manager;
		public GameObject[] friends;
		public GameObject[] enemies;
		List<Vector3> SetPoints = new List<Vector3>();
		List<Vector3> SafePoints = new List<Vector3>();
		Vector3[,] FreePos;
		Vector3 NewStart;
		List<List<int>> my_path = new List<List<int>>();
		public int[,] DangerMap;
		Color color2;
		Color color1;
		int x_low;
		int x_high;
		int x_N;
		int z_high;
		int z_low;
		int z_N;
		int x_length;
		int z_length;
		int x_M;
		int z_M;
		int[,] traversability;
		List<Vector3> car_pos = new List<Vector3>();
		Vector3 dir = new Vector3(10, 10, 10);
		public GameObject[] cars;

		Vector3 LastPos;
		int reverseCount = 0;
		int StuckCount = 0;
		bool Stuck = false;
		List<Vector3> OffPath = new List<Vector3>();
		int kPath = 0;
		int f = 0;
		int Count = 0;

		int leader = 0;
		int CarIndex = 0;
		bool NewPath = false;
		int EnemiesCounter = 0;
		bool firstTime = true;


		private void Start()
		{
			// get the car controller
			m_Car = GetComponent<CarController>();
			Health = GetComponent<Destructable>();

			terrain_manager = terrain_manager_game_object.GetComponent<TerrainManager>();

			x_low = (int)terrain_manager.myInfo.x_low;
			x_high = (int)terrain_manager.myInfo.x_high;
			x_N = (int)terrain_manager.myInfo.x_N;
			z_low = (int)terrain_manager.myInfo.z_low;
			z_high = (int)terrain_manager.myInfo.z_high;
			z_N = (int)terrain_manager.myInfo.z_N;
			x_length = (x_high - x_low);
			z_length = (z_high - z_low);
			x_M = x_length / x_N;
			z_M = z_length / z_N;

			// note that both arrays will have holes when objects are destroyed
			// but for initial planning they should work
			friends = GameObject.FindGameObjectsWithTag("Player");
			enemies = GameObject.FindGameObjectsWithTag("Enemy");
			EnemiesCounter = enemies.Length;
			foreach (GameObject obj in enemies)
			{
				//Debug.DrawLine(transform.position, obj.transform.position, Color.black, 10f);
			}

			// Plan your path here
			if (m_Car.transform.position.x == 407.8)
			{
				CarIndex = 0;
			}
			if (m_Car.transform.position.x == 414)
			{
				CarIndex = 1;
			}
			if (m_Car.transform.position.x == 419)
			{
				CarIndex = 2;
			}
			getTraversability();
			DiscretizeMap();
			ComputeDangerMap();
			GetShootingPoints();
			ComputePath();
			GetCarID();
			ComputeDangerMap();
			GetShootingPoints2();
		}

		public void GetShootingPoints()
		{
			//Get points of the map that can see just an enemy-->SetPoints
			//Get points of the map that cannot see any enemy-->SafePoints
			ShootingPoints.Clear();
			List<Vector3> Turrets = new List<Vector3>();
			int r = 0;
			bool hit;
			enemies = GameObject.FindGameObjectsWithTag("Enemy");
			foreach (GameObject obj in enemies)
			{
				//Debug.DrawLine(transform.position, obj.transform.position, Color.magenta, 100f);
				if (obj.transform.position != null)
				{
					Turrets.Add(obj.transform.position);
				}
			}

			for (int i = 0; i < x_N; i++)
			{
				for (int j = 0; j < z_N; j++)
				{
					if (FreePos[i, j].x != 0)
					{
						for (int t = 0; t < Turrets.Count; t++)
						{
							hit = Physics.Raycast(FreePos[i, j], Turrets[t] - FreePos[i, j], Vector3.Magnitude(Turrets[t] - FreePos[i, j]), 1 << 9);
							if (!hit)
							{
								r++;
							}
						}
						if (r == 1)
						{
							ShootingPoints.Add(FreePos[i, j]);
						}
						if (r == 0)
						{
							SafePoints.Add(FreePos[i, j]);
						}
						r = 0;
					}
				}
			}
		}
		public void ComputePath()
		{
			Vector3 Start = Vector3.zero;
			Path.Clear();
			for (int i = 0; i < ShootingPoints.Count; i++)
			{
				//Debug.DrawRay(ShootingPoints[i], dir, Color.magenta, 10f);
			}
			if (Count == 0)
			{
				Start = m_Car.transform.position;

				for (int i = 0; i < x_N; i++)
				{
					for (int j = 0; j < z_N; j++)
					{
						if (Vector3.Distance(Start, FreePos[i, j]) < 20)
						{
							Start = FreePos[i, j];
							break;
						}
					}

				}
				//Debug.DrawRay(Start, dir, Color.yellow, 100f);
			}
			else
			{
				Start = NewStart;
			}
			Vector3 TargetTurr = enemies[0].transform.position;
			Vector3 Target;
			bool hit;
			float Dist = Vector3.Distance(TargetTurr, Start);
			for (int i = 0; i < enemies.Length; i++)
			{
				if (Vector3.Distance(Start, enemies[i].transform.position) < Dist)
				{
					TargetTurr = enemies[i].transform.position;
					Dist = Vector3.Distance(TargetTurr, Start);
				}
			}
			for (int i = 0; i < x_N; i++)
			{
				for (int j = 0; j < z_N; j++)
				{
					if (Vector3.Distance(TargetTurr, FreePos[i, j]) < 20)
					{
						TargetTurr = FreePos[i, j];
						break;
					}
				}
			}
			//Debug.DrawRay(TargetTurr, dir, Color.yellow, 1000f);
			Dist = Vector3.Distance(ShootingPoints[0], TargetTurr);
			Target = ShootingPoints[0];
			if (Count == 0)
			{
				for (int i = 0; i < ShootingPoints.Count; i++)
				{
					hit = Physics.Raycast(ShootingPoints[i], TargetTurr - ShootingPoints[i], Vector3.Magnitude(TargetTurr - ShootingPoints[i]), 1 << 9);
					if (!hit && Vector3.Distance(Start, ShootingPoints[i]) < Dist)
					{
						Target = ShootingPoints[i];
						Dist = Vector3.Distance(ShootingPoints[i], TargetTurr);
					}
				}
				//Debug.DrawRay(Target, dir, Color.green, 1000f);
			}
			else
			{
				Dist = Vector3.Distance(ShootingPoints[0], m_Car.transform.position);
				for (int i = 0; i < ShootingPoints.Count; i++)
				{
					hit = Physics.Raycast(ShootingPoints[i], TargetTurr - ShootingPoints[i], Vector3.Magnitude(TargetTurr - ShootingPoints[i]), 1 << 9);
					if (!hit && Vector3.Distance(Start, ShootingPoints[i]) < Dist)
					{
						Target = ShootingPoints[i];
						Dist = Vector3.Distance(ShootingPoints[i], m_Car.transform.position);
					}
				}
				//Debug.DrawRay(Target, dir, Color.green, 1000f);
			}
			Node node = BFS(Target, Start);
			while (node.parent != null)
			{
				Path.Add(node.pos);
				node = node.parent;
			}
			Path.Add(Target);
			Path.Add(Target);
			for (int i = 0; i < Path.Count - 1; i++)
			{
				//Debug.DrawLine(Path[i], Path[i + 1], Color.black, 100f);
			}
		}
		public void GetShootingPoints2()
		{
			GetShootingPoints();
			ComputeDangerMap();
			Vector3 Start = m_Car.transform.position;
			float dist = Vector3.Distance(Start, ShootingPoints[0]);
			Vector3 goal = Start;
			for (int i = 0; i < ShootingPoints.Count; i++)
			{
				if (Vector3.Distance(Start, ShootingPoints[i]) < dist)
				{
					dist = Vector3.Distance(Start, ShootingPoints[i]);
					goal = ShootingPoints[i];
				}
			}
			Node node;
			node = Astar(Start, goal);
			while (node.parent != null)
			{
				Path.Add(node.pos);
				//Debug.DrawLine(target.pos, target.parent.pos, Color.cyan, 1000f);
				node = node.parent;
			}
			//Node target = new Node(null, 0, Start);
			//int cost = 10000000;
			//enemies = GameObject.FindGameObjectsWithTag("Enemy");
			//for (int i = 0; i < enemies.Length; i++)
			//{
			//    node = Astar(Start, enemies[i].transform.position);
			//    if (node.cost < cost)
			//    {
			//        cost = node.cost;
			//        target = node;
			//    }
			//}
			//while (target.parent != null)
			//{
			//    Path.Add(target.pos);
			//    //Debug.DrawLine(target.pos, target.parent.pos, Color.cyan, 1000f);
			//    target = target.parent;
			//}
			Path.Reverse();
			Path.Add(goal);
			for (int i = 0; i < Path.Count - 1; i++)
			{
				Debug.DrawLine(Path[i], Path[i + 1], Color.cyan, 1000f);
			}
		}
		public class Node
		{
			public Node parent;
			public int cost;
			public Vector3 pos;
			public Node(Node parent, int cost, Vector3 pos)
			{
				this.parent = parent;
				this.cost = cost;
				this.pos = pos;
			}
		}
		public Node BFS(Vector3 start, Vector3 goal)
		{
			bool[,] Visited = new bool[x_N * x_M, z_N * z_M];
			Queue<Node> queue = new Queue<Node>();
			Node root = new Node(null, 0, start);
			queue.Enqueue(root);
			while (queue.Count != 0)
			{
				Node curNode = queue.Dequeue();
				if (Vector3.Magnitude(curNode.pos - goal) < 10) { return curNode; }
				Vector3 up = new Vector3(curNode.pos.x, 0, curNode.pos.z + z_M);
				Vector3 down = new Vector3(curNode.pos.x, 0, curNode.pos.z - z_M);
				Vector3 left = new Vector3(curNode.pos.x - x_M, 0, curNode.pos.z);
				Vector3 right = new Vector3(curNode.pos.x + x_M, 0, curNode.pos.z);
				if (traversability[(int)up.x - x_low, (int)up.z - z_low] == 0 && !Visited[(int)up.x - x_low, (int)up.z - z_low])
				{
					Node newNode = new Node(curNode, curNode.cost + 1, up);
					queue.Enqueue(newNode);
					Visited[(int)up.x - 50, (int)up.z - 50] = true;
				}
				if (traversability[(int)down.x - x_low, (int)down.z - z_low] == 0 && !Visited[(int)down.x - x_low, (int)down.z - z_low])
				{
					Node newNode = new Node(curNode, curNode.cost + 1, down);
					queue.Enqueue(newNode);
					Visited[(int)down.x - 50, (int)down.z - 50] = true;
				}
				if (traversability[(int)left.x - x_low, (int)left.z - z_low] == 0 && !Visited[(int)left.x - x_low, (int)left.z - z_low])
				{
					Node newNode = new Node(curNode, curNode.cost + 1, left);
					queue.Enqueue(newNode);
					Visited[(int)left.x - 50, (int)left.z - 50] = true;
				}
				if (traversability[(int)right.x - x_low, (int)right.z - z_low] == 0 && !Visited[(int)right.x - x_low, (int)right.z - z_low])
				{
					Node newNode = new Node(curNode, curNode.cost + 1, right);
					queue.Enqueue(newNode);
					Visited[(int)right.x - 50, (int)right.z - 50] = true;
				}
			}
			return null;
		}
		public Node Astar(Vector3 start, Vector3 goal)
		{
			for (int i = 0; i < x_N; i++)
			{
				for (int j = 0; j < z_N; j++)
				{
					if (Vector3.Distance(start, FreePos[i, j]) < 20)
					{
						start = FreePos[i, j];
						break;
					}
				}

			}
			for (int i = 0; i < x_N; i++)
			{
				for (int j = 0; j < z_N; j++)
				{
					if (Vector3.Distance(goal, FreePos[i, j]) < 20)
					{
						goal = FreePos[i, j];
						break;
					}
				}

			}
			Node root = new Node(null, 0, start);
			Node neig = root;
			bool found;
			List<Node> OpenSet = new List<Node>();
			List<Vector3> Neig = new List<Vector3>();
			int tentativeScore = 0;
			int k = 0, p = 0;
			OpenSet.Add(root);
			Node current = root;
			int[,] gScore;
			//initialize gScore
			gScore = new int[x_N, z_N];
			for (int i = 0; i < x_N; i++)
			{
				for (int j = 0; j < z_N; j++)
				{
					if (FreePos[i, j] == start)
					{
						gScore[i, j] = 0;
					}
					else
					{
						gScore[i, j] = 100000;
					}
				}
			}


			while (OpenSet.Count > 0)
			{
				current = OpenSet[0];
				//SetCurrent as the best in OpenSet (the one with lowest cost)
				for (int i = 1; i < OpenSet.Count; i++)
				{
					if (OpenSet[i].cost < current.cost)
					{
						current = OpenSet[i];
					}
				}
				if (current.pos == goal)
				{
					break;
				}
				OpenSet.Remove(current);
				Neig = GetNeighbours(current.pos);
				tentativeScore = 0;
				for (int i = 0; i < Neig.Count; i++)
				{
					// get indexes of Neig[i]
					k = 0;
					p = 0;
					for (int n = 0; n < x_N; n++)
					{
						for (int m = 0; m < z_N; m++)
						{
							if (FreePos[n, m] == Neig[i])
							{
								k = n;
								p = m;
								break;
							}
						}
					}
					//update tentativeScore
					tentativeScore = current.cost + DangerMap[k, p];
					//check if you can find a better solution
					if (tentativeScore < gScore[k, p])
					{
						gScore[k, p] = tentativeScore;
						neig = new Node(current, tentativeScore, Neig[i]);
						found = false;
						for (int w = 0; w < OpenSet.Count; w++)
						{
							if (OpenSet[w].pos == Neig[i])
							{
								found = true;
							}
						}
						if (!found)
						{
							OpenSet.Add(neig);
						}
					}
				}
				Neig.Clear();
			}
			return current;
		}
		public List<Vector3> GetNeighbours(Vector3 point)
		{
			List<Vector3> Neighbours = new List<Vector3>();
			int i = 0, j = 0;
			for (int n = 0; n < x_N; n++)
			{
				for (int m = 0; m < z_N; m++)
				{
					if (FreePos[n, m] == point)
					{
						i = n;
						j = m;
					}
				}
			}
			if (FreePos[i, j + 1] != Vector3.zero)
			{
				Neighbours.Add(FreePos[i, j + 1]);
			}
			if (FreePos[i + 1, j] != Vector3.zero)
			{
				Neighbours.Add(FreePos[i + 1, j]);
			}
			if (FreePos[i, j - 1] != Vector3.zero)
			{
				Neighbours.Add(FreePos[i, j - 1]);
			}
			if (FreePos[i - 1, j] != Vector3.zero)
			{
				Neighbours.Add(FreePos[i - 1, j]);
			}
			return Neighbours;
		}
		public void getTraversability()
		{
			float[,] trav = terrain_manager.myInfo.traversability;
			traversability = new int[x_length, z_length];
			for (int i = 0; i < x_N; i++)
			{
				for (int j = 0; j < z_N; j++)
				{
					for (int k = 0; k < x_M; k++)
					{
						for (int l = 0; l < z_M; l++)
						{
							traversability[i * x_N + k, j * z_N + l] = (int)trav[i, j];
						}
					}
				}
			}
			int buffer = 3;
			for (int i = 0; i < x_length; i++)
			{
				for (int j = 0; j < z_length; j++)
				{
					int posX = (int)(i / x_M);
					int posZ = (int)(j / z_M);
					if (trav[posX, posZ] == 1.0)
					{
						int bufferXPlus = buffer;
						int bufferXMinus = -buffer;
						int bufferZPlus = buffer;
						int bufferZMinus = -buffer;
						if (bufferXMinus + i < 0) { bufferXMinus = 0; }
						if (bufferXPlus + i > x_length - 1) { bufferXPlus = 0; }
						if (bufferZMinus + j < 0) { bufferZMinus = 0; }
						if (bufferZPlus + j > z_length - 1) { bufferZPlus = 0; }
						for (int k = bufferXMinus; k <= bufferXPlus; k++)
						{
							for (int l = bufferZMinus; l <= bufferZPlus; l++)
							{
								traversability[k + i, l + j] = 1;
							}
						}
					}
				}
			}
		}
		public bool ObstacleFree(Vector3 nearest, Vector3 pos)
		{
			//UnityEngine.Debug.Log("From " + nearest + " To " + pos);
			float xDist = pos[0] - nearest[0];
			float zDist = pos[2] - nearest[2];
			float xAng = xDist + 0;
			float zAng = zDist + 0;
			if (zDist != 0)
			{
				xAng = xDist / Mathf.Abs(zDist);
			}
			if (xDist != 0)
			{
				zAng = zDist / Mathf.Abs(xDist);
			}
			if (Mathf.Abs(zAng) < Mathf.Abs(xAng))
			{
				xAng = 1 * Mathf.Sign(xDist);
			}
			else
			{
				zAng = 1 * Mathf.Sign(zDist);
			}
			bool obstacleFree = true;
			int length = (int)Mathf.Max(Mathf.Abs(zDist), Mathf.Abs(xDist));
			for (int i = 0; i < length; i++)
			{
				int xCord = (int)(nearest[0] - x_low - 1 + xAng * i);
				int zCord = (int)(nearest[2] - z_low - 1 + i * zAng);
				//UnityEngine.Debug.Log("X " + xCord + " Z " + zCord);
				if (traversability[xCord, zCord] == 1)
				{
					obstacleFree = false;
					break;
				}
			}
			return obstacleFree;
		}
		public void DiscretizeMap()
		{
			//Compute a matrix of Vector3 with the position of the discrete points of the map
			//(x, y, z) = (0, 0, 0) if obstacle
			float[,] trav = terrain_manager.myInfo.traversability;
			FreePos = new Vector3[x_N, z_N];

			for (int i = 0; i < x_N; i++)
			{
				for (int j = 0; j < z_N; j++)
				{
					if (trav[i, j] == 0)
					{
						Vector3 NewPos = new Vector3(x_low + (i * x_N) + x_N / 2, 0, z_low + (j * z_N) + z_N / 2);
						FreePos[i, j] = NewPos;
						//Debug.DrawRay(NewPos, dir, Color.black, 1000f);
					}
					else
					{
						Vector3 NewPos = new Vector3(0, 0, 0);
						FreePos[i, j] = NewPos;
					}
				}
			}
		}
		public void ComputeDangerMap()
		{
			List<Vector3> Turrets = new List<Vector3>();
			int f = 0;
			bool hit;
			foreach (GameObject obj in enemies)
			{
				//Debug.DrawLine(transform.position, obj.transform.position, Color.magenta, 100f);
				Turrets.Add(obj.transform.position);
			}
			DangerMap = new int[x_N, z_N];
			for (int i = 0; i < x_N; i++)
			{
				for (int j = 0; j < z_N; j++)
				{
					if (FreePos[i, j].x != 0)
					{
						for (int k = 0; k < Turrets.Count; k++)
						{
							hit = Physics.Raycast(FreePos[i, j], Turrets[k] - FreePos[i, j], Vector3.Magnitude(Turrets[k] - FreePos[i, j]), 1 << 9);
							if (!hit)
							{
								f++;
							}
						}
						DangerMap[i, j] = f;
						f = 0;
					}
					else
					{
						DangerMap[i, j] = 10000;
					}
				}
			}
			for (int i = 0; i < x_N; i++)
			{
				for (int j = 0; j < z_N; j++)
				{
					//               if (DangerMap[i, j] == 0)
					//               {
					//                   Debug.DrawRay(FreePos[i, j], dir, Color.green, 1000f);
					//               }
					//               if (DangerMap[i, j] == 1)
					//               {
					//                   Debug.DrawRay(FreePos[i, j], dir, Color.black, 1000f);
					//               }
					//if (DangerMap[i, j] == 2)
					//{
					//	Debug.DrawRay(FreePos[i, j], dir, Color.yellow, 1000f);
					//}
					//if (DangerMap[i, j] > 2)
					//{
					//	Debug.DrawRay(FreePos[i, j], dir, Color.red, 1000f);
					//}

					//if (DangerMap[i, j] == 3)
					//{
					//    Debug.DrawRay(FreePos[i, j], dir, Color.blue, 1000f);
					//}
					//if (DangerMap[i, j] == 4)
					//{
					//    Debug.DrawRay(FreePos[i, j], dir, Color.yellow, 1000f);
					//}
					//if (DangerMap[i, j] == 5)
					//{
					//    Debug.DrawRay(FreePos[i, j], dir, Color.red, 1000f);
					//}
					//if (DangerMap[i, j] == 6)
					//{
					//    Debug.DrawRay(FreePos[i, j], dir, Color.cyan, 1000f);
					//}
					//if (DangerMap[i, j] == 7)
					//{
					//    Debug.DrawRay(FreePos[i, j], dir, Color.white, 1000f);
					//}
					//if (DangerMap[i, j] == 8)
					//{
					//    Debug.DrawRay(FreePos[i, j], dir, Color.green, 1000f);
					//}
				}
			}
		}
		public void GetCarID()
		{
			HealthVec.Clear();
			float MaxHealt;
			int MaxHealtIndex = 0;
			cars = GameObject.FindGameObjectsWithTag("Player");
			for (int i = 0; i < cars.Length; i++)
			{
				HealthVec.Add(cars[i].GetComponent<Destructable>().health);
			}
			MaxHealt = HealthVec[0];
			for (int i = 1; i < HealthVec.Count; i++)
			{
				//Debug.Log("CarHealth: " + HealthVec[i]);
				if (HealthVec[i] > MaxHealt)
				{
					MaxHealt = HealthVec[i];
					MaxHealtIndex = i;
				}
			}
			leader = MaxHealtIndex;
			Debug.Log("leader: " + leader);
		}

		public Vector3 AdaptPath(Vector3 SetPoint)
		{
			Vector3 distance;
			Vector3 DesiredPos = Vector3.zero;
			cars = GameObject.FindGameObjectsWithTag("Player");
			if (cars.Length == 3)
			{
				if (leader == 0)
				{
					if (CarIndex == 1)
					{
						DesiredPos.x = -5f;
						DesiredPos.z = -0f;
						distance = cars[leader].transform.rotation * DesiredPos;
						SetPoint = cars[leader].transform.position + distance;
						Debug.DrawLine(cars[leader].transform.position, SetPoint);
					}
					if (CarIndex == 2)
					{
						DesiredPos.x = +0f;
						DesiredPos.z = -4f;
						distance = cars[leader].transform.rotation * DesiredPos;
						SetPoint = cars[leader].transform.position + distance;
						Debug.DrawLine(cars[leader].transform.position, SetPoint);
					}
				}
				if (leader == 1)
				{
					if (CarIndex == 0)
					{
						DesiredPos.x = -5f;
						DesiredPos.z = -0f;
						distance = cars[leader].transform.rotation * DesiredPos;
						SetPoint = cars[leader].transform.position + distance;
						Debug.DrawLine(cars[leader].transform.position, SetPoint);
					}
					if (CarIndex == 2)
					{
						DesiredPos.x = +0f;
						DesiredPos.z = -4f;
						distance = cars[leader].transform.rotation * DesiredPos;
						SetPoint = cars[leader].transform.position + distance;
						Debug.DrawLine(cars[leader].transform.position, SetPoint);
					}
				}
				if (leader == 2)
				{
					if (CarIndex == 0)
					{
						DesiredPos.x = +5f;
						DesiredPos.z = -0f;
						distance = cars[leader].transform.rotation * DesiredPos;
						SetPoint = cars[leader].transform.position + distance;
						Debug.DrawLine(cars[leader].transform.position, SetPoint);
					}
					if (CarIndex == 1)
					{
						DesiredPos.x = -0f;
						DesiredPos.z = -4f;
						distance = cars[leader].transform.rotation * DesiredPos;
						SetPoint = cars[leader].transform.position + distance;
						Debug.DrawLine(cars[leader].transform.position, SetPoint);
					}
				}
			}
			if (cars.Length == 2)
			{
				if (leader == 0)
				{
					if (CarIndex == 1)
					{
						DesiredPos.x = +5f;
						DesiredPos.z = -0f;
						distance = cars[leader].transform.rotation * DesiredPos;
						SetPoint = cars[leader].transform.position + distance;
						Debug.DrawLine(cars[leader].transform.position, SetPoint);
					}
					if (CarIndex == 2)
					{
						DesiredPos.x = -5f;
						DesiredPos.z = -0f;
						distance = cars[leader].transform.rotation * DesiredPos;
						SetPoint = cars[leader].transform.position + distance;
						Debug.DrawLine(cars[leader].transform.position, SetPoint);
					}
				}
				if (leader == 1)
				{
					if (CarIndex == 0)
					{
						DesiredPos.x = +5f;
						DesiredPos.z = -0f;
						distance = cars[leader].transform.rotation * DesiredPos;
						SetPoint = cars[leader].transform.position + distance;
						Debug.DrawLine(cars[leader].transform.position, SetPoint);
					}
					if (CarIndex == 2)
					{
						DesiredPos.x = -5f;
						DesiredPos.z = -0f;
						distance = cars[leader].transform.rotation * DesiredPos;
						SetPoint = cars[leader].transform.position + distance;
						Debug.DrawLine(cars[leader].transform.position, SetPoint);
					}
				}
			}
			return SetPoint;
		}
		private void FixedUpdate()
		{
			//Count = 0;

			bool YouLeader = false;
			bool waiting = false;
			OffPath = Path;
			cars = GameObject.FindGameObjectsWithTag("Player");

			//stuck recovery
			if (Stuck)
			{
				m_Car.Move(0f, 1f, -1f, 0f);
				reverseCount++;
				if (reverseCount > 100) //80
				{
					Stuck = false;
					reverseCount = 0;
				}
				return;
			}
			else if (LastPos.x.ToString("0.00") == m_Car.transform.position.x.ToString("0.00") && LastPos.z.ToString("0.00") == m_Car.transform.position.z.ToString("0.00"))
			{
				StuckCount++;
			}
			else
			{
				StuckCount = 0;
			}
			if (StuckCount > 5)
			{
				Stuck = true;
			}
			LastPos = m_Car.transform.position;
			Debug.Log("Count" + Count);
			if (Count > 0)
			{
				GetCarID();

				if (Health.health == cars[leader].GetComponent<Destructable>().health)
				{
					YouLeader = true;
				}

			}
			else
			{
				if (CarIndex == 0)
				{
					leader = 0;
					YouLeader = true;
				}
				else
				{
					YouLeader = false;
				}
			}

			Debug.Log("YouLeader" + YouLeader);


			Debug.Log("NewPath" + NewPath);
			Debug.Log("kPath" + kPath);
			if (NewPath)
			{
				//NewStart = Path[kPath - 1];
				Path.Clear();
				kPath = 0;
				Count++;
				//ShootingPoints.Clear();
				GetShootingPoints2();
				//ComputePath();
				OffPath = Path;
				NewPath = false;
			}
			else
			{
				enemies = GameObject.FindGameObjectsWithTag("Enemy");
				if (kPath >= OffPath.Count - 1 || enemies.Length < EnemiesCounter)
				{
					m_Car.Move(0f, 0f, 0f, 0f);
					EnemiesCounter = enemies.Length;
					NewPath = true;
					Count++;
					return;
				}
				if (YouLeader)
				{
					//followPath:
					//when close enough to Path[k] pass to next point in path
					if (Vector3.Magnitude(Path[kPath] - m_Car.transform.position) < 10)
					{
						kPath++;
					}
					//waiting function may be needed
					for (int i = 0; i < cars.Length; i++)
					{
						if (Vector3.Magnitude(m_Car.transform.position - cars[i].transform.position) > 20 && f > 30)
						{
							waiting = true;
						}
					}
					if (waiting)
					{
						//Vector3 AvPos = Vector3.zero;
						//for (int i = 0; i < cars.Length; i++)
						//{
						//	if (i != leader)
						//	{
						//		AvPos += cars[i].transform.position;
						//	}
						//}
						//AvPos /= (cars.Length - 1);
						//Single ControlV = 0.01f * Vector3.Magnitude(m_Car.transform.position - AvPos) + +0.5f * (m_Car.MaxSpeed / 20 - m_Car.CurrentSpeed);
						//Single steeringAngleErr = -Vector3.SignedAngle(AvPos - transform.position, transform.forward, transform.up);
						//Single control = 0.5f * steeringAngleErr;
						m_Car.Move(0f, 0f, -0.01f, 0f); // 0.01
														//Debug.DrawLine(m_Car.transform.position, AvPos, Color.magenta);
														//Debug.Log("WAITING");
					}
					else
					{
						//controller
						Single steeringAngleErr = -Vector3.SignedAngle(OffPath[kPath] - transform.position, transform.forward, transform.up);
						Single control = 0.5f * steeringAngleErr;
						Single controlV = 0.3f * Vector3.Magnitude(m_Car.transform.position - OffPath[kPath]) + 0.8f * (m_Car.MaxSpeed / 20 - m_Car.CurrentSpeed);

						if (Vector3.Magnitude(m_Car.transform.position - OffPath[kPath]) < 5)
						{
							if (m_Car.transform.position.x == OffPath[kPath].x && OffPath[kPath].x == OffPath[kPath + 1].x)
							{
								controlV = 1;
							}
							else if (m_Car.transform.position.z == OffPath[kPath].z && OffPath[kPath].z == OffPath[kPath + 1].z)
							{
								controlV = 1;
							}
							else
							{
								controlV = 0;
							}
						}
						if (m_Car.CurrentSpeed < 10)
						{
							controlV = 1;
						}
						Debug.DrawLine(m_Car.transform.position, OffPath[kPath]);
						Debug.Log("leaderrrrr");
						m_Car.Move(control, controlV, 0f, 0f);
					}
				}
				else
				{
					//followLeader
					f++; //to make the starting better
					if (Vector3.Magnitude(Path[kPath] - cars[leader].transform.position) < 10 && kPath < Path.Count - 1)
					{
						kPath++;
					}
					if (f > 40)
					{
						Vector3 Set;
						Set = AdaptPath(cars[leader].transform.position);
						Single steeringAngleErr = -Vector3.SignedAngle(Set - transform.position, transform.forward, transform.up);
						Single control = 0.5f * steeringAngleErr;
						Single controlV = 1f * Vector3.Magnitude(transform.position - Set) + 0.8f * (m_Car.MaxSpeed / 15 - m_Car.CurrentSpeed);
						Debug.DrawLine(transform.position, Set);
						m_Car.Move(control, controlV, 0f, 0f);
					}
					else
					{
						m_Car.Move(0f, 0f, 0f, 0f);
					}
				}
			}
			Debug.Log("CarIndex: " + CarIndex + " Leader: " + YouLeader + " Health: " + Health.health + " LeaderIndex: " + leader + " Count: " + Count);
		}

	}
}
