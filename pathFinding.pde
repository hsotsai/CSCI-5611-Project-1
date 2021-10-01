
import java.util.HashSet; // Import the HashSet class
//You will only be turning in this file
//Your solution will be graded based on it's runtime (smaller is better), 
//the optimality of the path you return (shorter is better), and the
//number of collisions along the path (it should be 0 in all cases).

//You must provide a function with the following prototype:
// ArrayList<Integer> planPath(Vec2 startPos, Vec2 goalPos, Vec2[] centers, float[] radii, int numObstacles, Vec2[] nodePos, int numNodes);
// Where: 
//    -startPos and goalPos are 2D start and goal positions
//    -centers and radii are arrays specifying the center and radius
//    -numObstacles specifies the number of obstacles
//    -nodePos is an array specifying the 2D position of roadmap nodes
//    -numNodes specifies the number of nodes in the PRM
// The function should return an ArrayList of node IDs (indexes into the nodePos array).
// This should provide a collision-free chain of direct paths from the start position
// to the position of each node, and finally to the goal position.
// If there is no collision-free path between the start and goal, return an ArrayList with
// the 0'th element of "-1".

// Your code can safely make the following assumptions:
//   - The function connectNeighbors() will always be called before planPath()
//   - The variable maxNumNodes has been defined as a large static int, and it will
//     always be bigger than the numNodes variable passed into planPath()
//   - None of the positions in the nodePos array will ever be inside an obstacle
//   - The start and the goal position will never be inside an obstacle

// There are many useful functions in CollisionLibrary.pde and Vec2.pde
// which you can draw on in your implementation. Please add any additional 
// functionality you need to this file (PRM.pde) for compatabilty reasons.

// Here we provide a simple PRM implementation to get you started.
// Be warned, this version has several important limitations.
// For example, it uses BFS which will not provide the shortest path.
// Also, it (wrongly) assumes the nodes closest to the start and goal
// are the best nodes to start/end on your path on. Be sure to fix 
// these and other issues as you work on this assignment. This file is
// intended to illustrate the basic set-up for the assignmtent, don't assume 
// this example funcationality is correct and end up copying it's mistakes!).



//Here, we represent our graph structure as a neighbor list
//You can use any graph representation you like
ArrayList<Integer>[] neighbors = new ArrayList[maxNumNodes+2];  //A list of neighbors can can be reached from a given node
//We also want some help arrays to keep track of some information about nodes we've visited
Boolean[] visited = new Boolean[maxNumNodes+2]; //A list which store if a given node has been visited
int[] parent = new int[maxNumNodes+2]; //A list which stores the best previous node on the optimal path to reach this node

//Set which nodes are connected to which neighbors (graph edges) based on PRM rules
void connectNeighbors(Vec2[] centers, float[] radii, int numObstacles, Vec2[] nodePos, int numNodes, float radius){
  for (int i = 0; i < numNodes; i++){
    neighbors[i] = new ArrayList<Integer>();  //Clear neighbors list
    for (int j = 0; j < numNodes; j++){
      if (i == j) continue; //don't connect to myself 
      Vec2 dir = nodePos[j].minus(nodePos[i]).normalized();
      float distBetween = nodePos[i].distanceTo(nodePos[j]);
      hitInfo circleListCheck = rayCircleListIntersect(centers, radii, numObstacles, nodePos[i], dir, distBetween, radius);
      if (!circleListCheck.hit){
        neighbors[i].add(j);
      }
    }
  }
}

void connectNeighbor(Vec2[] centers, float[] radii, int numObstacles, Vec2[] nodePos, int numNodes, float radius){
  neighbors[numNodes] = new ArrayList<Integer>();  //Clear neighbors list
  neighbors[numNodes+1] = new ArrayList<Integer>();  //Clear neighbors list
  Vec2 startNode = nodePos[numNodes];
  Vec2 endNode = nodePos[numNodes+1];
 
  for (int i = 0; i < numNodes; i++){
    Vec2 curNode = nodePos[i];
     //If old start/end nodes are linked in the neighbors list remove them
    if(neighbors[i].size()>0 && neighbors[i].get(neighbors[i].size()-1)==numNodes){
        neighbors[i].remove(neighbors[i].size()-1);
    }
    if(neighbors[i].size()>0 && neighbors[i].get(neighbors[i].size()-1)==numNodes+1){
        neighbors[i].remove(neighbors[i].size()-1);
    }
    
    //find Direction/Distance between start node and curNode
    Vec2 dir = nodePos[i].minus(startNode).normalized();
    float distBetween = startNode.distanceTo(curNode);
    //check if intersect with any obstacles
    hitInfo circleListCheck = rayCircleListIntersect(centers, radii, numObstacles, startNode, dir, distBetween, radius);
    if (!circleListCheck.hit){
      //add start to neighbors[i] and vice versa
      neighbors[i].add(numNodes);
      neighbors[numNodes].add(i);
    }
  //repeat for end node  
  }
  for (int i = 0; i < numNodes+1; i++){
    Vec2 curNode = nodePos[i];
    Vec2 dir = nodePos[i].minus(endNode).normalized();
    float distBetween = endNode.distanceTo(curNode);
    hitInfo circleListCheck = rayCircleListIntersect(centers, radii, numObstacles, endNode, dir, distBetween, radius);
    if (!circleListCheck.hit){
      neighbors[i].add(numNodes+1);
      neighbors[numNodes+1].add(i);
    }
  }
}

ArrayList<Integer> planPath(Vec2 startPos, Vec2 goalPos, Vec2[] centers, float[] radii, int numObstacles, Vec2[] nodePos, int numNodes, float radius){
  ArrayList<Integer> path = new ArrayList();
  
  //make new nodePos list with start and end nodes
  Vec2[] newNodePos=new Vec2[numNodes+2];
  for(int i=0; i<numNodes; i++){
    newNodePos[i] = nodePos[i];
  }
  newNodePos[numNodes]=startPos;
  newNodePos[numNodes+1]=goalPos;
  
  //connect start and end nodes with neigh bors
  connectNeighbor(centers, radii, numObstacles, newNodePos, numNodes, radius);
  //runA with new nodePos array new max nodes, and location of start and end nodes*
  path = runA(newNodePos, numNodes+2, numNodes, numNodes+1);
  return path;
}

//A* (A* Search)
ArrayList<Integer> runA(Vec2[] nodePos, int numNodes, int startID, int goalID){
  HashSet<Integer> fringe= new HashSet<Integer>();  //New empty fringe
  ArrayList<Integer> path = new ArrayList();
  HashMap<Integer, Float[]> hm = new HashMap<Integer, Float[]>();
    if(neighbors[startID].size() <=0 || neighbors[goalID].size() <=0){
      //return -1 if start or end has no neighbors.
    path.add(0,-1);
    return path;
  }
  Vec2 goalPos = nodePos[goalID];
  
  for (int i = 0; i < numNodes; i++) { //Clear visit tags and parent pointers
    visited[i] = false;
    parent[i] = -1; //No parent yet
  }
  //add start to fringe visited and add info to hashmap
  visited[startID] = true;
  fringe.add(startID);
  hm.put(startID,new Float[3]);
  hm.get(startID)[0]=0.0;
  hm.get(startID)[1]=goalPos.distanceTo(nodePos[startID]);
  hm.get(startID)[2]=goalPos.distanceTo(nodePos[startID]);
  //initialize min values
  int minID= startID;
  float min= 9999;
  while (fringe.size() > 0){
    //remove minimum path +heuristic node from fringe
    int currentNode = minID;
    fringe.remove(minID);
    min=9999;
    if (currentNode == goalID){
      //if goal found exit
      break;
    }
    //check all fringe looking for min score
    for (int id : fringe){
      if(hm.get(id)[2]<min){
        min=hm.get(id)[2];
        minID=id;
      }
    }
    //loop through all neighbors of recently added node
    for (int i = 0; i < neighbors[currentNode].size(); i++){
      int neighborNode = neighbors[currentNode].get(i);
      if (!visited[neighborNode]){
        //if not visited add to visited fringe and add info to hashmap
        visited[neighborNode] = true;
        parent[neighborNode] = currentNode;
        fringe.add(neighborNode);
        hm.put(neighborNode,new Float[3]);
        hm.get(neighborNode)[0]=hm.get(currentNode)[0]+nodePos[currentNode].distanceTo(nodePos[neighborNode]);
        hm.get(neighborNode)[1]=goalPos.distanceTo(nodePos[neighborNode]);
        hm.get(neighborNode)[2]= hm.get(neighborNode)[1]+ hm.get(neighborNode)[0];
        //check if new score value is less than stored min
        if(hm.get(neighborNode)[2]<min){
          min=hm.get(neighborNode)[2];
          minID = neighborNode;
        }
      }
      else{
        //if visited check if new path to node is shorter then stored path and update
        if(hm.get(neighborNode)[0] >  hm.get(currentNode)[0]+nodePos[currentNode].distanceTo(nodePos[neighborNode])){
            hm.get(neighborNode)[0] = hm.get(currentNode)[0]+nodePos[currentNode].distanceTo(nodePos[neighborNode]);
            hm.get(neighborNode)[2]= hm.get(neighborNode)[1]+ hm.get(neighborNode)[0];
            parent[neighborNode] = currentNode;
            //check if new score value is less then stored min
            if(hm.get(neighborNode)[2]<min){
              min=hm.get(neighborNode)[2];
              minID = neighborNode;
            }
          }
        }
    } 
  }
  
  if (fringe.size() == 0){
    //if no path return -1
    path.add(0,-1);
    return path;
  }
    
  int prevNode = parent[goalID];
  //recreate path without start and end nodes
  while (prevNode >= 0 && numNodes-2>prevNode){
    path.add(0,prevNode);
    prevNode = parent[prevNode];
  }
  return path;
}
