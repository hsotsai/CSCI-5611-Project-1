//CSCI 5611 - Graph Search & Planning
//PRM Sample Code [Proj 1]
//Instructor: Stephen J. Guy <sjguy@umn.edu>

//This is a test harness designed to help you test & debug your PRM.

//USAGE:
// On start-up your PRM will be tested on a random scene and the results printed
// Left clicking will set a red goal, right clicking the blue start
// The arrow keys will move the circular obstacle with the heavy outline
// Pressing 'r' will randomize the obstacles and re-run the tests

//Images for Textures
PImage ghost;
PImage fruit;
int counter =0;

//Change the below parameters to change the scenario/roadmap size
int numObstacles = 100;
int numNodes  = 900;
int numAgents = 25;
  
//Upper Bounds
static int maxNumObstacles = 1000;
static int maxNumNodes = 1000;
static int maxNumAgents = 100;
float maxSpeed =5000;

//A list of circle obstacles
Vec2 circlePos[] = new Vec2[maxNumObstacles]; //Circle positions
float circleRad[] = new float[maxNumObstacles];  //Circle radii

//A list of agents
Vec2 []agentPos = new Vec2[maxNumObstacles]; //Agent positions
float agentRad = 25; //Agent radii
Vec2 []agentVel = new Vec2[maxNumAgents];  //Agent velocities
int []pathLoc = new int[maxNumAgents];  //Agent step along path

//A list of goals
Vec2 []goalPos = new Vec2[maxNumObstacles]; //Goal positions

//A list of PRM nodes
Vec2[] nodePos = new Vec2[maxNumNodes];

//A list of paths for each agents
ArrayList<Integer>[] allPath = new ArrayList[maxNumAgents];

//Generate non-colliding PRM nodes
void generateRandomNodes(int numNodes, Vec2[] circleCenters, float[] circleRadii){
  for (int i = 0; i < numNodes; i++){
    Vec2 randPos = new Vec2(random(width),random(height));
    boolean insideAnyCircle = pointInCircleList(circleCenters,circleRadii,numObstacles,randPos,2);
    //boolean insideBox = pointInBox(boxTopLeft, boxW, boxH, randPos);
    while (insideAnyCircle){
      randPos = new Vec2(random(width),random(height));
      insideAnyCircle = pointInCircleList(circleCenters,circleRadii,numObstacles,randPos,2);
      //insideBox = pointInBox(boxTopLeft, boxW, boxH, randPos);
    }
    nodePos[i] = randPos;
  }
}

//Generate random obstacles
void placeRandomObstacles(int numObstacles){
  //Initial obstacle position
  for (int i = 0; i < numObstacles; i++){
    circlePos[i] = new Vec2(random(50,950),random(50,700));
    circleRad[i] = (10+40*pow(random(1),3));
  }
  //circleRad[0] = 30; //Make the first obstacle big
}

int strokeWidth = 2;
public void settings() {
  size(1024, 768, P2D);
}

void setup(){
  ghost=loadImage("ghost.png");
  fruit = loadImage("fruit.png");
  testPRM();
}

//Generate random nodes with a given radius
Vec2 sampleFreePos(float radius){
  Vec2 randPos = new Vec2(random(width),random(height));
  boolean insideAnyCircle = pointInCircleList(circlePos,circleRad,numObstacles,randPos,2+radius);
  while (insideAnyCircle){
    randPos = new Vec2(random(width),random(height));
    insideAnyCircle = pointInCircleList(circlePos,circleRad,numObstacles,randPos,2+radius);
  }
  return randPos;
}

//Run PRM
void testPRM(){  
  placeRandomObstacles(numObstacles);  //Generate random obstacles
  
  for(int x=0; x<numAgents; x++){
    agentPos[x] = sampleFreePos(agentRad);  //Generate random agents
    goalPos[x] = sampleFreePos(agentRad);  //Generate random goals
    pathLoc[x] = 0;  //Set starting position along path
    agentVel[x] = new Vec2(0,0);  //Set starting velocity of agent
  }
  
  generateRandomNodes(numNodes, circlePos, circleRad);  //Generate random PRM nodes
  connectNeighbors(circlePos, circleRad, numObstacles, nodePos, numNodes, agentRad);  //Connect neighbors with no collisions
  
  for(int x=0; x<numAgents; x++){
    allPath[x] = planPath(agentPos[x], goalPos[x], circlePos, circleRad, numObstacles, nodePos, numNodes, agentRad);  //Generate path for each agent
  }
}

//Calculate velocity for agents
Vec2 curVel(Vec2 curPos, Vec2 nextPos, float dt){
  Vec2 curVel=nextPos.minus(curPos).normalized().times(maxSpeed).times(dt);
  return curVel;
}

//Updates agents movement
void moveAgent(float dt){
  for (int i = 0; i < numAgents; i++){
    if(agentPos[i].distanceTo(goalPos[i]) <= agentRad) continue;        //agent has reached goal.
    if(allPath[i].size()==1 && allPath[i].get(0)==-1) continue;         //no path for agent
    hitInfo goalHit = rayCircleListIntersect(circlePos, circleRad, numObstacles, agentPos[i], goalPos[i].minus(agentPos[i]).normalized(), goalPos[i].distanceTo(agentPos[i]), agentRad);
    if(!goalHit.hit || allPath[i].size()==0 ||  pathLoc[i]==allPath[i].size()){      //set velocity to move towards goal if no collision.
      agentVel[i] = curVel(agentPos[i], goalPos[i], dt);
    }else{
      for(int x = pathLoc[i]+1; x<allPath[i].size(); x++){                           //check if agent can skip nodes along the path
        hitInfo tempHit = rayCircleListIntersect(circlePos, circleRad, numObstacles, agentPos[i], nodePos[allPath[i].get(x)].minus(agentPos[i]).normalized(), nodePos[allPath[i].get(x)].distanceTo(agentPos[i]), agentRad);
        if(!tempHit.hit){        //If node is skipped along path, update position along path
          pathLoc[i] = x;
        }
      }
      agentVel[i] = curVel(agentPos[i], nodePos[allPath[i].get(pathLoc[i])], dt);    //set agent velocity
    }
    agentPos[i].add(agentVel[i].times(dt));    //move agent position
  }
}

boolean paused = true;

void draw(){
  strokeWeight(1);
  background(255); //Grey background
  stroke(0,0,0);
  fill(255,255,255);
  
  //Update agent if not paused
  if (!paused){
    moveAgent(1.0/frameRate);
  }
  
  //Draw the circle obstacles
  for (int i = 0; i < numObstacles; i++){
    Vec2 c = circlePos[i];
    float r = circleRad[i];
    float rDist = r/sqrt(2);
    noStroke();
    textureMode(NORMAL); 
    beginShape();
    texture(ghost);
    vertex(c.x-rDist, c.y-rDist, 0, 0);
    vertex(c.x+rDist, c.y-rDist, 1, 0);
    vertex(c.x+rDist, c.y+rDist, 1, 1);
    vertex(c.x-rDist, c.y+rDist, 0, 1);
    endShape();
  }
  
  //Draw Goals
  for(int i = 0; i < numAgents; i++){
    Vec2 c =goalPos[i];
    float rDist = 1.5*agentRad/sqrt(2);
    noStroke();
    textureMode(NORMAL); 
    beginShape();
    texture(fruit);
    vertex(c.x-rDist, c.y-rDist, 0, 0);
    vertex(c.x+rDist, c.y-rDist, 1, 0);
    vertex(c.x+rDist, c.y+rDist, 1, 1);
    vertex(c.x-rDist, c.y+rDist, 0, 1);
    endShape();
  }
  
  //Draw Agents
  for(int i = 0; i < numAgents; i++){
    fill(255,255,0);
    circle(agentPos[i].x,agentPos[i].y,agentRad); 
    if((counter)%120>=60){
      fill(255);
      arc(agentPos[i].x,agentPos[i].y,agentRad, agentRad, -QUARTER_PI, QUARTER_PI);
    }
  }
  counter++;
 
  
}

boolean shiftDown = false;
void keyPressed(){
  if (key == 'r'){
    testPRM();
    return;
  }
  
  if (key == ' '){
    paused = !paused;
  }
  
  if (keyCode == SHIFT){
    shiftDown = true;
  }
}

void keyReleased(){
  if (keyCode == SHIFT){
    shiftDown = false;
  }
}

void mousePressed(){
  if (mouseButton == RIGHT){  //On right mouse click, create object on mouse if possible
    if(numObstacles<maxNumObstacles){
      circlePos[numObstacles] = new Vec2(mouseX, mouseY);
      circleRad[numObstacles] = (10+40*pow(random(1),3));
      numObstacles++;
      connectNeighbors(circlePos, circleRad, numObstacles, nodePos, numNodes, agentRad);
      for(int x=0; x<numAgents; x++){
        allPath[x] = planPath(agentPos[x], goalPos[x], circlePos, circleRad, numObstacles, nodePos, numNodes, agentRad);
        pathLoc[x] = 0;
      }
    }
  }
  else{
    if(numAgents<maxNumAgents){  //On left mouse click, create agent on mouse if possible 
      agentPos[numAgents] = new Vec2(mouseX, mouseY);
      goalPos[numAgents] = sampleFreePos(agentRad);
      connectNeighbors(circlePos, circleRad, numObstacles, nodePos, numNodes, agentRad);
      allPath[numAgents] = planPath(agentPos[numAgents], goalPos[numAgents], circlePos, circleRad, numObstacles, nodePos, numNodes, agentRad);
      numAgents++;
    }
  }
}
