#include "VarSpeedServo.h"

// servo motors
VarSpeedServo claw;
VarSpeedServo shoulder;
VarSpeedServo elbow;
VarSpeedServo base;
//Coordinates 



// global variables
float endEffectorPos[3];
float jointAngles[4];
float lengths[4] = {6, 8, 8, 6};
float coord_pickup[3] = {};// To Do: pickup global coordinates of where the arm exits the scan routine. These will be initialised as the starting point (either here or after the set up routine)
float domain[] = {};//To Do: Define n * 3 list where each row is a coordinate of edge points in the order that the scanning routine will hit them
//float bound_index[];
float base_angle, shoulder_angle, elbow_angle;
int foobar; 
int foo;
float boundaries [10][3]{
  {7,10,6},
  {7,0,6},
  {7,-4,6},
  {7,-10,6},
  {12,-10,6},
  {12,10,6},
  {15,10,6},
  {15,-10,6}
};
/// calculates inverse 
/// kinematics
void calc_IK(float coord[]) //calc_IK and write_angles() are called by the new classes so are defined at the top: this would be removed and sorted if I write a header file for the classes
{ 
  float l1 = lengths[0], l2 = lengths[1], l3 = lengths[2], l4 = lengths[3];
  float xx = pow(pow(coord[0],2)+pow(coord[1],2),0.5) - l4;
  float zz = coord[2]-l1; 
  
  jointAngles[0] = atan2(coord[1],coord[0]); 
  jointAngles[2] = -1*acos((pow(xx,2) + pow(zz,2) - pow(l2,2) - pow(l3,2))/(2*l2*l3));
  jointAngles[1] = (atan(zz/xx)-atan(l3*sin(jointAngles[2])/(l2+l3*cos(jointAngles[2]))));
  
  jointAngles[0]*=RAD_TO_DEG;
  jointAngles[1]*=RAD_TO_DEG;
  jointAngles[2]*=RAD_TO_DEG;
  jointAngles[3] = -1*(jointAngles[2] + jointAngles[1]);
}

/// writes current jointAngles[] to
/// servos
void write_angles()
{
  // add servo offsets
  base_angle = map(jointAngles[0] + 90, 0, 180, 180, 0);
  shoulder_angle = map(jointAngles[1], 0, 180, 180, 0);
  elbow_angle = map(jointAngles[3] + 90, 0, 180, 180, 0);
  
  // write angles
  shoulder.write(shoulder_angle);
  base.write(base_angle); 
  elbow.write(elbow_angle);
}



class Coordinates{ //Cartesian Coordinate Point/Vector. Contains methods to: 
                    //Define by the difference of two points (vector)
                    //Copy another point
                    //Get magnitude/distance from origin
                    //Scale each coordinate value by a scalar
                    //Increment by another coordinate vector
                    //Make robot go to given set of coordinates
    public:
    float xyz[3];
    Coordinates(){
        xyz[0] = 0;
        xyz[1] = 0;
        xyz[2] = 0;
        }
    Coordinates(float coord[]){
        xyz[0] = coord[0];
        xyz[1] = coord[1];
        xyz[2] = coord[3];
    }
    void by_difference(Coordinates point1, Coordinates point2){
        xyz[0] = -point1.xyz[0] + point2.xyz[0];
        xyz[1] = -point1.xyz[1] + point2.xyz[1];
        xyz[2] = -point1.xyz[2] + point2.xyz[2];
    }
    void go_to(){
        calc_IK(xyz);
        write_angles();
    }
    void copy_coords(Coordinates *xyz2){
        xyz[0] = (*xyz2).xyz[0];
        xyz[1] = (*xyz2).xyz[1];
        xyz[2] = (*xyz2).xyz[2];
    }
    float magnitude(){
        return pow((pow(xyz[0],2)+pow(xyz[1],2)+pow(xyz[2],2)),0.5);
    }
    void increment(Coordinates del){
        xyz[0] += del.xyz[0];
        xyz[1] += del.xyz[1];
        xyz[2] += del.xyz[2]; 
    }
    void scale(float k){
        xyz[0] *=k;
        xyz[1] *=k;
        xyz[2] *=k;
    }
};

class coordTicker{ //Class to store a set of points as boundaries that it moves to along a straight line.
                    //Goes to each point in steps (.tick()) to allow for routines to check for feedback
                    //Methods to: recognise when path has been followed to completion (.path_finished())
                    //Store and Return to coordinates if a routine is interrupted
                    //Go to the current desired coordinate
    Coordinates a,b,current,pickup,R,delta,vec_step;
    Coordinates bounds[10]; // Not necessarily 10, hence the use of a num_bounds
    float absR,step_size,eps,remaining_distance;
    int bound_index,num_bounds;
    public:
    coordTicker(float grain, float boundary_array[10][3],int boundary_number){
        step_size = grain;
        eps = step_size/2;
        bound_index = 1;
        num_bounds = boundary_number;
        for(int i = 1; i<num_bounds;i++){
            //bounds[i].Coordinates(boundary_array[i]);
            bounds[i].xyz[0] = boundary_array[i][0];
            bounds[i].xyz[1] = boundary_array[i][1];
            bounds[i].xyz[2] = boundary_array[i][2];
        }
        a.copy_coords(&(bounds[bound_index-1]));
        b.copy_coords(&(bounds[bound_index]));
        //Book-keeping coordinates: current and where the ticker has left off
        pickup.copy_coords(&a);
        current.copy_coords(&a);
        //Describing the remaining path to the next boundary
        delta.by_difference(current,b);
        remaining_distance = delta.magnitude();
        //Movement vector R
        R.by_difference(a,b);
        absR = R.magnitude();
        vec_step.copy_coords(&R);
        vec_step.scale(step_size/absR);
    }
    void store_coord(){
      pickup.copy_coords(&current);
    }
    void get_coord(){
      current.copy_coords(&pickup);
    }
    void go_to(){
        current.go_to();
    }
    void tick(){
      remaining_distance = delta.magnitude();
      if (remaining_distance<eps){//If close enough to the end point (b), update endpoints
        bound_index++;
        a.copy_coords(&(bounds[bound_index-1]));
        b.copy_coords(&(bounds[bound_index]));
        R.by_difference(a,b);
        absR = R.magnitude();
        vec_step.copy_coords(&R);
        vec_step.scale(step_size/absR);
      }
      current.increment(vec_step);
      delta.by_difference(current,b);
      remaining_distance = delta.magnitude();
    }
    bool path_finished(){
        return (bound_index+1 >= num_bounds);
    }
};

void scan_routine(class coordTicker scannerCoords){
  bool object_detected;
  //Pick up from where the scanning routine left off
  scannerCoords.get_coord();
  scannerCoords.go_to();
  //Check if there is an object, if not, start ticking 
  object_detected = get_IR();
  while (!object_detected && !scannerCoords.path_finished()) {
    scannerCoords.tick();
    object_detected = get_IR();
  }
  if (object_detected){
    scannerCoords.store_coord(); 
    collection_routine();
  }
};
 
bool get_IR(){
    Serial.println("We are checking the IR Sensor Here");
    return false;
}
void collection_routine(){
    Serial.println("We would enter the collection routine here");
    foo = 1; 
}
/// converts angle in degrees 
/// to radians
float d2r(float angle){
  return DEG_TO_RAD * angle;
}

/// calculates forward 
/// kinematics
void calc_FK(float Angles[])
{
  float gamma = lengths[1]*cos(d2r(Angles[1])) + lengths[2]*cos(d2r(Angles[1]+Angles[2]))+lengths[3];
  endEffectorPos[0] = gamma*cos(d2r(Angles[0]));     
  endEffectorPos[1] = gamma*sin(d2r(Angles[0]));  
  endEffectorPos[2] = lengths[0] + lengths[1]*sin(d2r(Angles[1])) + lengths[2]*sin(d2r(Angles[1]+Angles[2]));
}



/// draws an arc of n points for 
/// theta degrees
void arc(int n_pnts, float theta)
{
  // arbituary start point (x, y, z)
  float x = 12;
  float y =  0;
  float z =  6;

  float dtheta = theta / n_pnts;
  int cnt = 0;
  
  for (int i = 1; i < n_pnts; i++)
  {
    float pnt[3] = {x, y, z};
    calc_IK(pnt);

    if (cnt == 1)
     {
        claw.write(100);
        delay(1000);
        claw.write(60);
        delay(1000);
     }

    write_angles();
    delay(400);
    
    x = (x * cos(d2r(i * dtheta)));
    y = (x * sin(d2r(i * dtheta)));
    cnt++;
  }
}

/// draws a line between start and end
/// (x, y, z) coordinates
void line(float start_p[], float end_p[], bool flag)
{
  float x = start_p[0];
  float y = start_p[1];
  float z = start_p[2];
  float m = (end_p[1] - y)/(end_p[0] - x);//what if x is a constant - this produces an infinite gradient by this calculation
  float c = y - m*x;

  int step_size = 1;
  int cnt = 0;

  if (end_p[1] < start_p[1] or end_p[0] < start_p[0])
  {
    step_size *= -1;
  }
      
  while (x != end_p[0] or y != end_p[1]) //Might have to set some level of tolerance so the arm doesn't get stuck in this routine. 
  {
  
    if (cnt == 1 and flag)
     {
        claw.write(100);
        delay(1000);
        claw.write(60);
        delay(1000);
     }
    
    float pnt[3] = {x, y, z};
    calc_IK(pnt);

    write_angles();
    delay(400);
  
    if (end_p[0] == start_p[0])
    {
      // moving in y only
      y = y + step_size;
    }

    else if (end_p[1] == start_p[1])
    {
      // moving in x only
      x += step_size;
    }

    else
    {
      // moving in xy plane
      x = x + step_size; // One thing to consider here is that this method employs a step size which is greater (larger 2 norm) than when purely moving in the x or y direction. Perhaps a parametric definition would be more appropriate.
      y = m*x + c;
    }
    
    cnt++;
  }
}


void setup() 
{
  // put your setup code here, to run once:
  Serial.begin(9600);

  // set up motors 
  shoulder.attach(11);
  claw.attach(10); 
  base.attach(9); 
  elbow.attach(6); 

  coordTicker scanTicker(0.5,boundaries,6);
  coordTicker meepTicker(0.5,boundaries,6);
  scan_routine(scanTicker);
  
  // Part-1: Forward kinematics
  //  float angles[3] = {0, 0, 0};
  //  base_angle = map(angles[0] + 90, 0, 180, 180, 0)
  //  shoulder_angle = map(angles[1], 0, 180, 180, 0);
  //  elbow_angle = map(angles[2] + 90, 0, 180, 180, 0);

  //  calc_FK(angles);
  //  Serial.println(endEffectorPos[0]);
  //  Serial.println(endEffectorPos[1]);
  //  Serial.println(endEffectorPos[2]);

  // Part-2: Inverse kinematics
  //  float xyz[3] = {22, 0, 6};
  //  calc_IK(xyz);
  //  base_angle = map(jointAngles[0] + 90, 0, 180, 180, 0);
  //  shoulder_angle = map(jointAngles[1], 0, 180, 180, 0);
  //  elbow_angle = map(jointAngles[3] + 90, 0, 180, 180, 0);

  // Part-3: line
  //  float start_p[3] = {22, 0, 6};
  //  float end_p[3]   = {12, 0, 6};
  //  line(start_p, end_p, true);

  //Part-3: square
  //  float square_start[4][3] = {{18, 3, 6}, {18, -3, 6}, {10, -3, 6}, {10, 3, 6}};
  //  float square_end[4][3]   = {{18, -3, 6}, {10, -3, 6}, {10, 3, 6}, {18, 3, 6}};
  //
  //  line(square_start[0], square_end[0], true);
  //  delay(500);
  //  for(int i = 0; i < 3; i++)
  //  {
  //    line(square_start[i+1], square_end[i+1], false);
  //    delay(500);
  //  }

  //Part-3: arc
  //arc(10, 45.0);  
}

void loop() {
}
