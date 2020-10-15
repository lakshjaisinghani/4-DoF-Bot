#include "VarSpeedServo.h"

// servo motors
VarSpeedServo claw;
VarSpeedServo shoulder;
VarSpeedServo elbow;
VarSpeedServo base;

// global variables
float endEffectorPos[3];
float jointAngles[4];
float lengths[4] = {6, 8, 8, 6};

float base_angle, shoulder_angle, elbow_angle;

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

/// calculates inverse 
/// kinematics
void calc_IK(float coord[])
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
  float m = (end_p[1] - y)/(end_p[0] - x);
  float c = y - m*x;

  int step_size = 1;
  int cnt = 0;

  if (end_p[1] < start_p[1] or end_p[0] < start_p[0])
  {
    step_size *= -1;
  }
      
  while (x != end_p[0] or y != end_p[1])
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
      x = x + step_size;
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
  //    float xyz[3] = {10, 2, 8};
  //    calc_IK(xyz);
  //    write_angles();
  //  base_angle = map(jointAngles[0] + 90, 0, 180, 180, 0);
  //  shoulder_angle = map(jointAngles[1], 0, 180, 180, 0);
  //  elbow_angle = map(jointAngles[3] + 90, 0, 180, 180, 0);

  // Part-3: line
//  float start_p[3] = {20, 0, 6};
//  float end_p[3]   = {8, 4, 6};
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
  arc(10, 30.0);  
}

void loop() {
}
