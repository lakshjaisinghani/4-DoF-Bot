#include "Arduino.h"
#include "robot.h"
#include "VarSpeedServo.h"
#include "sensors.h"

robot::robot()
{
    // set up servo motors 
    shoulder.attach(6);
    claw.attach(9); 
    base.attach(10); 
    elbow.attach(11); 
}

/// converts angle in degrees 
/// to radians
float robot::d2r(float angle)
{
    return DEG_TO_RAD * angle;
}

/// calculates forward 
/// kinematics
void robot::calc_FK(float Angles[])
{
    float gamma = lengths[1]*cos(d2r(Angles[1])) + lengths[2]*cos(d2r(Angles[1]+Angles[2]))+lengths[3];
    endEffectorPos[0] = gamma*cos(d2r(Angles[0]));     
    endEffectorPos[1] = gamma*sin(d2r(Angles[0]));  
    endEffectorPos[2] = lengths[0] + lengths[1]*sin(d2r(Angles[1])) + lengths[2]*sin(d2r(Angles[1]+Angles[2]));
}


/// calculates inverse 
/// kinematics
void robot::calc_IK(float coord[])
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
void robot::write_angles()
{
    write_servo(jointAngles[1], 60, 1); // shoulder
    write_servo(jointAngles[3], 60, 2); // elbow
    write_servo(jointAngles[0], 60, 0); //base
}

void robot::stop_bot()
{
    base.stop();
    shoulder.stop();
    elbow.stop();
    claw.stop();
}

/// Adds servo offsets and 
/// writes angles
// TODO: if servo == base;
void robot::write_servo(int angle, int s_speed, int servo_ind)
{
    float s_angle = angle;

    if (servo_ind == 1)
    {
        //shoulder
        jointAngles[1] = angle;
        s_angle = map(angle, 0, 180, 180, 0);
    }
    else if (servo_ind == 2)
    {
        // elbow
        jointAngles[3] = angle;
        s_angle = map(angle + 45, 0, 180, 180, 0);
    }
    else if (servo_ind == 0)
    {
        // base
        jointAngles[0] = angle;
        s_angle = map(angle + 90, 0, 180, 180, 0);
    }
    
    // update end effector pos
    calc_FK(jointAngles);
    
    servos[servo_ind].write(s_angle, s_speed);
}

void robot::print_coord(float coord[3], int id)
{
  String coord_type;
  
    if (id == 1)
    {
      coord_type = "Begin Coord ";
    }
    else if (id == 2)
    {
      coord_type = "End Coord ";
    }
    else
    {
      coord_type = "End Effector coord ";
    }
    
    Serial.print(coord_type);
    Serial.print(coord[0]);
    Serial.print(", ");
    Serial.print(coord[1]);
    Serial.print(", ");
    Serial.print(coord[2]);
    Serial.println(" ");
}

/// draws a line between start and end
/// (x, y, z) coordinates
float * robot::line(float start_p[], float end_p[], float angle)
{
    float x = start_p[0];
    float y = start_p[1];
    float z = start_p[2];
    int lim = 2;

    // we use the equation y = mx
    // and keep y-intercept as zero.
    float m = tan(angle);

    static float pnt[3];

    float step_size = 0.5;
    int cnt = 0;

    // removed end_p[1] < start_p[1]
    if (end_p[0] < start_p[0])
    {
        step_size *= -1;
    }
    
    if (lim)
    {
        for(int i = 0; i < lim; i++)
        {
            pnt[0] = x;
            pnt[1] = y;
            pnt[2] = z;
            
            calc_IK(pnt);
            write_angles();

            // moving in xy plane
            x = x + step_size;
            y = m*x;
        }
        
        return pnt;
    }
}

void robot::update_joint_angles()
{
    float servo_angles[3];
    servo_angles[0] = map(base.read(), 0, 180, 180, 0) - 90;  // joint angles 0
    servo_angles[1] = map(shoulder.read(), 0, 180, 180, 0);   // joint angles 1
    servo_angles[2] = map(elbow.read(), 0, 180, 180, 0) -45; // joint angles 3

    // as we need joint angles 2
    servo_angles[2] = -1 * (servo_angles[2] + servo_angles[1]);

    calc_FK(servo_angles);
    calc_IK(endEffectorPos);
}

void robot::update_box_pos(int id)
{
  // update jointAngles
  update_joint_angles();

  if (id)
  {
    for (int i = 0; i < 3; i++)
    {
        box_pos_1[i] = endEffectorPos[i];
        Serial.println(box_pos_1[i]);
    }
  }
  else
  {
    for (int i = 0; i < 3; i++)
    {
        box_pos_2[i] = endEffectorPos[i];
        Serial.println(box_pos_2[i]);
    }
  }
  
}

float robot::read_angle(int id)
{
    switch(id)
    {
        case 0: 
            return (float) map(base.read(), 0, 180, 180, 0) - 90;
            break;

        case 1:
            return (float) map(shoulder.read(), 0, 180, 180, 0);
            break;

        case 2:
            return (float) map(elbow.read(), 0, 180, 180, 0) - 90;
            break;
            
        case 3: 
            return (float) claw.read();
            break;
            
        default:
            break;
    } 
}

float robot::read_joint_angles()
{
    return jointAngles[0];
}

float * robot::read_EE_pos()
{
    update_joint_angles();

    static float pos[3];

    pos[0] = endEffectorPos[0];
    pos[1] = endEffectorPos[1];
    pos[2] = endEffectorPos[2];

    return pos;
}
