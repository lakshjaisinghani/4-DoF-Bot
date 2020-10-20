#include "VarSpeedServo.h"

// servo motors
VarSpeedServo claw;
VarSpeedServo shoulder;
VarSpeedServo elbow;
VarSpeedServo base;

// global variables
float jointAngles[4];
float base_angle, shoulder_angle, elbow_angle;
float endEffectorPos[3];
float lengths[4]     = {6, 8, 8, 6};

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

/// draws a line between start and end
/// (x, y, z) coordinates
float * line(float start_p[], float end_p[], bool flag, int lim)
{
    float x = start_p[0];
    float y = start_p[1];
    float z = start_p[2];
    float m = (end_p[1] - y)/(end_p[0] - x);
    float c = y - m*x;

    static float pnt[3];

    int step_size = 1;
    int cnt = 0;

    if (end_p[1] < start_p[1] or end_p[0] < start_p[0])
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
            delay(400);

            // moving in xy plane
            x = x + step_size;
            y = m*x + c;
        }
        return pnt;
    }
}

// sweep
void sweep()
{
    float curr_angle = -70;
    float angle;
    float *f;
    
    float curr_coord[3] = {8, -10, 6};
    float end_coord[3]  = {7,   0,  6};
    float zero[3]       = {0, 0, 6} ;
    int limit           = 2;

    Serial.println("-----------");
    Serial.println(curr_coord[0]);
    Serial.println(curr_coord[1]);
    Serial.println(curr_coord[2]);
    Serial.println("-----------");

    while(curr_coord != end_coord)
    {
        curr_angle    = -70;
        calc_IK(curr_coord);
        write_angles();
    
        while (curr_angle != 0)
        {
            angle = map(curr_angle + 90, 0, 180, 180, 0);
            base.write(angle); 
            curr_angle += 2;
            delay(500);
        }

        f = line(curr_coord, zero, false, limit);
        
        curr_coord[0] = *f;
        curr_coord[1] = *(f+1);
        curr_coord[2] = *(f+2);

        Serial.println("-----------");
        Serial.println(curr_coord[0]);
        Serial.println(curr_coord[1]);
        Serial.println(curr_coord[2]);
        Serial.println("-----------");
    }
}


void setup()
{
    Serial.begin(9600);

    // set up motors 
    shoulder.attach(11);
    claw.attach(10); 
    base.attach(9); 
    elbow.attach(6); 

    sweep();
}

void loop() 
{

}
