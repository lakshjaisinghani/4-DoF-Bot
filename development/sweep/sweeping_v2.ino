//#include "VarSpeedServo.h"
//
///// servo motors
//VarSpeedServo claw;
//VarSpeedServo shoulder;
//VarSpeedServo elbow;
//VarSpeedServo base;
//VarSpeedServo servos[4] = {claw, shoulder, elbow, base};
//
/////global variables
//float jointAngles[4];
//float base_angle, shoulder_angle, elbow_angle;
//float endEffectorPos[3];
//float lengths[4]     = {6, 8, 8, 6};
//
///// sweep coords
//// left hand side only
//// TODO: RHS coords?
//float start_coord[3] = {4.06, -7.97, 5.98};
//float end_coord[3]   = {10, -20.03, 6};
//float begin_pose[3]  = {10, 0, 9};
//
///// converts angle in degrees 
///// to radians
//float d2r(float angle){
//    return DEG_TO_RAD * angle;
//}
//
///// calculates forward 
///// kinematics
//void calc_FK(float Angles[])
//{
//    float gamma = lengths[1]*cos(d2r(Angles[1])) + lengths[2]*cos(d2r(Angles[1]+Angles[2]))+lengths[3];
//    endEffectorPos[0] = gamma*cos(d2r(Angles[0]));     
//    endEffectorPos[1] = gamma*sin(d2r(Angles[0]));  
//    endEffectorPos[2] = lengths[0] + lengths[1]*sin(d2r(Angles[1])) + lengths[2]*sin(d2r(Angles[1]+Angles[2]));
//}
//
///// calculates inverse 
///// kinematics
//void calc_IK(float coord[])
//{ 
//    float l1 = lengths[0], l2 = lengths[1], l3 = lengths[2], l4 = lengths[3];
//    float xx = pow(pow(coord[0],2)+pow(coord[1],2),0.5) - l4;
//    float zz = coord[2]-l1; 
//    
//    jointAngles[0] = atan2(coord[1],coord[0]); 
//    jointAngles[2] = -1*acos((pow(xx,2) + pow(zz,2) - pow(l2,2) - pow(l3,2))/(2*l2*l3));
//    jointAngles[1] = (atan(zz/xx)-atan(l3*sin(jointAngles[2])/(l2+l3*cos(jointAngles[2]))));
//    
//    jointAngles[0]*=RAD_TO_DEG;
//    jointAngles[1]*=RAD_TO_DEG;
//    jointAngles[2]*=RAD_TO_DEG;
//    jointAngles[3] = -1*(jointAngles[2] + jointAngles[1]);
//}
//
///// writes current jointAngles[] to
///// servos
//void write_angles()
//{
//    write_servo(jointAngles[1], 80, 1); // shoulder
//    write_servo(jointAngles[3], 80, 2); // elbow
//    write_servo(jointAngles[0], 80, 3); //base
//}
//
///// Adds servo offsets and 
///// writes angles
//// TODO: if servo == base;
//void write_servo(int angle, int s_speed, int servo_ind)
//{
//    float s_angle;
//
//    if (servo_ind == 1)
//    {
//        //shoulder
//        jointAngles[1] = angle;
//         s_angle = map(angle, 0, 180, 180, 0);
//    }
//    else if (servo_ind == 2)
//    {
//        // elbow
//        jointAngles[3] = angle;
//        s_angle = map(angle + 90, 0, 180, 180, 0);
//    }
//    else if (servo_ind == 3)
//    {
//        // base
//        jointAngles[0] = angle;
//        s_angle = map(angle + 90, 0, 180, 180, 0);
//    }
//
//    // update end effector pos
//    calc_FK(jointAngles);
//    
//    servos[servo_ind].write(s_angle, s_speed);
//}
//
///// draws a line between start and end
///// (x, y, z) coordinates
//float * line(float start_p[], float end_p[], int lim)
//{
//    float x = start_p[0];
//    float y = start_p[1];
//    float z = start_p[2];
//    float m = (end_p[1] - y)/(end_p[0] - x);
//    float c = y - m*x;
//
//    static float pnt[3];
//
//    int step_size = 1;
//    int cnt = 0;
//
//    // removed end_p[1] < start_p[1]
//    if (end_p[0] < start_p[0])
//    {
//        step_size *= -1;
//    }
//    
//    if (lim)
//    {
//        for(int i = 0; i < lim; i++)
//        {
//            pnt[0] = x;
//            pnt[1] = y;
//            pnt[2] = z;
//            
//            calc_IK(pnt);
//            write_angles();
//
//            // moving in xy plane
//            x = x + step_size;
//            y = m*x + c;
//        }
//        
//        return pnt;
//    }
//}
//
//void print_coord(float coord[3], int id)
//{
//  String coord_type;
//  
//    if (id == 1)
//    {
//      coord_type = "Begin Coord ";
//    }
//    else if (id == 2)
//    {
//      coord_type = "End Coord ";
//    }
//    else
//    {
//      coord_type = "End Effector coord ";
//    }
//    
//    Serial.print(coord_type);
//    Serial.print(coord[0]);
//    Serial.print(", ");
//    Serial.print(coord[1]);
//    Serial.print(", ");
//    Serial.print(coord[2]);
//    Serial.println(" ");
//}
//
//void sweep(float begin_coord[3],bool half_plane)
//{
//    //float end_x = 9.06;
//    int servo_ind = 3;
//    float read_angle;
//    float *f;
//    if (half_plane) // half_plane = True -> Left half plane
//    {
//        float start_coord_int[3] = {4.06, -7.97, 5.98};
//        float end_coord_int[3]   = {10, -20.03, 6};
//        float end_x = 9.06;
//
//        float end_read_angle = 90; 
//        int end_write_angle = 0; 
//    } else 
//    {
//        //float working_space_width = 13.4;
//        float start_coord_int[3] = {4.06, -7.97, 5.98}; //Middle lower point
//        float end_coord_int[3]   = {10, -20.03, 6}; // Middle upper point
//        float end_x = 9.06; // + ???;
//
//        float end_read_angle = 90+60; //Right-hand bound on base servo
//        int end_write_angle = 60; //Destination base servo angle 
//    }
//
//    while (begin_coord[0] < end_x)
//    {
//        // go to begin coord
//        calc_IK(begin_coord);
//        write_angles();
//
//        delay(5000);
//
//        // slowly move base to zero (final angle)  
//        write_servo(end_write_angle, 3, servo_ind);
//        read_angle = base.read();
//
//        while  (read_angle != end_read_angle)
//        {
//            /*
//            // Do all checks in here
//            */
//
//            Serial.print("Current angle: ");
//            Serial.print(read_angle);
//            Serial.println(" ");
//
//            read_angle = base.read();
//        }
//
//        // go just below start_coord
//        f = line(start_coord, end_coord, 2);
//        begin_coord[0] = *f;
//        begin_coord[1] = *(f+1);
//        begin_coord[2] = *(f+2);   
//    }
//}
//
//
//void setup()
//{
//    Serial.begin(9600);
//
//    // set up motors 
//    shoulder.attach(11);
//    claw.attach(10); 
//    base.attach(9); 
//    elbow.attach(6); 
//
//    // sweep
//    sweep(start_coord);
//
//    // go back to start
//    calc_IK(begin_pose);
//    write_angles();
//
//}
//
//void loop() 
//{
//
//}
