#ifndef robot_h
#define robot_h

#include "Arduino.h"
#include "VarSpeedServo.h"
#include "sensors.h"

class robot
{
    private:
        // servo motors
        VarSpeedServo claw;
        VarSpeedServo shoulder;
        VarSpeedServo elbow;
        VarSpeedServo base;
        VarSpeedServo servos[4] = {claw, shoulder, elbow, base};

        // bot stats variables
        float endEffectorPos[3];
        float jointAngles[4];
        float box_pos[3];
        float lengths[4] = {6, 8, 8, 6};

        // workspace 
        // 0 -> left is restricted  (cubes on right) : -90 -> 0
        // 1 -> right is restricted (cubes on left)  :  90 -> 0
        int restricted_area;
        
    public:
        robot();
        float d2r(float angle);
        void calc_FK(float angles[]);
        void calc_IK(float coord[]);
        void write_angles();
        void update_joint_angles();
        void update_box_pos();
        void write_servo(int angle, int s_speed, int servo_ind);
        void print_coord(float coord[3], int id);
        void sweep_to_box(float begin_coord[3], float upto_angle, edge_detector edge);
        float * line(float start_p[], float end_p[], float angle);
};



#endif
