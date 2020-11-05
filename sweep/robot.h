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
        VarSpeedServo servos[4] = {base, shoulder, elbow, claw};

        // bot stats variables
        float endEffectorPos[3];
        float jointAngles[4];
        float lengths[4] = {6, 8, 8, 6};

        // box position
        float box_pos_1[3];
        float box_pos_2[3];

        // workspace 
        // 0 -> left is restricted  (cubes on right) : -90 -> 0
        // 1 -> right is restricted (cubes on left)  :  90 -> 0
        int restricted_area;
        
    public:
        robot();
        float d2r(float angle);
        float read_angle(int id);
        float read_joint_angles();
        float * read_EE_pos();
        void calc_FK(float angles[]);
        void calc_IK(float coord[]);
        void write_angles();
        void stop_bot();
        void update_joint_angles();
        void update_box_pos(int id);
        void write_servo(int angle, int s_speed, int servo_ind);
        void print_coord(float coord[3], int id);
        float * line(float start_p[], float end_p[], float angle);
};

#endif
