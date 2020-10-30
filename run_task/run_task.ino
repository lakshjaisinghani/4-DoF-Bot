#include "robot.h"
#include "sensors.h"

// function declarations
void sweep_to_box(float, robot, edge_detector);
void sweep(float, robot, edge_detector);
void find_box(robot, edge_detector);

// global variables
float center_pos[3] = {19, 0, 8};
int restricted_area = 0; 

void setup()
{
    Serial.begin(9600);

    //  Objects init 
    robot Bot;
    edge_detector edge_1(A0);

    // go to center
    Bot.calc_IK(center_pos);
    Bot.write_angles();
    delay(2000);

    // try to find box
    //find_box(Bot, edge_1);

    // sweep test
    float coord[3] = {4.06, -7.97, 6};
    sweep(coord, Bot, edge_1);
}

/// functions
///
void sweep_to_box(float begin_coord[3], robot Bot, edge_detector edge)
{
    float end_x = 9.06;
    float pose[3];
    float read_angle;
    float check_angle;
    int check = 0;
    float upto_angle = 63;

    // set claw to closed position
    Bot.write_servo(30, 0, 3);
    delay(1000);
  
    // go to begin coord
    Bot.calc_IK(begin_coord);
    Bot.write_angles();
    delay(1000);

    // slowly move base to zero
    Bot.write_servo(upto_angle, 3, 0);
    read_angle = Bot.read_angle(0);

    while (read_angle != upto_angle)
    {
        // argumented function (check)
        check = edge.is_below();

        Serial.print("Current angle: ");
        Serial.print(check);
        Serial.println(" ");

        read_angle = Bot.read_angle(0);

        if (check)
        {
                // 0 -> left is restricted  (cubes on right) : -90 -> 0
                // 1 -> right is restricted (cubes on left)  :  90 -> 0
                if (read_angle >= 0)
                {
                    restricted_area = 1;
                }
                else
                {
                    restricted_area = 0;
                }

                return;
        }
    }  
}


void find_box(robot Bot, edge_detector edge)
{
    // go to max dist
    // at -63.00 degrees
    float start_coord[3] = {9.06, -17.97, 8.9};

    // sweep full length
    sweep_to_box(start_coord, Bot, edge);

    //update jointAngles
    Bot.update_joint_angles();

    // stop if IR sensor gets triggered
    Bot.stop_bot();

    // TODO:
    // show that box is found
    // (bot state) some led action 
    delay(1000);
    
    // update robot.box_pos
    Bot.update_box_pos();

    // go to center
    Bot.calc_IK(center_pos);
    Bot.write_angles();
}

void sweep(float begin_coord[3], robot Bot, edge_detector edge)
{
    float end_x = 9.06;
    float read_angle;
    float prev_base_angle;
    float *f;
    float start_coord[3];
    float end_coord[3];
    float sweep_to;

    // line variables
    if (!restricted_area)
    {
      start_coord[0] = 4.06;
      start_coord[1] = -7.97;
      start_coord[2] = 6;
      
      end_coord[0] = 10.0;
      end_coord[1] = -19.6;
      end_coord[2] = 6;
      
      sweep_to       = 0;
    }
    else
    {
    //   float start_coord[3] = {4.06, 0, 6};
    //   float end_coord[3]   = {10.0, 0, 6};
    //   float sweep_to       = 63;
    }
    

    while (begin_coord[0] <= end_x)
    {
        Bot.print_coord(begin_coord, 2);

        // go to begin coord
        Bot.calc_IK(begin_coord);
        Bot.write_angles();
        
        //base angle to return to
        //jointAngles[0]
        prev_base_angle = Bot.d2r(Bot.read_joint_angles());

        delay(2000);

        // slowly move base to zero
        Bot.write_servo(sweep_to, 3, 0);
        read_angle = Bot.read_angle(0);

        while  (read_angle != sweep_to)
        {
            /*
            // Do all checks in here 
            */

            Serial.print("Current angle: ");
            Serial.print(read_angle);
            Serial.println(" ");

            read_angle = Bot.read_angle(0);
        }

        // go just below start_coord
        f = Bot.line(begin_coord, end_coord, prev_base_angle);
        begin_coord[0] = *f;
        begin_coord[1] = *(f+1);
        begin_coord[2] = *(f+2);   
    }
}

void loop()
{}
