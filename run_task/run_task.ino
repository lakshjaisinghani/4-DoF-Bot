#include "robot.h"
#include "sensors.h"

// function declarations
void find_box(robot, edge_detector);


void setup()
{
    Serial.begin(9600);

    //  Objects init 
    robot Bot;
    edge_detector edge_1(A0);

    find_box(Bot, edge_1);
}

void loop()
{
   /*
    do nothing
   */
}


/// functions
void find_box(robot Bot, edge_detector edge)
{
    // go to max dist
    // at -63.00 degrees
    float start_coord[3] = {9.06, -17.97, 6.00};

    // sweep full length
    Bot.sweep_to_box(start_coord, 63.00, edge);

    //update jointAngles
    Bot.update_joint_angles();

    // stop if IR sensor gets triggered
    Serial.print("Stopped");
    
    // update robot.box_pos
    Bot.update_box_pos();
}