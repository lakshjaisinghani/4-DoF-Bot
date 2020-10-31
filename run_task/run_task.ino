#include "robot.h"
#include "sensors.h"

/// function declarations
void sweep_to_box(float, robot, edge_detector);
void sweep(float, robot, edge_detector);
void find_box(robot, edge_detector);
void center_on_block(float, robot, edge_detector, edge_detector);
void pick_up();

/// global variables
// bot
float center_pos[3] = {18, 0, 5};
float prev_pos[3];
int restricted_area = 0;

//cubes
int num_cubes = 6;
int picked_cubes = 5;

void setup()
{
  Serial.begin(9600);

  //  Objects init 
  robot Bot;
  edge_detector edge_storage_1(A0);
  edge_detector edge_storage_2(A1);
  edge_detector edge_robot_side(A2);
  edge_detector edge_robot_top(A3);

  // go to center
  Bot.calc_IK(center_pos);
  Bot.write_angles();
  delay(2000);

  // try to find box
  //find_box(Bot, edge_storage_1);


  // sweep and pick cubes
  float prev_pos[3] = {4.06, -7.97, 5.8};


  // sweep 
  sweep(prev_pos, Bot, edge_robot_side);
  
  // get prev EE pos
  // we would be right above a cube
  float *f = Bot.read_EE_pos();
  prev_pos[0] = *f;
  prev_pos[1] = *(f+1);
  prev_pos[2] = *(f+2); 

  Serial.println("found box");
  //Serial.println(f);

  
  center_on_block(prev_pos, Bot, edge_robot_side, edge_robot_top);
  //Bot.stop_bot();


  // go to center
  //Bot.calc_IK(center_pos);
  //Bot.write_angles();
  //delay(2000);
}

/// functions
///
void center_on_block(float pos[3], robot Bot, edge_detector edge_side, edge_detector edge_top)
{
  int side = edge_side.is_below();
  int top  = edge_top.is_below();
  float m;
  
  // tuneable parameters
  int max_iter = 1000; 
  int iter = 0;
  float step_size = 0.01;

  //If neither IR sensor sees an edge, the scanning routine should be resumed. 
  if (!side && !top) 
  {
    return;
  }

  while (side && (iter < max_iter))
  {
    pos[1] -= step_size;
    Bot.calc_IK(pos);
    Bot.write_angles();
    side = edge_side.is_below();
    iter++;

  }

  if (iter == max_iter)
  {
    Serial.println("Side Detection Timed Out");
  }

  iter = 0;
  m = tan(Bot.d2r(Bot.read_joint_angles()));
  
  while(!top && (iter < max_iter))
  {
    pos[0] -= step_size; 
    pos[1] = m * pos[0];
    Bot.calc_IK(pos);
    Bot.write_angles();
    top = edge_top.is_below();
    iter++;
  }

  if (iter == max_iter)
  {
    Serial.println("Side Detection Timed Out");
    return;
  }
  else
  {
    Serial.println("Picking up");
    // pick_up();
  }

  Serial.println("Out of func!");
}

void pick_up()
{

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


void sweep(float begin_coord[3], robot Bot, edge_detector edge)
{
  float end_x;
  float read_angle;
  float prev_base_angle;
  float *f;
  float start_coord[3];
  float end_coord[3];
  float sweep_to;
  int check = 0;

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
    end_x = 9.06;
  }
  else
  {
    start_coord[0] = 9.05;
    start_coord[1] = 0;
    start_coord[2] = 6;
    
    end_coord[0] = 19.99;
    end_coord[1] = 0;
    end_coord[2] = 6;
    
    sweep_to       = 63;
    end_x = 19.99;
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

    delay(1000);

    // slowly move base to zero
    Bot.write_servo(sweep_to, 3, 0);
    read_angle = Bot.read_angle(0);

    while  (read_angle != sweep_to)
    {

      if (edge.is_below())
      {
        // return current pos
        Bot.stop_bot();
        return;
      }

      Serial.print("Current angle: ");
      Serial.print(read_angle);
      Serial.println(" ");
      Serial.println(edge.get_measure());
      Serial.println(" ");
      read_angle = Bot.read_angle(0);
    }

    // update joint angles
    Bot.update_joint_angles();
    
    // go just below start_coord
    f = Bot.line(begin_coord, end_coord, prev_base_angle);
    begin_coord[0] = *f;
    begin_coord[1] = *(f+1);
    begin_coord[2] = *(f+2);   
  }
}

void loop()
{}
