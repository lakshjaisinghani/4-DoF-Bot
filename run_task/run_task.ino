#include "robot.h"
#include "sensors.h"

/// function declarations
void sweep_to_box(float, robot, edge_detector);
void sweep(float, robot, edge_detector &edge1, edge_detector &edge2);
void find_box(robot &Bot, edge_detector edge1, edge_detector edge2);
void center_on_block(float, robot, edge_detector, edge_detector);
int find_colour(float, robot, colour_sensor);
int pick_up(robot, limit_switch);

/// global variables
// bot
float center_pos[3] = {9.05, 0, 8.9};
float prev_pos[3];
int restricted_area = 2;
char input;

//cubes
int num_cubes = 1;
int picked_cubes = 0;

// colour
const char colourIdStrings[4][1] = {'A', 'R', 'G', 'B'};

void setup()
{
  Serial.begin(9600);
  
  //  Objects init 
  robot Bot;
  edge_detector edge_storage_1(A0);
  edge_detector edge_storage_2(A1);
  edge_detector edge_robot_side(A2);
  edge_detector edge_robot_top(A3);
  colour_sensor colour(A4);
  limit_switch  limit(7);

  float edge_calib_pos[3] = {12, 0, 5};
  float col_calib_pos[3]  = {-1.0, -9.05, 4};
 
  // calibrate colour
//  Bot.write_servo(180, 0, 3);
//  Bot.calc_IK(col_calib_pos);
//  Bot.write_angles();
//  delay(2000);
//
//  Serial.println("Would you like to calibrate colour sensor? (y/n)");
//  while (!Serial.available())
//  {}
//
//  // calibrate colour sensor
//  colour.calibrate();
//  delay(2000);
//  Serial.println("Colour calibration done..");
//  colour.leds_togle(0);
//  delay(2000);

  //go to center
  Bot.calc_IK(center_pos);
  Bot.write_angles();
  delay(2000);

  // set claw to closed
  Bot.write_servo(0, 0, 3);
  delay(2000);
//
//  Serial.end();
//  Serial.begin(9600);
//  
//  Serial.print("Would you like to start task? (y/n)");
//  while (!Serial.available())
//  {}

  // try to find box
  find_box(Bot, edge_storage_1, edge_storage_2);

  //Serial.println(restricted_area);
  // while (picked_cubes < num_cubes)
  // {
  //   // 0 -> small
  //   // 1 -> big
  //   int block_type;
    
  //   // sweep and pick cubes
  //   // get prev_pos from find box
  //   float prev_pos[3] = {4.06, -7.97, 5.5};
  //   sweep(prev_pos, Bot, edge_robot_side, edge_robot_top);
    
  //   // get prev EE pos
  //   float *f = Bot.read_EE_pos();
  //   prev_pos[0] = *f;
  //   prev_pos[1] = *(f+1);
  //   prev_pos[2] = *(f+2); 

  //   Serial.println("found box");
    
  //   center_on_block(prev_pos, Bot, edge_robot_side, edge_robot_top);
  //   Bot.stop_bot();

  //   Serial.println("centered");

  //   // 1 -> R | 2 -> G | 3 -> B
  //   int block_colour = find_colour(prev_pos, Bot, colour);

  //   // pick up
  //   int angle = pick_up(Bot, limit);
  //   block_type = (angle < 20) ? 0 : 1;
    
  //   if (angle < 2) block_type = 2;

  //   // if there is a block in the gripper
  //   // go to blobk storage pos
  //   if (block_type < 2)
  //   {

  //     Bot.calc_IK(center_pos);
  //     Bot.write_angles();
  //     delay(2000);

  //     //drop box
  //     Bot.write_servo(120, 0, 3);
  //     picked_cubes++;
  //     delay(2000);
  //     Bot.write_servo(0, 0, 3);
  //   }
  // }
}

/// functions
///
int pick_up(robot Bot, limit_switch lim)
{
  Bot.write_servo(0, 30, 3);
  //delay(2000);

  int read_angle = Bot.read_angle(3);

  while (read_angle != 0)
  {
    // argumented function (check)
    if (lim.button_state()) 
    {
      Bot.stop_bot();
      Bot.write_servo(0, 0, 3);
      break;
    };

    read_angle = Bot.read_angle(3);
  }

  delay(1000);

  // get current pos
  float curr_pos[3];
  float *f = Bot.read_EE_pos();
  curr_pos[0] = *f;
  curr_pos[1] = *(f+1);
  curr_pos[2] = 8.5; 

  // move to safe height
  Bot.calc_IK(curr_pos);
  Bot.write_angles();
  delay(2000);

  return read_angle;
}

int find_colour(float pos[3], robot Bot, colour_sensor colour)
{
  int col_indx;

  // open claw
  Bot.write_servo(120, 0, 3);
  delay(2000);

  // go down
  pos[2] = 4;
  Bot.calc_IK(pos);
  Bot.write_angles();
  delay(2000);
  
  // sense coulour
  col_indx = colour.colourId();
  Serial.println(colourIdStrings[col_indx][0]);

  colour.leds_togle(0);
  
  return col_indx;
}

void center_on_block(float pos[3], robot Bot, edge_detector &edge_side, edge_detector &edge_top)
{
  int side = edge_side.side_edge_is_below();
  int top  = edge_top.is_below();
  float m;

  Serial.print(edge_side.side_edge_is_below());
  
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
    side = edge_side.side_edge_is_below();
    iter++;
    Serial.println("side");
    Serial.println(edge_side.get_measure());
    Serial.println(edge_side.side_edge_is_below());
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
    top = edge_top.top_edge_is_below();
    iter++;
    Serial.println("top");
    Serial.println(edge_top.get_measure());
  }

  if (iter == max_iter)
  {
    Serial.println("Side Detection Timed Out");
    return;
  }
  else
  {
    Serial.println("Picking up");
    return;
  }
}

void find_box(robot &Bot, edge_detector edge1, edge_detector edge2)
{
  // go to max dist
  // at -63.00 degrees
  float start_coord_top_1[3] = {9.06, -17.97, 7.65}; // left half top
  float start_coord_top_2[3] = {19.99, 0, 7.1}; // right half top
  float start_coord_bot[3] = {3.15, -6.18, 6.25};   // bottom


  // sweep half length
  sweep_to_box(start_coord_top_1, Bot, edge1);
  Bot.stop_bot();
  delay(1000);
  
  if (restricted_area > 1)
  {
  
    // sweep other half length because box not found
    sweep_to_box(start_coord_top_2, Bot, edge1);
    Bot.stop_bot();
  }
  Serial.println(restricted_area);

  // update robot.box_pos
  Bot.update_box_pos(0);

  // TODO:
  // show that box is found
  // (bot state) some led action 
  delay(1000);

  // go to center
  Bot.calc_IK(center_pos);
  Bot.write_angles();
  delay(1000);

  // sweep full length
  sweep_to_box(start_coord_bot, Bot, edge2);
  Bot.stop_bot();

  // update robot.box_pos
  Bot.update_box_pos(1);

  // TODO:
  // show that box is found
  // (bot state) some led action 
  delay(1000);

  // go to center
  Bot.calc_IK(center_pos);
  Bot.write_angles();
  delay(1000);

}

void sweep_to_box(float begin_coord[3], robot Bot, edge_detector edge)
{
  float end_x = 9.06;
  float pose[3];
  float read_angle;
  float check_angle;
  int check = 0;
  float upto_angle;

  if (begin_coord[0] > 5)
  {
    if (7.0 < begin_coord[2] && begin_coord[2]<= 7.5) 
    {
      upto_angle = 63;
    }
    else 
    {
      upto_angle = 0;
    }
  }
  else
  {
    upto_angle = 63;
  }
  
  Serial.println("upto");
  Serial.println(upto_angle);
 
  // set claw to closed position
  Bot.write_servo(0, 0, 3);
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
    check = edge.get_measure();

    if (check < 100)
    {
      Bot.stop_bot();
      Bot.write_servo(read_angle - 5, 0, 0);
      delay(1000);
      
      // 0 -> left is restricted  (cubes on right)
      // 1 -> right is restricted (cubes on left) 
      if (read_angle >= 0)
      {
        restricted_area = 0;
      }
      else
      {
        restricted_area = 1;
      }

      return;
    }

    read_angle = Bot.read_angle(0);
  }  
}

void sweep(float begin_coord[3], robot Bot, edge_detector &edge1, edge_detector &edge2)
{
  float end_x;
  float read_angle;
  float prev_base_angle;
  float *f;
  float start_coord[3];
  float end_coord[3];
  float sweep_to;
  int check = 0;
  int calib_edge;

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
    prev_base_angle = Bot.read_joint_angles();

    delay(1000);

    // slowly move base to zero
    Bot.write_servo(sweep_to, 5, 0);
    read_angle = Bot.read_angle(0);

    calib_edge = 0;

    while  (read_angle != sweep_to)
    {
      if (calib_edge == 0)
      {
        delay(500);
        edge1.calibrate();
        edge2.calibrate();
        Bot.stop_bot();
        Bot.write_servo(prev_base_angle, 80, 0);
        delay(2000);
        calib_edge++;
      }
      else
      {
        Bot.write_servo(sweep_to, 3, 0);

        read_angle = Bot.read_angle(0);
        
        if (edge1.is_below())
        {
          // return current pos
          Bot.stop_bot();
          return;
        }
      }
    }

    // update joint angles
    Bot.update_joint_angles();
    
    // go just below start_coord
    f = Bot.line(begin_coord, end_coord,  Bot.d2r(prev_base_angle));
    begin_coord[0] = *f;
    begin_coord[1] = *(f+1);
    begin_coord[2] = *(f+2);
  }
}

void loop()
{}
