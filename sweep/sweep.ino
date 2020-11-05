#include "robot.h"
#include "sensors.h"

void * sweep(float, robot, edge_detector &edge1, edge_detector &edge2);
//void center_on_block(float, robot, edge_detector, edge_detector);

/// global variables
// bot
float center_pos[3] = {12, 0, 8};
float prev_pos[3];
int restricted_area = 0;
char input;

//cubes
int num_cubes = 10;
int picked_cubes = 0;

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

  float * pnt;
  
  // go to center
  Bot.calc_IK(center_pos);
  Bot.write_angles();
  Bot.write_servo(0, 0, 3);
  delay(2000);

  float prev_pos[3] = {15.05, -5, 5.5};
  pnt = sweep(prev_pos, Bot, edge_robot_side, edge_robot_top);
  
  prev_pos[0] = *pnt;
  prev_pos[1] = *(pnt+1);
  prev_pos[2] = 5.5;

  Serial.println(prev_pos[0]);
  Serial.println(prev_pos[1]);
  Serial.println(prev_pos[2]);
}

float * sweep(float start_coord[3], robot Bot, edge_detector &edge1, edge_detector &edge2)
{
  float end_x;
  float read_angle;
  float prev_base_angle;
  float *f;
  int check = 0;
  int calib_edge;

  float begin_coord[3] = {9.05, 0, 5.5};
  float line_end_coord[3]   = {18.05, 0, 5.5};
  static float return_coord[3];
  end_x = line_end_coord[0];

  if (start_coord[0] != begin_coord[0] || start_coord[1] != begin_coord[1])
  {
    Bot.calc_IK(start_coord);
    Bot.write_angles();
    delay(1000);
    Bot.write_servo(0, 80, 0);
    delay(1000);

    f = Bot.read_EE_pos();
    begin_coord[0] = *f;
    begin_coord[1] = *(f+1);
    begin_coord[2] = 5.5; 
  }
  
  // line variables
  float sweep_to = (!restricted_area) ? -63 : 63;

  while (begin_coord[0] <= end_x)
  {
    Bot.print_coord(begin_coord, 2);

    // go to begin coord
    Bot.calc_IK(begin_coord);
    Bot.write_angles();
    
    calib_edge = 0;

    if (calib_edge == 0)
    {
      delay(500);
      edge1.calibrate(1);
      edge2.calibrate(1);
      Bot.stop_bot();

      Bot.calc_IK(begin_coord);
      Bot.write_angles();
      calib_edge++;
    }

    //base angle to return to
    prev_base_angle = Bot.read_joint_angles();
    delay(500);

    // slowly move base to zero
    Bot.write_servo(sweep_to, 3, 0);
    read_angle = Bot.read_angle(0);
 
    while  (read_angle != sweep_to)
    {
      read_angle = Bot.read_angle(0);

      Serial.println(edge1.get_measure());

      if (edge1.is_below())
      {
        // return current pos
        Bot.stop_bot();

        f = Bot.read_EE_pos();
        return_coord[0] = *f;
        return_coord[1] = *(f+1);
        return_coord[2] = 5.5;

        return return_coord;
      }
    }

    // update joint angles
    Bot.update_joint_angles();
    
    // go just below start_coord
    f = Bot.line(begin_coord, line_end_coord,  Bot.d2r(prev_base_angle));
    begin_coord[0] = *f;
    begin_coord[1] = *(f+1);
    begin_coord[2] = *(f+2);
  }
}

void loop()
{}
