#include "robot.h"
#include "sensors.h"

float * sweep(float, robot Bot, edge_detector &edge1, edge_detector &edge2);
void center_on_block(float pos[3], robot Bot, edge_detector &edge_side, edge_detector &edge_top);
int find_colour(float, robot, colour_sensor);
int pick_up(robot, limit_switch);
float * grab_cube(float pos[3], robot Bot,  limit_switch lim);

/// global variables
// bot
float center_pos[3] = {12, 0, 8};
float prev_pos[3];
float in_center_pos[3];
int restricted_area = 0;
char input;

//cubes
int num_cubes = 10;
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

  float * pnt;
  
  // go to center
  Bot.calc_IK(center_pos);
  Bot.write_angles();
  Bot.write_servo(0, 0, 3);


  float prev_pos[3] = {9.05, 0, 5.5};
  
  Serial.print("Would you like to start task? (y/n)");
  while (!Serial.available())
  {}
  
  while (picked_cubes < num_cubes)
  {
    pnt = sweep(prev_pos, Bot, edge_robot_side, edge_robot_top);

    Serial.println("Found cube");

    prev_pos[0] = *pnt;
    prev_pos[1] = *(pnt+1);
    prev_pos[2] = *(pnt+2);

    // arduino is somehow passing via refrence
    // hence new pose
    in_center_pos[0] = prev_pos[0];
    in_center_pos[1] = prev_pos[1];
    in_center_pos[2] = prev_pos[2];

    delay(1000);
    
    center_on_block(in_center_pos, Bot, edge_robot_side, edge_robot_top);
    Bot.stop_bot();

    Serial.println("Centered at : ");
    Serial.println(prev_pos[0]);
    Serial.println(prev_pos[1]);
    Serial.println(prev_pos[2]);
    
    // grab cube
    pnt = grab_cube(in_center_pos, Bot, limit);

    // update position
    in_center_pos[0] = *pnt;
    in_center_pos[1] = *(pnt+1);
    in_center_pos[2] = *(pnt+2);

    Serial.println("grabbed at : ");
    Serial.println(in_center_pos[0]);
    Serial.println(in_center_pos[1]);
    Serial.println(in_center_pos[2]);

    // find colour
    int cube_colour = find_colour(in_center_pos, Bot, colour);
    delay(3000);
    
    Bot.calc_IK(center_pos);
    Bot.write_angles();
    delay(2000);

    //drop box
    Bot.write_servo(120, 0, 3);
    picked_cubes++;
    delay(2000);
    Bot.write_servo(0, 0, 3);
  }
}

/// functions
///
float * grab_cube(float pos[3], robot Bot,  limit_switch lim)
{
  int angle;
  float m;
  static float return_coord[3];

  // open claw
  Bot.write_servo(120, 0, 3);
  delay(2000);

  // go down 
  pos[2] = 4;
  Bot.calc_IK(pos);
  Bot.write_angles();
  delay(1000);

  while (!lim.button_state())
  {
    angle = pick_up(Bot, lim);

    if (angle <= 5)
    {
      Serial.println(angle);
      // open claw
      Bot.write_servo(120, 0, 3);
      delay(2000);

      // move back
      m = tan(Bot.d2r(Bot.read_joint_angles()));
      pos[0] -= 0.5;
      pos[1] = m * pos[0];
      Bot.calc_IK(pos);
      Bot.write_angles();
      delay(1000);
    }
  }

  float * f = Bot.read_EE_pos();
  return_coord[0] = *f;
  return_coord[1] = *(f+1);
  return_coord[2] = *(f+2);

  return return_coord;
}

int pick_up(robot Bot, limit_switch lim)
{
  Bot.write_servo(0, 25, 3);

  int read_angle = Bot.read_angle(3);
  
  while (read_angle != 0)
  {
    // argumented function (check)
    if (lim.button_state()) 
    {
      Bot.stop_bot();
      break;
    };

    read_angle = Bot.read_angle(3);
  }

  return read_angle;
}

int find_colour(float pos[3], robot Bot, colour_sensor colour)
{
  // reinstantiate pos
  Bot.calc_IK(pos);
  Bot.write_angles();
  Bot.write_servo(0, 0, 3);
  delay(1000);

  int col_indx;

  // sense coulour
  col_indx = colour.colourId();
  colour.leds_togle(0);

  // go back
  float m = tan(Bot.d2r(Bot.read_joint_angles()));
  Serial.println(m);
  pos[0] -= 1;
  pos[1] = m * pos[0];
  Bot.calc_IK(pos);
  Bot.write_angles();
  delay(1000);

  // go to safe height
  pos[2] = 8.5;
  Bot.calc_IK(pos);
  Bot.write_angles();
  delay(1000);

  return col_indx;
}

void center_on_block(float pos[3], robot Bot, edge_detector &edge_side, edge_detector &edge_top)
{  

  int side = edge_side.is_below();
  int top  = edge_top.is_below();
  float read_angle;

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
  delay(1000);
  while (side && (iter < max_iter))
  {
    if (!restricted_area)
    {
     pos[1] -= step_size * 10;
     Bot.calc_IK(pos);
     Bot.write_angles();
    side = edge_side.is_below();
    iter++;
    }
    else
    {
      pos[1] -= step_size;
     Bot.calc_IK(pos);
     Bot.write_angles();
    side = edge_side.is_below();
    iter++;
    }
    
    Serial.println(" SIDE ");
    Serial.print(side);
    Serial.println(" ");
  }

  if (iter == max_iter)
  {
    Serial.println("Side Detection Timed Out");
  }

  iter = 0;
  m = tan(Bot.d2r(Bot.read_joint_angles()));

  // recalibrate top as we are lower
  delay(1000);
  edge_top.calibrate(1);
  
 while(!top && (iter < max_iter))
 {
   pos[0] -= step_size * 5; 
   pos[1] = m * pos[0];
   pos[2] += step_size * 0.32;
   Bot.calc_IK(pos);
   Bot.write_angles();
   top = edge_top.is_below();
   iter++;

   Serial.println(" TOP ");
   Serial.print(edge_top.get_measure());
   Serial.println(" ");
 }

  // go ahead a little
  delay(500);
  pos[0] += 0.8; 
  pos[1] = m * pos[0];
  Bot.calc_IK(pos);
  Bot.write_angles();
  
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

  delay(2000);

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
    begin_coord[2] = *(f+2); 
  }

  // line variables
  float sweep_to = (!restricted_area) ? -63 : 63;

  while (begin_coord[0] <= end_x)
  {
    if (begin_coord[0] >= 15.05)
    {
      begin_coord[2] = 4.8;
    }
    else if (begin_coord[0] >= 13.05 && begin_coord[0] < 15.05)
    {
      begin_coord[2] = 5;
    }
    else if (begin_coord[0] < 13.05 && begin_coord[0] >= 10.05)
    {
      begin_coord[2] = 5.2;
    }
    else
    {
      begin_coord[2] = 5.4;
    }
      
    Bot.print_coord(begin_coord, 2);

    // go to begin coord
    Bot.calc_IK(begin_coord);
    Bot.write_angles();

    // calibrate edge sensors at 
    // new start position
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

    // slowly move base
    Bot.write_servo(sweep_to, 3, 0);
    read_angle = Bot.read_angle(0);
  
    while  (read_angle != sweep_to)
    {
      read_angle = Bot.read_angle(0);
      
      if (edge1.is_below())
      {
        // return current pos
        Bot.stop_bot();

        f = Bot.read_EE_pos();
        return_coord[0] = *f;
        return_coord[1] = *(f+1);
        return_coord[2] = *(f+2);

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

// get current pos
// float curr_pos[3];
// float *f = Bot.read_EE_pos();
// curr_pos[0] = *f;
// curr_pos[1] = *(f+1);
// curr_pos[2] = 8.5;
// delay(1000);

// move to safe height
// Bot.calc_IK(curr_pos);
// Bot.write_angles();
// delay(2000);
