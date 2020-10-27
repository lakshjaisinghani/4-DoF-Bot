/***** Harris' Code *****/
const int IR1 = 0;
const int IR2 = 2; //Not sure which pin the second IR Sensor is placed
int IR_theshold = 500; //Will need to be calibrated: Should be halfway between typical positive and negative readings
const int LS1 = 1; //LimSwitch1 - will need the following code in begin(): //pinMode(LS1, INPUT);  
bool edge(int IR)
{
    int temp = analogRead(IR);
    return temp < IR_theshold;
}

void pick_up(){
    Serial.println("Pick up Sequence Starting");
}

void centre_on_block(int dir)//Assuming dir = 1 means left to right
{
    //Get status of side and top edge detector (1 and 2)
    bool side, top; 
    float step_size = 0.001;
    int max_iter = 100; 
    int iter = 0;
    side = edge(IR1);
    top = edge(IR2);
    calc_FK(jointAngles);
    if (!side && !top) //If neither IR sensor sees an edge, the scanning routine should be resumed. 
    {
        return;
    }
    if (!dir) //If right-left, the arm has to move all the way to the other side of the block 
    {
        while (side && (iter < max_iter)) 
        {
            /*** Need to check Index Correctness ***/
            endEffectorPos[1] -= step_size;
            calc_IK(endEffectorPos);
            write_angles();
            side = edge(IR2);
            iter ++;
        }
        endEffectorPos[1] += step_size;
        calc_IK(endEffectorPos);
        write_angles();
    }
    else if(dir) 
    {
        while (!side && (iter < max_iter))
        {
            endEffectorPos[1] += step_size;
            calc_IK(endEffectorPos);
            write_angles();
            side = edge(IR2);
            iter++;
        }
    }
    if (iter == max_iter)
    {
      Serial.println("Side Detection Timed Out");
    }
    iter = 0;

    while(!top && (iter < max_iter))//If the top doesn't see an edge, move down (Regardless of starting direction) 
    {
        endEffectorPos[0] -= step_size; //Assuming it is the X value but that might need to change
                                        //This increment might have to be tuned but I want to keep it small for sensitivity
        calc_IK(endEffectorPos);
        write_angles();
        top = edge(IR1);
        iter++;
    }
     if iter == max_iter)
    {
      Serial.println("Side Detection Timed Out");
      return
    } else
    {
      pick_up();
    }
}

void pick_up()
{
  //Timeout Variables
  int max_iter = 100; 
  int iter = 0;
  //Spatial Variables
  int theta = 90; //Initial Angle for Grip is 90
  int d_theta = 1;
  float pick_up_height = 1.75;
  float final_height = 6;
  int read_switch;// = digitalRead(LS1); 
  //Arguments for servo writing
  int servo_speed = 1;
  int claw_idx = 2; 

  //Open Claw
  write_servo(theta,servo_speed,claw_idx);

  //Descend:
  endEffectorPos[2] = pick_up_height;
  write_angles();

  //Close Jaw Slowly:
  read_switch = digitalRead(LS1);
  while ((!read_switch) && (iter < max_iter)) 
  {
    theta -= d_theta;
    write_servo(theta,servo_speed,claw_idx);
    read_switch = digitalRead(LS1);
    iter++;
  }
  if (iter == max_iter) 
  {
    Serial.println("Pickup Timed Out");
  }
  //Ascend:
  endEffectorPos[2] = final_height;
  write_angles();
}

int block_size() //0 - No Block Detected by LimSwitch (LS1), 1 - Little Block, 2 - Big Block
{
  float read_angle = jointAngles[2];
  int read_switch = digitalRead(LS1); 
  float offset_angle = 0; //Incase the Grip servo takes the closed position to be non-zero
  
  float width = 0; 
  float width_threshold = 2.12;
  float claw_length = 6;
  
  if (read_switch){
    width = 2*claw_length*sin(d2r(read_angle + offset_angle));
    if (width <= width_threshold) return 1;
    else return 2; 
  } else
  {
    return 0
  }
}

/***** Harris' Code *****/
