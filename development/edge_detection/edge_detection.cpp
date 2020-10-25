/***** Harris' Code *****/
const int IR1 = 0;
const int IR2 = 2; //Not sure which pin the second IR Sensor is placed
int IR_theshold = 500; //Will need to be calibrated: Should be halfway between typical positive and negative readings
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
    side = edge(IR1);
    top = edge(IR2);
    calc_FK(jointAngles);
    if (!side && !top) //If neither IR sensor sees an edge, the scanning routine should be resumed. 
    {
        return;
    }
    if (!dir) //If right-left, the arm has to move all the way to the other side of the block 
    {
        while (side) 
        {
            /*** Need to check Index Correctness ***/
            endEffectorPos[1] -= step_size;
            calc_IK(endEffectorPos);
            write_angles();
            side = edge(IR2);
        }
        endEffectorPos[1] += step_size;
        calc_IK(endEffectorPos);
        write_angles();
    }
    else if(dir) 
    {
        while (!side)
        {
            endEffectorPos[1] += step_size;
            calc_IK(endEffectorPos);
            write_angles();
            side = edge(IR2);
        }
    }
    while(!top)//If the top doesn't see an edge, move down (Regardless of starting direction) 
    {
        endEffectorPos[0] -= step_size; //Assuming it is the X value but that might need to change
                                        //This increment might have to be tuned but I want to keep it small for sensitivity
        calc_IK(endEffectorPos);
        write_angles();
        top = edge(IR1);
    }
    pick_up();
}
