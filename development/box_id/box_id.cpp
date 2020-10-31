float box_r[3] = {0,0,8.9};
float box_R[3] = {0,0,8.9};
float box_g[3] = {0,0,8.9};
float box_G[3] = {0,0,8.9};
float box_b[3] = {0,0,8.9};
float box_B[3] = {0,0,8.9};
float *boxes[6] = {box_r,box_R,box_g,box_G,box_b,box_B};
float box_IR1_coord[3] = {0,0,0};
float box_IR2_coord[3] = {0,0,0};
bool box_found = false;

void box_id() // This function is called once both box_IR1_coord and box_IR2_coord have been located.
{
  float *A = box_IR1_coord;
  float *B = box_IR2_coord;
  float diag = 22.14;
  float distance_from_point[6] = {0.5534, 3.8738, -1.6602, 1.6602, -3.8738, -0.5534};
  float distance_along_R[6] = {3.8738, 4.9806, 10.5146, 11.6214, 17.1554, 18.2622};
  //Box Vector
  float R[2];
  R[0] = -A[0] + B[0];
  R[1] = -A[1] + B[1];
  float m = R[1]/R[0];
  //Perpendicular vector (this is a unit vector) 
  float w[2]; 
  w[0] = 1/(pow(1+pow(1/m,2),0.5));
  w[1] = -1/(m*pow(1+pow(1/m,2),0.5));
  //Assign box coords
  for (int i = 0;i < 6;i++)
  {
    for (int j = 0; j < 2; j++) //Coordinate direction (x or y only)
    {
      //Go to A, go along R for the required distance (scale R by its approximate length)
      //then go along the normal by the scaled distance
      boxes[i][j] = A[j] + (R[j])/diag * distance_along_R[i] + w[j] * distance_from_point[i];
    }
  }
  box_found = true; 
}
void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}
