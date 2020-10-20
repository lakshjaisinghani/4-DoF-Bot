#include "coordTicker.h"
#include <Arduino.h>
Coordinates::Coordinates(){
    xyz[0] = 0;
    xyz[1] = 0;
    xyz[2] = 0;
};

void Coordinates::store_coord(float coord[3]){
    xyz[0] = coord[0];
    xyz[1] = coord[1];
    xyz[2] = coord[2];
};

void Coordinates::copy_coords(Coordinates xyz2){
    xyz[0] = xyz2.xyz[0];
    xyz[1] = xyz2.xyz[1];
    xyz[2] = xyz2.xyz[2];
};

void Coordinates::go_to(void (*f)(float xyz1[3]),void(*g)()){
    (*f)(xyz);
    (*g)();
};

void Coordinates::by_difference(Coordinates point1, Coordinates point2){
    xyz[0] = -point1.xyz[0] + point2.xyz[0];
    xyz[1] = -point1.xyz[1] + point2.xyz[1];
    xyz[2] = -point1.xyz[2] + point2.xyz[2];
};

float Coordinates::magnitude(){
    return pow((pow(xyz[0],2)+pow(xyz[1],2)+pow(xyz[2],2)),0.5);
};

void Coordinates::increment(Coordinates del){
    xyz[0] += del.xyz[0];
    xyz[1] += del.xyz[1];
    xyz[2] += del.xyz[2]; 
};

void Coordinates::scale(float k){

    xyz[0] *=k;
    xyz[1] *=k;
    xyz[2] *=k;
};

coordTicker::coordTicker(float grain, float boundary_array[10][3],int boundary_number){
    step_size = grain;
    eps = step_size/2;
    bound_index = 1;
    num_bounds = boundary_number;
    for(int i = 1; i<num_bounds;i++){
         //bounds[i].Coordinates(boundary_array[i]);
        bounds[i].xyz[0] = boundary_array[i][0];
        bounds[i].xyz[1] = boundary_array[i][1];
        bounds[i].xyz[2] = boundary_array[i][2];
    }
    a.copy_coords(bounds[bound_index-1]);
    b.copy_coords(bounds[bound_index]);
    //Book-keeping coordinates: current and where the ticker has left off
    pickup.copy_coords(a);
    current.copy_coords(a);
    //Describing the remaining path to the next boundary
    delta.by_difference(current,b);
    remaining_distance = delta.magnitude();
    //Movement vector R
    R.by_difference(a,b);
    absR = R.magnitude();
    vec_step.copy_coords(R);
    vec_step.scale(step_size/absR);
};

void coordTicker::store_coord(){
      pickup.copy_coords(current);
};

void coordTicker::get_coord(){
    current.copy_coords(pickup);
};

void coordTicker::go_to(void (*f)(float xyz1[3]),void(*g)()){
        current.go_to(f,g);
};

void coordTicker::tick(){
    remaining_distance = delta.magnitude();
    if (remaining_distance<eps){//If close enough to the end point (b), update endpoints
        bound_index++;
        a.copy_coords((bounds[bound_index-1]));
        b.copy_coords((bounds[bound_index]));
        R.by_difference(a,b);
        absR = R.magnitude();
        vec_step.copy_coords(R);
        vec_step.scale(step_size/absR);
    };
    current.increment(vec_step);
    delta.by_difference(current,b);
    remaining_distance = delta.magnitude();
};

bool coordTicker::path_finished(){
    return (bound_index+1 >= num_bounds);
};
