#include "Arduino.h"
#ifndef COORD_TICKER_H
#define COORD_TICKER_H
class Coordinates{
    public:
    float xyz[3];
    float x,y,z;
    Coordinates();
    void store_coord(float coord[3]);
    void copy_coords(Coordinates xyz2);
    void go_to(void (*f)(float xyz1[3]),void(*g)());
    void by_difference(Coordinates point1, Coordinates point2);
    float magnitude();
    void increment(Coordinates del);
    void scale(float k);
};

class coordTicker{
    Coordinates a,b,current,pickup,R,delta,vec_step;
    Coordinates bounds[10];
    float absR,step_size,eps,remaining_distance;
    int bound_index,num_bounds;
    public: 
    coordTicker(float grain, float boundary_array[10][3],int boundary_number);
    void store_coord();
    void get_coord();
    void go_to(void (*f)(float xyz1[3]),void(*g)());
    void tick();
    bool path_finished();
};
#endif
