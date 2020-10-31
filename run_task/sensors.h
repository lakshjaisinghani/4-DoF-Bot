#ifndef sensors_h
#define sensors_h

#include "Arduino.h"

class edge_detector
{
    private:
        int _pin;
        int threshold = 900;
    
    public:
        edge_detector(int pin);
        float get_measure();
        int  is_below();
};

class colour_sensor
{
    private:
        int _R_pin = 2;
        int _G_pin = 3;
        int _B_pin = 4;
        int _pin;

        int argbCalibrationVals[4][4]; 
        int testCases[4][3] = {{0, 0, 0},{1, 0 ,0}, {0, 1, 0},{0, 0, 1}};
        const char colourIdStrings[4][15] = {"Ambient", "Red", "Green", "Blue"};
    
    public:
        colour_sensor(int pin);
        int arr_minDex(int arr[]);
        void colourId();
        void getBaseline(int *colourVec);
        void printBaselines(int *colourVec);
        void calibrate();
};


#endif
