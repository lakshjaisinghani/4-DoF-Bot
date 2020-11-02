#ifndef sensors_h
#define sensors_h

#include "Arduino.h"

class edge_detector
{
    private:
        int _pin;
        int upper_threshold = 0;
        int lower_threshold = 0;
    
    public:
        edge_detector(int pin);
        float get_measure();
        void calibrate(int type);
        int  is_below();
        int  center_is_below();
};

class limit_switch
{
    private:
        int _pin;

    public:
        limit_switch(int pin);
        int button_state();
};

class colour_sensor
{
    private:
        int _R_pin = 2;
        int _G_pin = 4;
        int _B_pin = 3;
        int _pin;

        int argbCalibrationVals[4][4]; 
        int testCases[4][3] = {{0, 0, 0},{1, 0 ,0}, {0, 1, 0},{0, 0, 1}};
        int led_pins[3] = {_R_pin, _G_pin, _B_pin};
    
    public:
        colour_sensor(int pin);
        int arr_minDex(int arr[]);
        int colourId();
        void getBaseline(int *colourVec);
        void printBaselines(int *colourVec);
        void calibrate();
        void leds_togle(int on_off);
        void led(int ind, int on_off);
};


#endif
