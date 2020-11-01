#include "Arduino.h"
#include "sensors.h"

edge_detector::edge_detector(int pin)
{
    _pin = pin;
}

float edge_detector::get_measure()
{
    float val = 0;

    for(int i = 0; i < 5; i++)
    {
        val += analogRead(_pin);
    }

    return val/5.0;
}

int edge_detector::is_below()
{
    float val = get_measure();

    return (val < threshold) ? 1 : 0;
}

void edge_detector::calibrate()
{
    float val = 0;
    
    delay(5000);
    for (int i = 0; i < 5; i++)
    {
        val += get_measure();
    }

    threshold = (int) (val / 5.0) + 5;
}

colour_sensor::colour_sensor(int pin)
{
    pinMode(_R_pin, OUTPUT);
    pinMode(_G_pin, OUTPUT);
    pinMode(_B_pin, OUTPUT);
    _pin = pin;
}

int colour_sensor::arr_minDex(int arr[])
{
    // Find the minimum squared error 
  int minDex = 0;
  int minVal = arr[0];
  for (int c = 1; c<4;c++)
  {
    if (arr[c] < minVal)
    {
      minDex = c;
      minVal = arr[c];
    }
  }

  return minDex;
}

int colour_sensor::colourId()
{
    int abs_error_vec[4] = {0,0,0,0};
    int sample;
    int numTestCases = 4;
    static char * col;
    
    for(int i = 1; i < numTestCases; i++)
    {
        sample = 0; 
        digitalWrite(_R_pin,testCases[i][0]);
        digitalWrite(_G_pin,testCases[i][1]);
        digitalWrite(_B_pin,testCases[i][2]);

        for (int j = 0; j < 15; j++)
        {
        delay(100); 
        sample += analogRead(_pin);
        }

        for (int c = 0; c < 4; c++)
        {
        abs_error_vec[c] = abs(sample/15 - argbCalibrationVals[c][i]); 
        }
    }

    int minDex = arr_minDex(abs_error_vec);
    
    return minDex;
}

void colour_sensor::getBaseline(int *colourVec)
{
    int sample;
    int numTestCases = 4;

    for (int i = 0; i<numTestCases;i++)
    { 
        sample = 0;
        digitalWrite(_R_pin,testCases[i][0]);
        digitalWrite(_G_pin,testCases[i][1]);
        digitalWrite(_B_pin,testCases[i][2]);

        for (int j = 0; j <15;j++)
        {
        delay(100); 
        sample += analogRead(_pin);
        }
        colourVec[i] = sample/15; 
    }
}

void colour_sensor::printBaselines(int *colourVec)
{
    for (int col = 0; col < 4; col++)
    {
        Serial.println(colourVec[col]);
    }
}

void colour_sensor::calibrate()
{
    Serial.println("Please remove all blocks from colour sensor, ambient test starts in 5 seconds");
    delay(5000);
    Serial.println("Ambient Test Starting");
    getBaseline(argbCalibrationVals[0]);
    Serial.println("Please place Red block under colour sensor, test starts in 5 seconds");
    delay(5000);
    Serial.println("Red Test Starting");
    getBaseline(argbCalibrationVals[1]);
    Serial.println("Please place Green block under colour sensor, test starts in 5 seconds");
    delay(5000);
    Serial.println("Green Test Starting");
    getBaseline(argbCalibrationVals[2]);
    Serial.println("Please place Blue block under colour sensor, test starts in 5 seconds");
    delay(5000);
    Serial.println("Blue Test Starting");
    getBaseline(argbCalibrationVals[3]);

    // calibration output
    // Serial.println("No Block Responses to Null, R, G ,B:");
    // printBaselines(argbCalibrationVals[0]);
    // Serial.println("Red Block Responses to Null, R, G ,B:");
    // printBaselines(argbCalibrationVals[1]);
    // Serial.println("Green Block Responses to Null, R, G, B");
    // printBaselines(argbCalibrationVals[2]);
    // Serial.println("Blue Block Responses to Null, R, G, B");
    // printBaselines(argbCalibrationVals[3]);
}

void colour_sensor::led(int ind, int on_off)
{
    // {r, g, b}
    digitalWrite(led_pins[ind], on_off);
}

void colour_sensor::leds_togle(int on_off)
{
    led(0, on_off);
    led(1, on_off);
    led(2, on_off);
}
