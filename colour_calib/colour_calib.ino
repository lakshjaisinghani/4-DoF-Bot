#include "robot.h"
#include "sensors.h"

void setup()
{
  Serial.begin(9600);
  delay(2000);

  //  Objects init 
  robot Bot;
  colour_sensor bot_colour_sensor(A4);

  float col_calib_pos[3]  = {9.05, 0, 4};

  // calibrate colour
  Bot.write_servo(120, 0, 3);
  Bot.calc_IK(col_calib_pos);
  Bot.write_angles();
  delay(2000);

  Serial.println("Would you like to calibrate colour sensor? (y/n)");
  while (!Serial.available())
  {}

  // calibrate colour sensor
  bot_colour_sensor.calibrate();
  delay(2000);
  Serial.println("Colour calibration done..");
  bot_colour_sensor.leds_togle(0);
  delay(2000);

  while (true)
  {
    delay(3000);
    Serial.println(bot_colour_sensor.colourId());
  }
}

void loop()
{}
