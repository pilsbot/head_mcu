/*
 * rosserial ADC Example
 *
 * This is a poor man's Oscilloscope.  It does not have the sampling
 * rate or accuracy of a commerical scope, but it is great to get
 * an analog value into ROS in a pinch.
 */


#include <Arduino.h>
#include <ros.h>
#include <rosserial_arduino/Adc.h>

ros::NodeHandle nh;

rosserial_arduino::Adc adc_msg;
ros::Publisher p("head_mcu", &adc_msg);

void setup()
{
  pinMode(13, OUTPUT);
  memset(&adc_msg, 0, sizeof(rosserial_arduino::Adc));

  nh.initNode();
  nh.advertise(p);
}

//We average the analog reading to elminate some of the noise
int averageAnalog(int pin){
  int v=0;
  for(int i=0; i<4; i++) v+= analogRead(pin);
  return v/4;
}

void loop()
{
  adc_msg.adc0 = averageAnalog(0);

  p.publish(&adc_msg);

  nh.spinOnce();
}
