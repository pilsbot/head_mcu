/*
 * rosserial ADC Example
 *
 * This is a poor man's Oscilloscope.  It does not have the sampling
 * rate or accuracy of a commerical scope, but it is great to get
 * an analog value into ROS in a pinch.
 */


#include <Arduino.h>
#define USE_USBCON
#include <ros.h>
#include <rosserial_arduino/Adc.h>

ros::NodeHandle nh;

rosserial_arduino::Adc adc_msg;
ros::Publisher p("adc", &adc_msg);

void setup()
{
  pinMode(13, OUTPUT);
  pinMode(2, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);

  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.advertise(p);
}

//We average the analog reading to elminate some of the noise
uint16_t averageAnalog(int pin){
    static constexpr uint8_t average_samples = 16;
    uint16_t v=0;
    for(uint16_t i=0; i<average_samples; i++)
        v+= analogRead(pin)/average_samples;
    return v;
}

void loop()
{
  adc_msg.adc0 = averageAnalog(A0);
  adc_msg.adc1 = digitalRead(2);
  adc_msg.adc2 = digitalRead(3);

  p.publish(&adc_msg);

  nh.spinOnce();
  delay(2);
}
