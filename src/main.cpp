/*
 * Originally rosserial ADC Example
 *
 */

#define USE_USBCON
#include <ros.h>
#include <rosserial_arduino/Adc.h>

ros::NodeHandle nh;

rosserial_arduino::Adc adc_msg;
ros::Publisher p("adc", &adc_msg);

void setup()
{
  //analogReadResolution(10);	// pro micro is 10bit ADC
  pinMode(13, OUTPUT);
  pinMode(A0, INPUT);
  pinMode(2, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);

  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.advertise(p);
}

//We average the analog reading to elminate some of the noise
uint16_t averageAnalog(int pin)
{
  // 10 Bit analog resolution, 16 bit sample size, so we have more space
  static constexpr uint8_t average_samples = pow(2, 16-10);
  uint16_t v = 0;
  for (uint16_t i = 0; i < average_samples; i++)
    v += analogRead(pin); // / average_samples;
  return v;
}

void loop()
{
  static bool led = false;
  digitalWrite(13, led);
  led = !led;
  
  adc_msg.adc0 = averageAnalog(A0);
  adc_msg.adc1 = digitalRead(2);
  adc_msg.adc2 = digitalRead(3);

  p.publish(&adc_msg);

  nh.spinOnce();
  delay(5);
}
