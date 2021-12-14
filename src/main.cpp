/*
 * Originally rosserial ADC Example
 *
 */

#include "data_frame.hpp"

#define USE_USBCON
#include <Arduino.h>

using namespace head_mcu;
UpdatePeriodMs desired_cycle_duration_ms;
Frame frame;

void setup()
{
  //analogReadResolution(10);	// pro micro is 10bit ADC
  pinMode(A0, INPUT);
  pinMode(2, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);

  Serial.begin(115200);

  while(!Serial || !Serial.readBytes(
      reinterpret_cast<uint8_t*>(&desired_cycle_duration_ms), sizeof(UpdatePeriodMs))){
  };
}

//We average the analog reading to elminate some of the noise
uint16_t averageAnalog(int pin)
{
  // 10 Bit analog resolution, 16 bit sample size, so we have more space
  static constexpr uint8_t average_samples = 64; //pow(2, 16-10);
  uint16_t v = 0;
  for (uint16_t i = 0; i < average_samples; i++)
    v += analogRead(pin); // / average_samples;
  return v;
}

void loop()
{
  frame.analog0 = averageAnalog(INPUT);
  frame.analog1 = 0xFFFF;
  frame.digital0_8.as_bit.bit0 = digitalRead(2);
  frame.digital0_8.as_bit.bit1 = digitalRead(3);

  Serial.write(reinterpret_cast<uint8_t*>(&frame), sizeof(head_mcu::Frame));

  delay(desired_cycle_duration_ms); //roughly
}
