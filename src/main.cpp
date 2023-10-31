/*
 * Originally rosserial ADC Example
 *
 */

#include "data_frame.hpp"

#define USE_USBCON
#include <Arduino.h>

using namespace head_mcu;
UpdatePeriodMs desired_cycle_duration_ms = 500;
Frame frame;
Command cmd;

bool maybeConsumeCommand()
{
  if (!Serial)
    return false;

  static_assert (sizeof(Command) <= 64, "command does not fit into Serial buffer which is said to be 64 bytes big in some forum on the internet lol");
  if (Serial.available() < sizeof(Command))
    return false;

  auto ret = Serial.readBytes(
    reinterpret_cast<uint8_t*>(&cmd), sizeof(Command));
  if (ret <= 0)
    return false;

  if (cmd.magic != Command::MAGIC)
    return false;

  switch (cmd.type)
  {
    case Command::setUpdatePeriod:
      desired_cycle_duration_ms = cmd.updatePeriod_ms;
      break;
    case Command::setOutputFrame:
      frame = cmd.frame;
      digitalWrite(4, frame.digital0_8.as_bit.bit2);
      break;
    default:
      // unknown command
      return false;
  }
  return true;
}

void setup()
{
  //analogReadResolution(10);	// pro micro is 10bit ADC
  pinMode(A0, INPUT);
  pinMode(2, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);
  pinMode(4, OUTPUT);

  Serial.begin(115200);

  while(!Serial){};
  while (!maybeConsumeCommand()){};
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
  const uint16_t pre = millis();

  maybeConsumeCommand();

  frame.analog0 = averageAnalog(INPUT);
  frame.analog1 = 0xFFFF;
  frame.digital0_8.as_bit.bit0 = digitalRead(2);
  frame.digital0_8.as_bit.bit1 = digitalRead(3);

  Serial.write(reinterpret_cast<uint8_t*>(&frame), sizeof(head_mcu::Frame));

  const uint16_t time_it_took = millis() - pre;
  // min is for preventing integer underflow back to positive
  delay(min(desired_cycle_duration_ms, desired_cycle_duration_ms - time_it_took));
}
