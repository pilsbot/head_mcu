#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream>

#include "../include/data_frame.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <csignal>
#include <math.h>
#include <string.h>
#include <arpa/inet.h>

#include <chrono>

struct Parameter {
  std::string devicename;
  unsigned baud_rate;
  unsigned publish_rate;
} params_;

int serial_fd_ = -1;

head_mcu::Frame state_;

bool open_serial_port() {
  if ((serial_fd_ = ::open(params_.devicename.c_str(), O_RDWR | O_NOCTTY)) < 0) {
    printf("Cannot open serial device %s to head_mcu!", params_.devicename.c_str());
    return false;
  }
  return true;
}

void set_serial_properties() {
  tcflush(serial_fd_, TCIOFLUSH); // flush previous bytes

  struct termios tio;
  if(tcgetattr(serial_fd_, &tio) < 0)
    perror("tcgetattr");

  tio.c_iflag &= ~(INLCR | IGNCR | ICRNL | IXON | IXOFF);
  tio.c_oflag &= ~(ONLCR | OCRNL);
  tio.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);

  switch (params_.baud_rate)
  {
  case 9600:   cfsetospeed(&tio, B9600);   break;
  case 19200:  cfsetospeed(&tio, B19200);  break;
  case 38400:  cfsetospeed(&tio, B38400);  break;
  case 115200: cfsetospeed(&tio, B115200); break;
  case 230400: cfsetospeed(&tio, B230400); break;
  case 460800: cfsetospeed(&tio, B460800); break;
  case 500000: cfsetospeed(&tio, B500000); break;
  default:
    printf("Baudrate of %d not supported, using 115200!\n", params_.baud_rate);
    cfsetospeed(&tio, B115200);
    break;
  }
  cfsetispeed(&tio, cfgetospeed(&tio));

  if(tcsetattr(serial_fd_, TCSANOW, &tio) < 0) {
    printf("Could not set terminal attributes!\n");
    perror("tcsetattr");
  }
}

void set_update_period_of_target(head_mcu::UpdatePeriodMs period_ms) {
  head_mcu::Command cmd;
  memset(&cmd, 0, sizeof(decltype(cmd)));

  cmd.magic = head_mcu::Command::MAGIC;
  cmd.type = head_mcu::Command::setUpdatePeriod;
  cmd.updatePeriod_ms = ::htons(period_ms);

  if(::write(serial_fd_, &cmd, sizeof(decltype(cmd))) < 0){
    printf("could not set update period of %d ms\n", period_ms);
    return;
  }
  printf("Probably successfully set update period of %d ms\n", period_ms);
}

void set_pin(const bool val)
{
  head_mcu::Command cmd;
  memset(&cmd, 0, sizeof(decltype(cmd)));

  cmd.magic = head_mcu::Command::MAGIC;
  cmd.type = head_mcu::Command::setOutputFrame;
  cmd.frame.digital0_8.as_bit.bit2 = val;

  if(::write(serial_fd_, &cmd, sizeof(decltype(cmd))) < 0){
    printf("could not set pin to %s ms\n", val ? "on" : "off");
    return;
  }
  printf("Probably successfully set pin to %s\n", val ? "on" : "off");
}

void read_serial() {
  set_update_period_of_target(std::ceil(1000./params_.publish_rate));

  unsigned i = 0;
  using namespace std::chrono;
  milliseconds last_update = duration_cast<milliseconds>(system_clock::now().time_since_epoch());
  milliseconds now = last_update;
  while (true)
  {
    head_mcu::Frame frame;
    int ret = ::read(serial_fd_, &frame, sizeof(head_mcu::Frame));
    if(ret == sizeof(head_mcu::Frame)) {
      state_ = frame;
      now = duration_cast<milliseconds>(system_clock::now().time_since_epoch());
      std::cout << "\tAngle0: " << state_.analog0 << std::endl;
      std::cout << "\tAngle1: " << state_.analog1 << std::endl;
      std::cout << "\tEnd  L: " << (state_.digital0_8.as_bit.bit0 ? "I" : "-") << std::endl;
      std::cout << "\tEnd  R: " << (state_.digital0_8.as_bit.bit1 ? "I" : "-") << std::endl;
      std::cout << "\tPinOut: " << (state_.digital0_8.as_bit.bit2 ? "I" : "-") << std::endl;

    } else if(ret > 0)
    {
      printf("serial connection out of sync!");
      return;
    } else {
      printf("serial connection closed or something: %d", ret);
      return;
    }

    auto period = now - last_update;
    if ((i++ & 0b11) == 0b11)
    {
      std::cout << "actual period currently: " << period.count() << " ms" << std::endl;
      set_pin(i & 0b100);
    }
    last_update = now;
  }
}

int main(int argc, char** argv)
{
  // TODO: Make parameter
  params_.devicename = "/dev/ttyACM0";
  params_.baud_rate = 115200;
  params_.publish_rate = 30;

  if(params_.publish_rate <= 0)
  {
    printf("Invalid publish rate %d\n", params_.publish_rate);
    return -1;
  }

  if(!open_serial_port()) {
    throw std::runtime_error("Could not open serial port " + params_.devicename);
  }

  while (true)
  {
    set_serial_properties();
    printf("Opened device %s with baudrate %d\n", params_.devicename.c_str(),params_.baud_rate);

    read_serial();

    ::close(serial_fd_);
  }
  return 0;
}