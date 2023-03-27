#include "pioprogram.hpp"
#include "n64_host.pio.h"
#include "n64_client.pio.h"

#include <pico/stdlib.h>
#include <pico/multicore.h>

#include <iostream>

constexpr float PI = 3.14159265359;

enum class RequestCommand : uint8_t
{
  Info = 0x00,
  ControllerState = 0x01,
  ReadAccessory = 0x02,
  WriteAccessory = 0x03,
  ReadEEPROM = 0x04,
  WriteEEPROM = 0x05,
  ReadKeypress = 0x13, // For N64DD Randnet keyboard
  Reset = 0xFF,
};

enum class StatusFlag : uint8_t
{
  None = 0x00,
  PakInserted = 0x01,     // A controller pak is inserted
  PakRemoved = 0x02,      // A controller pak was removed since last status
  AddressCrcError = 0x04  // The last read or write command contained an address with a bad CRC
};

enum class ButtonFlag : uint16_t
{
  None = 0x00,
  CRight = (0x01 << 0),
  CLeft = (0x01 << 1),
  CDown = (0x01 << 2),
  CUp = (0x01 << 3),
  R = (0x01 << 4),
  L = (0x01 << 5),
  Reserved = (0x01 << 6),
  Reset = (0x01 << 7),
  PadRight = (0x01 << 8),
  PadLeft = (0x01 << 9),
  PadDown = (0x01 << 10),
  PadUp = (0x01 << 11),
  Start = (0x01 << 12),
  Z = (0x01 << 13),
  B = (0x01 << 14),
  A = (0x01 << 15),
};

struct __attribute__ ((packed)) Controller
{
  // In-memory rep on little endian is like
  //    reserved            status              header2              header1
  // 7 6 5 4 3 2 1 0    7 6 5 4 3 2 1 0    7 6 5 4 3 2 1 0       7 6 5 4 3 2 1 0
  //    yAxis               xAxis                       buttons   
  // 7 6 5 4 3 2 1 0    7 6 5 4 3 2 1 0    15 14 13 12 11 10 9 8 7 6 5 4 3 2 1 0
  // So when copied to the shift register as-is, then right-shifted, it's perfect

  uint8_t reserved = 0x00;
  StatusFlag status = StatusFlag::None;
  uint8_t header2 = 0x00;
  uint8_t header1 = 0x05;
  
  int8_t yAxis = 0;
  int8_t xAxis = 0;
  ButtonFlag buttons = ButtonFlag::None;

  uint32_t& statusMessage()
  {
    return *((uint32_t*)this);
  }

  uint32_t& pollMessage()
  {
    return *(((uint32_t*)this)+1);
  }

  void setFromPollValue(uint32_t msg)
  {
    *(((uint32_t*)this)+1) = msg;
  }

  bool getStatusFlag(StatusFlag flag) const
  {
    return (StatusFlag)((uint8_t)status & (uint8_t)flag) != StatusFlag::None;
  }

  void setStatusFlag(StatusFlag flag, bool value)
  {
    if (value)
    {
      status = (StatusFlag)((uint8_t)status | (uint8_t)flag);
    }
    else
    {
      status = (StatusFlag)((uint8_t) status & ~((uint8_t)flag));
    }
  }

  bool getButton(ButtonFlag button) const
  {
    return (ButtonFlag)((uint16_t)button & (uint16_t)buttons) != ButtonFlag::None;
  }

  void setButton(ButtonFlag button, bool value)
  {
    if (value)
    {
      buttons = (ButtonFlag)((uint16_t)buttons | (uint16_t)button);
    }
    else
    {
      buttons = (ButtonFlag)((uint16_t)buttons & ~((uint16_t)button));
    }
  }
};

std::ostream &operator<<(std::ostream& os, const Controller& c) 
{ 
    os << 
    "{" << std::endl <<
      "  \"PadRight\" : " << c.getButton(ButtonFlag::PadRight) << std::endl <<
      "  \"PadLeft\" : " << c.getButton(ButtonFlag::PadLeft) << std::endl <<
      "  \"PadDown\" : " << c.getButton(ButtonFlag::PadDown) << std::endl <<
      "  \"PadUp\" : " << c.getButton(ButtonFlag::PadUp) << std::endl <<
      "  \"Start\" : " << c.getButton(ButtonFlag::Start) << std::endl <<
      "  \"Z\" : " << c.getButton(ButtonFlag::Z) << std::endl <<
      "  \"B\" : " << c.getButton(ButtonFlag::B) << std::endl <<
      "  \"A\" : " << c.getButton(ButtonFlag::A) << std::endl <<
      "  \"CRight\" : " << c.getButton(ButtonFlag::CRight) << std::endl <<
      "  \"CLeft\" : " << c.getButton(ButtonFlag::CLeft) << std::endl <<
      "  \"CDown\" : " << c.getButton(ButtonFlag::CDown) << std::endl <<
      "  \"CUp\" : " << c.getButton(ButtonFlag::CUp) << std::endl <<
      "  \"R\" : " << c.getButton(ButtonFlag::R) << std::endl <<
      "  \"L\" : " << c.getButton(ButtonFlag::L) << std::endl <<
      "  \"Reserved\" : " << c.getButton(ButtonFlag::Reserved) << std::endl <<
      "  \"Reset\" : " << c.getButton(ButtonFlag::Reset) << std::endl <<
      "  \"StickX\" : " << (int)c.xAxis << std::endl <<
      "  \"StickY\" : " << (int)c.yAxis << std::endl <<
    "}";
    return os;
}

PioProgram n64Host;
PioProgram n64Client;
Controller pad1;

void updatePad()
{
  //float angle = 0;
  uint32_t pollMsg;

  while (1)
  {
    if (!n64Host.write((uint8_t)RequestCommand::ControllerState, 10))
    {
      std::cout << "Controller resetting due to send timeout..." << std::endl;
      n64Host.reset();
      sleep_ms(1000);
      continue;
    }
    if (!n64Host.read(pollMsg, 10))
    {
      std::cout << "Controller resetting due to receive timeout..." << std::endl;
      n64Host.reset();
      sleep_ms(1000);
      continue;
    }

    pad1.setFromPollValue(pollMsg);
    std::cout << "Controller poll value: " << pollMsg << std::endl;
    std::cout << pad1 << std::endl;
    sleep_ms(16);
  }
}

int main()
{
    // Confiigure stdio
    stdio_init_all();

    // Configure IOs for bidirectional Joybus comms
    //gpio_init(0);
    //gpio_pull_up(0);
    //gpio_set_dir(0, GPIO_IN);

    //gpio_init(16);
    gpio_set_pulls(16, false, false);
    // gpio_set_dir(16, GPIO_IN);

    // Load and start the N64 controller PIO programs
    n64Client = std::move(PioProgram(pio0, &n64_client_program, n64_client_program_init, 0));
    n64Host = std::move(PioProgram(pio1, &n64_host_program, n64_host_program_init, 16, 15.625f));

    multicore_reset_core1();
    multicore_launch_core1(updatePad);

    while (1)
    {
      // Wait for the N64 to request something
      RequestCommand value;
      n64Client.read((uint32_t&)value);

      if (value == RequestCommand::Info)
      {
        n64Client.writeLengthAndBytes(pad1.statusMessage(), 3);
      }
      else if (value == RequestCommand::ControllerState)
      {
        n64Client.writeLengthAndBytes(pad1.pollMessage(), 4);
      }
    }
    return 0;
}
