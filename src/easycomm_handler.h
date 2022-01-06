#pragma once

#include "encoder_axis.h"

#define COMM_BUF_SIZE 128
#define RESP_BUF_SIZE 128


class CEasyCommHandler
{
public:

template<class T>
static void handle_commands(T client)
{
  static char   command [COMM_BUF_SIZE];
  static char   response[RESP_BUF_SIZE];
  static size_t it = 0;

  bool complete_command_received = false;

  while (client.available())
  {
    if (it == COMM_BUF_SIZE-2)
    {
      Serial.println("ERR buffer is full");
      it = 0;
      break;
    }

    char recv_char = client.read();
    command[it++] = recv_char;

    if (recv_char == '\n' || recv_char == '\r' || recv_char == ' ')
    {
      if (it > 1)
      {
        command[it] = '\0';
        complete_command_received = true;
      }
      it = 0;
      break;
    }
  }

  if (complete_command_received)
  {
      CEasyCommHandler::handle_command(command, response);
      client.write(response);
  }
}

  static void begin(CEncoderAxis& azimuth_axis, CEncoderAxis& elevation_axis);

private:
  static CEncoderAxis* mAzimuthAxis;
  static CEncoderAxis* mElevationAxis;

  CEasyCommHandler() {}
  static void handle_command(char* command, char* response);
  static void handle_az_el_command(CEncoderAxis* axis, char* command, char* response);
  static bool string_to_number(char* string, int32_t& number);
  static bool number_to_string(int32_t& number, char* string);
};
