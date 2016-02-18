#include "Arduino.h"
#include "easycomm_handler.h"
#include "string.h"

#define MAX_NUMBER_STRING_SIZE 6

CAxis* CEasyCommHandler::mAzimuthAxis = NULL;
CAxis* CEasyCommHandler::mElevationAxis = NULL;
char CEasyCommHandler::mReceiveBuffer[BUF_SIZE];
size_t CEasyCommHandler::mBufferIndex;

void CEasyCommHandler::begin(CAxis& azimuth_axis, CAxis& elevation_axis, uint16_t baud_rate)
{
  mAzimuthAxis = &azimuth_axis;
  mElevationAxis = &elevation_axis;
  mBufferIndex = 0;
  Serial.begin(baud_rate);
}

void CEasyCommHandler::update()
{
  // Handle serial input
  if (Serial.available())
  {
    int read_val = 0;
    while ((read_val = Serial.read()) >= 0)
    {
      mReceiveBuffer[mBufferIndex] = static_cast<uint8_t>(read_val);
      if (mBufferIndex == BUF_SIZE-1)
      {
        //print_msg("recv buffer full");
        mBufferIndex = 0;
        break;
      }
      if (mReceiveBuffer[mBufferIndex] == '\n' || mReceiveBuffer[mBufferIndex] == '\r' || mReceiveBuffer[mBufferIndex] == ' ')
      {
        // Command seperator detected,
        // terminate with \0 and process command
        //print_msg("cmd received");
        mReceiveBuffer[mBufferIndex] = '\0';
        mBufferIndex = 0;
        handle_command();
        break;
      }
      mBufferIndex++;
    }
  }
}

int32_t CEasyCommHandler::CEasyCommHandler::parse_number(char* string)
{
  size_t len = strnlen(string, MAX_NUMBER_STRING_SIZE);

  // if the length is at max, something is wrong with the string
  if (len == MAX_NUMBER_STRING_SIZE)
    return 0;

  // Assumption: number has a dot and one decimal at the end. If not, something is wrong
  if (string[len-2] != '.')
    return 0;

  string[len-2] = string[len-1];
  string[len-1] = '\0';

  return atol(string) * 10l;
}

void CEasyCommHandler::CEasyCommHandler::handle_command()
{
  if (mReceiveBuffer[0] == 'A' && mReceiveBuffer[1] == 'Z')
  {
    // New azimuth setpoint
    int32_t setpoint = CEasyCommHandler::parse_number(&mReceiveBuffer[2]);
    mAzimuthAxis->move_to_position(setpoint);
  }
  else if (mReceiveBuffer[0] == 'E' && mReceiveBuffer[1] == 'L')
  {
    // New elevation setpoint
    int32_t setpoint = CEasyCommHandler::parse_number(&mReceiveBuffer[2]);
    mElevationAxis->move_to_position(setpoint);
  }
  else if (mReceiveBuffer[0] == 'V' && mReceiveBuffer[1] == 'E')
  {
    // Return version
    // TODO
  }
  else if (mReceiveBuffer[0] == 'M')
  {
    if(mReceiveBuffer[1] == 'L')
    {
      // Move left
      mAzimuthAxis->move_negative();
    }
    if(mReceiveBuffer[1] == 'R')
    {
      // Move right
      mAzimuthAxis->move_positive();
    }
    if(mReceiveBuffer[1] == 'U')
    {
      // Move up
      mElevationAxis->move_positive();

    }
    if(mReceiveBuffer[1] == 'D')
    {
      // Move down
      mElevationAxis->move_negative();
    }
  }
  else if (mReceiveBuffer[0] == 'S')
  {
    if(mReceiveBuffer[1] == 'A')
    {
      // Stop azimuth movement
      mAzimuthAxis->stop_moving();
    }
    if(mReceiveBuffer[1] == 'E')
    {
      // Stop elevation movement
      mElevationAxis->stop_moving();
    }
  }
}
