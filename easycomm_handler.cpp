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
    int32_t read_val = 0;
    while ((read_val = Serial.read()) >= 0)
    {
      mReceiveBuffer[mBufferIndex] = static_cast<uint8_t>(read_val);
      if (mBufferIndex == BUF_SIZE-1)
      {
        Serial.write("ERR buffer is full\n");
        mBufferIndex = 0;
        break;
      }
      if (mReceiveBuffer[mBufferIndex] == '\n' || mReceiveBuffer[mBufferIndex] == '\r' || mReceiveBuffer[mBufferIndex] == ' ')
      {
        // Command seperator detected,
        // terminate with \0 and process command
        //print_msg("cmd received");
        Serial.write("INFO command received\n");
        mReceiveBuffer[mBufferIndex] = '\0';
        handle_command();
        mBufferIndex = 0;
        break;
      }
      mBufferIndex++;
    }
  }
}

void CEasyCommHandler::handle_command()
{
  if (mReceiveBuffer[0] == 'A' && mReceiveBuffer[1] == 'Z')
  {
    CEasyCommHandler::handle_az_el_command(mAzimuthAxis);
  }
  else if (mReceiveBuffer[0] == 'E' && mReceiveBuffer[1] == 'L')
  {
    CEasyCommHandler::handle_az_el_command(mElevationAxis);
  }
  else if (mReceiveBuffer[0] == 'V' && mReceiveBuffer[1] == 'E')
  {
    // Return version
    Serial.write("PA3RVG Az/El rotor 0.0.1\n");
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

void CEasyCommHandler::handle_az_el_command(CAxis* axis)
{
  if (mReceiveBuffer[2] == '\0')
  {
    // Get current position
    char num_string[MAX_NUMBER_STRING_SIZE];
    int32_t cur_pos = mAzimuthAxis->get_current_position();
    if (CEasyCommHandler::number_to_string(cur_pos, num_string))
    {
      Serial.write(mReceiveBuffer[0]);
      Serial.write(mReceiveBuffer[1]);
      Serial.write(num_string);
      Serial.write("\n");
    }
  }
  else
  {
    // Start moving to position
    int32_t number = 0;
    if (CEasyCommHandler::string_to_number(&(mReceiveBuffer[2]), number))
    {
      axis->move_to_position(number);
      Serial.write("OK moving to setpoint\n");
    }
  }
}

bool CEasyCommHandler::string_to_number(char* string, int32_t& number)
{
  size_t len = strnlen(string, MAX_NUMBER_STRING_SIZE);

  // if the length is at max, something is wrong with the string
  if (len >= MAX_NUMBER_STRING_SIZE)
  {
    Serial.write("ERR source number string too long\n");
    return false;
  }

  // Assumption: number has a dot and one decimal at the end. If not, something is wrong
  if (string[len-2] != '.')
  {
    Serial.write("ERR No dot found in number\n");
    return false;
  }

  // Remove dot between last two chars, shrinking string size by one
  string[len-2] = string[len-1];
  string[len-1] = '\0';

  number = atol(string) * 10L;
  Serial.write("INFO succesfully parsed string to number\n");
  return true;
}

bool CEasyCommHandler::number_to_string(int32_t& number, char* string)
{
  int32_t scaled_number = (number/10L) + (number % 10L >= (10L/2L) ? 1L : 0L);
  snprintf(string, MAX_NUMBER_STRING_SIZE, "%ld", scaled_number);

  size_t len = strnlen(string, MAX_NUMBER_STRING_SIZE-1);

  // if the length is at max, something is wrong with the string
  if (len >= MAX_NUMBER_STRING_SIZE-1)
  {
    Serial.write("ERR destination number string too long\n");
    return false;
  }

  // Add dot between last two chars, extending string size by one
  string[len] = string[len-1];
  string[len-1] = '.';
  string[len+1] = '\0';

  Serial.write("INFO succesfully parsed number into string\n");
  return true;
}