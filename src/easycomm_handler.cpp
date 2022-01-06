#include "Arduino.h"
#include "easycomm_handler.h"
#include "string.h"

#define MAX_NUMBER_STRING_SIZE 6

CEncoderAxis* CEasyCommHandler::mAzimuthAxis = NULL;
CEncoderAxis* CEasyCommHandler::mElevationAxis = NULL;

void CEasyCommHandler::begin(CEncoderAxis& azimuth_axis, CEncoderAxis& elevation_axis)
{
  mAzimuthAxis = &azimuth_axis;
  mElevationAxis = &elevation_axis;
}

void CEasyCommHandler::handle_command(char* command, char* response)
{
  // Empty response by default
  response[0] = '\0';

  Serial.print("Got command ");
  Serial.println(command);

  if (command[0] == 'A' && command[1] == 'Z')
  {
    CEasyCommHandler::handle_az_el_command(mAzimuthAxis, command, response);
  }
  else if (command[0] == 'E' && command[1] == 'L')
  {
    CEasyCommHandler::handle_az_el_command(mElevationAxis, command, response);
  }
  else if (command[0] == 'V' && command[1] == 'E')
  {
    // Return version
    Serial.write("PA3RVG Az/El rotor 0.0.1\n");
  }
  else if (command[0] == 'M')
  {
    if(command[1] == 'L')
    {
      // Move left
      mAzimuthAxis->move_negative();
      Serial.println("Moving left");
    }
    if(command[1] == 'R')
    {
      // Move right
      mAzimuthAxis->move_positive();
      Serial.println("Moving right");
    }
    if(command[1] == 'U')
    {
      // Move up
      mElevationAxis->move_positive();
      Serial.println("Moving up");
    }
    if(command[1] == 'D')
    {
      // Move down
      mElevationAxis->move_negative();
      Serial.println("Moving down");
    }
  }
  else if (command[0] == 'S')
  {
    if(command[1] == 'A')
    {
      // Stop azimuth movement
      mAzimuthAxis->stop_moving();
      Serial.println("Stop moving azimuth");
    }
    if(command[1] == 'E')
    {
      // Stop elevation movement
      mElevationAxis->stop_moving();
      Serial.println("Stop moving elevation");
    }
  }
}

void CEasyCommHandler::handle_az_el_command(CEncoderAxis* axis, char* command, char* response)
{
  size_t len = strnlen(command, COMM_BUF_SIZE);

  if (len == 3)
  {
    // If the command is two bytes long get current position
    char num_string[MAX_NUMBER_STRING_SIZE];
    int32_t cur_pos = axis->get_current_position();
    if (CEasyCommHandler::number_to_string(cur_pos, num_string))
    {
      snprintf(response, RESP_BUF_SIZE, "%c%c%s%c", command[0], command[1], num_string, command[2]);
      Serial.println(response);
    }
  }
  else
  {
    // Command is longer than two bytes, interpret rest as position setpoint
    int32_t number = 0;
    // replace command separator with null character
    // so the buffer can be used as a string
    command[len-1] = '\0';
    if (CEasyCommHandler::string_to_number(&(command[2]), number))
    {
      Serial.print("Moving to position ");
      Serial.println(number);
      axis->move_to_position(number);
    }
  }
}

bool CEasyCommHandler::string_to_number(char* string, int32_t& number)
{
  size_t len = strnlen(string, MAX_NUMBER_STRING_SIZE);

  // if the length is at max, something is wrong with the string
  if (len >= MAX_NUMBER_STRING_SIZE)
  {
    Serial.println("ERR source number string too long");
    return false;
  }

  // Assumption: number has a dot and one decimal at the end. If not, something is wrong
  if (string[len-2] != '.')
  {
    Serial.println("ERR No dot found in number");
    return false;
  }

  // Remove dot between last two chars, shrinking string size by one
  string[len-2] = string[len-1];
  string[len-1] = '\0';

  number = static_cast<int32_t>(atol(string));
  //Serial.write("DBG succesfully parsed string to number: ");
  //Serial.print(number);
  //Serial.write("\n");
  return true;
}

bool CEasyCommHandler::number_to_string(int32_t& number, char* string)
{
  snprintf(string, MAX_NUMBER_STRING_SIZE, "%02ld", static_cast<long int>(number));

  size_t len = strnlen(string, MAX_NUMBER_STRING_SIZE);

  // if the length is at max, something is wrong with the string
  if (len >= MAX_NUMBER_STRING_SIZE)
  {
    Serial.write("ERR destination number string too long\n");
    return false;
  }

  // Add dot between last two chars, extending string size by one
  string[len] = string[len-1];
  string[len-1] = '.';
  string[len+1] = '\0';

  //Serial.write("DBG succesfully parsed number into string: ");
  //Serial.write(string);
  //Serial.write("\n");
  return true;
}
