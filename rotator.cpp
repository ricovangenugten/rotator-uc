#include <Arduino.h>
#include <LiquidCrystal.h>
#include "string.h"

#define MOT_0_POS  13
#define MOT_0_NEG  12
#define MOT_1_POS  11
#define MOT_1_NEG  10
#define ENC_0  9
#define ENC_1  8
#define LCD_D7 7
#define LCD_D6 6
#define LCD_D5 5
#define LCD_D4 4
#define LCD_E  3
#define LCD_RS 2

#define LCD_COLS 16
#define LCD_ROWS 2

#define BUF_SIZE 256

#define AXIS_COUNT 2
#define BAUD_RATE 9600

// 300 [rot/min] * (20*2) [transitions] / 60 [sec/min] = 200 [transitions/second]
// 1000 [ms] / 200 [transitions/second] = 5 [ms/transition]
#define ENC_DEAD_TIME 3; // ms

// 36000 [1/100 deg] / 300 [rot/rot] / (20*2) [transitions] = 3 [1/100 deg/transition]
#define CDEG_PER_COUNT 3

enum EEncState
{
  EEncStateDown    = 0,
  EEncStateUp      = 1,
  EEncStateUnknown = 2,

};

enum EMotorState
{
  EMotorStateStopped     = 0,
  EMotorStateRunningPos  = 1,
  EMotorStateStoppingPos = 2,
  EMotorStateRunningNeg  = 3,
  EMotorStateStoppingNeg = 4,
};

enum EAxis
{
  EAxisAzimuth   = 0,
  EAxisElevation = 1,
};

// globals
char recv_buf[BUF_SIZE];
size_t buf_i = 0;
EMotorState mot_cur_state[AXIS_COUNT] = { EMotorStateStopped };
EMotorState mot_req_state[AXIS_COUNT] = { EMotorStateStopped };
EEncState enc_cur_state[AXIS_COUNT] = { EEncStateUnknown };
unsigned long enc_last_change[AXIS_COUNT] = { 0 };
int32_t enc_angle_act[AXIS_COUNT] = { 0 };
int32_t enc_angle_set[AXIS_COUNT] = { 0 };

LiquidCrystal lcd(LCD_RS, LCD_E, LCD_D4, LCD_D5, LCD_D6, LCD_D7);

void enc_interrupt(uint8_t enc_i, uint8_t pin)
{
  EEncState new_state = static_cast<EEncState>(digitalRead(pin));
  EMotorState cur_state = mot_cur_state[enc_i];
  if (enc_cur_state[enc_i] != new_state)
  {
    if (millis() - enc_last_change[enc_i] > 50)
    {
      // valid encoder transition
      if (cur_state == EMotorStateRunningPos || cur_state == EMotorStateStoppingPos)
        enc_angle_act[enc_i] += CDEG_PER_COUNT;
      if (cur_state == EMotorStateRunningNeg || cur_state == EMotorStateStoppingNeg)
        enc_angle_act[enc_i] -= CDEG_PER_COUNT;
    }
    enc_last_change[enc_i] = millis();
    enc_cur_state[enc_i] = new_state;
  }
  return;
}

void enc_0_interrupt()
{
  enc_interrupt(0, ENC_0);
}

void enc_1_interrupt()
{
  enc_interrupt(1, ENC_1);
}

void motor_set_state(uint8_t mot_i, EMotorState state)
{
  mot_cur_state[mot_i] = state;
  bool is_mot_0 = (mot_i == 0);

  switch(state)
  {
    case EMotorStateRunningPos:
      digitalWrite(is_mot_0 ? MOT_0_POS : MOT_1_POS, 1);
      break;
    case EMotorStateRunningNeg:
      digitalWrite(is_mot_0 ? MOT_0_NEG : MOT_1_NEG, 1);
      break;
    case EMotorStateStoppingPos:
      digitalWrite(is_mot_0 ? MOT_0_POS : MOT_1_POS, 0);
      break;
    case EMotorStateStoppingNeg:
      digitalWrite(is_mot_0 ? MOT_0_NEG : MOT_1_NEG, 0);
      break;
    case EMotorStateStopped:
      default:
      break;
  }
}

void motor_request_state(uint8_t mot_i, EMotorState state)
{
  mot_req_state[mot_i] = state;
  EMotorState cur_state = mot_cur_state[mot_i];
  switch(state)
  {
    case EMotorStateStopped:
      if (cur_state == EMotorStateRunningPos)
        motor_set_state(mot_i, EMotorStateStoppingPos);
      else if (cur_state == EMotorStateRunningNeg)
        motor_set_state(mot_i, EMotorStateStoppingNeg);
      break;
    case EMotorStateRunningPos:
      if (cur_state == EMotorStateRunningNeg)
        motor_set_state(mot_i, EMotorStateStoppingNeg);
      else
        motor_set_state(mot_i, state);
      break;
    case EMotorStateRunningNeg:
      if (cur_state == EMotorStateRunningPos)
        motor_set_state(mot_i, EMotorStateStoppingPos);
      else
        motor_set_state(mot_i, state);
      break;
    // Transitional states, should not be requested
    case EMotorStateStoppingPos:
    case EMotorStateStoppingNeg:
    default:
      break;
  }
}

void axis_update_setpoint(EAxis axis, uint16_t setpoint)
{
  switch (axis)
  {
    case EAxisAzimuth:
      lcd.setCursor(0, 0);
      lcd.print("A ");
      break;
    case EAxisElevation:
      lcd.setCursor(0, 7);
      lcd.print("E ");
      break;
    default:
      break;
  }

lcd.print(setpoint/10);

  enc_angle_set[axis] = setpoint;
  if (enc_angle_set[axis] > enc_angle_act[axis])
  {
    motor_request_state(axis, EMotorStateRunningPos);
    lcd.print("+");
  }
  else if (enc_angle_set[axis] < enc_angle_act[axis])
  {
    motor_request_state(axis, EMotorStateRunningNeg);
    lcd.print("-");
  }
  else
  {
    motor_request_state(axis, EMotorStateStopped);
    lcd.print("=");
  }
}

void print_msg(const char* str)
{
  lcd.setCursor(0, 1);
  lcd.print(str);
}

#define MAX_NUMBER_STRING_SIZE 6

int16_t parse_easycom_number(char* string)
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

  return atoi(string);
}

void handle_command()
{
  if (recv_buf[0] == 'A' && recv_buf[1] == 'Z')
  {
    // New azimuth setpoint
    uint16_t setpoint = parse_easycom_number(&recv_buf[2]);
    axis_update_setpoint(EAxisAzimuth, setpoint);
  }
  else if (recv_buf[0] == 'E' && recv_buf[1] == 'L')
  {
    // New elevation setpoint
    uint16_t setpoint = parse_easycom_number(&recv_buf[2]);
    axis_update_setpoint(EAxisElevation, setpoint);
  }
  else if (recv_buf[0] == 'V' && recv_buf[1] == 'E')
  {
    // Return version
  }
  else if (recv_buf[0] == 'M')
  {
    if(recv_buf[1] == 'L')
    {
      // Move left
    }
    if(recv_buf[1] == 'R')
    {
      // Move right
    }
    if(recv_buf[1] == 'U')
    {
      // Move up
    }
    if(recv_buf[1] == 'D')
    {
      // Move down
    }
  }
  else if (recv_buf[0] == 'S')
  {
    if(recv_buf[1] == 'A')
    {
      // Stop azimuth movement
    }
    if(recv_buf[1] == 'E')
    {
      // Stop elevation movement
    }
  }
}

void setup() {
  pinMode(MOT_0_POS, OUTPUT);
  pinMode(MOT_0_NEG, OUTPUT);
  pinMode(MOT_1_POS, OUTPUT);
  pinMode(MOT_1_NEG, OUTPUT);
  pinMode(ENC_0, INPUT);
  pinMode(ENC_1, INPUT);

  attachInterrupt(0, enc_0_interrupt, CHANGE);
  attachInterrupt(1, enc_1_interrupt, CHANGE);

  Serial.begin(BAUD_RATE);
  lcd.begin(LCD_COLS, LCD_ROWS);
}

void loop() {
  // Handle serial input
  if (Serial.available())
  {
    int read_val = 0;
    while ((read_val = Serial.read()) >= 0)
    {
      recv_buf[buf_i] = static_cast<uint8_t>(read_val);
      if (buf_i == BUF_SIZE-1)
      {
        print_msg("recv buffer full");
        buf_i = 0;
        break;
      }
      if (recv_buf[buf_i] == '\n' || recv_buf[buf_i] == '\r' || recv_buf[buf_i] == ' ')
      {
        // Command seperator detected,
        // terminate with \0 and process command
        print_msg("cmd received");
        recv_buf[buf_i] = '\0';
        buf_i = 0;
        handle_command();
        break;
      }
      buf_i++;
    }
  }

  // Handle motor state machines
  for(uint8_t i = 0; i < AXIS_COUNT; ++i)
  {
    noInterrupts();
    int32_t enc_angle = enc_angle_act[i];
    interrupts();

    switch(mot_cur_state[i])
    {
      case EMotorStateStopped:
        // Do nothing
        break;
      case EMotorStateRunningPos:
        if (enc_angle >= enc_angle_set[i])
          motor_request_state(i, EMotorStateStopped);
        break;
      case EMotorStateStoppingPos:
        // check if transition to is due
        if (true)
        break;
      case EMotorStateRunningNeg:
        if (enc_angle <= enc_angle_set[i])
          motor_request_state(i, EMotorStateStopped);
        break;

      case EMotorStateStoppingNeg:
        // check if transition to stopped is due
        break;
      default:
        break;
    }
  }
}
