#include "Arduino.h"
#include "encoder_axis.h"

// 300 [rot/min] * (20*2) [transitions] / 60 [sec/min] = 200 [transitions/second]
// 1000 [ms] / 200 [transitions/second] = 5 [ms/transition]
#define ENC_DEAD_TIME 1 // ms

// 7.5 deg/s / 5 rot/s / 40 trans/rot = 0.0375 deg/trans = 375 * 1e-4 deg/trans
#define INCR_PER_COUNT 375 // 1e-4 deg

// external (1e-1 deg) to internal (1e-4 deg) scaling factor
#define EXT_TO_INT_FACTOR 1e3

#define STOPPING_TIME 500L //ms
#define HOMING_CHECK_TIME 500 // ms
#define HOMING_OFFSET -100 // [1/10 deg]

CEncoderAxis::CEncoderAxis(uint8_t enc_pin, uint8_t mot_pos_pin, uint8_t mot_neg_pin) :
  mMotCurState(CEncoderAxis::EMotorStateStopped),
  mMotReqState(CEncoderAxis::EMotorStateStopped),
  mEncCurState(CEncoderAxis::EEncStateUnknown),
  mEncLastChange(0),
  mEncAngleAct(0),
  mEncAngleSet(0),
  mTransitionDueTime(0),
  mEncPin(enc_pin),
  mMotPosPin(mot_pos_pin),
  mMotNegPin(mot_neg_pin),
  mStopAtSetpoint(true)
{
}

void CEncoderAxis::begin()
{
  pinMode(mEncPin, INPUT);
  digitalWrite(mMotPosPin, CEncoderAxis::ERelayStateOff);
  pinMode(mMotPosPin, OUTPUT);
  digitalWrite(mMotNegPin, CEncoderAxis::ERelayStateOff);
  pinMode(mMotNegPin, OUTPUT);
}

void CEncoderAxis::enc_interrupt()
{
  CEncoderAxis::EEncState new_state = static_cast<CEncoderAxis::EEncState>(digitalRead(mEncPin));
  if (mEncCurState != new_state)
  {
    if ((millis() - mEncLastChange) >= ENC_DEAD_TIME)
    {
      // valid encoder transition
      if (mMotCurState == CEncoderAxis::EMotorStateRunningPos || mMotCurState == CEncoderAxis::EMotorStateStoppingPos)
        mEncAngleAct += INCR_PER_COUNT;
      if (mMotCurState == CEncoderAxis::EMotorStateRunningNeg || mMotCurState == CEncoderAxis::EMotorStateStoppingNeg)
        mEncAngleAct -= INCR_PER_COUNT;
    }
    mEncLastChange = millis();
    mEncCurState = new_state;
  }
  return;
}

void CEncoderAxis::enc_reset()
{
  noInterrupts();
  mEncLastChange = millis();
  interrupts();
}

void CEncoderAxis::move_to_position(int32_t setpoint)
{
  mStopAtSetpoint = true;
  mEncAngleSet = setpoint * EXT_TO_INT_FACTOR;
  if (mEncAngleSet > mEncAngleAct)
    motor_request_state(CEncoderAxis::EMotorStateRunningPos);
  else if (mEncAngleSet < mEncAngleAct)
    motor_request_state(CEncoderAxis::EMotorStateRunningNeg);
  else
    motor_request_state(CEncoderAxis::EMotorStateStopped);
}

void CEncoderAxis::move_positive()
{
  mStopAtSetpoint = false;
  motor_request_state(CEncoderAxis::EMotorStateRunningPos);
}

void CEncoderAxis::move_negative()
{
  mStopAtSetpoint = false;
  motor_request_state(CEncoderAxis::EMotorStateRunningNeg);
}

void CEncoderAxis::stop_moving()
{
  motor_request_state(CEncoderAxis::EMotorStateStopped);
}

int32_t CEncoderAxis::get_position_setpoint()
{
  return mEncAngleSet / EXT_TO_INT_FACTOR;
}

int32_t CEncoderAxis::get_current_position()
{
  noInterrupts();
  int32_t enc_angle = mEncAngleAct / EXT_TO_INT_FACTOR;
  interrupts();
  return enc_angle;
}

void CEncoderAxis::set_current_position(int32_t position)
{
  noInterrupts();
  mEncAngleAct = position * EXT_TO_INT_FACTOR;
  interrupts();
  //Serial.write("DBG cur pos set to");
  //Serial.print(mEncAngleAct);
  //Serial.write("\n");

}

void CEncoderAxis::update()
{
  noInterrupts();
  int32_t enc_angle = mEncAngleAct;
  uint32_t enc_last_change = mEncLastChange;
  interrupts();

  bool not_moving = millis() > (enc_last_change + STOPPING_TIME);

  switch(mMotCurState)
  {
    case CEncoderAxis::EMotorStateStopped:
      // Do nothing
      break;
    case CEncoderAxis::EMotorStateRunningPos:
      if (not_moving || (mStopAtSetpoint && enc_angle >= mEncAngleSet))
        motor_request_state(CEncoderAxis::EMotorStateStopped);
      break;
    case CEncoderAxis::EMotorStateRunningNeg:
      if (not_moving || (mStopAtSetpoint && enc_angle <= mEncAngleSet))
        motor_request_state(CEncoderAxis::EMotorStateStopped);
      break;
    case CEncoderAxis::EMotorStateStoppingNeg: // fall-through
    case CEncoderAxis::EMotorStateStoppingPos:
      // check if transition is due
      if (millis() > mTransitionDueTime)
        motor_set_state(mMotReqState);
      break;
    default:
      break;
  }
}

void CEncoderAxis::motor_set_state(CEncoderAxis::EMotorState state)
{
  mMotCurState = state;
  //Serial.write("DBG setting state ");
  switch(state)
  {
    case CEncoderAxis::EMotorStateRunningPos:
      enc_reset();
      digitalWrite(mMotPosPin, CEncoderAxis::ERelayStateOn);
      //Serial.write("EMotorStateRunningPos");
      break;
    case CEncoderAxis::EMotorStateRunningNeg:
      enc_reset();
      digitalWrite(mMotNegPin, CEncoderAxis::ERelayStateOn);
      //Serial.write("EMotorStateRunningNeg");
      break;
    case CEncoderAxis::EMotorStateStoppingPos:
      digitalWrite(mMotPosPin, CEncoderAxis::ERelayStateOff);
      //Serial.write("EMotorStateStoppingPos");
      break;
    case CEncoderAxis::EMotorStateStoppingNeg:
      digitalWrite(mMotNegPin, CEncoderAxis::ERelayStateOff);
      //Serial.write("EMotorStateStoppingNeg");
      break;
    case CEncoderAxis::EMotorStateStopped:
      //Serial.write("EMotorStateStopped");
      break;
    default:
      break;
  }
  //Serial.write("\n");
}

void CEncoderAxis::motor_request_state(CEncoderAxis::EMotorState req_state)
{
  mMotReqState = req_state;
  switch(mMotCurState)
  {
    case CEncoderAxis::EMotorStateStopped:
      // From stopped, only transition to running pos or running neg is allowed
      if (mMotReqState == CEncoderAxis::EMotorStateRunningPos || mMotReqState == CEncoderAxis::EMotorStateRunningNeg)
      {
        motor_set_state(mMotReqState);
      }
      break;
    case CEncoderAxis::EMotorStateRunningPos:
      if (mMotReqState == CEncoderAxis::EMotorStateRunningNeg || mMotReqState == CEncoderAxis::EMotorStateStopped)
      {
        motor_set_state(CEncoderAxis::EMotorStateStoppingPos);
        mTransitionDueTime = millis() + STOPPING_TIME;
      }
      break;
    case CEncoderAxis::EMotorStateRunningNeg:
      if (mMotReqState == CEncoderAxis::EMotorStateRunningPos || mMotReqState == CEncoderAxis::EMotorStateStopped)
      {
        motor_set_state(CEncoderAxis::EMotorStateStoppingNeg);
        mTransitionDueTime = millis() + STOPPING_TIME;
      }
      break;
      // Transitional states, no action on request
    case CEncoderAxis::EMotorStateStoppingPos:
    case CEncoderAxis::EMotorStateStoppingNeg:
    default:
      break;
  }
}

void CEncoderAxis::do_homing_procedure()
{
  int32_t prev_pos = -1;
  int32_t cur_pos = 0;
  move_negative();
  while(prev_pos != cur_pos)
  {
    prev_pos = cur_pos;
    delay(HOMING_CHECK_TIME);
    cur_pos = get_current_position();
  }
  stop_moving();
  set_current_position(HOMING_OFFSET);
  move_to_position(0);
}
