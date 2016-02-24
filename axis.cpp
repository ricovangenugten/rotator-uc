#include "Arduino.h"
#include "axis.h"

// 300 [rot/min] * (20*2) [transitions] / 60 [sec/min] = 200 [transitions/second]
// 1000 [ms] / 200 [transitions/second] = 5 [ms/transition]
#define ENC_DEAD_TIME 1 // ms

// 7.5 deg/s / 5 rot/s / 40 trans/rot = 0.0375 deg/trans = 375 * 1e-4 deg/trans
#define INCR_PER_COUNT 375 // 1e-4 deg

// external (1e-1 deg) to internal (1e-4 deg) scaling factor
#define EXT_TO_INT_FACTOR 1e3

#define STOPPING_TIME 500L //ms

CAxis::CAxis(uint8_t enc_pin, uint8_t mot_pos_pin, uint8_t mot_neg_pin) :
  mMotCurState(CAxis::EMotorStateStopped),
  mMotReqState(CAxis::EMotorStateStopped),
  mEncCurState(CAxis::EEncStateUnknown),
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

void CAxis::begin()
{
  pinMode(mEncPin, INPUT);
  pinMode(mMotPosPin, OUTPUT);
  digitalWrite(mMotPosPin, 1);
  pinMode(mMotNegPin, OUTPUT);
  digitalWrite(mMotNegPin, 1);
}

void CAxis::enc_interrupt()
{
  CAxis::EEncState new_state = static_cast<CAxis::EEncState>(digitalRead(mEncPin));
  if (mEncCurState != new_state)
  {
    if ((millis() - mEncLastChange) >= ENC_DEAD_TIME)
    {
      // valid encoder transition
      if (mMotCurState == CAxis::EMotorStateRunningPos || mMotCurState == CAxis::EMotorStateStoppingPos)
        mEncAngleAct += INCR_PER_COUNT;
      if (mMotCurState == CAxis::EMotorStateRunningNeg || mMotCurState == CAxis::EMotorStateStoppingNeg)
        mEncAngleAct -= INCR_PER_COUNT;
    }
    mEncLastChange = millis();
    mEncCurState = new_state;
  }
  return;
}

void CAxis::enc_reset()
{
  noInterrupts();
  mEncLastChange = millis();
  interrupts();
}

void CAxis::move_to_position(int32_t setpoint)
{
  mStopAtSetpoint = true;
  mEncAngleSet = setpoint * EXT_TO_INT_FACTOR;
  if (mEncAngleSet > mEncAngleAct)
    motor_request_state(CAxis::EMotorStateRunningPos);
  else if (mEncAngleSet < mEncAngleAct)
    motor_request_state(CAxis::EMotorStateRunningNeg);
  else
    motor_request_state(CAxis::EMotorStateStopped);
}

void CAxis::move_positive()
{
  mStopAtSetpoint = false;
  motor_request_state(CAxis::EMotorStateRunningPos);
}

void CAxis::move_negative()
{
  mStopAtSetpoint = false;
  motor_request_state(CAxis::EMotorStateRunningNeg);
}

void CAxis::stop_moving()
{
  motor_request_state(CAxis::EMotorStateStopped);
}

int32_t CAxis::get_position_setpoint()
{
  return mEncAngleSet / EXT_TO_INT_FACTOR;
}

int32_t CAxis::get_current_position()
{
  noInterrupts();
  int32_t enc_angle = mEncAngleAct / EXT_TO_INT_FACTOR;
  interrupts();
  return enc_angle;
}

void CAxis::set_current_position(int32_t position)
{
  noInterrupts();
  mEncAngleAct = position * EXT_TO_INT_FACTOR;
  interrupts();
  Serial.write("DBG cur pos set to");
  Serial.print(mEncAngleAct);
  Serial.write("\n");

}

void CAxis::update()
{
  noInterrupts();
  int32_t enc_angle = mEncAngleAct;
  uint32_t enc_last_change = mEncLastChange;
  interrupts();

  bool not_moving = millis() > (enc_last_change + STOPPING_TIME);

  switch(mMotCurState)
  {
    case CAxis::EMotorStateStopped:
      // Do nothing
      break;
    case CAxis::EMotorStateRunningPos:
      if (not_moving || (mStopAtSetpoint && enc_angle >= mEncAngleSet))
        motor_request_state(CAxis::EMotorStateStopped);
      break;
    case CAxis::EMotorStateRunningNeg:
      if (not_moving || (mStopAtSetpoint && enc_angle <= mEncAngleSet))
        motor_request_state(CAxis::EMotorStateStopped);
      break;
    case CAxis::EMotorStateStoppingNeg: // fall-through
    case CAxis::EMotorStateStoppingPos:
      // check if transition is due
      if (millis() > mTransitionDueTime)
        motor_set_state(mMotReqState);
      break;
    default:
      break;
  }
}

void CAxis::motor_set_state(CAxis::EMotorState state)
{
  mMotCurState = state;
  Serial.write("DBG setting state ");
  switch(state)
  {
    case CAxis::EMotorStateRunningPos:
      enc_reset();
      digitalWrite(mMotPosPin, 0);
      Serial.write("EMotorStateRunningPos");
      break;
    case CAxis::EMotorStateRunningNeg:
      enc_reset();
      digitalWrite(mMotNegPin, 0);
      Serial.write("EMotorStateRunningNeg");
      break;
    case CAxis::EMotorStateStoppingPos:
      digitalWrite(mMotPosPin, 1);
      Serial.write("EMotorStateStoppingPos");
      break;
    case CAxis::EMotorStateStoppingNeg:
      digitalWrite(mMotNegPin, 1);
      Serial.write("EMotorStateStoppingNeg");
      break;
    case CAxis::EMotorStateStopped:
      Serial.write("EMotorStateStopped");
      break;
    default:
      break;
  }
  Serial.write("\n");
}

void CAxis::motor_request_state(CAxis::EMotorState req_state)
{
  mMotReqState = req_state;
  switch(mMotCurState)
  {
    case CAxis::EMotorStateStopped:
      // From stopped, only transition to running pos or running neg is allowed
      if (mMotReqState == CAxis::EMotorStateRunningPos || mMotReqState == CAxis::EMotorStateRunningNeg)
      {
        motor_set_state(mMotReqState);
      }
      break;
    case CAxis::EMotorStateRunningPos:
      if (mMotReqState == CAxis::EMotorStateRunningNeg || mMotReqState == CAxis::EMotorStateStopped)
      {
        motor_set_state(CAxis::EMotorStateStoppingPos);
        mTransitionDueTime = millis() + STOPPING_TIME;
      }
      break;
    case CAxis::EMotorStateRunningNeg:
      if (mMotReqState == CAxis::EMotorStateRunningPos || mMotReqState == CAxis::EMotorStateStopped)
      {
        motor_set_state(CAxis::EMotorStateStoppingNeg);
        mTransitionDueTime = millis() + STOPPING_TIME;
      }
      break;
      // Transitional states, no action on request
    case CAxis::EMotorStateStoppingPos:
    case CAxis::EMotorStateStoppingNeg:
    default:
      break;
  }
}
