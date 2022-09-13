#include "Arduino.h"
#include "encoder_axis.h"

// 300 [rot/min] * (20*2) [transitions] / 60 [sec/min] = 200 [transitions/second]
// 1000 [ms] / 200 [transitions/second] = 5 [ms/transition]
#define ENC_DEAD_TIME 2 // ms

// 7.5 deg/s / 5 rot/s / 20*2 trans/rot = 0.0375 deg/trans = 375 * 1e-4 deg/trans
#define INCR_PER_COUNT 375 // 1e-4 deg

#define ANGLE_HYSTERESIS 5000 // 1e-4 deg

// external (1e-1 deg) to internal (1e-4 deg) scaling factor
#define EXT_TO_INT_FACTOR 1e3

#define STOPPING_TIME 500L //ms
#define HOMING_CHECK_TIME 500 // ms
#define HOMING_TIMEOUT 60*1000L // ms
#define HOMING_POSITION 0 // [1/10 deg]

CEncoderAxis::CEncoderAxis(uint8_t enc_pin, uint8_t mot_pos_pin, uint8_t mot_neg_pin) :
  mMotCurState(CEncoderAxis::EMotorStateStopped),
  mMotReqState(CEncoderAxis::EMotorStateStopped),
  mEncLastChange(0),
  mEncAngleAct(),
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
  uint32_t cur_time = millis();

  if (cur_time > (mEncLastChange + ENC_DEAD_TIME))
  {
    // valid encoder transition
    if (mMotCurState == CEncoderAxis::EMotorStateRunningPos || mMotCurState == CEncoderAxis::EMotorStateStoppingPos)
    {
      mEncAngleAct += INCR_PER_COUNT;
    }
    if (mMotCurState == CEncoderAxis::EMotorStateRunningNeg || mMotCurState == CEncoderAxis::EMotorStateStoppingNeg)
    {
      mEncAngleAct -= INCR_PER_COUNT;
    }
    mEncLastChange = cur_time;
  }
  return;
}

void CEncoderAxis::enc_reset()
{
  noInterrupts();
  mEncLastChange = millis() - ENC_DEAD_TIME;
  interrupts();
}

void CEncoderAxis::move_to_position(int32_t setpoint)
{
  mStopAtSetpoint = true;
  mEncAngleSet = setpoint * EXT_TO_INT_FACTOR;
  if (mEncAngleSet > mEncAngleAct + ANGLE_HYSTERESIS)
  {
    motor_request_state(CEncoderAxis::EMotorStateRunningPos);
  }
  else if (mEncAngleSet < mEncAngleAct - ANGLE_HYSTERESIS)
  {
    motor_request_state(CEncoderAxis::EMotorStateRunningNeg);
  }
  else
  {
    motor_request_state(CEncoderAxis::EMotorStateStopped);
  }
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
  mStopAtSetpoint = true;
  motor_request_state(CEncoderAxis::EMotorStateStopped);
}

int32_t CEncoderAxis::get_position_setpoint()
{
  return mEncAngleSet / EXT_TO_INT_FACTOR;
}

int32_t CEncoderAxis::get_current_position()
{
  int32_t enc_angle = mEncAngleAct / EXT_TO_INT_FACTOR;
  return enc_angle;
}

void CEncoderAxis::set_current_position(int32_t position)
{
  mEncAngleAct = position * EXT_TO_INT_FACTOR;
  //Serial.write("DBG cur pos set to");
  //Serial.print(mEncAngleAct);
  //Serial.write("\n");

}

// Update the state machine by doing requests based on input conditions
void CEncoderAxis::update()
{
  int32_t enc_angle = mEncAngleAct;
  uint32_t enc_last_change = mEncLastChange;

  bool not_moving = millis() > (enc_last_change + STOPPING_TIME);

  switch(mMotCurState)
  {
    case CEncoderAxis::EMotorStateStopped:
      // Do nothing
      break;
    case CEncoderAxis::EMotorStateRunningPos:
      // Start transition to stopped if necessary
      if (not_moving || (mStopAtSetpoint && enc_angle >= mEncAngleSet))
        motor_request_state(CEncoderAxis::EMotorStateStopped);
      break;
    case CEncoderAxis::EMotorStateRunningNeg:
      // Start transition to stopped if necessary
      if (not_moving || (mStopAtSetpoint && enc_angle <= mEncAngleSet))
        motor_request_state(CEncoderAxis::EMotorStateStopped);
      break;
    case CEncoderAxis::EMotorStateStoppingNeg: // fall-through
    case CEncoderAxis::EMotorStateStoppingPos:
      // Finish delayed transition if ready
      motor_request_state(mMotReqState);
      break;
  }
}

// Request state transitions
void CEncoderAxis::motor_request_state(CEncoderAxis::EMotorState req_state)
{
  uint32_t cur_time = millis();
  switch(mMotCurState)
  {
    case CEncoderAxis::EMotorStateStopped:
      // From stopped, only transition to running pos or running neg is allowed
      if (req_state == CEncoderAxis::EMotorStateRunningPos ||
          req_state == CEncoderAxis::EMotorStateRunningNeg)
      {
        _motor_set_state(req_state);
      }
      break;
    case CEncoderAxis::EMotorStateRunningPos:
      if (req_state == CEncoderAxis::EMotorStateRunningNeg ||
          req_state == CEncoderAxis::EMotorStateStopped)
      {
        _motor_set_state(CEncoderAxis::EMotorStateStoppingPos);
        mTransitionDueTime = cur_time + STOPPING_TIME;
        mMotReqState = req_state;
      }
      break;
    case CEncoderAxis::EMotorStateRunningNeg:
      if (req_state == CEncoderAxis::EMotorStateRunningPos ||
          req_state == CEncoderAxis::EMotorStateStopped)
      {
        _motor_set_state(CEncoderAxis::EMotorStateStoppingNeg);
        mTransitionDueTime = cur_time + STOPPING_TIME;
        mMotReqState = req_state;
      }
      break;
    case CEncoderAxis::EMotorStateStoppingPos:
    case CEncoderAxis::EMotorStateStoppingNeg:
      // Process delayed state transition
      if (req_state == CEncoderAxis::EMotorStateRunningPos ||
          req_state == CEncoderAxis::EMotorStateRunningNeg ||
          req_state == CEncoderAxis::EMotorStateStopped)
      {
        if (cur_time > mTransitionDueTime)
        {
          _motor_set_state(req_state);
        }
      }
      break;
  }
}

// Perform state transitions
// should only be called by motor_request_state to adhere to state diagram
void CEncoderAxis::_motor_set_state(CEncoderAxis::EMotorState state)
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
      digitalWrite(mMotPosPin, CEncoderAxis::ERelayStateOff);
      digitalWrite(mMotNegPin, CEncoderAxis::ERelayStateOff);
      //Serial.write("EMotorStateStopped");
      break;
    default:
      break;
  }
  //Serial.write("\n");
}

void CEncoderAxis::do_homing_procedure()
{
  // Start moving in negative direction
  move_negative();

  // Wait until stopped (end stop used as homing position)
  auto timeout = millis() + HOMING_TIMEOUT;
  while(not is_stopped() && millis() < timeout)
  {
    update();
    delay(HOMING_CHECK_TIME);
  }

  // When stopped, update current position to homing position
  set_current_position(HOMING_POSITION);
}

bool CEncoderAxis::is_stopped()
{
  return (mMotCurState == CEncoderAxis::EMotorStateStopped);
}
