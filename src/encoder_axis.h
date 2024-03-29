#pragma once

class CEncoderAxis
{
public:
  CEncoderAxis(uint8_t enc_pin, uint8_t mot_pos_pin, uint8_t mot_neg_pin);
  void begin();
  void INTERRUPT_FUNC enc_interrupt();
  void enc_reset();
  void move_to_position(int32_t setpoint);
  void move_positive();
  void move_negative();
  void stop_moving();
  int32_t get_position_setpoint();
  int32_t get_current_position();
  void set_current_position(int32_t position);
  void update();
  void do_homing_procedure();
  bool is_stopped();

private:
  enum EEncState
  {
    EEncStateDown    = 0,
    EEncStateUp      = 1,
    EEncStateUnknown = 2,
  };

  enum ERelayState
  {
    ERelayStateOff    = 0,
    ERelayStateOn     = 1,
  };

  enum EMotorState
  {
    EMotorStateStopped     = 0,
    EMotorStateRunningPos  = 1,
    EMotorStateStoppingPos = 2,
    EMotorStateRunningNeg  = 3,
    EMotorStateStoppingNeg = 4,
  };

  void motor_request_state(EMotorState req_state);
  void _motor_set_state(EMotorState state);


  EMotorState mMotCurState;
  EMotorState mMotReqState;
  volatile uint32_t mEncLastChange;
  volatile int32_t mEncAngleAct;
  int32_t mEncAngleSet;
  uint32_t mTransitionDueTime;
  uint8_t mEncPin;
  uint8_t mMotPosPin;
  uint8_t mMotNegPin;
  bool mStopAtSetpoint;
};
