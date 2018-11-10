#pragma once

class CAxis
{
public:
  virtual void move_to_position(int32_t setpoint) = 0;
  virtual void move_positive() = 0;
  virtual void move_negative() = 0;
  virtual void stop_moving() = 0;
  virtual int32_t get_current_position() = 0;
};
