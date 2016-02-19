#include "axis.h"

#define BUF_SIZE 64

class CEasyCommHandler
{
public:
  static void begin(CAxis& azimuth_axis, CAxis& elevation_axis, uint16_t baud_rate);
  static void update();

private:
  static CAxis* mAzimuthAxis;
  static CAxis* mElevationAxis;
  static char mReceiveBuffer[BUF_SIZE];
  static size_t mBufferIndex;

  CEasyCommHandler() {}
  static void handle_command();
  static void handle_az_el_command(CAxis* axis);
  static bool string_to_number(char* string, int32_t& number);
  static bool number_to_string(int32_t& number, char* string);
};
