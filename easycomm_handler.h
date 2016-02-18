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
  static int32_t parse_number(char* string);
  static void handle_command();
};
