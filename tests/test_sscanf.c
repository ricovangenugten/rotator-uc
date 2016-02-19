#include <stdio.h>

void main()
{
  float az = 0;
  float el = 0;

  sscanf("AZ10.0\nEL7.0\n", "AZ%f EL%f", &az, &el);

  printf("AZ: %0.1f EL: %0.1f", az, el);
}
