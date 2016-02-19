#include <string.h>
#include <iostream>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#define MAX_NUMBER_STRING_SIZE 10

uint8_t string_to_number(char* string, int32_t& number)
{
  size_t len = strnlen(string, MAX_NUMBER_STRING_SIZE);

  // if the length is at max, something is wrong with the string
  if (len >= MAX_NUMBER_STRING_SIZE)
  {
    printf("ERR source number string too long\n");
    return false;
  }

  // Assumption: number has a dot and one decimal at the end. If not, something is wrong
  if (string[len-2] != '.')
  {
    printf("ERR No dot found in number\n");
    return false;
  }

  string[len-2] = string[len-1];
  string[len-1] = '\0';

  number = atol(string) * 10L;
  printf("INFO succesfully parsed string to number\n");
  return true;
}

uint8_t number_to_string(int32_t& number, char* string)
{
  int32_t scaled_number = (number/10L)/* + (number % 10L >= (10L/2L) ? 1L : 0L)*/;
  snprintf(string, MAX_NUMBER_STRING_SIZE, "%ld", (long int)scaled_number);

  size_t len = strnlen(string, MAX_NUMBER_STRING_SIZE-1);

  // if the length is at max, something is wrong with the string
  if (len >= MAX_NUMBER_STRING_SIZE-1)
  {
    printf("ERR destination number string too long\n");
    return false;
  }

  string[len] = string[len-1];
  string[len-1] = '.';
  string[len+1] = '\0';

  printf("INFO succesfully parsed number into string\n");
  return true;
}

int main()
{

  char numberstring[MAX_NUMBER_STRING_SIZE] = "AZ-238.0";
  int32_t number = 0;
  string_to_number(&(numberstring[2]), number);
  std::cout << "String to number: " << numberstring << " " << number << std::endl;

  char numberstring2[MAX_NUMBER_STRING_SIZE];
  int32_t number2 = -35000;
  number_to_string(number2, numberstring2);
  std::cout << "Number to string: " << number2 << " " << numberstring2 << std::endl;

}
