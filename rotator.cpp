#include <Arduino.h>
#include <LiquidCrystal.h>
#include "easycomm_handler.h"
//#include "axis.h"

#define LCD_RS 13
#define LCD_E  12
#define LCD_D4 11
#define LCD_D5 10
#define LCD_D6 9
#define LCD_D7 8
#define MOT_0_POS  7
#define MOT_0_NEG  6
#define MOT_1_POS  5
#define MOT_1_NEG  4
#define ENC_0  3
#define ENC_1  2

#define ENC_0_INTERRUPT 1
#define ENC_1_INTERRUPT 0

#define LCD_COLS 16
#define LCD_ROWS 2

#define BAUD_RATE 9600

#define DISPLAY_UPDATE_PERIOD 500 // ms
#define HOMING_CHECK_TIME 500 // ms
#define AZ_HOMING_OFFSET -100 // [1/10 deg]
#define EL_HOMING_OFFSET -100 // [1/10 deg]

LiquidCrystal lcd(LCD_RS, LCD_E, LCD_D4, LCD_D5, LCD_D6, LCD_D7);
CAxis azimuth_axis(ENC_0, MOT_0_POS, MOT_0_NEG);
CAxis elevation_axis(ENC_1, MOT_1_POS, MOT_1_NEG);

void azimuth_enc_interrupt()
{
  azimuth_axis.enc_interrupt();
}

void elevation_enc_interrupt()
{
  elevation_axis.enc_interrupt();
}

void do_homing_procedure(CAxis& axis, int32_t homing_offset)
{
  int32_t prev_pos = -1;
  int32_t cur_pos = 0;
  axis.move_negative();
  while(prev_pos != cur_pos)
  {
    prev_pos = cur_pos;
    delay(HOMING_CHECK_TIME);
    cur_pos = axis.get_current_position();
  }
  axis.stop_moving();
  axis.set_current_position(homing_offset);
  axis.move_to_position(0);
}

void setup() {
  azimuth_axis.begin();
  elevation_axis.begin();

  attachInterrupt(ENC_0_INTERRUPT, azimuth_enc_interrupt, CHANGE);
  attachInterrupt(ENC_1_INTERRUPT, elevation_enc_interrupt, CHANGE);

  CEasyCommHandler::begin(azimuth_axis, elevation_axis, BAUD_RATE);
  lcd.begin(LCD_COLS, LCD_ROWS);

  lcd.setCursor(0,0);
  lcd.print("PA3RVG Az/El Rot");
  Serial.print("PA3RVG Az/El Rot\n");
  delay(1000);

  lcd.setCursor(0,1);
  lcd.print("Homing El..");
  do_homing_procedure(elevation_axis, EL_HOMING_OFFSET);

  lcd.setCursor(0,1);
  lcd.print("Homing Az..");
  do_homing_procedure(azimuth_axis, AZ_HOMING_OFFSET);

}

void loop()
{
  uint32_t next_display_update_due = millis();
  while(1)
  {
    CEasyCommHandler::update();
    azimuth_axis.update();
    elevation_axis.update();

    if (millis() >= next_display_update_due)
    {
      lcd.setCursor(0,0);
      lcd.print("Set A ");
      lcd.print(azimuth_axis.get_position_setpoint()/10);
      lcd.print(" E ");
      lcd.print(elevation_axis.get_position_setpoint()/10);
      lcd.print("     ");

      lcd.setCursor(0,1);
      lcd.print("Cur A ");
      lcd.print(azimuth_axis.get_current_position()/10);
      lcd.print(" E ");
      lcd.print(elevation_axis.get_current_position()/10);
      lcd.print("     ");

      next_display_update_due += DISPLAY_UPDATE_PERIOD;
    }
    delay(1);
  }
}
