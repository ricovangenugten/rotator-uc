#include <Arduino.h>
#include <LiquidCrystal.h>
#include "easycomm_handler.h"
//#include "axis.h"

#define MOT_0_POS  13
#define MOT_0_NEG  12
#define MOT_1_POS  11
#define MOT_1_NEG  10
#define LCD_RS 9
#define LCD_E  8
#define LCD_D4 7
#define LCD_D5 6
#define LCD_D6 5
#define LCD_D7 4
#define ENC_0  3
#define ENC_1  2

#define AZIMUTH_ENC_INTERRUPT 0
#define ELEVATION_ENC_INTERRUPT 1

#define LCD_COLS 16
#define LCD_ROWS 2

#define BAUD_RATE 9600

#define DISPLAY_UPDATE_PERIOD 500 // ms
#define HOMING_CHECK_TIME 500 // ms
#define AZ_HOMING_OFFSET -1000 // [1/100 deg]
#define EL_HOMING_OFFSET -1000 // [1/100 deg]

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
  int32_t prev_pos = 0;
  int32_t cur_pos = 0;
  axis.move_negative();
  while(1)
  {
    cur_pos = axis.get_current_position();
    if (cur_pos == prev_pos)
      break;
    prev_pos = cur_pos;
    delay(HOMING_CHECK_TIME);
  }
  axis.stop_moving();
  axis.set_current_position(homing_offset);
  axis.move_to_position(0);
}

void setup() {
  azimuth_axis.begin();
  elevation_axis.begin();

  attachInterrupt(AZIMUTH_ENC_INTERRUPT, azimuth_enc_interrupt, CHANGE);
  attachInterrupt(ELEVATION_ENC_INTERRUPT, elevation_enc_interrupt, CHANGE);

  CEasyCommHandler::begin(azimuth_axis, elevation_axis, BAUD_RATE);
  lcd.begin(LCD_COLS, LCD_ROWS);

  lcd.setCursor(0,0);
  lcd.print("PA3RVG Az/El Rot");
  delay(1000);

  lcd.setCursor(1,0);
  lcd.print("Homing El..");
  do_homing_procedure(elevation_axis, EL_HOMING_OFFSET);

  lcd.setCursor(1,0);
  lcd.print("Homing Az..");
  do_homing_procedure(azimuth_axis, AZ_HOMING_OFFSET);

  lcd.setCursor(1,0);
  lcd.print("Ready      ");
  delay(1000);
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
      lcd.print(azimuth_axis.get_position_setpoint()/100);
      lcd.print(" E ");
      lcd.print(elevation_axis.get_position_setpoint()/100);

      lcd.setCursor(1,0);
      lcd.print("Cur A ");
      lcd.print(azimuth_axis.get_current_position()/100);
      lcd.print(" E ");
      lcd.print(elevation_axis.get_current_position()/100);

      next_display_update_due += DISPLAY_UPDATE_PERIOD;
    }
  }
}
