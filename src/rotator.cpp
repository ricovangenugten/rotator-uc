#include <Arduino.h>
#include "easycomm_handler.h"
#include "encoder_axis.h"

#ifdef USE_WIFI
#include <ESP8266WiFi.h>
#include "credentials.h"
WiFiServer wifiServer(23);
#endif

#ifdef USE_LCD
#include <LiquidCrystal.h>

#define LCD_RS 13
#define LCD_E  12
#define LCD_D4 11
#define LCD_D5 10
#define LCD_D6 9
#define LCD_D7 8
#define LCD_COLS 16
#define LCD_ROWS 2
#define DISPLAY_UPDATE_PERIOD 500 // ms
LiquidCrystal lcd(LCD_RS, LCD_E, LCD_D4, LCD_D5, LCD_D6, LCD_D7);
#endif

#ifdef IS_D1_MINI
#define MOT_EL_NEG D8
#define MOT_EL_POS D7
#define MOT_AZ_NEG D6
#define MOT_AZ_POS D5
#define ENC_AZ     D2
#define ENC_EL     D1
#else
#define MOT_EL_NEG 0
#define MOT_EL_POS 1
#define MOT_AZ_NEG 2
#define MOT_AZ_POS 3
#define ENC_AZ     4
#define ENC_EL     5
#endif

#define BAUD_RATE 9600

CEncoderAxis   azimuth_axis(ENC_AZ, MOT_AZ_POS, MOT_AZ_NEG, 50);
CEncoderAxis elevation_axis(ENC_EL, MOT_EL_POS, MOT_EL_NEG, 60);

void INTERRUPT_FUNC azimuth_enc_interrupt()
{
  azimuth_axis.enc_interrupt();
}

void INTERRUPT_FUNC elevation_enc_interrupt()
{
  elevation_axis.enc_interrupt();
}

void setup() {
  Serial.begin(BAUD_RATE);
  Serial.println();
  Serial.println("PA3RVG Az/El Rotator");

#ifdef USE_WIFI
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(100);
    Serial.print(".");
  }
  Serial.println();

  Serial.print("Connected, IP address: ");
  Serial.println(WiFi.localIP());
  wifiServer.begin();
#endif

  azimuth_axis.begin();
  elevation_axis.begin();

  attachInterrupt(digitalPinToInterrupt(ENC_AZ),   azimuth_enc_interrupt, FALLING);
  attachInterrupt(digitalPinToInterrupt(ENC_EL), elevation_enc_interrupt, FALLING);

  CEasyCommHandler::begin(azimuth_axis, elevation_axis);

#ifdef USE_LCD
  lcd.begin(LCD_COLS, LCD_ROWS);
  lcd.setCursor(0,0);
  lcd.print("PA3RVG Az/El Rot");
//  delay(1000);

//  lcd.setCursor(0,1);
//  lcd.print("Homing El..");
#endif
//  elevation_axis.do_homing_procedure();

#ifdef USE_LCD
//  lcd.setCursor(0,1);
//  lcd.print("Homing Az..");
#endif
//  azimuth_axis.do_homing_procedure();
}

#ifdef USE_WIFI
WiFiClient curClient;
#endif

void loop()
{

#ifdef USE_LCD
  uint32_t next_display_update_due = millis();
#endif

#ifdef USE_WIFI
  WiFiClient newClient = wifiServer.available();
  if (newClient)
  {
    Serial.println("New client connected");
    if (curClient.connected())
    {
      curClient.stop();
      Serial.println("Old client disconnected");
    }
    curClient = newClient;
  }

  if (curClient.connected())
  {
     CEasyCommHandler::handle_commands(curClient);
  }
#endif

  CEasyCommHandler::handle_commands(Serial);

  azimuth_axis.update();
  elevation_axis.update();

#ifdef USE_LCD
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
#endif
    delay(1);
}

