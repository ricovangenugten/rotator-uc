#include <Arduino.h>
#include "easycomm_handler.h"
#include "encoder_axis.h"

#ifdef USE_WIFI
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ArduinoOTA.h>

#include "credentials.h"
#define TCP_PORT 4533
WiFiServer wifiServer(TCP_PORT);
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);
#define MQTT_UPDATE_PERIOD 1000 // ms
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

CEncoderAxis   azimuth_axis(ENC_AZ, MOT_AZ_POS, MOT_AZ_NEG);
CEncoderAxis elevation_axis(ENC_EL, MOT_EL_POS, MOT_EL_NEG);

void INTERRUPT_FUNC azimuth_enc_interrupt()
{
  azimuth_axis.enc_interrupt();
}

void INTERRUPT_FUNC elevation_enc_interrupt()
{
  elevation_axis.enc_interrupt();
}

#ifdef USE_WIFI
bool is_ota_mode_requested = false;
bool is_ota_mode = false;

void mqtt_callback(char* topic, byte* payload, uint length)
{
  Serial.println("MQTT message received");
  if (strncmp((char*)payload, "OTA", 3) == 0)
  {
    is_ota_mode_requested = true;
    mqttClient.publish(MQTT_TOPIC_PREFIX"/state", "OTA");
    Serial.println("OTA mode on");
  }
  else
  {
    is_ota_mode_requested = false;
    Serial.println("OTA mode off");
    if (is_ota_mode)
    {
      ESP.restart();
    }
  }
}

void mqtt_connect()
{
  mqttClient.setServer(mqtt_server, mqtt_port);
  mqttClient.setCallback(mqtt_callback);

  for (int i = 0; i < 4; i++)
  {
    if (mqttClient.connect(
        wifi_hostname, mqtt_user, mqtt_pass))
    {
      mqttClient.subscribe(MQTT_TOPIC_PREFIX"/set");
      mqttClient.publish(MQTT_TOPIC_PREFIX"/state", "NORMAL");
      mqttClient.loop();
      Serial.println("MQTT connected");
      break;
    }
  }
}

void wifi_connect()
{
  WiFi.hostname(wifi_hostname);
  WiFi.begin(wifi_ssid, wifi_pass);

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
}
#endif

void setup() {
  Serial.begin(BAUD_RATE);
  Serial.println();
  Serial.println("PA3RVG Az/El Rotator");

#ifdef USE_WIFI
  wifi_connect();
  mqtt_connect();
#endif

  azimuth_axis.begin();
  elevation_axis.begin();

  attachInterrupt(digitalPinToInterrupt(ENC_AZ),   azimuth_enc_interrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_EL), elevation_enc_interrupt, CHANGE);

  CEasyCommHandler::begin(azimuth_axis, elevation_axis);

#ifdef USE_LCD
  lcd.begin(LCD_COLS, LCD_ROWS);
  lcd.setCursor(0,0);
  lcd.print("PA3RVG Az/El Rot");
  delay(1000);

//  lcd.setCursor(0,1);
//  lcd.print("Homing El..");
#endif
  elevation_axis.do_homing_procedure();

#ifdef USE_LCD
//  lcd.setCursor(0,1);
//  lcd.print("Homing Az..");
#endif
  azimuth_axis.do_homing_procedure();
}

#ifdef USE_WIFI
WiFiClient curClient;

void ota_setup()
{
  // OTA time
  ArduinoOTA.onStart([]() {
    String type;
     if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else { // U_FS
      type = "filesystem";
    }

    // NOTE: if updating FS this would be the place to unmount FS using FS.end()
    Serial.println("Start updating " + type);
  });

  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });

  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });

  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) {
      Serial.println("Auth Failed");
    } else if (error == OTA_BEGIN_ERROR) {
      Serial.println("Begin Failed");
    } else if (error == OTA_CONNECT_ERROR) {
      Serial.println("Connect Failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      Serial.println("Receive Failed");
    } else if (error == OTA_END_ERROR) {
      Serial.println("End Failed");
    }
  });

  ArduinoOTA.begin();
}

void ota_loop()
{
  mqttClient.loop();
  ArduinoOTA.handle();
  delay(1);
}

uint32_t next_mqtt_update_due = 0;
#endif

#ifdef USE_LCD
uint32_t next_display_update_due = 0;
#endif

// Init to a value we will never reach to trigger initial update
int32_t prev_az_set = 7200;
int32_t prev_el_set = 7200;
int32_t prev_az_pos = 7200;
int32_t prev_el_pos = 7200;

void rotator_loop()
{

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

  mqttClient.loop();
#endif

  CEasyCommHandler::handle_commands(Serial);

  azimuth_axis.update();
  elevation_axis.update();

#ifdef USE_WIFI
  // Move to OTA mode if requested and no axes are moving
  if (azimuth_axis.is_stopped() && elevation_axis.is_stopped() && is_ota_mode_requested)
  {
    ota_setup();
    is_ota_mode = true;
    is_ota_mode_requested = false;
  }

  if (millis() >= next_mqtt_update_due)
  {
    int32_t cur_az_set = azimuth_axis.get_position_setpoint();
    int32_t cur_el_set = elevation_axis.get_position_setpoint();
    int32_t cur_az_pos = azimuth_axis.get_current_position();
    int32_t cur_el_pos = elevation_axis.get_current_position();

    if (cur_az_set != prev_az_set ||
        cur_el_set != prev_el_set ||
        cur_az_pos != prev_az_pos ||
        cur_el_pos != prev_el_pos)
    {

      // send values
      static char json_str[256];

      snprintf(
        json_str,
        sizeof(json_str),
        "{\"az_setpoint\": %0.01f, \"el_setpoint\": %0.01f, \"az_position\": %0.01f, \"el_position\": %0.01f}",
        cur_az_set/10.0f,
        cur_el_set/10.0f,
        cur_az_pos/10.0f,
        cur_el_pos/10.0f);

      mqttClient.publish(MQTT_TOPIC_PREFIX"/measurements", json_str);

      prev_az_set = cur_az_set;
      prev_el_set = cur_el_set;
      prev_az_pos = cur_az_pos;
      prev_el_pos = cur_el_pos;
    }

    next_mqtt_update_due += MQTT_UPDATE_PERIOD;
  }
#endif

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

void loop()
{
#ifdef USE_WIFI
  if (is_ota_mode) ota_loop(); else
#endif
  rotator_loop();
}
