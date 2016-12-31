#include <Arduino.h>

#include <ESP8266WiFi.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
#include <EEPROM.h>
#include "password.inc"

#define RELAY 12
#define INDICATOR 13
#define TOUCH 0

int ledState = LOW;
int relayState = LOW; // Start turned off
int counter;
int reading;
unsigned long timer = 0;
int debounce_count = 50;

WiFiClient client;
Adafruit_MQTT_Client mqtt(&client, MQTT_SERVER, MQTT_SERVERPORT, MQTT_USERNAME, MQTT_KEY);
Adafruit_MQTT_Publish devicestatus = Adafruit_MQTT_Publish(&mqtt, "/UUID/status");
Adafruit_MQTT_Subscribe devicecommand = Adafruit_MQTT_Subscribe(&mqtt, "/UUID/command");

void setup() {
  EEPROM.begin(10);
  relayState = EEPROM.read(0);
  pinMode(RELAY, OUTPUT);
  pinMode(INDICATOR, OUTPUT);
  pinMode(TOUCH, INPUT);
  digitalWrite(RELAY, relayState);
  digitalWrite(INDICATOR, ledState);

  Serial.begin(115200);
  delay(10);

  Serial.println(F("Sonoff Touch MQTT Demo"));
  WiFi.begin(WLAN_SSID, WLAN_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    ledState = !ledState;
    digitalWrite(INDICATOR, ledState);
  }
  Serial.println();

  Serial.println("WiFi connected");
  Serial.println("IP address: "); Serial.println(WiFi.localIP());
  digitalWrite(INDICATOR, LOW);

  mqtt.subscribe(&devicecommand);

}

void loop()
{
  // If we have gone on to the next millisecond
  if (millis() != timer)
  {
    reading = digitalRead(TOUCH);

    if (reading == HIGH && counter > 0)
    {
      counter = 0;
    }
    if (reading == LOW)
    {
      counter++;
    }
    // If the Input has shown the same value for long enough let's switch it
    if (counter >= debounce_count)
    {
      counter = 0;
      relayState = !relayState;
      EEPROM.write(0, relayState);
      EEPROM.commit();
      digitalWrite(RELAY, relayState);
      if (! devicestatus.publish(relayState)) {
        Serial.println(F("Failed"));
      } else {
        Serial.print(F("Status: "));
        Serial.println(relayState);
        Serial.println(F("Sent OK!"));
      }

    }
    timer = millis();
  }

  MQTT_connect();
  Adafruit_MQTT_Subscribe *subscription;
  while ((subscription = mqtt.readSubscription(0))) {
    if (subscription == &devicecommand) {
      Serial.print(F("Command: "));
      Serial.println((char *)devicecommand.lastread);
      uint16_t commandval = atoi((char *)devicecommand.lastread);  // convert to a number
      relayState = commandval;
        EEPROM.write(0, relayState);
        EEPROM.commit();
      digitalWrite(RELAY, relayState);
      Serial.println(EEPROM.read(0));
    }
  }
}
// Function to connect and reconnect as necessary to the MQTT server.
// Should be called in the loop function and it will take care if connecting.
void MQTT_connect() {
  int8_t ret;

  // Stop if already connected.
  if (mqtt.connected()) {
    digitalWrite(INDICATOR, LOW);
    return;
  }

  Serial.print("Connecting to MQTT... ");
  digitalWrite(INDICATOR, HIGH);

  uint8_t retries = 3;
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
    Serial.println(mqtt.connectErrorString(ret));
    Serial.println("Retrying MQTT connection in 5 seconds...");
    mqtt.disconnect();
    delay(5000);  // wait 5 seconds
    retries--;
    if (retries == 0) {
      // basically die and wait for WDT to reset me
      while (1);
    }
  }
  Serial.println("MQTT Connected!");
}
