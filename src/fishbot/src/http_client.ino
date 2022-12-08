#include <stdlib.h>
#include <string.h>
#include <ESP32Servo.h>
#include <HTTPClient.h>
#include <WiFi.h>

#include "wifi_credentials.h"

Servo tail_motor;
const int trim      = 95;
const int servo_pin = 26;
const int period    = 200;

const char *delim = "\n";
const char *frwd = "FRWD";
const char *turn = "TURN";

void setup() {
  // put your setup code here, to run once:

  Serial.begin(115200);
  delay(10);

  // Serial.print("\n\nConnecting to ");
  // Serial.println(ssid);

  WiFi.begin(SSID, PWORD);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    // Serial.print('.');
  }

  // Serial.println("\nWiFi connected");
  // Serial.println("IP address: ");
  // Serial.println(WiFi.localIP());

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  tail_motor.setPeriodHertz(50);
  tail_motor.attach(servo_pin, 500, 2400);
  tail_motor.write(trim);
  delay(1000);
}

String http_GET_request(const char* server) {
  HTTPClient http;
    
  // Your IP address with path or Domain name with URL path 
  http.begin(server);
  
  int response_code = http.GET();
  
  String payload = "--"; 
  
  if (response_code > 0) {
    // Serial.print("HTTP Response code: ");
    // Serial.println(response_code);
    payload = http.getString();
  } else {
    // Serial.print("Error code: ");
    // Serial.println(response_code);
  }
  http.end();

  return payload;
}

void move_tail(int a, int b, unsigned long moving_time) {
  unsigned long moveStartTime = millis(); // time when start moving
  unsigned long progress = 0;

  while (progress <= moving_time) {
    progress = millis() - moveStartTime;
    long angle = map(progress, 0, moving_time, a, b);
    // Serial.println(angle);
    tail_motor.write(angle);
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  String key;
  if (WiFi.status() == WL_CONNECTED) {
    key = http_GET_request(URL);
  }

  int str_len = key.length()+1;
  char commands[str_len];
  key.toCharArray(commands, str_len);
  
  char *cmd = strtok(commands, delim);
  char *param = strtok(NULL, delim);
  
  int angle = atoi(param);
  if (strcmp(cmd, frwd) == 0) {
    move_tail(trim, trim + 30, 200);
    move_tail(trim + 30, trim - 30, 400);
    move_tail(trim - 30, trim, 200);
  } else if (strcmp(cmd, turn) == 0) {
    tail_motor.write(trim + angle);
    delay(500);
    move_tail(trim + angle, trim, 500);
  }
}
