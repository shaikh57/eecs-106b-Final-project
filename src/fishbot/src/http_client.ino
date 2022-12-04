#include <HTTPClient.h>
#include <ESP32Servo.h>
#include <WiFi.h>

#include "states.h"
#include "wifi_credentials.h"

// NETWORKING
const char* ssid     = SSID;
const char* password = PWORD;

// SERVO PARAMS
Servo tail_motor;
const int trim      = 60;
const int servo_pin = 26;

// TAIL PARAMS
const int amplitude     = 30;
const int offset        = 15;
const int f_upper_angle = amplitude;
const int f_lower_angle = -amplitude;
const int l_upper_angle = amplitude - offset;
const int l_lower_angle = -amplitude - offset;
const int r_upper_angle = amplitude + offset;
const int r_lower_angle = -amplitude + offset;

// FSM INITS
State current_state = NEUTRAL;
int current_angle   = 0;
String key          = "";

void setup() {
  // put your setup code here, to run once:

  Serial.begin(115200);
  delay(10);

  // Serial.print("\n\nConnecting to ");
  // Serial.println(ssid);

  WiFi.begin(ssid, password);

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
  tail_motor.detach();
}

State transition(State current_state, String action, int current_angle) {
  State next_state = current_state;
  switch (current_state) {
    case NEUTRAL:
      if (action == "w") {
        next_state = F_UPPER;
      } else if (action == "a") {
        next_state = L_UPPER;
      } else if (action == "d") {
        next_state = R_UPPER;
      }
      break;
    case F_UPPER:
      if (action == "w") {
        if (current_angle >= f_upper_angle) {
          next_state = F_LOWER;
        }
      } else if (action == "a") {
        next_state = current_angle >= l_upper_angle ? L_LOWER : L_UPPER;
      } else if (action == "d") {
        next_state = R_UPPER;
      } else {
        next_state = NEUTRAL;
      }
      break;
    case F_LOWER:
      if (action == "w") {
        if (current_angle <= f_lower_angle) {
          next_state = F_UPPER;
        }
      } else if (action == "a") {
        next_state = L_LOWER;
      } else if (action == "d") {
        next_state = current_angle <= r_lower_angle ? R_UPPER : R_LOWER;
      } else {
        next_state = NEUTRAL;
      }
      break;
    case L_UPPER:
      if (action == "w") {
        next_state = F_UPPER;
      } else if (action == "a") {
        if (current_angle >= l_upper_angle) {
          next_state = L_LOWER;
        }
      } else if (action == "d") {
        next_state = R_UPPER;
      } else {
        next_state = NEUTRAL;
      }
      break;
    case L_LOWER:
      if (action == "w") {
        next_state = current_angle <= f_lower_angle ? F_UPPER : F_LOWER;
      } else if (action == "a") {
        if (current_angle <= l_lower_angle) {
          next_state = L_UPPER;
        }
      } else if (action == "d") {
        next_state = current_angle <= r_lower_angle ? R_UPPER : R_LOWER;
      } else {
        next_state = NEUTRAL;
      }
      break;
    case R_UPPER:
      if (action == "w") {
        next_state = current_angle >= f_upper_angle ? F_LOWER : F_UPPER;
      } else if (action == "a") {
        next_state = current_angle >= l_upper_angle ? L_LOWER : L_UPPER;
      } else if (action == "d") {
        if (current_angle >= r_upper_angle) {
          next_state = R_LOWER;
        }
      } else {
        next_state = NEUTRAL;
      }
      break;
    case R_LOWER:
      if (action == "w") {
        next_state = F_LOWER;
      } else if (action == "a") {
        next_state = L_LOWER;
      } else if (action == "d") {
        if (current_angle <= r_lower_angle) {
          next_state = R_UPPER;
        }
      } else {
        next_state = NEUTRAL;
      }
      break;
    default:
      break;
  }
  return next_state;
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

void loop() {
  // put your main code here, to run repeatedly:
  unsigned long start_time = millis();
  if (WiFi.status() == WL_CONNECTED) {
    key = http_GET_request(URL);
  }
  State next_state = transition(current_state, key, current_angle);
  unsigned long end_time = millis();

  current_state = next_state;
  const long d_theta = (end_time - start_time + 100) / 50;
  if (current_state == F_UPPER || current_state == L_UPPER || current_state == R_UPPER) {
    current_angle += d_theta;
  } else if (current_state == F_LOWER || current_state == L_LOWER || current_state == R_LOWER) {
    current_angle -= d_theta;
  } else {
    if (current_angle < 0) {
      current_angle += d_theta;
    } else if (current_angle > 0) {
      current_angle -= d_theta;
    }
  }
  current_angle = max(-60, min(60, current_angle));
  // if (current_state == NEUTRAL && current_angle == 0) {
  //   tail_motor.detach();
  // } else {
  //   tail_motor.write(current_angle + trim);
  // }
  tail_motor.attach(servo_pin, 500, 2400);
  tail_motor.write(current_angle + trim);
  // Serial.println(current_angle);
  delay(100);
  tail_motor.detach();
}
