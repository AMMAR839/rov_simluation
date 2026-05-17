#include <Servo.h>

const uint8_t THRUSTER_PINS[6] = {3, 5, 6, 9, 10, 11};
const uint8_t GRIPPER_PIN = 12;
const uint16_t PWM_NEUTRAL_US = 1500;
const uint16_t PWM_MIN_US = 1100;
const uint16_t PWM_MAX_US = 1900;
const uint32_t WATCHDOG_MS = 500;

Servo thrusters[6];
Servo gripper;
String line;
uint32_t lastCommandMs = 0;

uint16_t clampPwm(long value) {
  if (value < PWM_MIN_US) return PWM_MIN_US;
  if (value > PWM_MAX_US) return PWM_MAX_US;
  return (uint16_t)value;
}

void writeNeutral() {
  for (uint8_t i = 0; i < 6; ++i) {
    thrusters[i].writeMicroseconds(PWM_NEUTRAL_US);
  }
}

bool parsePwmLine(const String &input, uint16_t *values, uint8_t expected) {
  if (!input.startsWith("PWM,")) {
    return false;
  }
  uint8_t index = 0;
  int start = 4;
  while (index < expected && start < input.length()) {
    int comma = input.indexOf(',', start);
    String token = comma >= 0 ? input.substring(start, comma) : input.substring(start);
    token.trim();
    values[index++] = clampPwm(token.toInt());
    if (comma < 0) {
      break;
    }
    start = comma + 1;
  }
  return index >= 6;
}

void applyPwm(uint16_t *values, uint8_t count) {
  for (uint8_t i = 0; i < 6; ++i) {
    thrusters[i].writeMicroseconds(values[i]);
  }
  if (count >= 7) {
    gripper.writeMicroseconds(values[6]);
  }
  lastCommandMs = millis();
}

void setup() {
  Serial.begin(115200);
  line.reserve(96);
  for (uint8_t i = 0; i < 6; ++i) {
    thrusters[i].attach(THRUSTER_PINS[i]);
  }
  gripper.attach(GRIPPER_PIN);
  writeNeutral();
  gripper.writeMicroseconds(1100);
  lastCommandMs = millis();
}

void loop() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n') {
      uint16_t values[7] = {
        PWM_NEUTRAL_US, PWM_NEUTRAL_US, PWM_NEUTRAL_US,
        PWM_NEUTRAL_US, PWM_NEUTRAL_US, PWM_NEUTRAL_US,
        1100
      };
      if (parsePwmLine(line, values, 7)) {
        applyPwm(values, 7);
      }
      line = "";
    } else if (c != '\r') {
      if (line.length() < 90) {
        line += c;
      } else {
        line = "";
      }
    }
  }

  if (millis() - lastCommandMs > WATCHDOG_MS) {
    writeNeutral();
  }
}
