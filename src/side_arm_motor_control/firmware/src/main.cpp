#include <Arduino.h>
#include <AccelStepper.h>
#include <cstring>

#include "pin_config.h"

namespace {

using namespace side_arm;

constexpr unsigned long STATE_INTERVAL_MS = 100;
constexpr size_t MAX_CMD_LEN = 96;
constexpr long HOMING_TRAVEL_STEPS = 40000;
constexpr float INSTANT_ACCEL = 1e6F;               // very high accel to approximate immediate speed

AccelStepper stepper1(AccelStepper::DRIVER, STEPPER1_STEP, STEPPER1_DIR);
AccelStepper stepper2(AccelStepper::DRIVER, STEPPER2_STEP, STEPPER2_DIR);

bool steppersEnabled = true;
bool stepper1Homing = false;
bool stepper2Homing = false;
int currentDcPercent = 0;
unsigned long lastStatePublish = 0;
bool limit1Latched = false;
bool limit2Latched = false;
bool limit3Latched = false;

String serialBuffer;

struct LimitStates {
  bool sw1;
  bool sw2;
  bool sw3;
};

LimitStates readLimitStates() {
  return {
      digitalRead(LIMIT_SW_1) == LOW,
      digitalRead(LIMIT_SW_2) == LOW,
      digitalRead(LIMIT_SW_3) == LOW,
  };
}

void sendState(bool force = false) {
  const unsigned long now = millis();
  if (!force && (now - lastStatePublish) < STATE_INTERVAL_MS) {
    return;
  }

  const LimitStates limits = readLimitStates();

  String payload = String("STATE {\"l1\":") + String(limits.sw1 ? 1 : 0) +
                   ",\"l2\":" + String(limits.sw2 ? 1 : 0) +
                   ",\"l3\":" + String(limits.sw3 ? 1 : 0) +
                   ",\"s1\":" + String(stepper1.currentPosition()) +
                   ",\"s2\":" + String(stepper2.currentPosition()) +
                   ",\"dc\":" + String(currentDcPercent) + "}";
  Serial.println(payload);
  lastStatePublish = now;
}

void enableSteppers(bool enable) {
  digitalWrite(STEPPER_ENABLE, enable ? LOW : HIGH);
  steppersEnabled = enable;
  if (!enable) {
    stepper1.stop();
    stepper2.stop();
  }
}

void configureStepper(AccelStepper& motor) {
  motor.setMaxSpeed(DEFAULT_MAX_SPEED);
  motor.setAcceleration(DEFAULT_ACCELERATION);
}

void applyDcCommand(int percent) {
  percent = constrain(percent, -100, 100);
  currentDcPercent = percent;

  if (percent == 0) {
    ledcWrite(DC_R_PWM_CHANNEL, 0);
    ledcWrite(DC_L_PWM_CHANNEL, 0);
    return;
  }

  const int duty = map(abs(percent), 0, 100, 0, (1 << DC_PWM_RES_BITS) - 1);
  if (percent > 0) {
    ledcWrite(DC_R_PWM_CHANNEL, duty);
    ledcWrite(DC_L_PWM_CHANNEL, 0);
  } else {
    ledcWrite(DC_R_PWM_CHANNEL, 0);
    ledcWrite(DC_L_PWM_CHANNEL, duty);
  }
}

void stopAllMotion() {
  stepper1.stop();
  stepper2.stop();
  applyDcCommand(0);
}

void haltStepperImmediate(AccelStepper& motor) {
  const long p = motor.currentPosition();
  motor.move(0);
  motor.setSpeed(0);
  motor.setCurrentPosition(p);
}

void haltSteppersImmediate(bool disableDrivers = true) {
  // Force targets to current position and zero speed to stop without decel.
  const long p1 = stepper1.currentPosition();
  const long p2 = stepper2.currentPosition();
  stepper1.move(0);
  stepper2.move(0);
  stepper1.setSpeed(0);
  stepper2.setSpeed(0);
  stepper1.setCurrentPosition(p1);
  stepper2.setCurrentPosition(p2);
  if (disableDrivers) {
    enableSteppers(false);  // EN=HIGH, immediate driver disable
  }
}

void requestHome(uint8_t target) {
  if (target == 1) {
    stepper1Homing = true;
    stepper1.setMaxSpeed(HOMING_SPEED);
    stepper1.setAcceleration(HOMING_SPEED * 2);
    stepper1.move(-HOMING_TRAVEL_STEPS);
  } else if (target == 2) {
    stepper2Homing = true;
    stepper2.setMaxSpeed(HOMING_SPEED);
    stepper2.setAcceleration(HOMING_SPEED * 2);
    stepper2.move(-HOMING_TRAVEL_STEPS);
  }
}

long parseLong(const char* token, long fallback = 0) {
  if (!token) {
    return fallback;
  }
  return strtol(token, nullptr, 10);
}

void handleStepperMove(uint8_t id, long steps, long speed) {
  AccelStepper* motor = (id == 1) ? &stepper1 : (id == 2 ? &stepper2 : nullptr);
  if (!motor) {
    Serial.println("ERR Unknown stepper id");
    return;
  }

  motor->setMaxSpeed(abs(speed) > 0 ? abs(speed) : DEFAULT_MAX_SPEED);
  motor->setAcceleration(INSTANT_ACCEL);  // jump to target speed with minimal ramp
  motor->move(steps);
}

void processCommand(const String& cmd) {
  if (cmd.isEmpty()) {
    return;
  }

  if (cmd.length() >= MAX_CMD_LEN) {
    Serial.println("ERR Command too long");
    return;
  }

  char buffer[MAX_CMD_LEN]{0};
  cmd.toCharArray(buffer, sizeof(buffer));

  char* savePtr;
  char* token = strtok_r(buffer, ",", &savePtr);
  if (!token) {
    return;
  }

  String verb(token);
  verb.toUpperCase();

  if (verb == "STEPPER_MOVE") {
    uint8_t id = static_cast<uint8_t>(parseLong(strtok_r(nullptr, ",", &savePtr), 0));
    long steps = parseLong(strtok_r(nullptr, ",", &savePtr), 0);
    long speed = parseLong(strtok_r(nullptr, ",", &savePtr), DEFAULT_MAX_SPEED);
    handleStepperMove(id, steps, speed);
  } else if (verb == "STEPPER_ENABLE") {
    long value = parseLong(strtok_r(nullptr, ",", &savePtr), 1);
    enableSteppers(value != 0);
  } else if (verb == "DC_SPEED") {
    long percent = parseLong(strtok_r(nullptr, ",", &savePtr), 0);
    applyDcCommand(percent);
  } else if (verb == "STOP_ALL") {
    stopAllMotion();
  } else if (verb == "STOP_NOW") {
    haltSteppersImmediate(true);
    applyDcCommand(0);
  } else if (verb == "HOME") {
    uint8_t id = static_cast<uint8_t>(parseLong(strtok_r(nullptr, ",", &savePtr), 0));
    requestHome(id);
  } else if (verb == "REQUEST_STATE") {
    sendState(true);
  } else {
    Serial.println("ERR Unknown command");
  }
}

void readSerial() {
  while (Serial.available() > 0) {
    char c = static_cast<char>(Serial.read());
    if (c == '\n') {
      serialBuffer.trim();
      processCommand(serialBuffer);
      serialBuffer = "";
    } else if (c != '\r') {
      if (serialBuffer.length() < MAX_CMD_LEN - 1) {
        serialBuffer += c;
      }
    }
  }
}

void checkLimits() {
  const LimitStates limits = readLimitStates();

  // l1 -> DC motor: stop and wait for next command
  if (limits.sw1) {
    if (!limit1Latched) {
      limit1Latched = true;
      applyDcCommand(0);
      Serial.println("EVENT Limit1 -> DC stopped, position zeroed, awaiting command");
    }
  } else {
    limit1Latched = false;
  }

  // l2 -> stepper2: stop, zero position, wait for next command
  if (limits.sw2) {
    if (!limit2Latched) {
      limit2Latched = true;
      haltStepperImmediate(stepper2);
      stepper2.setCurrentPosition(0);  // Zero the position
      stepper2Homing = false;
      Serial.println("EVENT Limit2 -> stepper2 stopped, position zeroed, awaiting command");
    }
  } else {
    limit2Latched = false;
  }

  // l3 -> stepper1: stop, zero position, wait for next command
  if (limits.sw3) {
    if (!limit3Latched) {
      limit3Latched = true;
      haltStepperImmediate(stepper1);
      stepper1.setCurrentPosition(0);  // Zero the position
      stepper1Homing = false;
      Serial.println("EVENT Limit3 -> stepper1 stopped, position zeroed, awaiting command");
    }
  } else {
    limit3Latched = false;
  }
}

}  // namespace

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(5);

  pinMode(LIMIT_SW_1, INPUT_PULLUP);
  pinMode(LIMIT_SW_2, INPUT_PULLUP);
  pinMode(LIMIT_SW_3, INPUT_PULLUP);

  pinMode(STEPPER_ENABLE, OUTPUT);
  enableSteppers(true);

  configureStepper(stepper1);
  configureStepper(stepper2);

  // Apply direction inversion if configured
  stepper1.setPinsInverted(STEPPER1_INVERT_DIR, false, false);
  stepper2.setPinsInverted(STEPPER2_INVERT_DIR, false, false);

  pinMode(DC_EN, OUTPUT);
  digitalWrite(DC_EN, HIGH);
  pinMode(DC_R_PWM, OUTPUT);
  pinMode(DC_L_PWM, OUTPUT);
  ledcSetup(DC_R_PWM_CHANNEL, DC_PWM_FREQ, DC_PWM_RES_BITS);
  ledcSetup(DC_L_PWM_CHANNEL, DC_PWM_FREQ, DC_PWM_RES_BITS);
  ledcAttachPin(DC_R_PWM, DC_R_PWM_CHANNEL);
  ledcAttachPin(DC_L_PWM, DC_L_PWM_CHANNEL);
  applyDcCommand(0);

  Serial.println("READY SideArm controller online");
}

void loop() {
  readSerial();
  checkLimits();

  if (steppersEnabled) {
    stepper1.run();
    stepper2.run();
  }

  sendState();
}

