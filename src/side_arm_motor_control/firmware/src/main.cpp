#include <Arduino.h>
#include <AccelStepper.h>
#include <cstring>
#include <esp_system.h>

#include "pin_config.h"

namespace {

using namespace side_arm;

constexpr unsigned long STATE_INTERVAL_MS = 100;
constexpr size_t MAX_CMD_LEN = 96;
constexpr long HOMING_TRAVEL_STEPS = 75000;
constexpr float INSTANT_ACCEL = 6000;
constexpr int DC_IS_ADC_PIN = 35;    // R_IS
constexpr int DC_IS_UPPER = 1700;    // ADC threshold
constexpr int DC_IS_LOWER = 1500;    // hysteresis lower bound
bool dcHighLoad = false;
constexpr int DC_IS_SAMPLES = 10; 

constexpr int DC_ALERT_SERVO_MIN_US = 500;
constexpr int DC_ALERT_SERVO_MAX_US = 2500; 
bool alertServoIncreasing = true;
const double ALERT_STEP_US = 10;

unsigned long lastAdcRead = 0;
constexpr unsigned long ADC_INTERVAL_MS = 10;
int adcVal = 0;


AccelStepper stepper1(AccelStepper::DRIVER, STEPPER1_STEP, STEPPER1_DIR);
AccelStepper stepper2(AccelStepper::DRIVER, STEPPER2_STEP, STEPPER2_DIR);

// ===== SERVO SUPPORT (added from File 1) =====
constexpr int SERVO_PIN = 13;
constexpr int SERVO_PWM_CHANNEL = 6;
constexpr int SERVO_PWM_FREQ = 50;
constexpr int SERVO_PWM_RES_BITS = 16;

constexpr int SERVO_NEUTRAL_US = 1500;
constexpr int SERVO_MIN_US = 500;
constexpr int SERVO_MAX_US = 2500;
constexpr int SERVO_DEADBAND_US = 6;

int servoPulseUs = SERVO_NEUTRAL_US;
int servoTargetPulseUs = SERVO_NEUTRAL_US;
int servoCurrentPulseUs = SERVO_NEUTRAL_US;
// ===== END SERVO SUPPORT =====

bool steppersEnabled = true;
bool stepper1Homing = false;
bool stepper2Homing = false;
bool dcHoming = false;
int currentDcPercent = 0;
constexpr int DC_HOMING_SPEED = 30;
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

// ===== SERVO HELPER (added from File 1) =====
void setServoPulseUs(int pulseUs) {
  pulseUs = constrain(pulseUs, SERVO_MIN_US, SERVO_MAX_US);

  if (abs(pulseUs - SERVO_NEUTRAL_US) <= SERVO_DEADBAND_US) {
    pulseUs = SERVO_NEUTRAL_US;
  }

  servoTargetPulseUs = pulseUs;
}
// ===== END SERVO HELPER =====

void sendState(bool force = false) {
  const unsigned long now = millis();
  if (!force && (now - lastStatePublish) < STATE_INTERVAL_MS) {
    return;
  }

  const LimitStates limits = readLimitStates();

  // Added servo to state output
  String payload = String("STATE {\"l1\":") + String(limits.sw1 ? 1 : 0) +
                   ",\"l2\":" + String(limits.sw2 ? 1 : 0) +
                   ",\"l3\":" + String(limits.sw3 ? 1 : 0) +
                   ",\"s1\":" + String(stepper1.currentPosition()) +
                   ",\"s2\":" + String(stepper2.currentPosition()) +
                   ",\"dc\":" + String(currentDcPercent) +
                   ",\"servo\":" + String(servoPulseUs - SERVO_NEUTRAL_US) + "}";
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

int readDcCurrent() {
    long sum = 0;
    for (int i = 0; i < DC_IS_SAMPLES; i++) {
       sum += analogRead(DC_IS_ADC_PIN);
    }
    return sum / DC_IS_SAMPLES;
}

void applyDcCommand(int percent) {
  int originalPercent = percent;
  percent = constrain(percent, -100, 100);

  // if (percent != currentDcPercent) {
  //   Serial.print("DEBsUG DC: ");
  //   Serial.print(currentDcPercent);
  //   Serial.print(" -> ");
  //   Serial.println(percent);
  // }

  // Safety: Don't allow motion toward an engaged limit switch
  const LimitStates limits = readLimitStates();
  if (limits.sw1 && percent > 0) {
    // Limit 1 (DC retract limit) is engaged, block motion toward it
    percent = 0;
    Serial.println("BLOCKED DC motion toward engaged limit");
  }

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
  const long p1 = stepper1.currentPosition();
  const long p2 = stepper2.currentPosition();
  stepper1.move(0);
  stepper2.move(0);
  stepper1.setSpeed(0);
  stepper2.setSpeed(0);
  stepper1.setCurrentPosition(p1);
  stepper2.setCurrentPosition(p2);
  if (disableDrivers) {
    enableSteppers(false);
  }
}

// Homing states
enum class HomingState { IDLE, BACKING_OFF, APPROACHING, DONE };
HomingState stepper1HomingState = HomingState::IDLE;
HomingState stepper2HomingState = HomingState::IDLE;
HomingState dcHomingState = HomingState::IDLE;

constexpr long BACKOFF_STEPS = 800;    // steps to back off from limit
constexpr float HOMING_APPROACH_SPEED = 200.0;  // slow approach speed

void requestHome(uint8_t target) {
  const LimitStates limits = readLimitStates();

  if (target == 1) {
    if (limits.sw2) {
      // Already on limit — back off first
      stepper1HomingState = HomingState::BACKING_OFF;
      stepper1.setMaxSpeed(HOMING_SPEED);
      stepper1.setAcceleration(HOMING_SPEED * 2);
      stepper1.move(-BACKOFF_STEPS);  // positive = away from limit
      Serial.println("HOME stepper1: backing off limit");
    } else {
      // Not on limit — approach directly
      stepper1HomingState = HomingState::APPROACHING;
      stepper1.setMaxSpeed(HOMING_SPEED);
      stepper1.setAcceleration(HOMING_SPEED * 2);
      stepper1.move(HOMING_TRAVEL_STEPS);
      Serial.println("HOME stepper1: approaching limit");
    }
  } else if (target == 2) {
    if (limits.sw3) {
      stepper2HomingState = HomingState::BACKING_OFF;
      stepper2.setMaxSpeed(HOMING_SPEED);
      stepper2.setAcceleration(HOMING_SPEED * 2);
      stepper2.move(-BACKOFF_STEPS);
      Serial.println("HOME stepper2: backing off limit");
    } else {
      stepper2HomingState = HomingState::APPROACHING;
      stepper2.setMaxSpeed(HOMING_SPEED);
      stepper2.setAcceleration(HOMING_SPEED * 2);
      stepper2.move(HOMING_TRAVEL_STEPS);
      Serial.println("HOME stepper2: approaching limit");
    }
  } else if (target == 0) {
    if (limits.sw1) {
      dcHomingState = HomingState::BACKING_OFF;
      applyDcCommand(-DC_HOMING_SPEED);  // reverse away from limit
      Serial.println("HOME DC: backing off limit");
    } else {
      dcHomingState = HomingState::APPROACHING;
      applyDcCommand(DC_HOMING_SPEED);
      Serial.println("HOME DC: approaching limit");
    }
  }
}

void updateHoming() {
  const LimitStates limits = readLimitStates();

  // === Stepper 1 ===
  if (stepper1HomingState == HomingState::BACKING_OFF) {
    if (!limits.sw2 && stepper1.distanceToGo() == 0) {
      // Cleared the limit, now approach slowly
      stepper1HomingState = HomingState::APPROACHING;
      stepper1.setMaxSpeed(HOMING_APPROACH_SPEED);
      stepper1.setAcceleration(HOMING_APPROACH_SPEED * 2);
      stepper1.move(HOMING_TRAVEL_STEPS);
      Serial.println("HOME stepper1: approaching limit");
    }
  } else if (stepper1HomingState == HomingState::APPROACHING) {
    if (limits.sw2) {
      stepper1.stop();
      stepper1.setCurrentPosition(0);
      stepper1HomingState = HomingState::DONE;
      limit3Latched = true;
      Serial.println("HOME stepper1: complete, position zeroed");
    }
  }

  // === Stepper 2 ===
  if (stepper2HomingState == HomingState::BACKING_OFF) {
    if (!limits.sw3 && stepper2.distanceToGo() == 0) {
      stepper2HomingState = HomingState::APPROACHING;
      stepper2.setMaxSpeed(HOMING_APPROACH_SPEED);
      stepper2.setAcceleration(HOMING_APPROACH_SPEED * 2);
      stepper2.move(HOMING_TRAVEL_STEPS);
      Serial.println("HOME stepper2: approaching limit");
    }
  } else if (stepper2HomingState == HomingState::APPROACHING) {
    if (limits.sw3) {
      stepper2.stop();
      stepper2.setCurrentPosition(0);
      stepper2HomingState = HomingState::DONE;
      limit2Latched = true;
      Serial.println("HOME stepper2: complete, position zeroed");
    }
  }

  // === DC motor ===
  if (dcHomingState == HomingState::BACKING_OFF) {
    if (!limits.sw1) {
      // Cleared limit, brief pause then approach
      applyDcCommand(0);
      delay(100);
      dcHomingState = HomingState::APPROACHING;
      applyDcCommand(DC_HOMING_SPEED);
      Serial.println("HOME DC: approaching limit");
    }
  } else if (dcHomingState == HomingState::APPROACHING) {
    if (limits.sw1) {
      applyDcCommand(0);
      dcHomingState = HomingState::DONE;
      limit1Latched = true;
      Serial.println("HOME DC: complete, position zeroed");
    }
  }
}

void requestHomeAll() {
  requestHome(0);
  requestHome(1);
  requestHome(2);
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

  // Safety: Don't allow motion toward an engaged limit switch
  const LimitStates limits = readLimitStates();
  if (id == 1 && limits.sw2 && steps > 0) {
    // Stepper 1 limit (sw2/l2) is engaged, block motion toward it
    Serial.println("BLOCKED stepper1 motion toward engaged limit");
    return;
  }
  if (id == 2 && limits.sw3 && steps > 0) {
    // Stepper 2 limit (sw3/l3) is engaged, block motion toward it
    Serial.println("BLOCKED stepper2 motion toward engaged limit");
    return;
  }

  motor->setMaxSpeed(abs(speed) > 0 ? min(abs(speed), 2500L) : 2500L);
  motor->setAcceleration(INSTANT_ACCEL);
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

  // if (verb != "REQUEST_STATE") {
  //   Serial.print(verb);
  //   Serial.print(',  len: ');
  //   Serial.println(cmd.length());
  // }

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
  // ===== SERVO COMMAND =====
  } else if (verb == "SERVO") {
    long offset = parseLong(strtok_r(nullptr, ",", &savePtr), 0);
    setServoPulseUs(SERVO_NEUTRAL_US + offset);
  // ===== END SERVO COMMAND =====
  } else if (verb == "STOP_ALL") {
    stopAllMotion();
  } else if (verb == "STOP_NOW") {
    haltSteppersImmediate(true);
    applyDcCommand(0);
  } else if (verb == "HOME") {
    uint8_t id = static_cast<uint8_t>(parseLong(strtok_r(nullptr, ",", &savePtr), 255));
    if (id == 255) {
      requestHomeAll();
    } else {
      requestHome(id);
    }
  } else if (verb == "HOME_ALL") {
    requestHomeAll();
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

  // l1 -> DC motor
  if (limits.sw1) {
    if (!limit1Latched) {
      limit1Latched = true;
      applyDcCommand(0);
      Serial.println("EVENT Limit1 -> DC stopped");
    }
  } else {
    limit1Latched = false;
  }

  // l3 -> stepper2
  if (limits.sw3) {
    if (!limit2Latched) {
      limit2Latched = true;
      stepper2.stop();
      stepper2.setCurrentPosition(0);
      Serial.println("EVENT Limit3 -> stepper2 stopped");
    }
  } else {
      limit2Latched = false;
  }

  // l2 -> stepper1
  if (limits.sw2) {
    if (!limit3Latched) {
      limit3Latched = true;
      stepper1.stop();
      stepper1.setCurrentPosition(0);
      Serial.println("EVENT Limit2 -> stepper1 stopped");
    }
  } else {
      limit3Latched = false;
  }
}

}  // namespace

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(5);

  Serial.print("BOOT reset_reason=");
  Serial.println(esp_reset_reason());

  pinMode(LIMIT_SW_1, INPUT_PULLUP);
  pinMode(LIMIT_SW_2, INPUT_PULLUP);
  pinMode(LIMIT_SW_3, INPUT_PULLUP);

  pinMode(STEPPER_ENABLE, OUTPUT);
  enableSteppers(true);

  configureStepper(stepper1);
  configureStepper(stepper2);

  // Direction inversion (from File 2)
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

  // ===== SERVO SETUP (added from File 1) =====
  ledcSetup(SERVO_PWM_CHANNEL, SERVO_PWM_FREQ, SERVO_PWM_RES_BITS);
  ledcAttachPin(SERVO_PIN, SERVO_PWM_CHANNEL);
  setServoPulseUs(SERVO_NEUTRAL_US);
  // ===== END SERVO SETUP =====

  Serial.println("READY SideArm controller online");
}

void loop() {
  readSerial();
  checkLimits();
  updateHoming();
  
  
  if (currentDcPercent != 0) {
    unsigned long now = millis();
    if (now - lastAdcRead >= ADC_INTERVAL_MS) {
      adcVal = readDcCurrent();
      lastAdcRead = now;
    if (adcVal > DC_IS_UPPER) { // determining the threshold for high dc motor load
      dcHighLoad = true;
    } else if (adcVal < DC_IS_LOWER) {
      dcHighLoad = false;
    }
    if (dcHighLoad) {
      if (alertServoIncreasing) {
          servoTargetPulseUs += ALERT_STEP_US;
          if (servoTargetPulseUs >= DC_ALERT_SERVO_MAX_US) {
              servoTargetPulseUs = DC_ALERT_SERVO_MAX_US;
              alertServoIncreasing = false;
          }
      } else {
          servoTargetPulseUs -= ALERT_STEP_US;
          if (servoTargetPulseUs <= DC_ALERT_SERVO_MIN_US) {
              servoTargetPulseUs = DC_ALERT_SERVO_MIN_US;
              alertServoIncreasing = true;
          }
      }
    }
    }
    
  }
  
  if (steppersEnabled) {
    stepper1.run();
    stepper2.run();
  }

  const int SERVO_RAMP_STEP_US = 2;

  if (servoCurrentPulseUs < servoTargetPulseUs) {
    servoCurrentPulseUs = min(servoCurrentPulseUs + SERVO_RAMP_STEP_US, servoTargetPulseUs);
  } else if (servoCurrentPulseUs > servoTargetPulseUs) {
    servoCurrentPulseUs = max(servoCurrentPulseUs - SERVO_RAMP_STEP_US, servoTargetPulseUs);
  }

  if (servoCurrentPulseUs != servoPulseUs) {
    servoPulseUs = servoCurrentPulseUs;

    constexpr uint32_t MAX_DUTY = (1UL << SERVO_PWM_RES_BITS) - 1;
    uint32_t duty = (static_cast<uint64_t>(servoPulseUs) * MAX_DUTY) / 20000UL;
    ledcWrite(SERVO_PWM_CHANNEL, duty);
  }
  sendState();
}