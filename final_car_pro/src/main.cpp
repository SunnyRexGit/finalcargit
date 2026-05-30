#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#include "config.h"

HardwareSerial MotorSerial(2);
HardwareSerial RcSerial(1);
Adafruit_PWMServoDriver pwm(PCA9685_ADDRESS);

const uint8_t motorAddr[6] = {1, 2, 3, 4, 5, 6};
const int SERVO_CH[4] = {0, 1, 2, 3};

int rcCh[16] = {1500};
uint16_t sbusRaw[16] = {0};
uint8_t sbusFrame[25] = {0};
uint8_t sbusFramePos = 0;
uint32_t lastRcFrameMs = 0;
bool rcLostFlag = true;
bool rcFailsafeFlag = true;
bool failsafeTriggered = false;

int panAngle = SERVO_DEFAULT_S1;
int armServoLogicalAngle = SERVO_DEFAULT_S4_LOGICAL;
int gripperAngle = GRIPPER_OPEN_ANGLE;
int32_t motor5Pos = M5_PULSE_MIN;
int32_t motor6Pos = 0;

int lastWheelCmd[4] = {0, 0, 0, 0};
bool chassisStopped = false;
uint32_t lastServoUpdateMs = 0;
uint32_t lastM5UpdateMs = 0;
uint32_t lastDebugPrintMs = 0;

struct EdgeLatch {
  bool lowArmed = true;
  bool highArmed = true;

  bool low(int value) {
    updateCenter(value);
    if (lowArmed && value < EDGE_LOW_THRESHOLD) {
      lowArmed = false;
      return true;
    }
    return false;
  }

  bool high(int value) {
    updateCenter(value);
    if (highArmed && value > EDGE_HIGH_THRESHOLD) {
      highArmed = false;
      return true;
    }
    return false;
  }

  void updateCenter(int value) {
    if (value > EDGE_CENTER_LOW && value < EDGE_CENTER_HIGH) {
      lowArmed = true;
      highArmed = true;
    }
  }
};

EdgeLatch trayCh1Edge;
EdgeLatch trayCh2Edge;
EdgeLatch autoCh1Edge;
EdgeLatch autoCh2Edge;

enum class AutoPhase {
  Idle,
  MoveHigh,
  MoveTarget,
  WaitServoSettle,
  Done
};

bool autoRunning = false;
AutoPhase autoPhase = AutoPhase::Idle;
uint8_t currentAutoState = 2;
uint8_t targetAutoState = 2;
uint32_t autoPhaseStartMs = 0;
uint32_t autoPhaseDurationMs = 0;
bool autoNeedsFinalMove = false;
int32_t autoStartM5 = 0;
int32_t autoTargetM5 = TURNTABLE_MOTOR5;

int32_t clamp32(int32_t value, int32_t low, int32_t high) {
  if (value < low) return low;
  if (value > high) return high;
  return value;
}

int channel(uint8_t indexMacro) {
  return rcCh[indexMacro];
}

void setupRcReceiver();
bool readRcChannels();
bool parseSbusFrame(const uint8_t* frame);
int normalizeRcValue(int raw);
int channelPercent(int value);
bool allSticksCentered();

void setupPCA9685();
void setupRS485Motors();
void sendMotorCommand(uint8_t addr, const uint8_t* cmd, size_t len);
void motorEnable(uint8_t idx);
void motorStop(uint8_t idx);
void setCurrentAsZero(uint8_t idx);
void setMotorSpeed(uint8_t idx, int cmdPercent);
void setMotorAbsPosition(uint8_t idx, int32_t targetPosition, uint16_t speedOverride = 0);
void emergencyStopAllMotors();
void stopChassisOnce();

void setServoRawChannel(int pcaChannel, int angle);
void setServoLogical(uint8_t servoId, int angle);
void setGripperAngle(int angle);

void updateLaserForMode();
void handleFailsafe();
bool handleFailsafeAndRecovery();
void handleDriveMode();
void handleTrayMode();
void handleArmManualMode();
void handleArmAutoMode();
void updateM5ByChannel(int ch3);
void startAutoState(uint8_t state);
void cancelAutoMacro(bool stopMotors);
void updateAutoStateMachine();
uint32_t estimateMotor5MoveTime(int32_t from, int32_t to);
void getAutoTargets(uint8_t state, int32_t& m5, int& s1, int& s4);
bool transitionUsesHighPos(uint8_t fromState, uint8_t toState);
void printDebugInfoPeriodically();

void setup() {
  Serial.begin(115200);
  delay(200);
  for (uint8_t i = 0; i < 16; i++) {
    rcCh[i] = RC_OUT_MID;
  }
  Serial.println();
  Serial.println("ESP32-S3 RC car controller starting...");

  pinMode(LASER_PIN, OUTPUT);
  digitalWrite(LASER_PIN, LOW);

  setupRcReceiver();
  setupPCA9685();
  setupRS485Motors();

  setServoLogical(1, panAngle);
  setGripperAngle(gripperAngle);
  setServoLogical(4, armServoLogicalAngle);

  Serial.println("Ready. Waiting for valid SBUS/M.BUS receiver frames.");
}

void loop() {
  const bool rcOk = readRcChannels();

  if (!rcOk) {
    handleFailsafe();
    printDebugInfoPeriodically();
    return;
  }

  if (!handleFailsafeAndRecovery()) {
    printDebugInfoPeriodically();
    return;
  }

  updateLaserForMode();
  updateAutoStateMachine();

  if (autoRunning) {
    if (autoCh2Edge.low(channel(RC_CH2_INDEX))) {
      cancelAutoMacro(true);
      Serial.println("AUTO: cancelled by CH2 low");
    }
    printDebugInfoPeriodically();
    return;
  }

  if (channel(RC_CH5_INDEX) < RC_OUT_MID) {
    if (channel(RC_CH6_INDEX) < RC_OUT_MID) {
      handleDriveMode();
    } else {
      handleTrayMode();
    }
  } else {
    stopChassisOnce();
    if (channel(RC_CH6_INDEX) < RC_OUT_MID) {
      handleArmManualMode();
    } else {
      handleArmAutoMode();
    }
  }

  printDebugInfoPeriodically();
}

void setupRcReceiver() {
  RcSerial.begin(RC_BAUD_RATE, RC_SERIAL_CONFIG, RC_RX_PIN, RC_TX_PIN, RC_SERIAL_INVERTED);
  Serial.printf("RC receiver: SBUS/M.BUS on RX GPIO%d, %d baud, inverted=%d\n",
                RC_RX_PIN, RC_BAUD_RATE, RC_SERIAL_INVERTED);
}

bool readRcChannels() {
  bool frameUpdated = false;

  while (RcSerial.available()) {
    uint8_t b = RcSerial.read();

    if (sbusFramePos == 0 && b != 0x0F) {
      continue;
    }

    sbusFrame[sbusFramePos++] = b;
    if (sbusFramePos >= sizeof(sbusFrame)) {
      sbusFramePos = 0;
      if (parseSbusFrame(sbusFrame)) {
        frameUpdated = true;
        lastRcFrameMs = millis();
      }
    }
  }

  const bool timedOut = (millis() - lastRcFrameMs) > RC_TIMEOUT_MS;
  rcLostFlag = timedOut || rcLostFlag;
  return frameUpdated ? (!rcLostFlag && !rcFailsafeFlag) : (!timedOut && !rcLostFlag && !rcFailsafeFlag);
}

bool parseSbusFrame(const uint8_t* frame) {
  if (frame[0] != 0x0F) return false;

  sbusRaw[0]  = ((frame[1]     | frame[2] << 8) & 0x07FF);
  sbusRaw[1]  = ((frame[2] >> 3 | frame[3] << 5) & 0x07FF);
  sbusRaw[2]  = ((frame[3] >> 6 | frame[4] << 2 | frame[5] << 10) & 0x07FF);
  sbusRaw[3]  = ((frame[5] >> 1 | frame[6] << 7) & 0x07FF);
  sbusRaw[4]  = ((frame[6] >> 4 | frame[7] << 4) & 0x07FF);
  sbusRaw[5]  = ((frame[7] >> 7 | frame[8] << 1 | frame[9] << 9) & 0x07FF);
  sbusRaw[6]  = ((frame[9] >> 2 | frame[10] << 6) & 0x07FF);
  sbusRaw[7]  = ((frame[10] >> 5 | frame[11] << 3) & 0x07FF);
  sbusRaw[8]  = ((frame[12]    | frame[13] << 8) & 0x07FF);
  sbusRaw[9]  = ((frame[13] >> 3 | frame[14] << 5) & 0x07FF);
  sbusRaw[10] = ((frame[14] >> 6 | frame[15] << 2 | frame[16] << 10) & 0x07FF);
  sbusRaw[11] = ((frame[16] >> 1 | frame[17] << 7) & 0x07FF);
  sbusRaw[12] = ((frame[17] >> 4 | frame[18] << 4) & 0x07FF);
  sbusRaw[13] = ((frame[18] >> 7 | frame[19] << 1 | frame[20] << 9) & 0x07FF);
  sbusRaw[14] = ((frame[20] >> 2 | frame[21] << 6) & 0x07FF);
  sbusRaw[15] = ((frame[21] >> 5 | frame[22] << 3) & 0x07FF);

  rcLostFlag = (frame[23] & 0x04) != 0;
  rcFailsafeFlag = (frame[23] & 0x08) != 0;

  for (uint8_t i = 0; i < 16; i++) {
    rcCh[i] = normalizeRcValue(sbusRaw[i]);
  }

  return true;
}

int normalizeRcValue(int raw) {
  long value = map(raw, RC_RAW_MIN, RC_RAW_MAX, RC_OUT_MIN, RC_OUT_MAX);
  return constrain(value, RC_OUT_MIN, RC_OUT_MAX);
}

int channelPercent(int value) {
  const int delta = value - RC_OUT_MID;
  if (abs(delta) <= RC_DEADZONE) return 0;
  if (delta > 0) {
    return map(value, RC_OUT_MID + RC_DEADZONE, RC_OUT_MAX, 0, WHEEL_CMD_MAX);
  }
  return map(value, RC_OUT_MID - RC_DEADZONE, RC_OUT_MIN, 0, -WHEEL_CMD_MAX);
}

bool allSticksCentered() {
  return abs(channel(RC_CH1_INDEX) - RC_OUT_MID) <= RC_DEADZONE &&
         abs(channel(RC_CH2_INDEX) - RC_OUT_MID) <= RC_DEADZONE &&
         abs(channel(RC_CH3_INDEX) - RC_OUT_MID) <= RC_DEADZONE &&
         abs(channel(RC_CH4_INDEX) - RC_OUT_MID) <= RC_DEADZONE;
}

void setupPCA9685() {
  Wire.begin(PCA9685_SDA_PIN, PCA9685_SCL_PIN);
  pwm.begin();
  pwm.setPWMFreq(SERVO_PWM_FREQ);
  delay(10);
  Serial.printf("PCA9685: SDA GPIO%d, SCL GPIO%d, address 0x%02X, %u Hz\n",
                PCA9685_SDA_PIN, PCA9685_SCL_PIN, PCA9685_ADDRESS, SERVO_PWM_FREQ);
}

void setupRS485Motors() {
  MotorSerial.begin(115200, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);
  Serial.printf("RS485 motors: RX GPIO%d, TX GPIO%d, 115200 SERIAL_8N1\n",
                RS485_RX_PIN, RS485_TX_PIN);

  for (uint8_t i = 0; i < 6; i++) {
    motorEnable(i);
    delay(50);
    if (i < 4) {
      setCurrentAsZero(i);
    }
    delay(50);
  }
}

void sendMotorCommand(uint8_t addr, const uint8_t* cmd, size_t len) {
  Serial.printf("RS485 TX addr=%u:", addr);
  for (size_t i = 0; i < len; i++) {
    Serial.printf(" %02X", cmd[i]);
  }
  Serial.println();

  MotorSerial.write(cmd, len);
  MotorSerial.flush();
  delay(20);

  while (MotorSerial.available()) {
    uint8_t resp = MotorSerial.read();
    Serial.printf("Resp: %02X", resp);
    if (resp == 0xE2) Serial.println(" -> Parameter error");
    else if (resp == 0xEE) Serial.println(" -> Format error");
    else if (resp == 0x02) Serial.println(" -> OK");
    else Serial.println();
  }
}

void motorEnable(uint8_t idx) {
  if (idx >= 6) return;
  uint8_t cmd[] = {motorAddr[idx], 0xF3, 0xAB, 0x01, 0x00, 0x6B};
  sendMotorCommand(motorAddr[idx], cmd, sizeof(cmd));
}

void motorStop(uint8_t idx) {
  if (idx >= 6) return;
  uint8_t cmd[] = {motorAddr[idx], 0xFE, 0x98, 0x00, 0x6B};
  sendMotorCommand(motorAddr[idx], cmd, sizeof(cmd));
}

void setCurrentAsZero(uint8_t idx) {
  if (idx >= 6) return;
  uint8_t cmd[] = {motorAddr[idx], 0x0A, 0x6D, 0x6B};
  sendMotorCommand(motorAddr[idx], cmd, sizeof(cmd));
}

void setMotorSpeed(uint8_t idx, int cmdPercent) {
  if (idx >= 4) return;

  cmdPercent = constrain(cmdPercent, -WHEEL_CMD_MAX, WHEEL_CMD_MAX);
  if (abs(cmdPercent) < MIN_EFFECTIVE_CMD) cmdPercent = 0;

  if (lastWheelCmd[idx] == cmdPercent) return;
  lastWheelCmd[idx] = cmdPercent;
  chassisStopped = (lastWheelCmd[0] == 0 && lastWheelCmd[1] == 0 &&
                    lastWheelCmd[2] == 0 && lastWheelCmd[3] == 0);

  uint8_t direction = (cmdPercent > 0) ? 0x01 : 0x00;
  uint16_t speedVal = map(abs(cmdPercent), 0, WHEEL_CMD_MAX, 0, MAX_SPEED_RPM);
  if (cmdPercent == 0) speedVal = 0;

  uint8_t cmd[] = {
      motorAddr[idx], 0xF6, direction,
      (uint8_t)(speedVal >> 8), (uint8_t)(speedVal & 0xFF),
      ACCEL_WHEEL, 0x00, 0x6B
  };
  sendMotorCommand(motorAddr[idx], cmd, sizeof(cmd));
}

void setMotorAbsPosition(uint8_t idx, int32_t targetPosition, uint16_t speedOverride) {
  if (idx != 4 && idx != 5) return;

  int32_t pos = targetPosition;
  uint8_t acc = (idx == 4) ? ACCEL_MOTOR5 : ACCEL_MOTOR6;
  uint16_t speed = speedOverride > 0 ? speedOverride : ((idx == 4) ? POSITION_SPEED_M5 : POSITION_SPEED_M6);
  uint32_t pulse = 0;

  if (idx == 4) {
    pos = clamp32(pos, M5_PULSE_MIN, M5_PULSE_MAX);
    motor5Pos = pos;
    pulse = (uint32_t)abs(pos);
  } else {
    pos = clamp32(pos, M6_MIN, M6_MAX);
    motor6Pos = pos;
    pulse = (uint32_t)abs(pos);
  }

  uint8_t direction = (pos >= 0) ? 0x01 : 0x00;
  uint8_t cmd[] = {
      motorAddr[idx], 0xFD, direction,
      (uint8_t)(speed >> 8), (uint8_t)(speed & 0xFF),
      acc,
      (uint8_t)(pulse >> 24), (uint8_t)(pulse >> 16),
      (uint8_t)(pulse >> 8), (uint8_t)(pulse & 0xFF),
      POSITION_MODE_ABSOLUTE, POSITION_SYNC_DISABLED, 0x6B
  };
  sendMotorCommand(motorAddr[idx], cmd, sizeof(cmd));
}

void emergencyStopAllMotors() {
  for (uint8_t i = 0; i < 6; i++) {
    motorStop(i);
    if (i < 4) lastWheelCmd[i] = 0;
  }
  chassisStopped = true;
}

void stopChassisOnce() {
  if (chassisStopped) return;
  for (uint8_t i = 0; i < 4; i++) {
    setMotorSpeed(i, 0);
  }
  chassisStopped = true;
}

void setServoRawChannel(int pcaChannel, int angle) {
  angle = constrain(angle, 0, 180);
  uint16_t pulse = map(angle, 0, 180, SERVOMIN, SERVOMAX);
  pwm.setPWM(pcaChannel, 0, pulse);
}

void setServoLogical(uint8_t servoId, int angle) {
  if (servoId < 1 || servoId > 4) return;

  angle = constrain(angle, 0, 180);
  int actualAngle = angle;
  if (servoId == 4) {
    armServoLogicalAngle = angle;
    actualAngle = 180 - angle;
  } else if (servoId == 1) {
    panAngle = angle;
  }

  setServoRawChannel(SERVO_CH[servoId - 1], actualAngle);
}

void setGripperAngle(int angle) {
  gripperAngle = constrain(angle, 0, 180);
  setServoLogical(2, gripperAngle);
#if GRIPPER_S3_REVERSED
  setServoLogical(3, 180 - gripperAngle);
#else
  setServoLogical(3, gripperAngle);
#endif
}

void updateLaserForMode() {
  const int ch5 = channel(RC_CH5_INDEX);
  const int ch6 = channel(RC_CH6_INDEX);
  const bool ch5Centered = ch5 > EDGE_CENTER_LOW && ch5 < EDGE_CENTER_HIGH;
  const bool ch6Centered = ch6 > EDGE_CENTER_LOW && ch6 < EDGE_CENTER_HIGH;
  const bool driveMode = ch5 < RC_OUT_MID && ch6 < RC_OUT_MID;
  const bool laserOn = !driveMode && !ch5Centered && !ch6Centered;
  digitalWrite(LASER_PIN, laserOn ? HIGH : LOW);
}

void handleFailsafe() {
  if (!failsafeTriggered) {
    digitalWrite(LASER_PIN, LOW);
    emergencyStopAllMotors();
    cancelAutoMacro(false);
    failsafeTriggered = true;
    Serial.println("FAILSAFE TRIGGERED");
  }
}

bool handleFailsafeAndRecovery() {
  if (!failsafeTriggered) return true;

  if (allSticksCentered()) {
    failsafeTriggered = false;
    Serial.println("FAILSAFE CLEARED");
    return true;
  }

  return false;
}

void handleDriveMode() {
  int vx = channelPercent(channel(RC_CH1_INDEX)) * CHASSIS_X_SIGN;
  int vy = channelPercent(channel(RC_CH2_INDEX)) * CHASSIS_Y_SIGN;
  int w = channelPercent(channel(RC_CH4_INDEX)) * CHASSIS_W_SIGN;

  int m[4] = {
      (vy - vx + w) * M1_SIGN,
      (-vy + vx + w) * M2_SIGN,
      (-vy - vx + w) * M3_SIGN,
      (vy + vx + w) * M4_SIGN
  };

  int maxAbs = 1;
  for (uint8_t i = 0; i < 4; i++) {
    maxAbs = max(maxAbs, abs(m[i]));
  }
  if (maxAbs > WHEEL_CMD_MAX) {
    for (uint8_t i = 0; i < 4; i++) {
      m[i] = (long)m[i] * WHEEL_CMD_MAX / maxAbs;
    }
  }

  for (uint8_t i = 0; i < 4; i++) {
    setMotorSpeed(i, m[i]);
  }
}

void handleTrayMode() {
  stopChassisOnce();

  if (trayCh1Edge.low(channel(RC_CH1_INDEX))) {
    setMotorAbsPosition(5, motor6Pos - M6_STEP);
    Serial.printf("M6 tray left -> %ld\n", (long)motor6Pos);
  }
  if (trayCh1Edge.high(channel(RC_CH1_INDEX))) {
    setMotorAbsPosition(5, motor6Pos + M6_STEP);
    Serial.printf("M6 tray right -> %ld\n", (long)motor6Pos);
  }
  if (trayCh2Edge.low(channel(RC_CH2_INDEX))) {
    setMotorAbsPosition(5, M6_CENTER);
    Serial.printf("M6 tray center -> %ld\n", (long)motor6Pos);
  }
}

void handleArmManualMode() {
  stopChassisOnce();

  const uint32_t now = millis();
  if (now - lastServoUpdateMs >= SERVO_UPDATE_MS) {
    lastServoUpdateMs = now;

    const int ch1 = channel(RC_CH1_INDEX);
    const int ch2 = channel(RC_CH2_INDEX);
    if (ch1 < EDGE_LOW_THRESHOLD) {
      setServoLogical(1, panAngle - SERVO_STEP);
    } else if (ch1 > EDGE_HIGH_THRESHOLD) {
      setServoLogical(1, panAngle + SERVO_STEP);
    }

    if (ch2 > EDGE_HIGH_THRESHOLD) {
      setServoLogical(4, armServoLogicalAngle + SERVO_STEP);
    } else if (ch2 < EDGE_LOW_THRESHOLD) {
      setServoLogical(4, armServoLogicalAngle - SERVO_STEP);
    }

    const int ch4 = channel(RC_CH4_INDEX);
    if (abs(ch4 - RC_OUT_MID) > RC_DEADZONE) {
      int angle = map(ch4, RC_OUT_MIN, RC_OUT_MAX, GRIPPER_CLOSE_ANGLE, GRIPPER_OPEN_ANGLE);
      setGripperAngle(constrain(angle, min(GRIPPER_OPEN_ANGLE, GRIPPER_CLOSE_ANGLE),
                                max(GRIPPER_OPEN_ANGLE, GRIPPER_CLOSE_ANGLE)));
    }
  }

  updateM5ByChannel(channel(RC_CH3_INDEX));
}

void updateM5ByChannel(int ch3) {
  const uint32_t now = millis();
  if (now - lastM5UpdateMs < M5_UPDATE_MS) return;

  if (ch3 > EDGE_HIGH_THRESHOLD) {
    lastM5UpdateMs = now;
    setMotorAbsPosition(4, motor5Pos - (M5_STEP * M5_DIR_SIGN));
  } else if (ch3 < EDGE_LOW_THRESHOLD) {
    lastM5UpdateMs = now;
    setMotorAbsPosition(4, motor5Pos + (M5_STEP * M5_DIR_SIGN));
  }
}

void handleArmAutoMode() {
  stopChassisOnce();

  if (autoCh2Edge.low(channel(RC_CH2_INDEX))) {
    cancelAutoMacro(true);
    Serial.println("AUTO: emergency cancel");
    return;
  }
  if (autoRunning) return;

  if (autoCh1Edge.low(channel(RC_CH1_INDEX))) {
    startAutoState(1);
  } else if (autoCh2Edge.high(channel(RC_CH2_INDEX))) {
    startAutoState(2);
  } else if (autoCh1Edge.high(channel(RC_CH1_INDEX))) {
    startAutoState(3);
  }
}

void startAutoState(uint8_t state) {
  if (state < 1 || state > 3 || autoRunning) return;
  if (state == currentAutoState) {
    Serial.printf("AUTO: already at state %u\n", state);
    return;
  }

  targetAutoState = state;
  int targetS1 = 0;
  int targetS4 = 0;
  getAutoTargets(targetAutoState, autoTargetM5, targetS1, targetS4);
  const bool useHigh = transitionUsesHighPos(currentAutoState, targetAutoState);

  autoRunning = true;
  autoNeedsFinalMove = useHigh;
  autoStartM5 = motor5Pos;
  autoPhaseStartMs = millis();

  Serial.printf("AUTO: start state %u -> state %u\n", currentAutoState, targetAutoState);

  if (useHigh) {
    Serial.println("AUTO_STEP: move M5 to HIGH_POS");
    setMotorAbsPosition(4, HIGH_POS, POSITION_SPEED_M5_AUTO);
    autoPhase = AutoPhase::MoveHigh;
    autoPhaseDurationMs = estimateMotor5MoveTime(autoStartM5, HIGH_POS);
  } else {
    Serial.println("AUTO_STEP: move M5 to target");
    setMotorAbsPosition(4, autoTargetM5, POSITION_SPEED_M5_AUTO);
    autoPhase = AutoPhase::MoveTarget;
    autoPhaseDurationMs = estimateMotor5MoveTime(autoStartM5, autoTargetM5);
  }
}

void cancelAutoMacro(bool stopMotors) {
  autoRunning = false;
  autoPhase = AutoPhase::Idle;
  if (stopMotors) {
    motorStop(4);
    motorStop(5);
  }
}

void updateAutoStateMachine() {
  if (!autoRunning) return;

  const uint32_t now = millis();
  if (now - autoPhaseStartMs < autoPhaseDurationMs) return;

  int targetS1 = 0;
  int targetS4 = 0;
  getAutoTargets(targetAutoState, autoTargetM5, targetS1, targetS4);

  switch (autoPhase) {
    case AutoPhase::MoveHigh:
    case AutoPhase::MoveTarget:
      Serial.println("AUTO_STEP: move servos");
      setServoLogical(1, targetS1);
      setServoLogical(4, targetS4);
      autoPhase = AutoPhase::WaitServoSettle;
      autoPhaseStartMs = now;
      autoPhaseDurationMs = AUTO_SERVO_SETTLE_MS;
      break;

    case AutoPhase::WaitServoSettle:
      if (autoNeedsFinalMove) {
        Serial.println("AUTO_STEP: move M5 to target");
        setMotorAbsPosition(4, autoTargetM5, POSITION_SPEED_M5_AUTO);
        autoPhase = AutoPhase::Done;
        autoPhaseStartMs = now;
        autoPhaseDurationMs = estimateMotor5MoveTime(HIGH_POS, autoTargetM5);
      } else {
        currentAutoState = targetAutoState;
        autoRunning = false;
        autoPhase = AutoPhase::Idle;
        Serial.println("AUTO: finished");
      }
      break;

    case AutoPhase::Done:
      currentAutoState = targetAutoState;
      autoRunning = false;
      autoPhase = AutoPhase::Idle;
      Serial.println("AUTO: finished");
      break;

    case AutoPhase::Idle:
      break;
  }
}

uint32_t estimateMotor5MoveTime(int32_t from, int32_t to) {
  (void)from;
  (void)to;
  return AUTO_M5_MOVE_WAIT_MS;
}

void getAutoTargets(uint8_t state, int32_t& m5, int& s1, int& s4) {
  if (state == 1) {
    m5 = PRE_CLAMP_MOTOR5;
    s1 = PRE_CLAMP_SERVO0;
    s4 = PRE_CLAMP_SERVO3;
  } else if (state == 2) {
    m5 = TURNTABLE_MOTOR5;
    s1 = TURNTABLE_SERVO0;
    s4 = TURNTABLE_SERVO3;
  } else {
    m5 = PRE_DROP_MOTOR5;
    s1 = PRE_DROP_SERVO0;
    s4 = PRE_DROP_SERVO3;
  }
}

bool transitionUsesHighPos(uint8_t fromState, uint8_t toState) {
  (void)fromState;
  (void)toState;
  return true;
}

void printDebugInfoPeriodically() {
  const uint32_t now = millis();
  if (now - lastDebugPrintMs < DEBUG_PRINT_MS) return;
  lastDebugPrintMs = now;

  const char* mode = "DRIVE";
  if (channel(RC_CH5_INDEX) < RC_OUT_MID && channel(RC_CH6_INDEX) >= RC_OUT_MID) mode = "TRAY";
  if (channel(RC_CH5_INDEX) >= RC_OUT_MID && channel(RC_CH6_INDEX) < RC_OUT_MID) mode = "ARM";
  if (channel(RC_CH5_INDEX) >= RC_OUT_MID && channel(RC_CH6_INDEX) >= RC_OUT_MID) mode = "AUTO";

  Serial.printf("CH1=%d CH2=%d CH3=%d CH4=%d CH5=%d CH6=%d mode=%s failsafe=%d auto=%d M1=%d M2=%d M3=%d M4=%d M5=%ld M6=%ld S1=%d S4=%d grip=%d\n",
                channel(RC_CH1_INDEX), channel(RC_CH2_INDEX), channel(RC_CH3_INDEX),
                channel(RC_CH4_INDEX), channel(RC_CH5_INDEX), channel(RC_CH6_INDEX),
                mode, failsafeTriggered || rcFailsafeFlag || rcLostFlag, autoRunning,
                lastWheelCmd[0], lastWheelCmd[1], lastWheelCmd[2], lastWheelCmd[3],
                (long)motor5Pos, (long)motor6Pos, panAngle, armServoLogicalAngle, gripperAngle);
}
