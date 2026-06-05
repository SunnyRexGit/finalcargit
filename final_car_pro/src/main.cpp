#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#include "config.h"

HardwareSerial MotorSerial(2);
HardwareSerial RcSerial(1);
Adafruit_PWMServoDriver pwm(PCA9685_ADDRESS);

const uint8_t motorAddr[6] = {1, 2, 3, 4, 5, 6};
const int SERVO_CH[4] = {0, 1, 2, 3};
constexpr uint8_t AUTO_STATE_CH2_LOW = 4;

int rcCh[16] = {1500};
uint16_t sbusRaw[16] = {0};
uint8_t sbusFrame[25] = {0};
uint8_t sbusFramePos = 0;
uint32_t lastRcFrameMs = 0;
bool rcLostFlag = true;
bool rcFailsafeFlag = true;
bool failsafeTriggered = false;
bool rcWasOk = false;

int panAngle = SERVO_DEFAULT_S1;
int armServoAngle = SERVO_DEFAULT_S4;
uint8_t gripperState = GRIPPER_STATE_OPEN;
int gripperAngle = GRIPPER_OPEN_ANGLE;
int gripperTargetAngle = GRIPPER_OPEN_ANGLE;
int32_t motor5Pos = M5_PULSE_MIN;
int32_t motor6Pos = 0;

int lastWheelCmd[4] = {0, 0, 0, 0};
bool chassisStopped = false;
uint32_t lastGripperUpdateMs = 0;
uint32_t lastDebugPrintMs = 0;
uint32_t lastLaserBlinkMs = 0;
uint32_t laserRcConnectBlinkStartMs = 0;
bool laserBlinkState = true;
bool laserRcConnectBlinkActive = false;
bool gripperMotionActive = false;

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

  bool high(int value, int threshold = EDGE_HIGH_THRESHOLD) {
    updateCenter(value);
    if (highArmed && value > threshold) {
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
EdgeLatch gripperCh4Edge;
EdgeLatch autoCh1Edge;
EdgeLatch autoCh2Edge;
EdgeLatch autoCh3Edge;

enum class AutoPhase {
  Idle,
  MoveHigh,
  MoveTarget,
  WaitServoSettle,
  Done,
  TurntableMovePickup,
  TurntableWaitBeforeClose,
  TurntableWaitGripperClose,
  TurntableMoveHigh,
  TurntableWaitServoSettle,
  TurntableMoveToTarget,
  TurntableWaitBeforeOpen,
  TurntableWaitGripperOpen,
  TurntableMoveTopAfterOpen,
  TurntableStepTray,
  TurntableRotatePreClamp,
  TurntableMovePreClamp,
  PreDropMoveTopForOpen,
  PreDropWaitGripperOpen,
  PreDropSetS4ToTurntable,
  PreDropSetS1ToTurntable,
  PreDropMoveToTurntableClamp,
  PreDropWaitGripperClose,
  PreDropWaitAfterClose,
  PreDropMoveTopAfterClamp,
  PreDropSetS1ToDrop,
  PreDropSetS4ToDrop,
  PreDropMoveToDrop,
  PreDropOpenAtDrop,
  PreDropWaitOpenAtDrop
};

bool autoRunning = false;
AutoPhase autoPhase = AutoPhase::Idle;
uint8_t currentAutoState = 2;
uint8_t targetAutoState = 2;
uint32_t autoPhaseStartMs = 0;
uint32_t autoPhaseDurationMs = 0;
bool autoNeedsFinalMove = false;
int32_t autoStartM5 = 0;
int32_t autoIntermediateM5 = HIGH_POS;
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
int channelPercentWithMax(int value, int maxCmd);
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
void setServoAngle(uint8_t servoId, int angle);
void setGripperAngle(int angle);
void setGripperState(uint8_t state);
void setGripperStateSlow(uint8_t state);
void setGripperStateFast(uint8_t state);
void updateGripperMotion();

void updateLaserForMode();
void startLaserRcConnectBlink();
void handleFailsafe();
bool handleFailsafeAndRecovery();
void handleDriveMode();
void handleTrayMode();
void handleChassisFineMode();
void handleArmAutoMode();
void handleGripperByChannel(int ch4);
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
  digitalWrite(LASER_PIN, HIGH);

  setupRcReceiver();
  setupPCA9685();
  setupRS485Motors();

  setServoAngle(1, panAngle);
  setGripperState(gripperState);
  setServoAngle(4, armServoAngle);

  Serial.println("Ready. Waiting for valid SBUS/M.BUS receiver frames.");
}

void loop() {
  const bool rcOk = readRcChannels();
  updateGripperMotion();

  if (rcOk && !rcWasOk) {
    startLaserRcConnectBlink();
  }
  rcWasOk = rcOk;

  if (!rcOk) {
    handleFailsafe();
    printDebugInfoPeriodically();
    return;
  }

  if (!handleFailsafeAndRecovery()) {
    updateLaserForMode();
    printDebugInfoPeriodically();
    return;
  }

  updateLaserForMode();
  updateAutoStateMachine();

  if (channel(RC_CH5_INDEX) < RC_OUT_MID) {
    if (channel(RC_CH6_INDEX) < RC_OUT_MID) {
      handleDriveMode();
    } else {
      handleTrayMode();
    }
  } else {
    if (channel(RC_CH6_INDEX) < RC_OUT_MID) {
      handleChassisFineMode();
    } else {
      stopChassisOnce();
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
  return channelPercentWithMax(value, WHEEL_CMD_MAX);
}

int channelPercentWithMax(int value, int maxCmd) {
  maxCmd = constrain(maxCmd, 0, WHEEL_CMD_MAX);
  value = constrain(value, RC_OUT_MIN, RC_OUT_MAX);
  const int delta = value - RC_OUT_MID;
  if (abs(delta) <= RC_DEADZONE) return 0;

  if (delta > 0) {
    const int activeRange = RC_OUT_MAX - (RC_OUT_MID + RC_DEADZONE);
    const int activeValue = value - (RC_OUT_MID + RC_DEADZONE);
    return constrain((int)((long)activeValue * maxCmd / activeRange), 0, maxCmd);
  }

  const int activeRange = (RC_OUT_MID - RC_DEADZONE) - RC_OUT_MIN;
  const int activeValue = (RC_OUT_MID - RC_DEADZONE) - value;
  return -constrain((int)((long)activeValue * maxCmd / activeRange), 0, maxCmd);
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
  uint16_t speedVal = (uint16_t)((uint32_t)abs(cmdPercent) * CHASSIS_MAX_SPEED_RPM_LIMIT / WHEEL_CMD_MAX);
  if (cmdPercent == 0) speedVal = 0;

  uint8_t cmd[] = {
      motorAddr[idx], 0xF6, direction,
      (uint8_t)(speedVal >> 8), (uint8_t)(speedVal & 0xFF),
      CHASSIS_ACCEL_WHEEL_LIMIT, 0x00, 0x6B
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

void setServoAngle(uint8_t servoId, int angle) {
  if (servoId < 1 || servoId > 4) return;

  angle = (servoId == 4) ? constrain(angle, SERVO_S4_MIN, SERVO_S4_MAX)
                         : constrain(angle, 0, 180);
  int actualAngle = angle;
  if (servoId == 4) {
    armServoAngle = angle;
  } else if (servoId == 1) {
    panAngle = angle;
  }

  setServoRawChannel(SERVO_CH[servoId - 1], actualAngle);
}

void setGripperAngle(int angle) {
  gripperAngle = constrain(angle, 0, 180);
  setServoAngle(2, gripperAngle);
#if GRIPPER_S3_REVERSED
  setServoAngle(3, 180 - gripperAngle);
#else
  setServoAngle(3, gripperAngle);
#endif
}

void setGripperState(uint8_t state) {
  setGripperStateSlow(state);
}

void setGripperStateSlow(uint8_t state) {
  gripperState = (state == GRIPPER_STATE_CLOSED) ? GRIPPER_STATE_CLOSED : GRIPPER_STATE_OPEN;
  gripperTargetAngle = (gripperState == GRIPPER_STATE_OPEN) ? GRIPPER_OPEN_ANGLE : GRIPPER_CLOSE_ANGLE;

  if (gripperAngle == gripperTargetAngle) {
    gripperMotionActive = false;
    setGripperAngle(gripperTargetAngle);
    Serial.printf("GRIPPER: already %s angle=%d\n",
                  gripperState == GRIPPER_STATE_OPEN ? "open" : "closed",
                  gripperAngle);
    return;
  }

  gripperMotionActive = true;
  lastGripperUpdateMs = 0;
  Serial.printf("GRIPPER: %s stepped target=%d current=%d\n",
                gripperState == GRIPPER_STATE_OPEN ? "open" : "close",
                gripperTargetAngle, gripperAngle);
}

void setGripperStateFast(uint8_t state) {
  gripperState = (state == GRIPPER_STATE_CLOSED) ? GRIPPER_STATE_CLOSED : GRIPPER_STATE_OPEN;
  gripperTargetAngle = (gripperState == GRIPPER_STATE_OPEN) ? GRIPPER_OPEN_ANGLE : GRIPPER_CLOSE_ANGLE;
  gripperMotionActive = false;
  setGripperAngle(gripperTargetAngle);
  Serial.printf("GRIPPER: %s fast angle=%d\n",
                gripperState == GRIPPER_STATE_OPEN ? "open" : "close",
                gripperAngle);
}

void updateGripperMotion() {
  if (!gripperMotionActive) return;

  const uint32_t now = millis();
  const uint32_t updateMs = (gripperState == GRIPPER_STATE_OPEN)
                                ? GRIPPER_OPEN_UPDATE_MS
                                : GRIPPER_CLOSE_UPDATE_MS;
  if (now - lastGripperUpdateMs < updateMs) return;
  lastGripperUpdateMs = now;

  const int delta = gripperTargetAngle - gripperAngle;
  if (delta == 0) {
    gripperMotionActive = false;
    Serial.printf("GRIPPER: %s finished angle=%d\n",
                  gripperState == GRIPPER_STATE_OPEN ? "open" : "close",
                  gripperAngle);
    return;
  }

  const int maxStep = (gripperState == GRIPPER_STATE_OPEN)
                          ? GRIPPER_OPEN_STEP
                          : GRIPPER_CLOSE_STEP;
  const int step = min(abs(delta), maxStep);
  setGripperAngle(gripperAngle + (delta > 0 ? step : -step));

  if (gripperAngle == gripperTargetAngle) {
    gripperMotionActive = false;
    Serial.printf("GRIPPER: %s finished angle=%d\n",
                  gripperState == GRIPPER_STATE_OPEN ? "open" : "close",
                  gripperAngle);
  }
}

void updateLaserForMode() {
  const uint32_t now = millis();
  if (laserRcConnectBlinkActive) {
    if (now - laserRcConnectBlinkStartMs < LASER_RC_CONNECTED_BLINK_DURATION_MS) {
      if (now - lastLaserBlinkMs >= LASER_RC_CONNECTED_BLINK_INTERVAL_MS) {
        lastLaserBlinkMs = now;
        laserBlinkState = !laserBlinkState;
        digitalWrite(LASER_PIN, laserBlinkState ? HIGH : LOW);
      }
      return;
    }
    laserRcConnectBlinkActive = false;
  }

  const int ch5 = channel(RC_CH5_INDEX);
  const int ch6 = channel(RC_CH6_INDEX);
  const bool autoMode = ch5 >= RC_OUT_MID && ch6 >= RC_OUT_MID;
  const uint32_t blinkIntervalMs = autoRunning ? LASER_AUTO_RUNNING_BLINK_INTERVAL_MS
                                               : LASER_BLINK_INTERVAL_MS;

  if (!autoMode) {
    laserBlinkState = true;
    digitalWrite(LASER_PIN, HIGH);
    return;
  }

  if (now - lastLaserBlinkMs >= blinkIntervalMs) {
    lastLaserBlinkMs = now;
    laserBlinkState = !laserBlinkState;
    digitalWrite(LASER_PIN, laserBlinkState ? HIGH : LOW);
  }
}

void startLaserRcConnectBlink() {
  laserRcConnectBlinkStartMs = millis();
  laserRcConnectBlinkActive = true;
  lastLaserBlinkMs = 0;
  laserBlinkState = false;
  Serial.println("RC connected: laser fast blink for 3 seconds");
}

void handleFailsafe() {
  if (!failsafeTriggered) {
    digitalWrite(LASER_PIN, HIGH);
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

void handleChassisFineMode() {
  int vx = channelPercentWithMax(channel(RC_CH1_INDEX), FINE_WHEEL_CMD_MAX) * CHASSIS_X_SIGN;
  int vy = channelPercentWithMax(channel(RC_CH2_INDEX), FINE_WHEEL_CMD_MAX) * CHASSIS_Y_SIGN;
  int w = channelPercentWithMax(channel(RC_CH4_INDEX), FINE_WHEEL_CMD_MAX) * CHASSIS_W_SIGN;

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
  if (maxAbs > FINE_WHEEL_CMD_MAX) {
    for (uint8_t i = 0; i < 4; i++) {
      m[i] = (long)m[i] * FINE_WHEEL_CMD_MAX / maxAbs;
    }
  }

  for (uint8_t i = 0; i < 4; i++) {
    setMotorSpeed(i, m[i]);
  }
}

void handleGripperByChannel(int ch4) {
  if (gripperCh4Edge.low(ch4)) {
    setGripperState(GRIPPER_STATE_CLOSED);
  } else if (gripperCh4Edge.high(ch4)) {
    setGripperState(GRIPPER_STATE_OPEN);
  }
}

void handleArmAutoMode() {
  stopChassisOnce();
  handleGripperByChannel(channel(RC_CH4_INDEX));

  if (autoCh3Edge.high(channel(RC_CH3_INDEX), AUTO_CH3_TOP_THRESHOLD)) {
    Serial.printf("AUTO_STEP: CH3 top -> move M5 to top pos %ld\n", (long)M5_TOP_MOTOR5);
    setMotorAbsPosition(4, M5_TOP_MOTOR5, POSITION_SPEED_M5_AUTO);
    return;
  }

  if (autoCh1Edge.low(channel(RC_CH1_INDEX))) {
    startAutoState(1);
  } else if (autoCh2Edge.high(channel(RC_CH2_INDEX))) {
    startAutoState(2);
  } else if (autoCh1Edge.high(channel(RC_CH1_INDEX))) {
    startAutoState(3);
  }
}

void startAutoState(uint8_t state) {
  if (state < 1 || state > AUTO_STATE_CH2_LOW) return;

  targetAutoState = state;
  int targetS1 = 0;
  int targetS4 = 0;
  getAutoTargets(targetAutoState, autoTargetM5, targetS1, targetS4);
  const bool useHigh = transitionUsesHighPos(currentAutoState, targetAutoState);

  autoRunning = true;
  autoNeedsFinalMove = useHigh;
  autoStartM5 = motor5Pos;
  autoIntermediateM5 = (targetAutoState == AUTO_STATE_CH2_LOW) ? CH2_LOW_AUTO_LIFT_MOTOR5 : HIGH_POS;
  autoPhaseStartMs = millis();

  Serial.printf("AUTO: start state %u -> state %u\n", currentAutoState, targetAutoState);

  if (targetAutoState == 2) {
    Serial.printf("AUTO_STEP: move M5 to pickup pos %ld\n", (long)PICKUP_MOTOR5);
    setMotorAbsPosition(4, PICKUP_MOTOR5, POSITION_SPEED_M5_AUTO);
    autoPhase = AutoPhase::TurntableMovePickup;
    autoPhaseDurationMs = estimateMotor5MoveTime(autoStartM5, PICKUP_MOTOR5);
    return;
  }

  if (targetAutoState == 3) {
    Serial.printf("AUTO_STEP: move M5 to top pos %ld\n", (long)M5_TOP_MOTOR5);
    setMotorAbsPosition(4, M5_TOP_MOTOR5, POSITION_SPEED_M5_AUTO);
    autoPhase = AutoPhase::PreDropMoveTopForOpen;
    autoPhaseDurationMs = estimateMotor5MoveTime(autoStartM5, M5_TOP_MOTOR5);
    return;
  }

  if (useHigh) {
    Serial.printf("AUTO_STEP: move M5 to lift pos %ld\n", (long)autoIntermediateM5);
    setMotorAbsPosition(4, autoIntermediateM5, POSITION_SPEED_M5_AUTO);
    autoPhase = AutoPhase::MoveHigh;
    autoPhaseDurationMs = estimateMotor5MoveTime(autoStartM5, autoIntermediateM5);
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
      setServoAngle(1, targetS1);
      setServoAngle(4, targetS4);
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
        autoPhaseDurationMs = estimateMotor5MoveTime(autoIntermediateM5, autoTargetM5);
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

    case AutoPhase::TurntableMovePickup:
      Serial.println("AUTO_STEP: wait before close gripper");
      autoPhase = AutoPhase::TurntableWaitBeforeClose;
      autoPhaseStartMs = now;
      autoPhaseDurationMs = AUTO_GRIPPER_SETTLE_MS;
      break;

    case AutoPhase::TurntableWaitBeforeClose:
      Serial.println("AUTO_STEP: close gripper");
      setGripperStateFast(GRIPPER_STATE_CLOSED);
      autoPhase = AutoPhase::TurntableWaitGripperClose;
      autoPhaseStartMs = now;
      autoPhaseDurationMs = 0;
      break;

    case AutoPhase::TurntableWaitGripperClose:
      if (gripperMotionActive) {
        autoPhaseStartMs = now;
        autoPhaseDurationMs = GRIPPER_CLOSE_UPDATE_MS;
        break;
      }
      Serial.printf("AUTO_STEP: move M5 to lift pos %ld\n", (long)HIGH_POS);
      setMotorAbsPosition(4, HIGH_POS, POSITION_SPEED_M5_AUTO);
      autoPhase = AutoPhase::TurntableMoveHigh;
      autoPhaseStartMs = now;
      autoPhaseDurationMs = estimateMotor5MoveTime(PICKUP_MOTOR5, HIGH_POS);
      break;

    case AutoPhase::TurntableMoveHigh:
      Serial.println("AUTO_STEP: move servos to turntable");
      setServoAngle(1, targetS1);
      setServoAngle(4, targetS4);
      autoPhase = AutoPhase::TurntableWaitServoSettle;
      autoPhaseStartMs = now;
      autoPhaseDurationMs = AUTO_SERVO_SETTLE_MS;
      break;

    case AutoPhase::TurntableWaitServoSettle:
      Serial.printf("AUTO_STEP: move M5 to turntable pos %ld\n", (long)TURNTABLE_MOTOR5);
      setMotorAbsPosition(4, TURNTABLE_MOTOR5, POSITION_SPEED_M5_AUTO);
      autoPhase = AutoPhase::TurntableMoveToTarget;
      autoPhaseStartMs = now;
      autoPhaseDurationMs = estimateMotor5MoveTime(HIGH_POS, TURNTABLE_MOTOR5);
      break;

    case AutoPhase::TurntableMoveToTarget:
      Serial.println("AUTO_STEP: wait before open gripper");
      autoPhase = AutoPhase::TurntableWaitBeforeOpen;
      autoPhaseStartMs = now;
      autoPhaseDurationMs = AUTO_GRIPPER_SETTLE_MS;
      break;

    case AutoPhase::TurntableWaitBeforeOpen:
      Serial.println("AUTO_STEP: open gripper");
      setGripperStateFast(GRIPPER_STATE_OPEN);
      autoPhase = AutoPhase::TurntableWaitGripperOpen;
      autoPhaseStartMs = now;
      autoPhaseDurationMs = 0;
      break;

    case AutoPhase::TurntableWaitGripperOpen:
      if (gripperMotionActive) {
        autoPhaseStartMs = now;
        autoPhaseDurationMs = GRIPPER_OPEN_UPDATE_MS;
        break;
      }
      Serial.printf("AUTO_STEP: move M5 to top pos %ld\n", (long)M5_TOP_MOTOR5);
      setMotorAbsPosition(4, M5_TOP_MOTOR5, POSITION_SPEED_M5_AUTO);
      autoPhase = AutoPhase::TurntableMoveTopAfterOpen;
      autoPhaseStartMs = now;
      autoPhaseDurationMs = estimateMotor5MoveTime(TURNTABLE_MOTOR5, M5_TOP_MOTOR5);
      break;

    case AutoPhase::TurntableMoveTopAfterOpen:
      Serial.printf("AUTO_STEP: step M6 tray to %ld\n", (long)(motor6Pos + AUTO_TURNTABLE_M6_STEP));
      setMotorAbsPosition(5, motor6Pos + AUTO_TURNTABLE_M6_STEP);
      autoPhase = AutoPhase::TurntableStepTray;
      autoPhaseStartMs = now;
      autoPhaseDurationMs = AUTO_M6_STEP_WAIT_MS;
      break;

    case AutoPhase::TurntableStepTray:
      Serial.println("AUTO_STEP: rotate S1 to pre-clamp and push S4 to limit");
      setServoAngle(1, PRE_CLAMP_SERVO0);
      setServoAngle(4, PRE_DROP_SERVO3);
      autoPhase = AutoPhase::TurntableRotatePreClamp;
      autoPhaseStartMs = now;
      autoPhaseDurationMs = AUTO_SERVO_SETTLE_MS;
      break;

    case AutoPhase::TurntableRotatePreClamp:
      Serial.printf("AUTO_STEP: move M5 to pre-clamp pos %ld\n", (long)PRE_CLAMP_MOTOR5);
      setMotorAbsPosition(4, PRE_CLAMP_MOTOR5, POSITION_SPEED_M5_AUTO);
      autoPhase = AutoPhase::TurntableMovePreClamp;
      autoPhaseStartMs = now;
      autoPhaseDurationMs = estimateMotor5MoveTime(M5_TOP_MOTOR5, PRE_CLAMP_MOTOR5);
      break;

    case AutoPhase::TurntableMovePreClamp:
      currentAutoState = 1;
      autoRunning = false;
      autoPhase = AutoPhase::Idle;
      Serial.println("AUTO: turntable macro finished at pre-clamp");
      break;

    case AutoPhase::PreDropMoveTopForOpen:
      Serial.println("AUTO_STEP: confirm gripper open");
      setGripperStateFast(GRIPPER_STATE_OPEN);
      autoPhase = AutoPhase::PreDropWaitGripperOpen;
      autoPhaseStartMs = now;
      autoPhaseDurationMs = 0;
      break;

    case AutoPhase::PreDropWaitGripperOpen:
      if (gripperMotionActive) {
        autoPhaseStartMs = now;
        autoPhaseDurationMs = GRIPPER_OPEN_UPDATE_MS;
        break;
      }
      Serial.printf("AUTO_STEP: set S4 inward to turntable angle %d\n", TURNTABLE_SERVO3);
      setServoAngle(4, TURNTABLE_SERVO3);
      autoPhase = AutoPhase::PreDropSetS4ToTurntable;
      autoPhaseStartMs = now;
      autoPhaseDurationMs = AUTO_SERVO_SETTLE_MS;
      break;

    case AutoPhase::PreDropSetS4ToTurntable:
      Serial.printf("AUTO_STEP: set S1 to turntable angle %d\n", TURNTABLE_SERVO0);
      setServoAngle(1, TURNTABLE_SERVO0);
      autoPhase = AutoPhase::PreDropSetS1ToTurntable;
      autoPhaseStartMs = now;
      autoPhaseDurationMs = PRE_DROP_S1_TO_CLAMP_SETTLE_MS;
      break;

    case AutoPhase::PreDropSetS1ToTurntable:
      Serial.printf("AUTO_STEP: move M5 to turntable clamp pos %ld\n", (long)CH2_LOW_AUTO_TARGET_MOTOR5);
      setMotorAbsPosition(4, CH2_LOW_AUTO_TARGET_MOTOR5, POSITION_SPEED_M5_AUTO);
      autoPhase = AutoPhase::PreDropMoveToTurntableClamp;
      autoPhaseStartMs = now;
      autoPhaseDurationMs = PRE_DROP_M5_CLAMP_BEFORE_CLOSE_MS;
      break;

    case AutoPhase::PreDropMoveToTurntableClamp:
      Serial.println("AUTO_STEP: close gripper");
      setGripperStateFast(GRIPPER_STATE_CLOSED);
      autoPhase = AutoPhase::PreDropWaitGripperClose;
      autoPhaseStartMs = now;
      autoPhaseDurationMs = 0;
      break;

    case AutoPhase::PreDropWaitGripperClose:
      if (gripperMotionActive) {
        autoPhaseStartMs = now;
        autoPhaseDurationMs = GRIPPER_CLOSE_UPDATE_MS;
        break;
      }
      Serial.println("AUTO_STEP: wait after close gripper");
      autoPhase = AutoPhase::PreDropWaitAfterClose;
      autoPhaseStartMs = now;
      autoPhaseDurationMs = PRE_DROP_AFTER_CLOSE_WAIT_MS;
      break;

    case AutoPhase::PreDropWaitAfterClose:
      Serial.printf("AUTO_STEP: move M5 to top pos %ld\n", (long)M5_TOP_MOTOR5);
      setMotorAbsPosition(4, M5_TOP_MOTOR5, POSITION_SPEED_M5_AUTO);
      autoPhase = AutoPhase::PreDropMoveTopAfterClamp;
      autoPhaseStartMs = now;
      autoPhaseDurationMs = estimateMotor5MoveTime(CH2_LOW_AUTO_TARGET_MOTOR5, M5_TOP_MOTOR5);
      break;

    case AutoPhase::PreDropMoveTopAfterClamp:
      Serial.printf("AUTO_STEP: set S1 to pre-drop angle %d\n", PRE_DROP_SERVO0);
      setServoAngle(1, PRE_DROP_SERVO0);
      autoPhase = AutoPhase::PreDropSetS1ToDrop;
      autoPhaseStartMs = now;
      autoPhaseDurationMs = AUTO_SERVO_SETTLE_MS;
      break;

    case AutoPhase::PreDropSetS1ToDrop:
      Serial.printf("AUTO_STEP: set S4 outward to pre-drop angle %d\n", PRE_DROP_SERVO3);
      setServoAngle(4, PRE_DROP_SERVO3);
      autoPhase = AutoPhase::PreDropSetS4ToDrop;
      autoPhaseStartMs = now;
      autoPhaseDurationMs = AUTO_SERVO_SETTLE_MS;
      break;

    case AutoPhase::PreDropSetS4ToDrop:
      Serial.printf("AUTO_STEP: move M5 to pre-drop pos %ld\n", (long)PRE_DROP_MOTOR5);
      setMotorAbsPosition(4, PRE_DROP_MOTOR5, POSITION_SPEED_M5_AUTO);
      autoPhase = AutoPhase::PreDropMoveToDrop;
      autoPhaseStartMs = now;
      autoPhaseDurationMs = estimateMotor5MoveTime(M5_TOP_MOTOR5, PRE_DROP_MOTOR5);
      break;

    case AutoPhase::PreDropMoveToDrop:
      Serial.println("AUTO_STEP: slow open gripper at pre-drop");
      setGripperStateSlow(GRIPPER_STATE_OPEN);
      autoPhase = AutoPhase::PreDropOpenAtDrop;
      autoPhaseStartMs = now;
      autoPhaseDurationMs = 0;
      break;

    case AutoPhase::PreDropOpenAtDrop:
      if (gripperMotionActive) {
        autoPhaseStartMs = now;
        autoPhaseDurationMs = GRIPPER_OPEN_UPDATE_MS;
        break;
      }
      autoPhase = AutoPhase::PreDropWaitOpenAtDrop;
      autoPhaseStartMs = now;
      autoPhaseDurationMs = 0;
      break;

    case AutoPhase::PreDropWaitOpenAtDrop:
      currentAutoState = 3;
      autoRunning = false;
      autoPhase = AutoPhase::Idle;
      Serial.println("AUTO: pre-drop merged macro finished");
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
    s4 = AUTO_PRE_CLAMP_S4_AFTER_LIFT;
  } else if (state == 2) {
    m5 = TURNTABLE_MOTOR5;
    s1 = TURNTABLE_SERVO0;
    s4 = AUTO_TURNTABLE_S4_AFTER_LIFT;
  } else if (state == 3) {
    m5 = PRE_DROP_MOTOR5;
    s1 = PRE_DROP_SERVO0;
    s4 = AUTO_PRE_DROP_S4_AFTER_LIFT;
  } else {
    m5 = CH2_LOW_AUTO_TARGET_MOTOR5;
    s1 = TURNTABLE_SERVO0;
    s4 = AUTO_TURNTABLE_S4_AFTER_LIFT;
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
  if (channel(RC_CH5_INDEX) >= RC_OUT_MID && channel(RC_CH6_INDEX) < RC_OUT_MID) mode = "FINE";
  if (channel(RC_CH5_INDEX) >= RC_OUT_MID && channel(RC_CH6_INDEX) >= RC_OUT_MID) mode = "AUTO";

  Serial.printf("CH1=%d CH2=%d CH3=%d CH4=%d CH5=%d CH6=%d mode=%s failsafe=%d auto=%d M1=%d M2=%d M3=%d M4=%d M5=%ld M6=%ld S1=%d S4=%d gripState=%u gripAngle=%d\n",
                channel(RC_CH1_INDEX), channel(RC_CH2_INDEX), channel(RC_CH3_INDEX),
                channel(RC_CH4_INDEX), channel(RC_CH5_INDEX), channel(RC_CH6_INDEX),
                mode, failsafeTriggered || rcFailsafeFlag || rcLostFlag, autoRunning,
                lastWheelCmd[0], lastWheelCmd[1], lastWheelCmd[2], lastWheelCmd[3],
                (long)motor5Pos, (long)motor6Pos, panAngle, armServoAngle, gripperState, gripperAngle);
}

