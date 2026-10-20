/*
  ARDUINO_PIPELINE_FIRMWARE.ino

  Arduino Uno replacement firmware for opensurgbot_pipeline/main.py.

  This sketch speaks the existing STM32-style serial protocol used by:
    opensurgbot_pipeline/main.py
    opensurgbot_pipeline/lib/opensurgbot_pipeline.py
    opensurgbot_pipeline/protocol_description.md

  Hardware:
    - Arduino Uno
    - CAN-BUS Shield V1.2 / MCP2515
    - 4 servos
    - 1 ZDT stepper driver over CAN

  PC serial:
    - COM3 from Python main.py
    - 115200 baud

  Required Arduino libraries:
    - Servo
    - mcp_can

  Protocol responses:
    250       command ack
    249       command success
    248,0     invalid command
    248,1     invalid params / processing error
    248,10    timeout
*/

#include <SPI.h>
#include <mcp_can.h>
#include <Servo.h>
#include <stdlib.h>

// ===================== CONFIG =====================

const byte CAN_CS_PIN = 10;
const byte ZDT_MOTOR_ID = 0x01;
const unsigned long ZDT_CAN_BASE_ID = ((unsigned long)ZDT_MOTOR_ID) << 8;
const byte ZDT_CHECKSUM_6B = 0x6B;

const byte SERVO_COUNT = 4;
const byte SERVO_PINS[SERVO_COUNT] = {3, 5, 9, 7}; // D10 is CAN CS; D11-D13 are SPI; D6 is unavailable on this build.

// STM32 firmware used angle range -150..+150 with 2.5%..12.5% duty.
// On Arduino, map that to microseconds around neutral.
const float SERVO_MIN_LOGICAL_DEG = -150.0f;
const float SERVO_MAX_LOGICAL_DEG = 150.0f;
const int SERVO_CENTER_US = 1500;
const int SERVO_US_PER_150_DEG = 1000;

// Start conservative. If your servos/mechanism need wider range, expand carefully.
const int SERVO_SAFE_MIN_US = 800;
const int SERVO_SAFE_MAX_US = 2200;

const float SERVO_OFFSETS[SERVO_COUNT] = {
  10.0f,
  -5.0f,
  -5.0f,
  3.0f
};

const unsigned long SERIAL_READ_TIMEOUT_MS = 80;
const unsigned long DEFAULT_ZDT_REPLY_TIMEOUT_MS = 600;
const unsigned long SERVO_STEP_DELAY_MS = 5;

// Software home tracking:
// At every Arduino startup, the current ZDT position is treated as software zero.
// All relative ZDT moves update this counter. Command 199 moves back by the
// accumulated offset, so the motor returns to the startup position. Python may
// also pass its saved step count to command 199 after a serial reset.
const unsigned long ZDT_RETURN_HOME_TIMEOUT_MS = 60000;

// ZDT linear-axis calibration for Arduino-side debugging commands.
// Update ZDT_LEAD_MM_PER_REV after measuring your real screw/actuator travel.
const float ZDT_LEAD_MM_PER_REV = 1.0f;
const float ZDT_MOTOR_STEP_DEG = 1.8f;
const unsigned int ZDT_MICROSTEPS = 16;
const float ZDT_MAX_TRAVEL_CM = 13.0f;
const float ZDT_ZERO_GUARD_MM = 1.0f;
const float ZDT_POSITION_TOLERANCE_MM = 1.0f;

MCP_CAN CAN(CAN_CS_PIN);
Servo servos[SERVO_COUNT];
float currentServoAngles[SERVO_COUNT] = {0, 0, 0, 0};
long zdtCurrentPositionPulses = 0;
long zdtDriverZeroPositionPulses = 0;
bool hasZdtDriverZeroPosition = false;

String serialBuffer;

bool zdtStartPositionMove(bool direction, unsigned long pulseCount, unsigned int speedRpm, byte accelLevel);
bool zdtStartMoveToSoftwareTarget(unsigned long targetPulses, unsigned int speedRpm, byte accelLevel, bool *willMove, long *signedDelta);
bool zdtTargetInRange(unsigned long targetPulses);
bool zdtDeltaStaysInRange(long signedDelta);
bool zdtGuardedTargetInRange(long targetPulses);
bool zdtGuardedDeltaStaysInRange(long signedDelta);
bool zdtReturnToSavedHome(long savedPositionPulses, unsigned int speedRpm, byte accelLevel, unsigned long timeoutMs);
bool waitForZdtDone(unsigned long pulseCount, unsigned int speedRpm, unsigned long timeoutMs);
bool zdtReadDriverPosition(long *positionPulses, unsigned long timeoutMs);
bool zdtValidateExpectedPosition(long expectedRelativePulses);
unsigned long zdtMmToPulses(float travelMm);

// ===================== SERIAL RESPONSES =====================

void notifyAck() {
  Serial.println(F("250"));
}

void notifySuccess() {
  Serial.println(F("249"));
}

void notifyError(byte errorCode) {
  Serial.print(F("248,"));
  Serial.println(errorCode);
}

// ===================== SERVO HELPERS =====================

int servoAngleToUs(float logicalAngleDeg) {
  logicalAngleDeg = constrain(logicalAngleDeg, SERVO_MIN_LOGICAL_DEG, SERVO_MAX_LOGICAL_DEG);
  float us = SERVO_CENTER_US + logicalAngleDeg * ((float)SERVO_US_PER_150_DEG / 150.0f);
  return constrain((int)round(us), SERVO_SAFE_MIN_US, SERVO_SAFE_MAX_US);
}

void writeServoAngle(byte servoIndex, float logicalAngleDeg) {
  if (servoIndex >= SERVO_COUNT) {
    return;
  }

  logicalAngleDeg = constrain(logicalAngleDeg, SERVO_MIN_LOGICAL_DEG, SERVO_MAX_LOGICAL_DEG);
  servos[servoIndex].writeMicroseconds(servoAngleToUs(logicalAngleDeg));
  currentServoAngles[servoIndex] = logicalAngleDeg;
}

bool gotoServoAngle(byte servoIndex, float targetAngleDeg, float maxSpeedDegPerSec) {
  if (servoIndex >= SERVO_COUNT) {
    return false;
  }
  if (targetAngleDeg < SERVO_MIN_LOGICAL_DEG || targetAngleDeg > SERVO_MAX_LOGICAL_DEG) {
    return false;
  }

  maxSpeedDegPerSec = max(maxSpeedDegPerSec, 1.0f);
  float startAngle = currentServoAngles[servoIndex];
  float delta = targetAngleDeg - startAngle;
  unsigned long steps = (unsigned long)round(abs(delta) / maxSpeedDegPerSec / (SERVO_STEP_DELAY_MS / 1000.0f));

  if (steps < 1) {
    writeServoAngle(servoIndex, targetAngleDeg);
    return true;
  }

  for (unsigned long step = 1; step <= steps; step++) {
    float progress = (float)step / (float)steps;
    writeServoAngle(servoIndex, startAngle + delta * progress);
    delay(SERVO_STEP_DELAY_MS);
  }

  writeServoAngle(servoIndex, targetAngleDeg);
  return true;
}

bool groupGotoServos(const float targetAngles[SERVO_COUNT], float maxSpeedDegPerSec) {
  for (byte i = 0; i < SERVO_COUNT; i++) {
    if (targetAngles[i] < SERVO_MIN_LOGICAL_DEG || targetAngles[i] > SERVO_MAX_LOGICAL_DEG) {
      return false;
    }
  }

  maxSpeedDegPerSec = max(maxSpeedDegPerSec, 1.0f);

  float startAngles[SERVO_COUNT];
  float deltas[SERVO_COUNT];
  unsigned long maxSteps = 1;

  for (byte i = 0; i < SERVO_COUNT; i++) {
    startAngles[i] = currentServoAngles[i];
    deltas[i] = targetAngles[i] - startAngles[i];
    unsigned long steps = (unsigned long)round(abs(deltas[i]) / maxSpeedDegPerSec / (SERVO_STEP_DELAY_MS / 1000.0f));
    if (steps > maxSteps) {
      maxSteps = steps;
    }
  }

  for (unsigned long step = 1; step <= maxSteps; step++) {
    float progress = (float)step / (float)maxSteps;
    for (byte i = 0; i < SERVO_COUNT; i++) {
      writeServoAngle(i, startAngles[i] + deltas[i] * progress);
    }
    delay(SERVO_STEP_DELAY_MS);
  }

  for (byte i = 0; i < SERVO_COUNT; i++) {
    writeServoAngle(i, targetAngles[i]);
  }
  return true;
}

bool homeServos(float maxSpeedDegPerSec) {
  float homeTargets[SERVO_COUNT];
  for (byte i = 0; i < SERVO_COUNT; i++) {
    homeTargets[i] = SERVO_OFFSETS[i];
  }
  return groupGotoServos(homeTargets, maxSpeedDegPerSec);
}

// ===================== CAN / ZDT HELPERS =====================

bool sendZdtFrame(unsigned long idOffset, const byte *data, byte len) {
  unsigned long canId = ZDT_CAN_BASE_ID + idOffset;
  byte result = CAN.sendMsgBuf(canId, 1, len, (byte *)data);
  delay(8);
  return result == CAN_OK;
}

bool waitForZdtReply(byte expectedCommand, unsigned long timeoutMs, bool acceptReached) {
  unsigned long start = millis();

  while (millis() - start < timeoutMs) {
    if (CAN_MSGAVAIL == CAN.checkReceive()) {
      unsigned long rxId = 0;
      byte len = 0;
      byte buf[8];
      CAN.readMsgBuf(&rxId, &len, buf);

      if (len >= 2 && buf[0] == 0x00 && buf[1] == 0xEE) {
        return false;
      }
      if (len >= 2 && buf[0] == expectedCommand && buf[1] == 0xE2) {
        return false;
      }
      if (len >= 2 && buf[0] == expectedCommand && buf[1] == 0x02) {
        return true;
      }
      if (acceptReached && len >= 2 && buf[0] == expectedCommand && buf[1] == 0x9F) {
        return true;
      }
    }
  }

  return false;
}

bool waitForZdtReached(unsigned long timeoutMs) {
  unsigned long start = millis();

  while (millis() - start < timeoutMs) {
    if (CAN_MSGAVAIL == CAN.checkReceive()) {
      unsigned long rxId = 0;
      byte len = 0;
      byte buf[8];
      CAN.readMsgBuf(&rxId, &len, buf);

      if (len >= 2 && buf[0] == 0xFD && buf[1] == 0x9F) {
        return true;
      }
      if (len >= 2 && buf[0] == 0x00 && buf[1] == 0xEE) {
        return false;
      }
      if (len >= 2 && buf[0] == 0xFD && buf[1] == 0xE2) {
        return false;
      }
    }
  }

  return false;
}

unsigned long estimateZdtMoveTimeMs(unsigned long pulseCount, unsigned int speedRpm) {
  if (pulseCount == 0) {
    return 0;
  }
  if (speedRpm == 0) {
    return 0xFFFFFFFFUL;
  }

  float pulsesPerRev = (360.0f / ZDT_MOTOR_STEP_DEG) * (float)ZDT_MICROSTEPS;
  float revolutions = (float)pulseCount / pulsesPerRev;
  float moveMs = revolutions * 60000.0f / (float)speedRpm;
  return (unsigned long)moveMs + 1500UL;
}

bool waitForZdtDone(unsigned long pulseCount, unsigned int speedRpm, unsigned long timeoutMs) {
  unsigned long start = millis();
  unsigned long estimatedMs = estimateZdtMoveTimeMs(pulseCount, speedRpm);

  while (millis() - start < timeoutMs) {
    if (CAN_MSGAVAIL == CAN.checkReceive()) {
      unsigned long rxId = 0;
      byte len = 0;
      byte buf[8];
      CAN.readMsgBuf(&rxId, &len, buf);

      if (len >= 2 && buf[0] == 0xFD && buf[1] == 0x9F) {
        return true;
      }
      if (len >= 2 && buf[0] == 0x00 && buf[1] == 0xEE) {
        return false;
      }
      if (len >= 2 && buf[0] == 0xFD && buf[1] == 0xE2) {
        return false;
      }
    }

    if (estimatedMs != 0xFFFFFFFFUL && millis() - start >= estimatedMs) {
      return true;
    }
  }

  return false;
}

bool zdtReadDriverPosition(long *positionPulses, unsigned long timeoutMs) {
  byte cmd[] = {
    0x36, ZDT_CHECKSUM_6B
  };

  if (!sendZdtFrame(0, cmd, sizeof(cmd))) {
    return false;
  }

  unsigned long start = millis();
  while (millis() - start < timeoutMs) {
    if (CAN_MSGAVAIL == CAN.checkReceive()) {
      unsigned long rxId = 0;
      byte len = 0;
      byte buf[8];
      CAN.readMsgBuf(&rxId, &len, buf);

      if (len >= 2 && buf[0] == 0x00 && buf[1] == 0xEE) {
        return false;
      }

      if (len >= 7 && buf[0] == 0x36) {
        byte direction = buf[1];
        unsigned long magnitude = ((unsigned long)buf[2] << 24) |
                                  ((unsigned long)buf[3] << 16) |
                                  ((unsigned long)buf[4] << 8) |
                                  ((unsigned long)buf[5]);
        *positionPulses = direction == 0 ? (long)magnitude : -(long)magnitude;
        return true;
      }
    }
  }

  return false;
}

unsigned long zdtPositionTolerancePulses() {
  return zdtMmToPulses(ZDT_POSITION_TOLERANCE_MM);
}

bool zdtValidateExpectedPosition(long expectedRelativePulses) {
  if (!hasZdtDriverZeroPosition) {
    return false;
  }

  long driverPosition = 0;
  if (!zdtReadDriverPosition(&driverPosition, DEFAULT_ZDT_REPLY_TIMEOUT_MS)) {
    return false;
  }

  long actualRelative = driverPosition - zdtDriverZeroPositionPulses;
  long error = actualRelative - expectedRelativePulses;
  if (error < 0) {
    error = -error;
  }

  return (unsigned long)error <= zdtPositionTolerancePulses();
}

bool zdtEnable(bool enable) {
  byte cmd[] = {
    0xF3, 0xAB, enable ? 0x01 : 0x00, 0x00, ZDT_CHECKSUM_6B
  };
  if (!sendZdtFrame(0, cmd, sizeof(cmd))) {
    return false;
  }
  return waitForZdtReply(0xF3, DEFAULT_ZDT_REPLY_TIMEOUT_MS, false);
}

bool zdtStop() {
  byte cmd[] = {
    0xFE, 0x98, 0x00, ZDT_CHECKSUM_6B
  };
  if (!sendZdtFrame(0, cmd, sizeof(cmd))) {
    return false;
  }
  return waitForZdtReply(0xFE, DEFAULT_ZDT_REPLY_TIMEOUT_MS, false);
}

bool zdtPositionMove(bool direction, unsigned long pulseCount, unsigned int speedRpm, byte accelLevel) {
  if (!zdtStartPositionMove(direction, pulseCount, speedRpm, accelLevel)) {
    return false;
  }

  bool ok = waitForZdtDone(pulseCount, speedRpm, 10000);
  if (ok) {
    long signedDelta = direction ? -(long)pulseCount : (long)pulseCount;
    long expectedPosition = zdtCurrentPositionPulses + signedDelta;
    bool validated = zdtValidateExpectedPosition(expectedPosition);
    (void)validated;
    zdtCurrentPositionPulses = expectedPosition;
  }
  return ok;
}

bool zdtStartPositionMove(bool direction, unsigned long pulseCount, unsigned int speedRpm, byte accelLevel) {
  byte firstFrame[] = {
    0xFD,
    direction ? 0x01 : 0x00,
    (byte)((speedRpm >> 8) & 0xFF),
    (byte)(speedRpm & 0xFF),
    accelLevel,
    (byte)((pulseCount >> 24) & 0xFF),
    (byte)((pulseCount >> 16) & 0xFF),
    (byte)((pulseCount >> 8) & 0xFF)
  };

  byte secondFrame[] = {
    0xFD,
    (byte)(pulseCount & 0xFF),
    0x00, // relative move
    0x00, // multi-machine flag off
    ZDT_CHECKSUM_6B
  };

  if (!sendZdtFrame(0, firstFrame, sizeof(firstFrame))) {
    return false;
  }
  if (!sendZdtFrame(1, secondFrame, sizeof(secondFrame))) {
    return false;
  }

  return waitForZdtReply(0xFD, DEFAULT_ZDT_REPLY_TIMEOUT_MS, false);
}

bool zdtMoveToSoftwareTarget(unsigned long targetPulses, unsigned int speedRpm, byte accelLevel) {
  if (!zdtGuardedTargetInRange((long)targetPulses)) {
    return false;
  }

  long target = (long)targetPulses;
  long delta = target - zdtCurrentPositionPulses;

  if (delta == 0) {
    return true;
  }

  bool direction = delta < 0;
  unsigned long pulseCount = delta < 0 ? (unsigned long)(-delta) : (unsigned long)delta;
  return zdtPositionMove(direction, pulseCount, speedRpm, accelLevel);
}

bool zdtStartMoveToSoftwareTarget(unsigned long targetPulses, unsigned int speedRpm, byte accelLevel, bool *willMove, long *signedDelta) {
  if (!zdtGuardedTargetInRange((long)targetPulses)) {
    *willMove = false;
    *signedDelta = 0;
    return false;
  }

  long target = (long)targetPulses;
  long delta = target - zdtCurrentPositionPulses;

  *signedDelta = delta;
  *willMove = delta != 0;
  if (delta == 0) {
    return true;
  }

  bool direction = delta < 0;
  unsigned long pulseCount = delta < 0 ? (unsigned long)(-delta) : (unsigned long)delta;
  return zdtStartPositionMove(direction, pulseCount, speedRpm, accelLevel);
}

unsigned long zdtMmToPulses(float travelMm) {
  float motorStepsPerRev = 360.0f / ZDT_MOTOR_STEP_DEG;
  float pulsesPerRev = motorStepsPerRev * (float)ZDT_MICROSTEPS;
  float pulsesPerMm = pulsesPerRev / ZDT_LEAD_MM_PER_REV;
  return (unsigned long)round(abs(travelMm) * pulsesPerMm);
}

unsigned long zdtMaxTravelPulses() {
  return zdtMmToPulses(ZDT_MAX_TRAVEL_CM * 10.0f);
}

unsigned long zdtZeroGuardPulses() {
  return zdtMmToPulses(ZDT_ZERO_GUARD_MM);
}

bool zdtTargetInRange(unsigned long targetPulses) {
  return targetPulses <= zdtMaxTravelPulses();
}

bool zdtDeltaStaysInRange(long signedDelta) {
  long target = zdtCurrentPositionPulses + signedDelta;
  return target >= 0 && (unsigned long)target <= zdtMaxTravelPulses();
}

bool zdtGuardedTargetInRange(long targetPulses) {
  return targetPulses > (long)zdtZeroGuardPulses() &&
         targetPulses <= (long)zdtMaxTravelPulses();
}

bool zdtGuardedDeltaStaysInRange(long signedDelta) {
  long target = zdtCurrentPositionPulses + signedDelta;
  return zdtGuardedTargetInRange(target);
}

bool zdtMoveMillimeters(bool direction, float travelMm, unsigned int speedRpm, byte accelLevel) {
  unsigned long pulseCount = zdtMmToPulses(travelMm);
  if (pulseCount == 0) {
    return true;
  }
  long signedDelta = direction ? -(long)pulseCount : (long)pulseCount;
  if (!zdtGuardedDeltaStaysInRange(signedDelta)) {
    return false;
  }
  return zdtPositionMove(direction, pulseCount, speedRpm, accelLevel);
}

bool zdtReturnToStartupHome(unsigned int speedRpm, byte accelLevel, unsigned long timeoutMs) {
  return zdtReturnToSavedHome(zdtCurrentPositionPulses, speedRpm, accelLevel, timeoutMs);
}

bool zdtReturnToSavedHome(long savedPositionPulses, unsigned int speedRpm, byte accelLevel, unsigned long timeoutMs) {
  if (savedPositionPulses == 0) {
    zdtCurrentPositionPulses = 0;
    return true;
  }

  bool returnDirection = savedPositionPulses > 0;
  unsigned long returnPulses = savedPositionPulses < 0
                                 ? (unsigned long)(-savedPositionPulses)
                                 : (unsigned long)savedPositionPulses;

  bool ok = zdtPositionMove(returnDirection, returnPulses, speedRpm, accelLevel);
  if (ok) {
    zdtCurrentPositionPulses = 0;
  }
  return ok;
}

bool zdtTriggerHoming(unsigned long timeoutMs) {
  // Safe software homing: return to the Arduino-tracked zero position.
  // Command 19 defines that zero at the current physical position.
  return zdtReturnToStartupHome(1200, 0, timeoutMs);
}

bool zdtSetCurrentPositionZero() {
  // Reset Arduino's software position tracker at the current physical position.
  // This is intentionally not written into the ZDT driver's persistent memory.
  long driverPosition = 0;
  if (!zdtReadDriverPosition(&driverPosition, DEFAULT_ZDT_REPLY_TIMEOUT_MS)) {
    return false;
  }

  zdtDriverZeroPositionPulses = driverPosition;
  hasZdtDriverZeroPosition = true;
  zdtCurrentPositionPulses = 0;
  return true;
}

// ===================== CSV PARSER =====================

byte splitCsv(char *line, char *fields[], byte maxFields) {
  byte count = 0;
  char *token = strtok(line, ",");

  while (token != NULL && count < maxFields) {
    fields[count++] = token;
    token = strtok(NULL, ",");
  }

  return count;
}

bool parseFloatField(char *field, float *out) {
  char *endPtr = NULL;
  float value = strtod(field, &endPtr);
  if (endPtr == field || *endPtr != '\0') {
    return false;
  }
  *out = value;
  return true;
}

bool parseUnsignedLongField(char *field, unsigned long *out) {
  char *endPtr = NULL;
  unsigned long value = strtoul(field, &endPtr, 10);
  if (endPtr == field || *endPtr != '\0') {
    return false;
  }
  *out = value;
  return true;
}

// ===================== COMMAND HANDLERS =====================

void handleServoSet(char *fields[], byte count) {
  if (count != 3) {
    notifyError(1);
    return;
  }

  unsigned long servoId;
  float angle;
  if (!parseUnsignedLongField(fields[1], &servoId) || !parseFloatField(fields[2], &angle)) {
    notifyError(1);
    return;
  }
  if (servoId >= SERVO_COUNT) {
    notifyError(1);
    return;
  }

  notifyAck();
  writeServoAngle((byte)servoId, angle + SERVO_OFFSETS[servoId]);
  notifySuccess();
}

void handleServoGoto(char *fields[], byte count) {
  if (count != 4) {
    notifyError(1);
    return;
  }

  unsigned long servoId;
  float angle;
  float speed;
  if (!parseUnsignedLongField(fields[1], &servoId) ||
      !parseFloatField(fields[2], &angle) ||
      !parseFloatField(fields[3], &speed)) {
    notifyError(1);
    return;
  }
  if (servoId >= SERVO_COUNT) {
    notifyError(1);
    return;
  }

  notifyAck();
  if (!gotoServoAngle((byte)servoId, angle + SERVO_OFFSETS[servoId], speed)) {
    notifyError(1);
    return;
  }
  notifySuccess();
}

void handleZdtEnable(char *fields[], byte count) {
  if (count != 2) {
    notifyError(1);
    return;
  }

  unsigned long enableValue;
  if (!parseUnsignedLongField(fields[1], &enableValue)) {
    notifyError(1);
    return;
  }

  notifyAck();
  if (!zdtEnable(enableValue != 0)) {
    notifyError(1);
    return;
  }
  notifySuccess();
}

void handleZdtPosition(char *fields[], byte count) {
  if (count != 6) {
    notifyError(1);
    return;
  }

  unsigned long dirValue;
  unsigned long pulseCount;
  unsigned long speed;
  unsigned long accel;
  unsigned long isRelative;

  if (!parseUnsignedLongField(fields[1], &dirValue) ||
      !parseUnsignedLongField(fields[2], &pulseCount) ||
      !parseUnsignedLongField(fields[3], &speed) ||
      !parseUnsignedLongField(fields[4], &accel) ||
      !parseUnsignedLongField(fields[5], &isRelative)) {
    notifyError(1);
    return;
  }

  notifyAck();

  if (isRelative != 0) {
    long signedDelta = dirValue != 0 ? -(long)pulseCount : (long)pulseCount;
    if (!zdtGuardedDeltaStaysInRange(signedDelta)) {
      notifyError(1);
      return;
    }
  }

  bool ok = isRelative != 0
              ? zdtPositionMove(dirValue != 0, pulseCount, (unsigned int)speed, (byte)accel)
              : zdtMoveToSoftwareTarget(pulseCount, (unsigned int)speed, (byte)accel);

  if (!ok) {
    notifyError(1);
    return;
  }
  notifySuccess();
}

void handleZdtMoveMillimeters(char *fields[], byte count) {
  if (count != 5) {
    notifyError(1);
    return;
  }

  unsigned long dirValue;
  float travelMm;
  unsigned long speed;
  unsigned long accel;

  if (!parseUnsignedLongField(fields[1], &dirValue) ||
      !parseFloatField(fields[2], &travelMm) ||
      !parseUnsignedLongField(fields[3], &speed) ||
      !parseUnsignedLongField(fields[4], &accel)) {
    notifyError(1);
    return;
  }

  notifyAck();
  if (!zdtMoveMillimeters(dirValue != 0, travelMm, (unsigned int)speed, (byte)accel)) {
    notifyError(1);
    return;
  }
  notifySuccess();
}

void handleDriveAllBlocking(char *fields[], byte count) {
  if (count != 11) {
    notifyError(1);
    return;
  }

  float servoTargets[SERVO_COUNT];
  float maxServoSpeed;
  unsigned long zdtDir;
  unsigned long zdtPulses;
  unsigned long zdtSpeed;
  unsigned long zdtAccel;
  unsigned long reachedTimeout;

  for (byte i = 0; i < SERVO_COUNT; i++) {
    float angle;
    if (!parseFloatField(fields[i + 1], &angle)) {
      notifyError(1);
      return;
    }
    servoTargets[i] = angle + SERVO_OFFSETS[i];
  }

  if (!parseFloatField(fields[5], &maxServoSpeed) ||
      !parseUnsignedLongField(fields[6], &zdtDir) ||
      !parseUnsignedLongField(fields[7], &zdtPulses) ||
      !parseUnsignedLongField(fields[8], &zdtSpeed) ||
      !parseUnsignedLongField(fields[9], &zdtAccel) ||
      !parseUnsignedLongField(fields[10], &reachedTimeout)) {
    notifyError(1);
    return;
  }

  notifyAck();

  (void)zdtDir;
  bool zdtWillMove = false;
  long zdtSignedDelta = 0;
  if (!zdtStartMoveToSoftwareTarget(zdtPulses, (unsigned int)zdtSpeed, (byte)zdtAccel, &zdtWillMove, &zdtSignedDelta)) {
    notifyError(1);
    return;
  }

  if (!groupGotoServos(servoTargets, maxServoSpeed)) {
    notifyError(1);
    return;
  }

  if (zdtWillMove) {
    unsigned long zdtMovePulses = zdtSignedDelta < 0 ? (unsigned long)(-zdtSignedDelta) : (unsigned long)zdtSignedDelta;
    if (!waitForZdtDone(zdtMovePulses, (unsigned int)zdtSpeed, reachedTimeout)) {
      notifyError(1);
      return;
    }
    long expectedPosition = zdtCurrentPositionPulses + zdtSignedDelta;
    bool validated = zdtValidateExpectedPosition(expectedPosition);
    (void)validated;
    zdtCurrentPositionPulses = expectedPosition;
  }

  notifySuccess();
}

void handleReturnHome(char *fields[], byte count) {
  unsigned long speed = 1200;
  unsigned long accel = 0;
  unsigned long timeoutMs = ZDT_RETURN_HOME_TIMEOUT_MS;
  long savedPositionPulses = zdtCurrentPositionPulses;

  if (count != 1 && count != 4 && count != 5) {
    notifyError(1);
    return;
  }

  if (count == 4 || count == 5) {
    if (!parseUnsignedLongField(fields[1], &speed) ||
        !parseUnsignedLongField(fields[2], &accel) ||
        !parseUnsignedLongField(fields[3], &timeoutMs)) {
      notifyError(1);
      return;
    }
  }

  if (count == 5) {
    char *endPtr = NULL;
    savedPositionPulses = strtol(fields[4], &endPtr, 10);
    if (endPtr == fields[4] || *endPtr != '\0') {
      notifyError(1);
      return;
    }
  }

  notifyAck();
  if (!zdtReturnToSavedHome(savedPositionPulses, (unsigned int)speed, (byte)accel, timeoutMs)) {
    notifyError(10);
    return;
  }
  if (!homeServos(100.0f)) {
    notifyError(1);
    return;
  }
  notifySuccess();
}

void handleCommand(char *line) {
  char *fields[16];
  byte count = splitCsv(line, fields, 16);
  if (count < 1) {
    return;
  }

  unsigned long commandId;
  if (!parseUnsignedLongField(fields[0], &commandId)) {
    notifyError(0);
    return;
  }

  switch (commandId) {
    case 1:
      handleServoSet(fields, count);
      break;
    case 2:
      handleServoGoto(fields, count);
      break;
    case 10:
      handleZdtEnable(fields, count);
      break;
    case 11:
      handleZdtPosition(fields, count);
      break;
    case 18:
      if (count != 1) {
        notifyError(1);
        return;
      }
      notifyAck();
      if (!zdtTriggerHoming(15000)) {
        notifyError(1);
        return;
      }
      notifySuccess();
      break;
    case 19:
      if (count != 1) {
        notifyError(1);
        return;
      }
      notifyAck();
      if (!zdtSetCurrentPositionZero()) {
        notifyError(1);
        return;
      }
      notifySuccess();
      break;
    case 101:
      handleDriveAllBlocking(fields, count);
      break;
    case 198:
      handleZdtMoveMillimeters(fields, count);
      break;
    case 199:
      handleReturnHome(fields, count);
      break;
    default:
      notifyError(0);
      break;
  }
}

void processSerialInput() {
  while (Serial.available()) {
    char ch = (char)Serial.read();

    if (ch == '\n' || ch == '\r') {
      if (serialBuffer.length() > 0) {
        char line[160];
        serialBuffer.toCharArray(line, sizeof(line));
        serialBuffer = "";
        handleCommand(line);
      }
    } else {
      if (serialBuffer.length() < 150) {
        serialBuffer += ch;
      } else {
        serialBuffer = "";
        notifyError(1);
      }
    }
  }
}

// ===================== SETUP / LOOP =====================

void setup() {
  Serial.begin(115200);

  for (byte i = 0; i < SERVO_COUNT; i++) {
    servos[i].attach(SERVO_PINS[i], SERVO_SAFE_MIN_US, SERVO_SAFE_MAX_US);
    writeServoAngle(i, SERVO_OFFSETS[i]);
  }

  while (CAN_OK != CAN.begin(MCP_ANY, CAN_500KBPS, MCP_16MHZ)) {
    delay(1000);
  }
  CAN.setMode(MCP_NORMAL);

  // Match STM32 startup behavior: enable ZDT, then treat current physical ZDT position as software zero.
  zdtEnable(true);
  zdtSetCurrentPositionZero();
  Serial.println(F("255"));
}

void loop() {
  processSerialInput();
}
