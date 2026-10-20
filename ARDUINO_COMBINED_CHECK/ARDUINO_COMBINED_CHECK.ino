/*
  ARDUINO_COMBINED_CHECK.ino

  Combined safety/check sketch for:
    - Arduino Uno
    - 4 hobby servos
    - CAN-BUS Shield V1.2 / MCP2515
    - 1 ZDT stepper driver over CAN

  Wiring summary:
    Servo 1 signal -> D3
    Servo 2 signal -> D5
    Servo 3 signal -> D9
    Servo 4 signal -> D7
    Servo 1,2 power    -> external 5V/6V servo PSU
    Servo 3,4 power    -> Arduino 5V
    Servo GND      -> common GND

    CAN shield CAN_H -> ZDT CAN_H
    CAN shield CAN_L -> ZDT CAN_L
    CAN shield GND   -> common GND / ZDT GND
    ZDT power        -> programmable DC supply

  ZDT menu:
    P_Serial = CAN / CAN1_MAP
    CAN_Baud = 500K
    ID_Addr  = 1
    Checksum = 0x6B
    Response = Both

  Serial Monitor: 115200 baud
*/

#include <SPI.h>
#include <mcp_can.h>
#include <Servo.h>

// ===================== CAN / ZDT CONFIG =====================

const byte CAN_CS_PIN = 10;              // CAN shield chip-select uses D10. Do not attach a servo to D10.
const byte ZDT_MOTOR_ID = 0x01;
const unsigned long ZDT_CAN_BASE_ID = ((unsigned long)ZDT_MOTOR_ID) << 8; // ID 1 -> 0x100
const byte ZDT_CHECKSUM_6B = 0x6B;

// ===================== ZDT DISTANCE CALIBRATION =====================
// These match the Python pipeline values:
//   ZDTStepPerTurn = 0.1 cm per full motor revolution
//   ZDTStepRot = 1.8 degrees per full motor step
//   ZDTMicrostep = 16 microsteps per full step
// Formula:
//   pulses = distance_mm * (360 / motor_step_deg) * microsteps / lead_mm_per_rev
const float ZDT_STEP_PER_TURN_CM = 0.1f;
const float ZDT_LEAD_MM_PER_REV = ZDT_STEP_PER_TURN_CM * 10.0f;
const float ZDT_MOTOR_STEP_DEG = 1.8f;
const unsigned int ZDT_MICROSTEPS = 16;
const unsigned int ZDT_DEFAULT_SPEED_RPM = 1200;
const byte ZDT_DEFAULT_ACCEL = 5;

MCP_CAN CAN(CAN_CS_PIN);

// ===================== SERVO CONFIG =====================

const byte SERVO_COUNT = 4;
const byte SERVO_PINS[SERVO_COUNT] = {3, 5, 9, 7};

const int SERVO_CENTER = 90;
const int SERVO_SAFE_LOW = 50;
const int SERVO_SAFE_HIGH = 180;

// Match current tested sketch values. Tune after cable/mechanical setup.
const int SERVO_OFFSETS[SERVO_COUNT] = {
  -5,
   1,
  -2,
  -6
};

const unsigned int SERVO_STEP_DELAY_MS = 25;
const unsigned int SERVO_HOLD_MS = 600;

Servo servos[SERVO_COUNT];
int servoLogicalAngles[SERVO_COUNT] = {
  SERVO_CENTER,
  SERVO_CENTER,
  SERVO_CENTER,
  SERVO_CENTER
};

unsigned long lastZdtPositionMagnitudePulses = 0;
byte lastZdtPositionDirection = 0;
unsigned long zdtSoftwareZeroMagnitudePulses = 0;
byte zdtSoftwareZeroDirection = 0;
bool hasZdtSoftwareZero = false;
bool hasZdtPosition = false;
String serialCommandBuffer;
unsigned long lastSerialInputMs = 0;

void printHelp();
void zdtForwardBackTest();
void zdtReadCurrentPosition();
void zdtSetDriverCurrentPositionAsHomingZero(bool storeToDriver);
void printJointState();
void fullSafeCheck();
float zdtPulsesToDistanceMm(unsigned long pulses);

// ===================== HELPERS =====================

int calibratedServoAngle(byte servoIndex, int logicalAngle) {
  return constrain(logicalAngle + SERVO_OFFSETS[servoIndex], 0, 180);
}

void writeServoLogical(byte servoIndex, int logicalAngle) {
  logicalAngle = constrain(logicalAngle, SERVO_SAFE_LOW, SERVO_SAFE_HIGH);
  int outputAngle = calibratedServoAngle(servoIndex, logicalAngle);
  servos[servoIndex].write(outputAngle);
  servoLogicalAngles[servoIndex] = logicalAngle;

  Serial.print(F("Servo "));
  Serial.print(servoIndex + 1);
  Serial.print(F(" logical="));
  Serial.print(logicalAngle);
  Serial.print(F(" output="));
  Serial.println(outputAngle);
}

void moveServoSmooth(byte servoIndex, int targetLogicalAngle) {
  targetLogicalAngle = constrain(targetLogicalAngle, SERVO_SAFE_LOW, SERVO_SAFE_HIGH);

  while (servoLogicalAngles[servoIndex] != targetLogicalAngle) {
    if (servoLogicalAngles[servoIndex] < targetLogicalAngle) {
      writeServoLogical(servoIndex, servoLogicalAngles[servoIndex] + 1);
    } else {
      writeServoLogical(servoIndex, servoLogicalAngles[servoIndex] - 1);
    }
    delay(SERVO_STEP_DELAY_MS);
  }
}

void moveAllServosSmooth(int targetLogicalAngle) {
  targetLogicalAngle = constrain(targetLogicalAngle, SERVO_SAFE_LOW, SERVO_SAFE_HIGH);
  bool moving = true;

  while (moving) {
    moving = false;

    for (byte i = 0; i < SERVO_COUNT; i++) {
      if (servoLogicalAngles[i] < targetLogicalAngle) {
        writeServoLogical(i, servoLogicalAngles[i] + 1);
        moving = true;
      } else if (servoLogicalAngles[i] > targetLogicalAngle) {
        writeServoLogical(i, servoLogicalAngles[i] - 1);
        moving = true;
      }
    }

    delay(SERVO_STEP_DELAY_MS);
  }
}

void centerAllServos() {
  Serial.println(F("Centering all servos."));
  moveAllServosSmooth(SERVO_CENTER);
}

void servoOneAtATimeTest() {
  Serial.println(F("Testing servos one at a time."));

  for (byte i = 0; i < SERVO_COUNT; i++) {
    Serial.print(F("Testing Servo "));
    Serial.println(i + 1);
    moveServoSmooth(i, SERVO_SAFE_LOW);
    delay(SERVO_HOLD_MS);
    moveServoSmooth(i, SERVO_SAFE_HIGH);
    delay(SERVO_HOLD_MS);
    moveServoSmooth(i, SERVO_CENTER);
    delay(SERVO_HOLD_MS);
  }
}

void servoAllWaveTest() {
  Serial.println(F("Testing all servos together."));
  moveAllServosSmooth(SERVO_SAFE_LOW);
  delay(SERVO_HOLD_MS);
  moveAllServosSmooth(SERVO_SAFE_HIGH);
  delay(SERVO_HOLD_MS);
  moveAllServosSmooth(SERVO_CENTER);
}

void printRawCanMessage(unsigned long rxId, byte len, const byte *buf) {
  Serial.print(F("RX ID 0x"));
  Serial.print(rxId, HEX);
  Serial.print(F(" len "));
  Serial.print(len);
  Serial.print(F(" data:"));
  for (byte i = 0; i < len; i++) {
    Serial.print(F(" 0x"));
    if (buf[i] < 0x10) Serial.print('0');
    Serial.print(buf[i], HEX);
  }
  Serial.println();
}

bool parseZdtPositionReply(byte len, const byte *buf) {
  if (len < 7 || buf[0] != 0x36) {
    return false;
  }

  byte direction = buf[1];
  unsigned long magnitude = ((unsigned long)buf[2] << 24) |
                            ((unsigned long)buf[3] << 16) |
                            ((unsigned long)buf[4] << 8) |
                            ((unsigned long)buf[5]);

  lastZdtPositionMagnitudePulses = magnitude;
  lastZdtPositionDirection = direction;
  hasZdtPosition = true;

  Serial.print(F("ZDT direction byte: "));
  Serial.println(direction);
  Serial.print(F("ZDT current position magnitude pulses: "));
  Serial.println(magnitude);
  Serial.print(F("ZDT current position approx mm from driver zero: "));
  Serial.println(zdtPulsesToDistanceMm(magnitude), 3);
  Serial.print(F("ZDT current position approx cm from driver zero: "));
  Serial.println(zdtPulsesToDistanceMm(magnitude) / 10.0f, 3);
  if (hasZdtSoftwareZero) {
    double relativePulses = zdtRelativeToSoftwareZeroPulses();
    float relativeMm = zdtPulsesToDistanceMm((unsigned long)abs(relativePulses));
    if (relativePulses < 0) {
      relativeMm = -relativeMm;
    }
    Serial.print(F("ZDT position relative to software zero pulses: "));
    Serial.println(relativePulses, 0);
    Serial.print(F("ZDT position relative to software zero mm: "));
    Serial.println(relativeMm, 3);
  } else {
    Serial.println(F("ZDT software zero not set. Send 0 after z at your chosen home."));
  }
  return true;
}

void readCanReplies(unsigned long timeoutMs) {
  unsigned long start = millis();
  bool sawMessage = false;

  while (millis() - start < timeoutMs) {
    if (CAN_MSGAVAIL == CAN.checkReceive()) {
      sawMessage = true;
      unsigned long rxId = 0;
      byte len = 0;
      byte buf[8];
      CAN.readMsgBuf(&rxId, &len, buf);
      printRawCanMessage(rxId, len, buf);
      parseZdtPositionReply(len, buf);
    }
  }

  if (!sawMessage) {
    Serial.println(F("No CAN reply seen."));
  }
}

bool sendZdtFrame(unsigned long idOffset, const byte *data, byte len, unsigned long replyTimeoutMs = 250) {
  unsigned long canId = ZDT_CAN_BASE_ID + idOffset;
  byte result = CAN.sendMsgBuf(canId, 1, len, (byte *)data); // 1 = extended frame in this library

  Serial.print(F("TX ID 0x"));
  Serial.print(canId, HEX);
  Serial.print(F(" data:"));
  for (byte i = 0; i < len; i++) {
    Serial.print(F(" 0x"));
    if (data[i] < 0x10) Serial.print('0');
    Serial.print(data[i], HEX);
  }
  Serial.print(F(" -> "));
  Serial.println(result == CAN_OK ? F("OK") : F("FAIL"));

  delay(20);
  if (replyTimeoutMs > 0) {
    readCanReplies(replyTimeoutMs);
  }

  return result == CAN_OK;
}

// ===================== ZDT COMMANDS =====================

void zdtEnable(bool enable) {
  byte cmd[] = {
    0xF3, 0xAB, enable ? 0x01 : 0x00, 0x00, ZDT_CHECKSUM_6B
  };
  sendZdtFrame(0, cmd, sizeof(cmd));
}

void zdtStop() {
  byte cmd[] = {
    0xFE, 0x98, 0x00, ZDT_CHECKSUM_6B
  };
  sendZdtFrame(0, cmd, sizeof(cmd));
}

void zdtRunSlowOneSecond() {
  // Command 0xF6: speed mode. Always auto-stop after 1 second.
  // dir=0, speed=30, accel=5, multi-machine flag=0.
  byte cmd[] = {
    0xF6, 0x00, 0x00, 0x1E, 0x05, 0x00, ZDT_CHECKSUM_6B
  };

  sendZdtFrame(0, cmd, sizeof(cmd));
  delay(1000);
  zdtStop();
  Serial.println(F("ZDT speed test stopped automatically."));
}

void zdtRelativeMove(byte direction, unsigned long pulseCount = 3200UL) {
  // Command 0xFD: position mode split across CAN ID 0x100 then 0x101.
  // ZDT uses big-endian speed/pulses. relative=0x00, absolute=0x01.
  unsigned int speedRpm = 30;
  byte accel = 0x05;

  byte firstFrame[] = {
    0xFD,
    direction,
    (byte)((speedRpm >> 8) & 0xFF),
    (byte)(speedRpm & 0xFF),
    accel,
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

  sendZdtFrame(0, firstFrame, sizeof(firstFrame));
  sendZdtFrame(1, secondFrame, sizeof(secondFrame), 600);
}

unsigned long zdtDistanceMmToPulses(float distanceMm) {
  float motorStepsPerRev = 360.0f / ZDT_MOTOR_STEP_DEG;
  float pulsesPerRev = motorStepsPerRev * (float)ZDT_MICROSTEPS;
  float pulsesPerMm = pulsesPerRev / ZDT_LEAD_MM_PER_REV;
  return (unsigned long)round(abs(distanceMm) * pulsesPerMm);
}

float zdtPulsesToDistanceMm(unsigned long pulses) {
  float motorStepsPerRev = 360.0f / ZDT_MOTOR_STEP_DEG;
  float pulsesPerRev = motorStepsPerRev * (float)ZDT_MICROSTEPS;
  float pulsesPerMm = pulsesPerRev / ZDT_LEAD_MM_PER_REV;
  return (float)pulses / pulsesPerMm;
}

int zdtDirectionSign(byte direction) {
  return direction == 0 ? 1 : -1;
}

double zdtSignedPulses(byte direction, unsigned long magnitude) {
  return (double)zdtDirectionSign(direction) * (double)magnitude;
}

double zdtRelativeToSoftwareZeroPulses() {
  double current = zdtSignedPulses(lastZdtPositionDirection, lastZdtPositionMagnitudePulses);
  double zero = zdtSignedPulses(zdtSoftwareZeroDirection, zdtSoftwareZeroMagnitudePulses);
  return current - zero;
}

void zdtSetSoftwareZeroFromLastRead() {
  if (!hasZdtPosition) {
    Serial.println(F("Read ZDT position first with z, then send 0 to set software zero."));
    return;
  }

  zdtSoftwareZeroMagnitudePulses = lastZdtPositionMagnitudePulses;
  zdtSoftwareZeroDirection = lastZdtPositionDirection;
  hasZdtSoftwareZero = true;
  Serial.println(F("ZDT software zero set to last read position."));
}

void printZdtDistanceCalibration() {
  float pulsesPerRev = (360.0f / ZDT_MOTOR_STEP_DEG) * (float)ZDT_MICROSTEPS;
  float pulsesPerMm = pulsesPerRev / ZDT_LEAD_MM_PER_REV;

  Serial.println(F("ZDT distance calibration:"));
  Serial.print(F("  lead_mm_per_rev="));
  Serial.println(ZDT_LEAD_MM_PER_REV, 4);
  Serial.print(F("  motor_step_deg="));
  Serial.println(ZDT_MOTOR_STEP_DEG, 4);
  Serial.print(F("  microsteps="));
  Serial.println(ZDT_MICROSTEPS);
  Serial.print(F("  pulses_per_rev="));
  Serial.println(pulsesPerRev, 2);
  Serial.print(F("  pulses_per_mm="));
  Serial.println(pulsesPerMm, 2);
  Serial.print(F("  pulses_per_cm="));
  Serial.println(pulsesPerMm * 10.0f, 2);
}

void zdtRelativeMoveWithSpeed(byte direction, unsigned long pulseCount, unsigned int speedRpm, byte accel) {
  byte firstFrame[] = {
    0xFD,
    direction,
    (byte)((speedRpm >> 8) & 0xFF),
    (byte)(speedRpm & 0xFF),
    accel,
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

  sendZdtFrame(0, firstFrame, sizeof(firstFrame));
  sendZdtFrame(1, secondFrame, sizeof(secondFrame), 600);
}

void zdtMoveMillimeters(byte direction, float distanceMm) {
  unsigned long pulseCount = zdtDistanceMmToPulses(distanceMm);

  Serial.print(F("ZDT distance move: direction="));
  Serial.print(direction == 0x00 ? F("forward") : F("backward"));
  Serial.print(F(" distance_mm="));
  Serial.print(distanceMm, 3);
  Serial.print(F(" pulses="));
  Serial.println(pulseCount);

  if (pulseCount == 0) {
    Serial.println(F("Distance is zero; no move sent."));
    return;
  }

  zdtRelativeMoveWithSpeed(direction, pulseCount, ZDT_DEFAULT_SPEED_RPM, ZDT_DEFAULT_ACCEL);
}

void zdtMoveCentimeters(byte direction, float distanceCm) {
  zdtMoveMillimeters(direction, distanceCm * 10.0f);
}

bool parseDistanceArgument(const String &commandLine, float *value) {
  String distanceText = commandLine.substring(1);
  distanceText.trim();

  if (distanceText.length() == 0) {
    Serial.println(F("Missing distance. Example: m50 for 50 mm, k5 for 5 cm."));
    return false;
  }

  *value = distanceText.toFloat();
  if (*value <= 0.0f) {
    Serial.println(F("Distance must be greater than 0."));
    return false;
  }
  return true;
}

void processSerialCommand(String commandLine) {
  commandLine.trim();
  if (commandLine.length() == 0) {
    return;
  }

  char command = commandLine.charAt(0);

  switch (command) {
    case '?':
      printHelp();
      break;
    case 'c':
    case 'C':
      centerAllServos();
      break;
    case 'a':
    case 'A':
      moveAllServosSmooth(SERVO_SAFE_LOW);
      break;
    case 'd':
    case 'D':
      moveAllServosSmooth(SERVO_SAFE_HIGH);
      break;
    case '1':
      moveServoSmooth(0, SERVO_SAFE_LOW);
      delay(SERVO_HOLD_MS);
      moveServoSmooth(0, SERVO_SAFE_HIGH);
      delay(SERVO_HOLD_MS);
      moveServoSmooth(0, SERVO_CENTER);
      break;
    case '2':
      moveServoSmooth(1, SERVO_SAFE_LOW);
      delay(SERVO_HOLD_MS);
      moveServoSmooth(1, SERVO_SAFE_HIGH);
      delay(SERVO_HOLD_MS);
      moveServoSmooth(1, SERVO_CENTER);
      break;
    case '3':
      moveServoSmooth(2, SERVO_SAFE_LOW);
      delay(SERVO_HOLD_MS);
      moveServoSmooth(2, SERVO_SAFE_HIGH);
      delay(SERVO_HOLD_MS);
      moveServoSmooth(2, SERVO_CENTER);
      break;
    case '4':
      moveServoSmooth(3, SERVO_SAFE_LOW);
      delay(SERVO_HOLD_MS);
      moveServoSmooth(3, SERVO_SAFE_HIGH);
      delay(SERVO_HOLD_MS);
      moveServoSmooth(3, SERVO_CENTER);
      break;
    case 'w':
    case 'W':
      servoAllWaveTest();
      break;
    case 'e':
    case 'E':
      zdtEnable(true);
      break;
    case 'x':
    case 'X':
      zdtEnable(false);
      break;
    case 'f':
    case 'F':
      zdtRelativeMove(0x00);
      break;
    case 'b':
    case 'B':
      zdtRelativeMove(0x01);
      break;
    case 'm':
    case 'M': {
      float distanceMm;
      if (parseDistanceArgument(commandLine, &distanceMm)) {
        zdtMoveMillimeters(0x00, distanceMm);
      }
      break;
    }
    case 'n':
    case 'N': {
      float distanceMm;
      if (parseDistanceArgument(commandLine, &distanceMm)) {
        zdtMoveMillimeters(0x01, distanceMm);
      }
      break;
    }
    case 'k':
    case 'K': {
      float distanceCm;
      if (parseDistanceArgument(commandLine, &distanceCm)) {
        zdtMoveCentimeters(0x00, distanceCm);
      }
      break;
    }
    case 'l':
    case 'L': {
      float distanceCm;
      if (parseDistanceArgument(commandLine, &distanceCm)) {
        zdtMoveCentimeters(0x01, distanceCm);
      }
      break;
    }
    case 'i':
    case 'I':
      printZdtDistanceCalibration();
      break;
    case 'p':
    case 'P':
      zdtForwardBackTest();
      break;
    case 'r':
    case 'R':
      zdtRunSlowOneSecond();
      break;
    case 's':
    case 'S':
      zdtStop();
      break;
    case 'z':
    case 'Z':
      zdtReadCurrentPosition();
      if (commandLine.length() > 1 && commandLine.charAt(1) == '0') {
        zdtSetSoftwareZeroFromLastRead();
      }
      break;
    case '0':
      zdtSetSoftwareZeroFromLastRead();
      break;
    case 'H':
      Serial.println(F("Setting ZDT driver current position as temporary homing zero."));
      zdtSetDriverCurrentPositionAsHomingZero(false);
      break;
    case 'q':
    case 'Q':
      readCanReplies(1000);
      break;
    case 'j':
    case 'J':
      printJointState();
      break;
    case 't':
    case 'T':
      fullSafeCheck();
      break;
    default:
      Serial.println(F("Unknown command. Type ? for help."));
      break;
  }
}

void zdtForwardBackTest() {
  Serial.println(F("ZDT relative forward."));
  zdtRelativeMove(0x00);
  delay(2500);
  readCanReplies(500);

  Serial.println(F("ZDT relative backward."));
  zdtRelativeMove(0x01);
  delay(2500);
  readCanReplies(500);
}

void zdtReadCurrentPosition() {
  byte cmd[] = {
    0x36, ZDT_CHECKSUM_6B
  };

  sendZdtFrame(0, cmd, sizeof(cmd), 600);
}

void zdtSetDriverCurrentPositionAsHomingZero(bool storeToDriver) {
  // ZDT command 0x93: set current position as single-turn homing zero.
  // storeToDriver=false keeps it temporary; true asks the ZDT to store it persistently.
  byte cmd[] = {
    0x93, 0x88, storeToDriver ? 0x01 : 0x00, ZDT_CHECKSUM_6B
  };

  sendZdtFrame(0, cmd, sizeof(cmd), 600);
}

// ===================== JOINT STATE / FULL TEST =====================

void printJointState() {
  Serial.println(F("Joint state snapshot:"));
  for (byte i = 0; i < SERVO_COUNT; i++) {
    Serial.print(F("  servo"));
    Serial.print(i + 1);
    Serial.print(F("_logical_deg="));
    Serial.print(servoLogicalAngles[i]);
    Serial.print(F(" output_deg="));
    Serial.println(calibratedServoAngle(i, servoLogicalAngles[i]));
  }

  if (hasZdtPosition) {
    Serial.print(F("  zdt_position_direction="));
    Serial.println(lastZdtPositionDirection);
    Serial.print(F("  zdt_position_magnitude_pulses="));
    Serial.println(lastZdtPositionMagnitudePulses);
    Serial.print(F("  zdt_position_approx_mm="));
    Serial.println(zdtPulsesToDistanceMm(lastZdtPositionMagnitudePulses), 3);
    if (hasZdtSoftwareZero) {
      double relativePulses = zdtRelativeToSoftwareZeroPulses();
      float relativeMm = zdtPulsesToDistanceMm((unsigned long)abs(relativePulses));
      if (relativePulses < 0) {
        relativeMm = -relativeMm;
      }
      Serial.print(F("  zdt_relative_to_software_zero_pulses="));
      Serial.println(relativePulses, 0);
      Serial.print(F("  zdt_relative_to_software_zero_mm="));
      Serial.println(relativeMm, 3);
    }
  } else {
    Serial.println(F("  zdt_position_pulses=unknown; send z first"));
  }
}

void fullSafeCheck() {
  Serial.println(F("Starting full safe check."));
  centerAllServos();
  zdtEnable(true);
  zdtReadCurrentPosition();
  servoOneAtATimeTest();
  zdtRelativeMove(0x00, 1600UL);
  delay(1800);
  zdtReadCurrentPosition();
  zdtRelativeMove(0x01, 1600UL);
  delay(1800);
  zdtReadCurrentPosition();
  centerAllServos();
  printJointState();
  Serial.println(F("Full safe check complete."));
}

void printHelp() {
  Serial.println();
  Serial.println(F("Combined 4-servo + ZDT CAN check"));
  Serial.println(F("Commands:"));
  Serial.println(F("  ? = help"));
  Serial.println(F("  c = center all servos"));
  Serial.println(F("  a = all servos to safe low"));
  Serial.println(F("  d = all servos to safe high"));
  Serial.println(F("  1 = test Servo 1"));
  Serial.println(F("  2 = test Servo 2"));
  Serial.println(F("  3 = test Servo 3"));
  Serial.println(F("  4 = test Servo 4"));
  Serial.println(F("  w = all-servo wave test"));
  Serial.println(F("  e = enable ZDT"));
  Serial.println(F("  x = disable ZDT"));
  Serial.println(F("  f = ZDT small forward relative move"));
  Serial.println(F("  b = ZDT small backward relative move"));
  Serial.println(F("  m<number> = ZDT forward by millimeters, example m50"));
  Serial.println(F("  n<number> = ZDT backward by millimeters, example n50"));
  Serial.println(F("  k<number> = ZDT forward by centimeters, example k5"));
  Serial.println(F("  l<number> = ZDT backward by centimeters, example l5"));
  Serial.println(F("  i = print ZDT distance calibration"));
  Serial.println(F("  p = ZDT forward then back test"));
  Serial.println(F("  r = ZDT slow speed for 1 second, then stop"));
  Serial.println(F("  s = ZDT immediate stop"));
  Serial.println(F("  z = read ZDT current position"));
  Serial.println(F("  z0 = read ZDT current position, then set Arduino software zero"));
  Serial.println(F("  0 = set Arduino software zero to last ZDT position read"));
  Serial.println(F("  H = set ZDT driver current position as temporary homing zero"));
  Serial.println(F("  q = read pending CAN replies"));
  Serial.println(F("  j = print joint state snapshot"));
  Serial.println(F("  t = full safe check sequence"));
  Serial.println();
}

// ===================== ARDUINO SETUP / LOOP =====================

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(2000);
  while (!Serial) {}

  Serial.println(F("Starting combined Arduino pipeline check."));

  for (byte i = 0; i < SERVO_COUNT; i++) {
    servos[i].attach(SERVO_PINS[i], 1000, 2000);
    servoLogicalAngles[i] = SERVO_CENTER;
    servos[i].write(calibratedServoAngle(i, SERVO_CENTER));
  }
  delay(800);
  Serial.println(F("Servos attached and centered."));

  Serial.println(F("Starting CAN at 500 kbps."));
  while (CAN_OK != CAN.begin(MCP_ANY, CAN_500KBPS, MCP_16MHZ)) {
    Serial.println(F("CAN init failed. Check shield CS pin, wiring, and MCP_16MHZ vs MCP_8MHZ."));
    delay(1000);
  }
  CAN.setMode(MCP_NORMAL);
  Serial.println(F("CAN init OK."));

  printHelp();
  printJointState();
}

void loop() {
  if (CAN_MSGAVAIL == CAN.checkReceive()) {
    readCanReplies(1);
  }

  while (Serial.available()) {
    char ch = (char)Serial.read();
    if (ch == '\r' || ch == '\n') {
      processSerialCommand(serialCommandBuffer);
      serialCommandBuffer = "";
    } else {
      serialCommandBuffer += ch;
      lastSerialInputMs = millis();
      if (serialCommandBuffer.length() > 24) {
        Serial.println(F("Command too long; clearing input."));
        serialCommandBuffer = "";
      }
    }
  }

  if (serialCommandBuffer.length() > 0 && millis() - lastSerialInputMs > 100) {
    processSerialCommand(serialCommandBuffer);
    serialCommandBuffer = "";
  }
}
