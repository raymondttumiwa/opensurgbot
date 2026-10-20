# Arduino Full Pipeline: 4 Servos + 1 ZDT Stepper over CAN

This README describes how to rebuild the current STM32-style pipeline on an Arduino Uno using:

- 4 hobby servos controlled by Arduino PWM pins
- 1 ZDT stepper driver controlled over CAN-BUS
- Arduino CAN-BUS Shield V1.2
- External power for servos
- Programmable DC power supply for the ZDT stepper driver

You have already confirmed:

- All 4 servos work from Arduino.
- CAN communication to the ZDT driver works.
- The ZDT stepper responds to CAN commands.

The next goal is clean cable management, reliable shared grounding, and a repeatable full-system startup/check procedure.

## System Overview

```text
PC / Laptop USB
    |
    v
Arduino Uno + CAN-BUS Shield V1.2
    |                |
    | PWM signals    | CAN_H / CAN_L / GND
    v                v
4 Servos          ZDT Stepper Driver
    |                |
    | Servo power    | Motor driver power
    v                v
Servo 5-6V PSU    Programmable DC PSU
```

Important rule:

```text
All grounds must be connected together.
Power rails should stay separated by voltage/current purpose.
```

That means:

- Arduino `GND`
- Servo power supply `GND`
- ZDT programmable DC supply negative / `GND`
- ZDT CAN `GND`

should all share a common ground reference.

Do **not** power the servos or the ZDT driver from the Arduino `5V` pin.

## Recommended Hardware

### Control

- Arduino Uno
- CAN-BUS Shield V1.2, usually MCP2515 + MCP2551/TJA1050 based
- USB cable for Arduino programming and serial monitor

### Motion

- 4 hobby servos
- 1 ZDT stepper driver + stepper motor

### Power

- Servo power supply: regulated `5V` to `6V`, enough current for all servos
- ZDT stepper power supply: programmable DC supply, voltage according to ZDT/motor rating
- Optional separate Arduino supply, or simply power Arduino by USB

### Cable Management

Recommended for moving beyond loose Dupont wires:

- Small terminal block or power distribution block for servo `+V`
- Small terminal block or power distribution block for common `GND`
- Twisted pair cable for `CAN_H` and `CAN_L`
- Separate thicker wires for motor/servo power
- Thin signal wires only for PWM/CAN logic
- Heat shrink or ferrules for screw terminals
- Labels for every cable: `S1`, `S2`, `S3`, `S4`, `CANH`, `CANL`, `GND`, `ZDT+`, `ZDT-`
- Zip ties or cable sleeve
- Optional inline fuse for servo supply and motor supply

## Power Architecture

Use separate power paths:

```text
Servo PSU +5V/+6V  -> servo red wires only
Servo PSU GND      -> servo black/brown wires

ZDT DC PSU +       -> ZDT driver power +
ZDT DC PSU -       -> ZDT driver power - / GND

Arduino USB/Power  -> Arduino only
```

Then connect common ground:

```text
Arduino GND        -> common GND block
Servo PSU GND      -> common GND block
ZDT PSU - / GND    -> common GND block
CAN shield GND     -> common GND block, usually through Arduino GND
ZDT CAN GND        -> common GND block
```

### Why Common Ground Is Needed

PWM servo signals and CAN transceivers need a shared voltage reference. Without common ground:

- Servos may jitter or ignore commands.
- CAN communication may become unreliable.
- Arduino may reset or receive noisy data.

### What Not To Do

- Do not connect ZDT motor supply `+` to Arduino `5V`.
- Do not connect servo power `+` to Arduino `5V`.
- Do not pass high motor current through Arduino ground pins if avoidable.
- Do not use breadboard rails for high servo or motor current in final wiring.

For final wiring, use a ground distribution block. Arduino ground should be a signal/reference ground connection, not the main motor-current return path.

## Servo Wiring

Current Arduino servo mapping:

```text
Arduino D3 -> Servo 1 signal
Arduino D5 -> Servo 2 signal
Arduino D6 -> Servo 3 signal
Arduino D9 -> Servo 4 signal
```

Servo power:

```text
Servo red wires         -> Servo PSU +5V/+6V
Servo black/brown wires -> Servo PSU GND / common GND
Servo signal wires      -> Arduino PWM pins
```

Servo signal wire is usually orange, yellow, or white.

## CAN / ZDT Wiring

```text
Arduino CAN Shield CAN_H -> ZDT CAN_H
Arduino CAN Shield CAN_L -> ZDT CAN_L
Arduino/CAN Shield GND   -> ZDT GND / common GND
ZDT power +              -> programmable DC supply +
ZDT power -              -> programmable DC supply - / common GND
```

Use a twisted pair for `CAN_H` and `CAN_L` if possible.

## CAN Termination

CAN bus should have termination at both ends.

For a simple Arduino CAN shield <-> ZDT driver bus:

```text
120 ohm between CAN_H and CAN_L near Arduino/CAN shield side
120 ohm between CAN_H and CAN_L near ZDT side
```

With power off, measure resistance between `CAN_H` and `CAN_L`:

| Measurement | Meaning |
| --- | --- |
| about `60 ohm` | Good: two 120 ohm terminators |
| about `120 ohm` | Only one terminator |
| very high/open | No terminator |
| much below `60 ohm` | Too many terminators or short/problem |

A breadboard resistor is acceptable for testing, but for final cable management put the resistor close to the connector or terminal block.

## ZDT Menu Settings

Use these ZDT interface settings:

```text
P_Serial  = CAN / CAN1_MAP
CAN_Baud  = 500K
ID_Addr   = 1
Checksum  = 0x6B
Response  = Both
En        = Hold / Always active, if available
```

Response choices:

| Response | Meaning | Recommendation |
| --- | --- | --- |
| `None` | No reply | Not good for debugging |
| `Receive` | Reply when command is accepted | Good for command testing |
| `Reached` | Reply when movement completes | Useful for position moves |
| `Both` | Receive + reached replies | Best while integrating |

For integration, keep `Response = Both` until the full pipeline is stable.

## Arduino Software Setup

Install these Arduino libraries:

- `Servo`
  - Usually built into Arduino IDE.
- `mcp_can`
  - Used for MCP2515 CAN-BUS shield.
  - Commonly called Seeed CAN-BUS Shield / MCP_CAN library.

Current sketches:

```text
ARDUINO_TEST/ARDUINO_TEST.ino
ARDUINO_CAN_ZDT_TEST/ARDUINO_CAN_ZDT_TEST.ino
```

For the final pipeline, combine the servo and CAN logic into one Arduino sketch.

## Arduino Initialization Needed

The Arduino version needs to initialize these subsystems:

1. Serial monitor
2. Servo PWM outputs
3. CAN shield
4. ZDT motor state
5. Safe starting positions

Conceptual Arduino `setup()`:

```cpp
void setup() {
  Serial.begin(115200);

  // 1. Attach servos
  servo1.attach(3, 1000, 2000);
  servo2.attach(5, 1000, 2000);
  servo3.attach(6, 1000, 2000);
  servo4.attach(9, 1000, 2000);

  // 2. Move servos to safe neutral
  centerAllServos();

  // 3. Start CAN shield at 500 kbps
  CAN.begin(MCP_ANY, CAN_500KBPS, MCP_16MHZ);
  CAN.setMode(MCP_NORMAL);

  // 4. Enable ZDT motor over CAN
  zdtEnable(true);

  // 5. Read current ZDT position
  zdtReadCurrentPosition();

  // 6. Print command menu
  printHelp();
}
```

### Arduino Pin Plan

| Function | Arduino Pin |
| --- | --- |
| Servo 1 PWM | `D3` |
| Servo 2 PWM | `D5` |
| Servo 3 PWM | `D6` |
| Servo 4 PWM | `D9` |
| CAN shield SPI CS | usually `D10`, sometimes `D9` |
| CAN shield SPI MOSI | `D11` |
| CAN shield SPI MISO | `D12` |
| CAN shield SPI SCK | `D13` |

Important conflict:

- If your CAN shield uses `D9` as CS, it conflicts with Servo 4 on `D9`.
- Your working sketch currently uses `CAN_CS_PIN = 10`.
- Keep CAN CS on `D10` if possible.
- If the shield requires `D9`, move Servo 4 to another PWM-capable pin, for example `D10` only if not used by CAN, or better use a separate servo driver board such as PCA9685.

For Arduino Uno, SPI pins are fixed:

```text
D11 = MOSI
D12 = MISO
D13 = SCK
```

Do not use these for servo signals while using the CAN shield.

## ZDT CAN Commands Used

Motor ID is `1`, so CAN base ID is:

```text
0x01 << 8 = 0x100
```

The Arduino test sends extended CAN frames to `0x100` and sometimes `0x101`.

| Action | ZDT Command |
| --- | --- |
| Enable motor | `0xF3` |
| Speed mode | `0xF6` |
| Position mode | `0xFD` |
| Immediate stop | `0xFE` |
| Read current position | `0x36` |

Avoid relying on speed mode for robot control because it runs continuously until stopped.

For robot-like motion, prefer position mode:

- Relative move forward
- Relative move backward
- Absolute move after homing/zeroing

## Position Feedback For Kinevizu

Use ZDT command `0x36` to read current position.

The Arduino sketch prints:

```text
Current position pulses: <value>
```

Convert pulses to joint angle:

```text
pulses_per_rev = motor_full_steps_per_rev * microsteps
angle_degrees = pulses * 360 / pulses_per_rev
angle_radians = angle_degrees * pi / 180
```

Example for a 1.8 degree motor with `MStep = 16`:

```text
motor_full_steps_per_rev = 200
microsteps = 16
pulses_per_rev = 3200
angle_degrees = pulses * 360 / 3200
```

For Kinevizu integration, publish or stream:

```text
servo1_angle
servo2_angle
servo3_angle
servo4_angle
stepper_angle_or_position
```

Important: stepper position is only meaningful after defining zero.

## Homing / Zeroing

Before using position data in Kinevizu, define a reference zero.

Options:

1. Manual zero
   - Move mechanism to known safe zero position.
   - Power on ZDT.
   - Send/read position and treat that as zero offset in Arduino software.

2. ZDT homing command
   - Use ZDT homing features if limit switches or sensorless homing are configured.
   - More robust, but requires proper ZDT menu setup and physical limits.

3. External limit switch
   - Add a limit switch to Arduino.
   - Slowly move stepper until switch triggers.
   - Stop motor and set software zero.

For first full pipeline check, use manual zero and small relative moves only.

## Recommended Full-System Startup Procedure

### Before Power

1. Check all labels.
2. Check no loose wire strands touch adjacent terminals.
3. Confirm servo power is `5V` to `6V`, not higher.
4. Confirm ZDT supply voltage is correct for your driver/motor.
5. Confirm Arduino `GND`, servo `GND`, and ZDT `GND` are common.
6. Confirm `CAN_H` and `CAN_L` are not swapped.
7. Confirm CAN termination by measuring `CAN_H` to `CAN_L` with power off.

### Power-Up Order

Recommended:

1. Turn servo power supply on with low current limit.
2. Turn ZDT programmable DC supply on with safe current limit.
3. Connect Arduino USB.
4. Open Serial Monitor at `115200`.
5. Upload/run the Arduino pipeline sketch.

### First Commands

1. Center servos.
2. Enable ZDT.
3. Read ZDT current position.
4. Move one servo at a time.
5. Move ZDT one small relative step forward.
6. Move ZDT one small relative step backward.
7. Run combined low-speed/no-load test.

## Suggested Command Interface

For the combined Arduino sketch, use a serial command menu like this:

| Command | Action |
| --- | --- |
| `?` | Print help |
| `c` | Center all servos |
| `1` | Test Servo 1 |
| `2` | Test Servo 2 |
| `3` | Test Servo 3 |
| `4` | Test Servo 4 |
| `e` | Enable ZDT stepper |
| `x` | Disable ZDT stepper |
| `f` | ZDT small relative forward move |
| `b` | ZDT small relative backward move |
| `s` | Immediate ZDT stop |
| `z` | Read ZDT current position |
| `a` | Print all current joint states |
| `t` | Full safe test sequence |

Avoid using continuous speed mode except for debugging. If included, make it auto-stop after a short time.

## Full Check Sequence For 4 Servos + 1 Stepper

Use this after cable management is complete.

### Stage 1: Power-Only Check

- Arduino disconnected from servo/ZDT signals.
- Servo PSU on.
- ZDT PSU on.
- No heating, no smell, no unexpected motion.
- Measure voltage at servo power block.
- Measure voltage at ZDT input.

### Stage 2: Servo Signal Check

- Connect Arduino servo signals.
- Keep ZDT CAN connected but do not move stepper yet.
- Run servo center command.
- Test each servo individually.
- Watch for brownout/reset.

### Stage 3: CAN Check

- Send ZDT enable.
- Read current position.
- Confirm CAN reply appears.
- Send tiny relative move forward.
- Send tiny relative move backward.
- Confirm position changes and returns approximately.

### Stage 4: Combined Check

- Center all servos.
- Read ZDT position.
- Move one servo.
- Move stepper tiny amount.
- Move all servos within safe range.
- Move stepper back.
- Print final joint states.

### Stage 5: Kinevizu Check

- Convert servo angles to joint angles.
- Convert ZDT pulses to angle.
- Stream or manually input values into Kinevizu.
- Confirm visual direction matches physical direction.
- If direction is reversed, fix in software sign convention first, not wiring.

## Cable Management Plan

Recommended layout:

```text
[Arduino + CAN Shield]
        |
        | short signal wires
        v
[Signal terminal area]
        |
        +--> Servo 1 signal
        +--> Servo 2 signal
        +--> Servo 3 signal
        +--> Servo 4 signal
        +--> CAN_H/CAN_L twisted pair to ZDT

[Servo PSU] --> [5V/GND servo power distribution] --> 4 servos

[ZDT PSU] --> [ZDT driver power input]

All GNDs --> [common GND block]
```

Keep these separated:

- Servo power bundle
- ZDT motor power bundle
- CAN twisted pair
- Servo signal wires
- USB cable

Avoid running CAN_H/CAN_L tightly parallel to high-current motor power for long distances.

## Current Limits And Safety

Start conservative:

### Servo PSU

- Voltage: `5.0V` first, then `6.0V` only if servos support it.
- Current limit: start around `1A` to `2A` for no-load testing.
- Increase only if needed.

### ZDT PSU

- Voltage: according to driver/motor rating.
- Current limit: start low for no-load communication/motion testing.
- Increase gradually only when motion requires it.

### Mechanical Safety

- Remove servo horns or disconnect linkages for first combined test.
- Keep the stepper away from hard limits.
- Use small relative moves first.
- Keep one hand near power cutoff.
- Add physical limit switches before autonomous motion.

## Common Problems

### CAN Init Fails

- Try `CAN_CS_PIN = 10` or `CAN_CS_PIN = 9`.
- Confirm CAN shield oscillator setting: `MCP_16MHZ` vs `MCP_8MHZ`.
- Confirm library is installed.
- Confirm shield is seated properly.

### ZDT Does Not Reply

- Check `P_Serial = CAN / CAN1_MAP`.
- Check `CAN_Baud = 500K`.
- Check `ID_Addr = 1`.
- Check `Checksum = 0x6B`.
- Check `Response = Both`.
- Swap/check `CAN_H` and `CAN_L`.
- Check common ground.
- Check termination resistance.

### Motor Moves Forever

- That is speed mode.
- Send immediate stop.
- Prefer position mode for robot integration.
- In code, make speed tests auto-stop after a timeout.

### Position Value Is Wrong Or Offset

- Position is relative to current zero.
- Perform homing or manually define zero.
- Check microstep value before converting pulses to angle.

### Arduino Resets When Servos Move

- Servo current is too high for wiring/supply.
- Servo power is accidentally passing through Arduino.
- Ground wiring is too thin or loose.
- Add a large capacitor on servo power rail, for example `470uF` to `1000uF`, rated above servo voltage.

## What To Build Next

1. Create one combined Arduino sketch for 4 servos + ZDT CAN.
2. Add a `JointState` print command for Kinevizu/debugging.
3. Add software limits for every joint.
4. Add stepper homing or manual-zero command.
5. Replace breadboard/high-current Dupont wiring with terminal blocks.
6. Add emergency stop behavior: stop ZDT and hold/center servos.

## Minimum Final Initialization Checklist

Your Arduino pipeline should initialize:

```text
Serial.begin(115200)
Servo attach on D3/D5/D6/D9
Servo center positions with offsets
SPI bus for CAN shield
MCP2515 CAN at 500 kbps
CAN normal mode
ZDT motor ID = 1
ZDT enable command
ZDT read current position command
Safe serial command menu
```

If all of those pass, you have recreated the key STM32 pipeline behavior on Arduino for initial integration.
