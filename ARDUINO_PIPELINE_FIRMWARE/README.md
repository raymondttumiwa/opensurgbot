# ARDUINO_TEST

Arduino UNO/GIGA quick test for moving 4 servos together using an external programmable DC power supply.

This sketch is intentionally limited to a safe small range: logical `70` to `110` degrees, centered at `90` degrees.

## Files

- `ARDUINO_TEST.ino` - upload this sketch with Arduino IDE.

## Temporary Dupont-Wire Wiring

This is only for a short, no-load test. For real testing with load, use thicker power wiring or a terminal block.

Power wiring:

```text
DC supply +5V or +6V  -> all servo red wires
DC supply GND         -> all servo brown/black wires
DC supply GND         -> Arduino GND
```

Signal wiring:

```text
Arduino D3 -> Servo 1 signal wire
Arduino D5 -> Servo 2 signal wire
Arduino D6 -> Servo 3 signal wire
Arduino D9 -> Servo 4 signal wire
```

Servo signal wire is usually orange, yellow, or white.

## Power Supply Settings

1. Disconnect the servos first.
2. Set the power supply to `5.0V` first.
3. Set current limit to about `1A` to `2A` for initial testing.
4. Turn output off.
5. Connect the servo power wires.
6. Turn output on.
7. Increase current limit slowly only if the supply hits current limit or the servos jitter.

For 4 larger `DG-995MG` / `MG995` style servos, you may eventually need `5A` or more, but do not start there for a loose Dupont-wire test.

## Pay Attention To This

- Do not power the servos from the Arduino `5V` pin.
- Do not set the power supply above the servo voltage rating.
- Keep Dupont power wires short.
- If possible, use two Dupont wires in parallel for `+` and two for `GND`.
- Do not let exposed Dupont pins touch each other.
- Test with servo horns removed or with the mechanism disconnected first.
- Stop immediately if wires get warm, servos buzz loudly, or the Arduino resets.
- The Arduino and servo power supply must share `GND`.

## Serial Monitor Commands

Open Serial Monitor at `115200` baud.

| Command | Action |
| --- | --- |
| `c` | Center all servos at logical `90` |
| `a` | Move all servos to safe low logical `70` |
| `d` | Move all servos to safe high logical `110` |
| `w` | Slow wave together: `70 -> 110 -> 90` |
| `s` | Small stagger test, one servo at a time |
| `?` | Print help |

## DMMC-Style Offsets

The sketch includes these calibration offsets from the DMMC firmware:

```text
Servo 1: +8 degrees
Servo 2: +6 degree
Servo 3: 0 degree
Servo 4: 0 degree
```

The actual command sent to each servo is:

```text
logical angle + servo offset
```

So logical `90` becomes approximately:

```text
Servo 1: 85
Servo 2: 91
Servo 3: 88
Servo 4: 84
```

If the mechanism does not line up smoothly, adjust `SERVO_OFFSETS` in `ARDUINO_TEST.ino`.
