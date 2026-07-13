// Open-loop dive control.
// We have no pressure gauge and no IMU accurate enough to integrate for depth,
// so there is no depth HOLD here: the joystick Y axis maps straight to raw
// thrust on the four vertical thrusters. Stick up = climb, stick down = dive,
// centred = neutral (no vertical thrust).
//
// Controller: Xbox One pad over a USB Host Shield, same as Forward_motion.ino.
// Left stick Y is used so the same stick's X axis stays free for the rudder.

#include <XBOXONE.h>
#include <Servo.h>

USB Usb;            // the USB Host Shield stack
XBOXONE Xbox(&Usb); // Xbox One controller driver on that stack

// Vertical thrusters -- same pins and conventions as stability/stability.cpp
// so both sketches run on identical wiring.
const uint8_t PIN_FL = 3, PIN_FR = 4, PIN_RL = 5, PIN_RR = 6;
Servo motorFL, motorFR, motorRL, motorRR;

const int NEUTRAL    = 1500; // us pulse = "stop" on a bidirectional ESC
const int MAX_OUTPUT = 200;  // biggest us offset we allow either side of neutral

// Joystick geometry
const long STICK_MAX = 32767; // getAnalogHat() returns -32768..32767
const long DEADZONE  = 4000;  // ignore wobble near centre so we don't creep when "still"

int toMicros(int power) {
  power = constrain(power, -MAX_OUTPUT, MAX_OUTPUT); // never exceed the safe offset
  return NEUTRAL + power;                            // signed thrust -> servo pulse
}

// Convert the stick position into a signed thrust value.
// Returns 0 (neutral) when the controller is missing -- that is the failsafe:
// if the pad dies mid-dive the thrusters stop instead of holding the last command.
int readDivePower() {
  if (!Xbox.XboxOneConnected) return 0;

  // Read into a long BEFORE taking abs(): on an Uno `int` is 16-bit, so
  // abs(-32768) overflows right back to -32768 and would flip our direction.
  long y = Xbox.getAnalogHat(LeftHatY); // stick up = positive

  long magnitude = abs(y);
  if (magnitude < DEADZONE) return 0;   // centred -> no vertical thrust

  // Map only the LIVE part of the throw (deadzone..max) onto 0..MAX_OUTPUT,
  // so thrust ramps from zero at the deadzone edge instead of jumping.
  long power = map(magnitude, DEADZONE, STICK_MAX, 0, MAX_OUTPUT);

  // up = climb (+), down = dive (-). If the vehicle moves the wrong way in the
  // water, flip the sign here rather than rewiring the ESCs.
  return (y > 0) ? (int)power : -(int)power;
}

void writeAll(int power) {
  int us = toMicros(power);
  motorFL.writeMicroseconds(us);
  motorFR.writeMicroseconds(us);
  motorRL.writeMicroseconds(us);
  motorRR.writeMicroseconds(us);
}

void setup() {
  Serial.begin(115200);

  if (Usb.Init() == -1) {              // bring up the USB Host Shield
    Serial.println(F("USB Host Shield failed to start -- halting."));
    while (true);                      // no controller path, so refuse to run
  }

  motorFL.attach(PIN_FL); motorFR.attach(PIN_FR);
  motorRL.attach(PIN_RL); motorRR.attach(PIN_RR);
  writeAll(0);                         // ESCs must see neutral while they arm
  delay(3000);

  Serial.println(F("Dive control ready."));
}

void loop() {
  Usb.Task();                          // service the USB stack (polls the pad)

  int power = readDivePower();
  writeAll(power);                     // same raw thrust on all four thrusters

  static unsigned long lastPrint = 0;  // telemetry at 10 Hz so you can watch/tune
  if (millis() - lastPrint >= 100) {
    lastPrint = millis();
    Serial.print(F("divePower=")); Serial.println(power);
  }
}
