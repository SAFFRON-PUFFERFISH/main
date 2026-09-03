//Vertical Based Microcontroller
//Secondary controller which is autonomously stabilising the submersible through IMU sensor data.
//Receives a single dive base speed from XCM over UART ("DIVE:<int>\n"),
//applies it to all 4 thrusters, and adds PID roll/pitch stability
//corrections on top before writing to the ESCs.

#include <Wire.h>
#include <ESP32Servo.h>

// MPU6050 constants (stability)
const uint8_t MPU_ADDR         = 0x68;     // I2C address when the AD0 pin is tied low
const float   ACC_LSB_PER_G    = 16384.0f; // at +/-2g, full 16-bit range spans 2g -> 32768/2
const float   GYRO_LSB_PER_DPS = 131.0f;   // at +/-250dps -> 32768/250 counts per deg/s

// ---- Thrusters ----
// Pin order assumed to be FL, FR, RL, RR - matches the mixing math below.
const int NUM_THRUSTERS = 4;
const int thrusterPins[NUM_THRUSTERS] = {8, 9, 10, 11};
Servo thrusters[NUM_THRUSTERS];

const int PWM_MIN_US     = 1000;   // full reverse
const int PWM_NEUTRAL_US = 1500;   // stop / neutral
const int PWM_MAX_US     = 2000;   // full forward
const int MAX_OUTPUT     = 200;    // biggest us offset PID may add either side of neutral (start small)

const unsigned long COMMAND_TIMEOUT_MS = 250;
unsigned long lastCommandTime = 0;

// Loop timing
unsigned long lastLoopUs = 0; // timestamp (micros) of the previous loop iteration

// Complementary filter state
float roll = 0, pitch = 0;          // fused angle estimate in degrees (what we control)
const float ALPHA = 0.98f;          // how much we trust the gyro vs the accelerometer
float gyroBiasX = 0, gyroBiasY = 0; // constant drift of each gyro axis, measured at startup

// PID controller
struct PID {
  float Kp, Ki, Kd;   // proportional / integral / derivative gains
  float integral;     // running sum of error*dt
  float prevMeas;     // previous measurement, so we can compute a derivative
  float outLimit;     // clamp on the final output
  float iLimit;       // clamp on the integral term (anti-windup)
};

// One controller per axis. guess gains - tune later.
PID rollPID  = { 2.0f, 0.0f, 0.4f, 0.0f, 0.0f, (float)MAX_OUTPUT, (float)MAX_OUTPUT };
PID pitchPID = { 2.0f, 0.0f, 0.4f, 0.0f, 0.0f, (float)MAX_OUTPUT, (float)MAX_OUTPUT };

// Set when a full "DIVE:<int>\n" line has just been decoded.
bool newCommandAvailable = false;
int verticalPower = 0; // single base value, -DIVE_MAX_OUTPUT..DIVE_MAX_OUTPUT, applied to all 4 thrusters

// ---- UART text command decode (from XCM) ----
// Expects lines of the form "DIVE:<signed int>\n"
char cmdBuf[32];
uint8_t cmdLen = 0;

void pollUartCommands() {
  while (Serial1.available() > 0) {
    char c = Serial1.read();

    if (c == '\n') {
      cmdBuf[cmdLen] = '\0';

      if (strncmp(cmdBuf, "DIVE:", 5) == 0) {
        verticalPower = atoi(cmdBuf + 5);
        verticalPower = constrain(verticalPower, -MAX_OUTPUT, MAX_OUTPUT);
        newCommandAvailable = true;
      }

      cmdLen = 0;
    } else if (cmdLen < sizeof(cmdBuf) - 1) {
      cmdBuf[cmdLen++] = c;
    }
    // characters beyond buffer size are dropped
  }
}

// ---- MPU6050 raw I2C ----
void mpuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

// Read one 16-bit sample. Two Wire.read() calls in a single expression are
// unsequenced in C++, so we grab the bytes into named variables FIRST.
int16_t read16() {
  uint8_t hi = Wire.read();
  uint8_t lo = Wire.read();
  return (int16_t)(((uint16_t)hi << 8) | lo);
}

void mpuReadRaw(int16_t &ax, int16_t &ay, int16_t &az,
                int16_t &gx, int16_t &gy, int16_t &gz) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);                       // point at ACCEL_XOUT_H, start of the data block
  Wire.endTransmission(false);            // repeated start, keeps the bus for the read
  Wire.requestFrom((int)MPU_ADDR, 14, (int)true); // accel(6) + temp(2) + gyro(6)
  ax = read16(); ay = read16(); az = read16();
  read16();                               // temperature -- read and discard to keep alignment
  gx = read16(); gy = read16(); gz = read16();
}

void calibrateGyro(int samples = 500) {
  long sx = 0, sy = 0;
  int16_t ax, ay, az, gx, gy, gz;
  for (int i = 0; i < samples; i++) {
    mpuReadRaw(ax, ay, az, gx, gy, gz);
    sx += gx; sy += gy;
    delay(2);
  }
  gyroBiasX = (float)sx / samples / GYRO_LSB_PER_DPS;
  gyroBiasY = (float)sy / samples / GYRO_LSB_PER_DPS;
}

void anglesFromAccel(int16_t ax, int16_t ay, int16_t az,
                     float &rollAcc, float &pitchAcc) {
  float axg = ax / ACC_LSB_PER_G;
  float ayg = ay / ACC_LSB_PER_G;
  float azg = az / ACC_LSB_PER_G;
  rollAcc  = atan2(ayg, azg) * RAD_TO_DEG;
  pitchAcc = atan2(-axg, sqrt(ayg * ayg + azg * azg)) * RAD_TO_DEG;
}

float pidStep(PID &c, float measurement, float setpoint, float dt) {
  float error = setpoint - measurement;

  c.integral += c.Ki * error * dt;
  c.integral = constrain(c.integral, -c.iLimit, c.iLimit); // anti-windup clamp

  float dMeas = (measurement - c.prevMeas) / dt; // rate of change of angle, not error
  c.prevMeas = measurement;

  float out = c.Kp * error + c.integral - c.Kd * dMeas;
  return constrain(out, -c.outLimit, c.outLimit);
}

// UART link to XCM - must match XCM's Serial1.begin() pins, crossed
// (XCM TX -> this RX, XCM RX -> this TX)
const int UART_RX_PIN = 16;
const int UART_TX_PIN = 17;

void setup() {
  Serial.begin(115200);       // USB serial for telemetry/tuning
  Serial1.begin(115200, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN); // link to XCM
  Wire.begin();                // I2C bus for the MPU6050

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  for (int i = 0; i < NUM_THRUSTERS; i++) {
    thrusters[i].setPeriodHertz(50);
    thrusters[i].attach(thrusterPins[i], PWM_MIN_US, PWM_MAX_US);
    thrusters[i].writeMicroseconds(PWM_NEUTRAL_US);
  }

  lastCommandTime = millis();
  Serial.println(F("Dive control ready."));

  mpuWrite(0x6B, 0x00);       // PWR_MGMT_1: clear SLEEP bit -> wake sensor
  delay(100);
  mpuWrite(0x1B, 0x00);       // GYRO_CONFIG: +/-250 dps
  mpuWrite(0x1C, 0x00);       // ACCEL_CONFIG: +/-2 g

  Serial.println(F("Calibrating gyro -- keep the vehicle STILL and level..."));
  calibrateGyro();

  // Seed the fused angle from gravity so we don't start from a false 0.
  int16_t ax, ay, az, gx, gy, gz;
  mpuReadRaw(ax, ay, az, gx, gy, gz);
  float rollAcc, pitchAcc;
  anglesFromAccel(ax, ay, az, rollAcc, pitchAcc);
  roll = rollAcc;  pitch = pitchAcc;
  rollPID.prevMeas = roll;  pitchPID.prevMeas = pitch;

  Serial.println(F("Ready."));
  lastLoopUs = micros();
}

void loop() {
  pollUartCommands();

  if (newCommandAvailable) {
    lastCommandTime = millis();
    newCommandAvailable = false;
  }

  // Failsafe: no fresh command in time -> treat base command as neutral.
  bool linkOk = (millis() - lastCommandTime <= COMMAND_TIMEOUT_MS);

  // ---- Compute actual elapsed time for the PID/filter maths ----
  unsigned long nowUs = micros();
  float dt = (nowUs - lastLoopUs) / 1000000.0f;
  lastLoopUs = nowUs;
  if (dt <= 0) dt = 0.001f; // guard against a zero/negative dt on first loop or overflow

  // ---- IMU stability ----
  int16_t ax, ay, az, gx, gy, gz;
  mpuReadRaw(ax, ay, az, gx, gy, gz);

  float rollAcc, pitchAcc;
  anglesFromAccel(ax, ay, az, rollAcc, pitchAcc);

  float gRateX = gx / GYRO_LSB_PER_DPS - gyroBiasX;
  float gRateY = gy / GYRO_LSB_PER_DPS - gyroBiasY;

  roll  = ALPHA * (roll  + gRateX * dt) + (1.0f - ALPHA) * rollAcc;
  pitch = ALPHA * (pitch + gRateY * dt) + (1.0f - ALPHA) * pitchAcc;

  float rollCorrection  = pidStep(rollPID,  roll,  0.0f, dt); // setpoint 0 = "be level"
  float pitchCorrection = pidStep(pitchPID, pitch, 0.0f, dt);

  // ---- Combine XCM's base dive command with PID stability correction ----
  // Same base speed for all 4 thrusters (verticalPower); PID splits the
  // difference per thruster to keep the vehicle level.
  // Thruster order assumed: 0=FL, 1=FR, 2=RL, 3=RR
  int baseOffset = linkOk ? verticalPower : 0; // verticalPower is already a us offset from neutral

  for (int i = 0; i < NUM_THRUSTERS; i++) {

    int correction;
    switch (i) {
      case 0: correction = (int)( rollCorrection + pitchCorrection); break; // FL
      case 1: correction = (int)(-rollCorrection + pitchCorrection); break; // FR
      case 2: correction = (int)( rollCorrection - pitchCorrection); break; // RL
      default: correction = (int)(-rollCorrection - pitchCorrection); break; // RR
    }

    int finalOffset = constrain(baseOffset + correction, -MAX_OUTPUT - 500, MAX_OUTPUT + 500);
    thrusters[i].writeMicroseconds(PWM_NEUTRAL_US + finalOffset);
  }

  // Print angles + corrections at 20 Hz so you can watch/tune the loop.
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint >= 50) {
    lastPrint = millis();
    Serial.print(F("roll="));   Serial.print(roll, 1);
    Serial.print(F(" pitch=")); Serial.print(pitch, 1);
    Serial.print(F(" rC="));    Serial.print(rollCorrection, 0);
    Serial.print(F(" pC="));    Serial.println(pitchCorrection, 0);
  }
}
