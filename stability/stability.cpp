
// MPU6050 is what i am assuming we use.
#include <Wire.h>    
#include <Servo.h>   

// MPU6050 constants 
const uint8_t MPU_ADDR         = 0x68;     // I2C address when the AD0 pin is tied low
const float   ACC_LSB_PER_G    = 16384.0f; // at +/-2g, full 16-bit range spans 2g -> 32768/2
const float   GYRO_LSB_PER_DPS = 131.0f;   // at +/-250dps -> 32768/250 counts per deg/s

// Vertical thrusters
const uint8_t PIN_FL = 3, PIN_FR = 4, PIN_RL = 5, PIN_RR = 6; // one ESC signal pin each
Servo motorFL, motorFR, motorRL, motorRR;                     // one Servo object drives each ESC
const int NEUTRAL    = 1500;   // us pulse = "stop" on a bidirectional ESC
const int MAX_OUTPUT = 200;    // biggest us offset we allow either side of neutral (keep small at first)

int verticalPower = 0;         // pilot climb/dive command; 0 here = just hold level

// Loop timing
const float   LOOP_DT    = 0.01f;  // run the control loop every 10 ms (100 Hz)
unsigned long lastLoopUs  = 0;     // timestamp (micros) of the previous loop iteration

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

// One controller per axis. guess gains
PID rollPID  = { 2.0f, 0.0f, 0.4f, 0.0f, 0.0f, (float)MAX_OUTPUT, (float)MAX_OUTPUT };
PID pitchPID = { 2.0f, 0.0f, 0.4f, 0.0f, 0.0f, (float)MAX_OUTPUT, (float)MAX_OUTPUT };

float pidStep(PID &c, float measurement, float setpoint, float dt) {
  float error = setpoint - measurement;                    // how far off level we are (want 0)

  c.integral += c.Ki * error * dt;                         // accumulate error over time
  c.integral = constrain(c.integral, -c.iLimit, c.iLimit); // clamping integral term prevents "windup" if the controller saturates

  float dMeas = (measurement - c.prevMeas) / dt;           // rate of change of the ANGLE, not the error
  c.prevMeas = measurement;                                // (using the measurement avoids a "kick" on setpoint changes)

  float out = c.Kp * error       // PID
            + c.integral         
            - c.Kd * dMeas;      
  return constrain(out, -c.outLimit, c.outLimit);          // clamp the correction to a safe range
}

// MPU6050 raw I2C
void mpuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR); // address the chip
  Wire.write(reg);                  // which register to write
  Wire.write(val);                  // the value to put there
  Wire.endTransmission();           // send it
}

// Read one 16-bit sample. Two Wire.read() calls in a single expression are
// unsequenced in C++, so we grab the bytes into named variables FIRST.
int16_t read16() {
  uint8_t hi = Wire.read();                          // high byte comes out first
  uint8_t lo = Wire.read();                          // then the low byte
  return (int16_t)(((uint16_t)hi << 8) | lo);        // stitch them into a signed 16-bit value
}

void mpuReadRaw(int16_t &ax, int16_t &ay, int16_t &az,
                int16_t &gx, int16_t &gy, int16_t &gz) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);                       // point at ACCEL_XOUT_H, start of the data block
  Wire.endTransmission(false);            // "false" = repeated start, keeps the bus for the read
  Wire.requestFrom((int)MPU_ADDR, 14, (int)true); // pull 14 bytes: accel(6) + temp(2) + gyro(6)
  ax = read16(); ay = read16(); az = read16();     // accelerometer X,Y,Z
  read16();                               // temperature -- read and discard to keep alignment
  gx = read16(); gy = read16(); gz = read16();     // gyroscope X,Y,Z
}

void calibrateGyro(int samples = 500) {
  long sx = 0, sy = 0;                    // 32-bit accumulators so 500 samples can't overflow
  int16_t ax, ay, az, gx, gy, gz;
  for (int i = 0; i < samples; i++) {     // average many readings while the vehicle is still...
    mpuReadRaw(ax, ay, az, gx, gy, gz);
    sx += gx; sy += gy;
    delay(2);
  }
  gyroBiasX = (float)sx / samples / GYRO_LSB_PER_DPS; // The mean is the zero-rate offset (in deg/s)
  gyroBiasY = (float)sy / samples / GYRO_LSB_PER_DPS;
}

void anglesFromAccel(int16_t ax, int16_t ay, int16_t az,
                     float &rollAcc, float &pitchAcc) {
  float axg = ax / ACC_LSB_PER_G;         // convert raw counts to g's
  float ayg = ay / ACC_LSB_PER_G;
  float azg = az / ACC_LSB_PER_G;
  rollAcc  = atan2(ayg, azg) * RAD_TO_DEG;                       // tilt of gravity in the Y-Z plane = roll
  pitchAcc = atan2(-axg, sqrt(ayg * ayg + azg * azg)) * RAD_TO_DEG; // tilt of gravity toward X = pitch
}

int toMicros(int power) {
  power = constrain(power, -MAX_OUTPUT, MAX_OUTPUT); // never exceed the safe offset
  return NEUTRAL + power;                            // turn a signed thrust into a servo pulse
}

void setup() {
  Serial.begin(115200);       // open the USB serial link for telemetry/tuning
  Wire.begin();               // start the I2C bus (SDA=A4, SCL=A5 on an Uno)
  mpuWrite(0x6B, 0x00);       // PWR_MGMT_1: clear the SLEEP bit -> wake the sensor
  delay(100);
  mpuWrite(0x1B, 0x00);       // GYRO_CONFIG:  select +/-250 dps  (matches the constant above)
  mpuWrite(0x1C, 0x00);       // ACCEL_CONFIG: select +/-2 g      (matches the constant above)

  motorFL.attach(PIN_FL); motorFR.attach(PIN_FR);   // bind each Servo object to its pin
  motorRL.attach(PIN_RL); motorRR.attach(PIN_RR);
  motorFL.writeMicroseconds(NEUTRAL); motorFR.writeMicroseconds(NEUTRAL); 
  motorRL.writeMicroseconds(NEUTRAL); motorRR.writeMicroseconds(NEUTRAL);
  delay(3000);                

  Serial.println(F("Calibrating gyro -- keep the vehicle STILL and level..."));
  calibrateGyro();            // measure and store the gyro offsets

  // Seed the fused angle from gravity so we don't start from a false 0.
  int16_t ax, ay, az, gx, gy, gz;
  mpuReadRaw(ax, ay, az, gx, gy, gz);
  float rollAcc, pitchAcc;
  anglesFromAccel(ax, ay, az, rollAcc, pitchAcc);
  roll = rollAcc;  pitch = pitchAcc;             // start the filter at the real orientation
  rollPID.prevMeas = roll;  pitchPID.prevMeas = pitch; // and prime the derivative term

  Serial.println(F("Ready."));
  lastLoopUs = micros();      // start the loop clock
}

void loop() {
  unsigned long now = micros();
  // Gate the loop to a fixed rate. Unsigned subtraction stays correct even
  // when micros() rolls over (~every 71 minutes).
  if ((now - lastLoopUs) < (unsigned long)(LOOP_DT * 1e6f)) return;
  float dt = (now - lastLoopUs) / 1e6f;  // actual elapsed time in seconds (for the PID maths)
  lastLoopUs = now;

  int16_t ax, ay, az, gx, gy, gz;
  mpuReadRaw(ax, ay, az, gx, gy, gz);    // fresh sensor sample

  float rollAcc, pitchAcc;
  anglesFromAccel(ax, ay, az, rollAcc, pitchAcc);       // absolute angle from gravity (noisy, no drift)

  float gRateX = gx / GYRO_LSB_PER_DPS - gyroBiasX;     // roll rate  in deg/s, bias removed
  float gRateY = gy / GYRO_LSB_PER_DPS - gyroBiasY;     // pitch rate in deg/s, bias removed

  // Complementary filter: integrate the gyro for smooth short-term motion,
  // then nudge it toward the accelerometer angle so it can't drift.
  roll  = ALPHA * (roll  + gRateX * dt) + (1.0f - ALPHA) * rollAcc;
  pitch = ALPHA * (pitch + gRateY * dt) + (1.0f - ALPHA) * pitchAcc;

  float rollCorrection  = pidStep(rollPID,  roll,  0.0f, dt);  // setpoint 0 = "be level"
  float pitchCorrection = pidStep(pitchPID, pitch, 0.0f, dt);

  // Thruster mix: base lift +/- a roll term (splits left/right)
  //                        +/- a pitch term (splits front/rear).
  int FL = verticalPower + (int)rollCorrection + (int)pitchCorrection;
  int FR = verticalPower - (int)rollCorrection + (int)pitchCorrection;
  int RL = verticalPower + (int)rollCorrection - (int)pitchCorrection;
  int RR = verticalPower - (int)rollCorrection - (int)pitchCorrection;

  motorFL.writeMicroseconds(toMicros(FL));   // send each mixed value out as a servo pulse
  motorFR.writeMicroseconds(toMicros(FR));
  motorRL.writeMicroseconds(toMicros(RL));
  motorRR.writeMicroseconds(toMicros(RR));

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