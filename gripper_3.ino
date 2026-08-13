// Code for the robotic arm, wrist and gripper. 
// Last updated: 6.8.2026

 #include <ESP32Servo.h> // library to interface with servos
 #include <XBOXONE.h> // library to use the xbox one controller 
 
 // Servo objects 
 Servo servo_wrist;
 Servo servo_grip;

 // USB USB Host and Xbox controller objects.
 USB Usb;
 XBOXONE Xbox(&Usb); 

 // servo angle limit 
 const int wrist_min_angle = 0;
 const int wrist_max_angle = 180;
 const int grip_min_angle = 0;
 const int grip_max_angle = 90;

 // Define starting angles for Servos 
 int wrist_angle = 0;
 int grip_angle = 0;
 
 // joystick angles 
 // xBox -32768 to +32767
 const int joystick_deadzone = 7500;
 const int joystick_centre = 0;

void setup() {
  // Attach servos to pins - CONFIRM PIN NUMBERS
  servo_wrist.attach(); 
  servo_grip.attach(); 

  // Set initial position 
  servo_wrist.write(wrist_angle); // start at 0 degrees
  servo_grip.write(grip_angle); // start at 0 degrees
   
  Serial.begin(115200); // set baud rate for USB 

  if (Usb.Init() == -1) {
    Serial.print(F("USB host shield failed to start"));
    while (1); //halt
  }
  Serial.println("USB host shield is ready");
}

void loop() {
 Usb.Task(); // processes USB events

 if (Xbox.XboxOneConnected) {
    Serial.println("Controller connected");
    // wrist control 
    wrist_control();
    // grip control
    grip_control();
  }
  delay(20);
}

void grip_control (){ 
  
 int16_t xval = Xbox.getAnalogHat(RightHatX); //horizontal, y axis is irrelevent to us, range -32768 to 32767
  Serial.println(xval); // print the joystick value 
  // check for deadzones
  if (abs(xval) > joystick_deadzone){
    if (xval > joystick_deadzone){
      grip_angle++; 
    }
   else if (xval < -joystick_deadzone) {
      grip_angle--;
    }
  }
 grip_angle = constrain(grip_angle, grip_min_angle, grip_max_angle); // make sure the angle doesnt go outside the range (0-180)
 servo_grip.write(grip_angle);
}

// Control the wrist rotation based on the triggers. Hold R down, will turn CW onedegree every loop. so when released it stays there, and left will turn CCW. 
void wrist_control() {
 
 bool R_rotation = Xbox.getButtonPress(RB);
 bool L_rotation = Xbox.getButtonPress(LB);

 int step = 2; // degrees per loop
  // check if button is pressed
  if (R_rotation){
    // rotate servo angle
    wrist_angle += step;
  }
  else if (L_rotation){
    wrist_angle -= step;
  }
 wrist_angle = constrain(wrist_angle, wrist_min_angle, wrist_max_angle); // keep in servo limits
 servo_wrist.write(wrist_angle); // move servo 
}

