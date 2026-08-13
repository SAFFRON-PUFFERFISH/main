#include <ESP32Servo.h> // library to interface with servos
 #include <XBOXONE.h> // library to use the xbox one controller 
 
 // Servo objects 
 Servo servo_wrist;
 Servo servo_grip;

 // USB USB Host and Xbox controller objects.
 USB Usb;
 XBOXONE Xbox(&Usb); 

 // Define the servo angle limit 
 const int wrist_min_angle = 0;
 const int wrist_max_angle = 180;
 const int grip_min_angle = 0;
 const int grip_max_angle = 180;

 // Define starting angles for Servos 
 int wrist_angle = 0;
 int grip_angle = 0;
 
 // Joystick settings  
 // xBox range -32768 to +32767
 const int joystick_deadzone = 7500;
 const int joystick_centre = 0;
 const int xbox_lower_range = -32768;
 const int xbox_higher_range = 32767;

void setup() {
  // put your setup code here, to run once:
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
  // put your main code here, to run repeatedly:
 Usb.Task(); // processes USB events

 // Check the Xbox is connected
 if (Xbox.XboxOneConnected) {
    Serial.println("Controller connected");
    // wrist control 
    wrist_control();
    // grip control
    grip_control();
  }
  delay(20);
}

void grip_control() { 
 // mapping the joystick position for grip control - testing
 int16_t xval = Xbox.getAnalogHat(RightHatX); // horixontal axis

 if (abs(xval)>joystick_deadzone){
   grip_angle = map(xval, xbox_lower_range, xbox_higher_range, grip_min_angle, grib_max_angle); // map the position it is held in to the servo angle
  }
 grip_angle = constrain(grip_angle, grip_min_angle, grip_max_angle); // check the angle is within range (0-180)
 Serial.println(grip_angle); // Print the angle the wrist will rotate too
 servo_grip.write(grip_angle); // Commands servo to move 
   
}



// Control the wrist rotation based on the triggers. Hold R down, will turn CW onedegree every loop. so when released it stays there, and left will turn CCW. 
void wrist_control() {
 
 bool R_rotation = Xbox.getButtonPress(RB); // Reads if the triggers are pressed 
 bool L_rotation = Xbox.getButtonPress(LB);

 int step = 2; // degrees per loop
 
  if (R_rotation){ // if RB is pressed, the wrist will rotate with step 2 degrees CW 
    // rotate servo angle
    wrist_angle += step;
  }
  else if (L_rotation){ // if LB is pressed, the wrist will rotate with step 2 degrees CCW 
    wrist_angle -= step;
  }
 wrist_angle = constrain(wrist_angle, wrist_min_angle, wrist_max_angle); // check the angle is within range (0-180)
 Serial.println(wrist_angle); //print the angle value
 servo_wrist.write(wrist_angle); // Commands the servo to move 
}


