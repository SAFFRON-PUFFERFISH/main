

#include <XBOXONE.h>

USB Usb;
XBOXONE Xbox(&Usb);


//motor driver pins
const int in1 = ; //analogue pins TBD
const int in2 = ; 

//enable pin (If using PWM) have not decided yet...
int en = ; //digital pin


void setup() {
  Serial.begin(115200); //set baud rate

  while (!Serial); // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection
    if (Usb.Init() == -1) {
      Serial.print(F("\r\nOSC did not start"));
    while (1); //halt
  }

  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT); //set pins to outouts
  pinMode(en, OUTPUT);

  digitalWrite(en,HIGH);

}

void loop() {

  Usb.Task(); //services USB stack
  front();

}


void front(){

  uint16_t rt = Xbox.getButtonPress(RT); //0-1023, forward //recieves trigger values from the controller allowing us to control the motors corrospondingly 
  uint16_t lt = Xbox.getButtonPress(LT); //0-1023, reverse

  if (rt > 50) { //the above 50 is acting as a deadzone, need to play around with sensitivity and actual values later on
     digitalWrite(in1, HIGH);
     digitalWrite(in2, LOW); //forward

  } 
  else if (lt > 50) { 
 
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH); //reverse

  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW); //stopped
  }



}

