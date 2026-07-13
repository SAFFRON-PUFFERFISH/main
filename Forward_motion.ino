

#include <XBOXONE.h>

USB Usb;
XBOXONE Xbox(&Usb);


//motor driver pins
int in1 = ; //analogue pins
int in2 = ;

//enable pin (If using PWM)
int en = ; //digital pin


void setup() {
  Serial.begin(115200); //set baud rate

  while (!Serial); // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection
    if (Usb.Init() == -1) {
      Serial.print(F("\r\nOSC did not start"));
    while (1); //halt
  }

  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT); //set pins
  pinMode(en, OUTPUT);

  digitalWrite(en,HIGH);

}

void loop() {

  Usb.Task(); //services USB stack
  front();

}


void front(){

  uint16_t rt = Xbox.getButtonPress(RT); //0-1023, forward
  uint16_t lt = Xbox.getButtonPress(LT); //0-1023, reverse

  if (rt > 50) {
     digitalWrite(in1, HIGH);
     digitalWrite(in2, LOW); //forward

  } 
  else if (lt > 50) { //play around with deadzone values
 
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH); //reverse

  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW); //stopped
  }



}

