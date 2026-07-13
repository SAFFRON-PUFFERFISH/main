

#include <XBOXONE.h>

//motor driver pins
int in1 = ; //analogue pins
int in2 = ;

//enable pin (If using PWM)
int en = ; //digital pin


void setup() {
  Serial.begin(9600); //set baud rate

  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT); //set pins
  pinMode(en, OUTPUT);

}

void loop() {

  Usb.Task(); //services USB stack
  front();

}


void front();{

  uint16_t rt = Xbox.getButtonPress(RT); //0-1023, forward
  uint16_t lt = Xbox.getButtonPress(LT); //0-1023, reverse

  if (rt > 50) {
    uint8_t speed = map(rt, 0, 1023, 0, 255);
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

