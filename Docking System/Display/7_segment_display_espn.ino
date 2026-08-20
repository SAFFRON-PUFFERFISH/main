// INITIALISE WIFI PACKAGES

#include <WiFi.h>
#include <WiFiAP.h>
#include <WiFiClient.h>
#include <WiFiGeneric.h>
#include <WiFiMulti.h>
#include <WiFiSTA.h>
#include <WiFiScan.h>
#include <WiFiServer.h>
#include <WiFiType.h>
#include <WiFiUdp.h>

// INITIALISE ESP-NOW PACKAGES

#include <esp_now.h>

// INITIALISE ARDUINO PACKAGES

#include <Arduino.h>

/*
7 Segment Display Driver Circuit

PINOUT:
D2-9 (DigitalWrite) [a-g;dpx] (segment selection, hold LOW, HIGH to enable)
A0-3 (DigitalWrite) [CC] (digit switching, hold HIGH, LOW to enable)
*/

// ASSIGN SEGMENT PINS
int DPX = D2; //D6?
int a = D3; //D2?
int b = D4; //D9?
int c = D5; //D7?
int d = D6; //D5?
int e = D7; //D4?
int f = D8; //D3?
int g = D9; //D8?

// ASSIGN DIGIT PINS
int Q1 = A0; // digit 1
int Q2 = A1; // digit 2
int Q3 = A2; // digit 3
int Q4 = A3; // digit 4

// ASSIGN SEGMENT PIN ARRAY
const int pins[7] = {a, b, c, d, e, f, g}; 
const int digP[4] = {Q1, Q2, Q3, Q4};

// set segment pattern for each digit (boolean segments)
const bool digitSegments[10][7] = {
  {1,1,1,1,1,1,0}, //0
  {0,1,1,0,0,0,0}, //1
  {1,1,0,1,1,0,1}, //2
  {1,1,1,1,0,0,1}, //3
  {0,1,1,0,0,1,1}, //4
  {1,0,1,1,0,1,1}, //5
  {1,0,1,1,1,1,1}, //6
  {1,1,1,0,0,0,0}, //7
  {1,1,1,1,1,1,1}, //8
  {1,1,1,1,0,1,1}, //9
};

// set segment pattern for each letter along with a key

const char letters[] = {
  'a','b','c','d','e','f','g','h','i','j',
  'l','n','o','p','q','r','s','t','u','y','z'
};

const bool letterSegments[21][7] = {
  {1,1,1,0,1,1,1}, // a — tall
  {0,0,1,1,1,1,1}, // b — small
  {1,0,0,1,1,1,0}, // c — tall
  {0,1,1,1,1,0,1}, // d — small
  {1,0,0,1,1,1,1}, // e — tall
  {1,0,0,0,1,1,1}, // f — tall
  {1,0,1,1,1,1,0}, // g — tall
  {0,1,1,0,1,1,1}, // h — small
  {0,0,0,0,1,1,0}, // i — tal0
  {0,1,1,1,0,0,0}, // j — tall
  {0,0,0,1,1,1,0}, // l — tall
  {0,0,1,0,1,0,1}, //{1,1,1,0,1,1,0}, // n — tall // N SHORT {0,0,1,0,1,0,1},
  {1,1,1,1,1,1,0}, // o — tall (same as digit 0)
  {1,1,0,0,1,1,1}, // p — tall
  {1,1,1,0,0,1,1}, // q — tall
  {0,0,0,0,1,0,1}, // r — small
  {1,0,1,1,0,1,1}, // s — tall (same as digit 5)
  {0,0,0,1,1,1,1}, // t — small
  {0,1,1,1,1,1,0}, // u — tall
  {0,1,1,1,0,1,1}, // y — tall
  {1,1,0,1,1,0,1}, // z — tall (same as digit 2)
};

// ESP-NOW NOTES

// INITIALISE GLOBAL DATA RECEIPT VARIABLES
uint16_t binaryData_A = 0; // data receipt from Dock A [KEYPAD]        CURRENT MAC: {0xE8, 0xF6, 0x0A, 0xBE, 0x54, 0xDC} [keypad arduino] [4 digit number]
uint16_t binaryData_B = 0; // data receipt from Dock B [SUBMERSIBLE]   CURRENT MAC: {0xE8, 0xF6, 0x0A, 0xD3, 0x68, 0x54} [receiving buzz]
uint16_t binaryData_C = 0; // data receipt from Dock C [DISPLAY]       CURRENT MAC: {0xE8, 0xF6, 0x0A, 0xBE, 0xB0, 0xCC} [7 seg display]


// INITIALISE GLOBAL MAC ADDRESS VARIABLES
const uint8_t macAddr_DockA[] = {0xE8, 0xF6, 0x0A, 0xBF, 0xB8, 0xA4};  //REPLACE W. E8:F6:0A:BF:B8:A4
const uint8_t macAddr_DockB[] = {0xE8, 0xF6, 0x0A, 0xD3, 0x68, 0x54};
const uint8_t macAddr_DockC[] = {0xE8, 0xF6, 0x0A, 0xBE, 0xB0, 0xCC};

// ESP-NOW FUNCTIONS

void esp_init () {
  // initialise ESP-NOW and check that it is operating

  WiFi.mode(WIFI_MODE_STA);
  
  if (esp_now_init() != ESP_OK) {
    //Serial.println("ESP-NOW Initialisation Failed, Retrying...");
    ESP.restart();
  }
  else {
    esp_now_register_recv_cb(receive_data); // REMOVE IF NOT RECEIVING DATA
    // esp_now_register_send_cb(send_success); // REMOVE IF SEND VALIDATION N/R
  };
}

void get_MAC () {
  // display MAC address
  //Serial.println(WiFi.macAddress());
}

void add_peer(const uint8_t *receiverMAC) {
  // ADD A PEER TO TRANSMIT DATA TO
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, receiverMAC, 6); // fill in MAC address into peerInfo
  peerInfo.channel = 0; // sets data channel to 0
  peerInfo.encrypt = false; // disable encryption
  esp_now_add_peer(&peerInfo);
}

bool send_data(const uint8_t *receiverMAC, const uint8_t *data, size_t len) {
  // take input of receiver MAC Address, send data
  // conducts 10 attempts with a spacing of 50ms before halting transfer attempts
  // returns bool depending on success/failure
  esp_err_t result;
  int attempts = 0;

  do {
    result = esp_now_send(receiverMAC, data, len);
    attempts++;
    if (result != ESP_OK) delay(50);
  } while (result != ESP_OK && attempts <= 10);

  if (attempts > 10) {
    //Serial.println("Data transmission failed. :(");
    }
  
  return (result == ESP_OK); // returns true if data sent successfully
}

void receive_data(const uint8_t *senderMAC, const uint8_t *data, int len) {
  if (len == sizeof(uint16_t)) {
      uint16_t value;
      memcpy(&value, data, 2);
    if (memcmp(senderMAC, macAddr_DockA, 6) == 0) {
      binaryData_A = value;
      //Serial.println("Data Received from Dock A");  
    }
    else if (memcmp(senderMAC, macAddr_DockB, 6) == 0) {
      binaryData_B = value;
      //Serial.println("Data Received from Dock B");
    }
    else if (memcmp(senderMAC, macAddr_DockC, 6) == 0) {
      binaryData_C = value;
      //Serial.println("Data Received from Dock C");
    }
    else {
      //Serial.println("Unknown MAC Address");
    }
  }
}

void assignPins() {
  pinMode(D2, OUTPUT); pinMode(D3, OUTPUT); pinMode(D4, OUTPUT); pinMode(D5, OUTPUT); pinMode(D6, OUTPUT); pinMode(D7, OUTPUT); pinMode(D8, OUTPUT); pinMode(D9, OUTPUT);
  pinMode(A0, OUTPUT); pinMode(A1, OUTPUT); pinMode(A2, OUTPUT); pinMode(A3, OUTPUT);  
}

void numOn(int number) {
  /*
  Set up boolean array??
  */
  if (number < 0 || number > 9) return;
  for (int i = 0; i < 7; i++) {
    digitalWrite(pins[i], digitSegments[number][i] ? HIGH : LOW); // IF digitSegments[number][i] is true, activate, otherwise, don't.
  }
}

void charOn(char letterDisp) {
  for (int x = 0; x < sizeof(letters); x++) {
    if (letterDisp == letters[x]) {
      for (int i = 0; i < 7; i++) {
        digitalWrite(pins[i], letterSegments[x][i] ? HIGH : LOW);
      }
    }
  }
}

void allOff() {
  for (int i = 0; i < 7; i++) {digitalWrite(pins[i], LOW);}
  for (int x = 0; x < 4; x++) {digitalWrite(digP[x], LOW);}
}

void integerDisp(int inputNum) {
  if (inputNum < 0 || inputNum > 9999) return;
  int thous = (inputNum / 1000) % 10;
  int hundr = (inputNum / 100) % 10;
  int tens = (inputNum / 10) % 10;
  int ones = (inputNum / 1) % 10;

  //display at 100hz refresh rate per digit, so overall 400hz
  unsigned long cycleStart = micros();
  unsigned long currentTime = micros();
  while (true) {
    currentTime = micros();
    unsigned long dt = currentTime - cycleStart;
    if (dt > 0 && dt < 1250) {allOff(); digOn(4); numOn(ones);} // ONES: ENABLE DIGIT4
    else if (dt > 1250 && dt < 2500) {allOff(); digOn(3); numOn(tens);} // TENS: ENABLE DIGIT 3
    else if (dt > 2500 && dt < 3750) {allOff(); digOn(2); numOn(hundr);} // HUNDREDS: ENABLE DIGIT 2
    else if (dt > 3750 && dt < 5000) {allOff(); digOn(1); numOn(thous);} // THOUSANDS: ENABLE DIGIT 1

    if (dt > 5000) {
      //cycleStart = micros();
      break;    
    }
  }
}

void wordDisp(char word[]) {
  if (strlen(word) <= 4) {
    // less than 5 so no scrolling required
    fourCharWord(word);
  }
  else if (strlen(word) > 4) {
    // greater than 4 so scrolling required
    for (int i = 0; i <= strlen(word) - 4; i++) {
      char wordNew[] = {word[0+i], word[1+i], word[2+i], word[3+i]};
      unsigned long start = millis();
      while (millis() - start < 1000) {
        fourCharWord(wordNew);
      }
      if (binaryData_A != 0) {
        break;
      }
    }
  }
  allOff();
}

void fourCharWord(char word4[]) {
  //display at 100hz refresh rate per digit, so overall 400hz
    unsigned long cycleStart = micros();
    unsigned long currentTime = micros();
    while (true) {
      currentTime = micros();
      unsigned long dt = currentTime - cycleStart;
      if (dt > 0 && dt < 1250) {allOff(); digOn(4); charOn(word4[3]);} // ENABLE DIGIT4 for LETTER D
      else if (dt > 1250 && dt < 2500) {allOff(); digOn(3); charOn(word4[2]);} // ENABLE DIGIT 3 for LETTER C
      else if (dt > 2500 && dt < 3750) {allOff(); digOn(2); charOn(word4[1]);} // ENABLE DIGIT 2 for LETTER B
      else if (dt > 3750 && dt < 5000) {allOff(); digOn(1); charOn(word4[0]);} // ENABLE DIGIT 1 for LETTER A

      if (dt > 5000) {
        //cycleStart = micros();
        break;    
      }
    }
}

void digOn(int digit) {
  switch(digit) {
    case 1:
      digitalWrite(Q1, HIGH);
      break;
    case 2:
      digitalWrite(Q2, HIGH);
      break;
    case 3:
      digitalWrite(Q3, HIGH);
      break;
    case 4:
      digitalWrite(Q4, HIGH);
      break;
  }
}


void setup() {
  // put your setup code here, to run once:
  //Serial.begin(19200);
  esp_init();
  add_peer(macAddr_DockA); // ADD DOCK A as a PEER
  add_peer(macAddr_DockB); // ADD DOCK B as a PEER
  add_peer(macAddr_DockC); // ADD DOCK C as a PEER
  assignPins(); 
}

void loop() {
  // SHOULD RUN AN ESP-NOW CHECK, INITIALLY DISPLAY SEARCH
  while (binaryData_A == 0) {wordDisp("scanning");}
  int denarynum = binaryData_A;
  integerDisp(binaryData_A);
}
