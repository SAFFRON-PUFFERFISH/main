// Initialise WiFi Packages

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

//const uint8_t seg7_mac[] = {0xE8, 0xF6, 0x0A, 0xBE, 0xB0, 0xCC}; //KEYPAD ARDUINO MAC ADDRESS: [E8:F6:0A:BE:54:DC] | {0xE8, 0xF6, 0x0A, 0xBE, 0xB0, 0xCC}

//3x3 LED MATRIX ARD. MAC ADDRESS: [E8:F6:0A:BE:B0:CC]
//RECEIVING BUZZ ARD. MAC ADDRESS: [E8:F6:0A:D3:68:54]

// Initialise Keypad Packages

#include "Adafruit_Keypad.h"

// Initialise ESP-NOW Packages

#include "esp_now.h"

// Defining I/O Pins
// PIN LAYOUT L -> R (1 -> 3) : C1 - C3; R1 - R4

/*
const int R1 = D5;
const int R2 = D6;
const int R3 = D7;
const int R4 = D8;

const int C1 = D2;
const int C2 = D3;
const int C3 = D4;

const int BUZZ = A0;
*/

// define row/column pins
const int C1 = D2; // REQUIRES JUMPER WIRE
const int R4 = D3;
const int R3 = D4;
const int R2 = D5;
const int R1 = D6;
const int C3 = D7;
const int C2 = D8;

const int BUZZ = A0;

// Initialise Keypad Packages II
#define KEYPAD_PID1824
#include "keypad_config.h"

// ESP-NOW DATA STRUCTURE (16 bit integer)

// ESP-NOW NOTES

// INITIALISE GLOBAL DATA RECEIPT VARIABLES
uint16_t binaryData_A; // data receipt from Dock A [KEYPAD]        CURRENT MAC: {0xE8, 0xF6, 0x0A, 0xBE, 0x54, 0xDC} [keypad arduino] [4 digit number]
uint16_t binaryData_B; // data receipt from Dock B [SUBMERSIBLE]   CURRENT MAC: {0xE8, 0xF6, 0x0A, 0xD3, 0x68, 0x54} [receiving buzz]
uint16_t binaryData_C; // data receipt from Dock C [DISPLAY]       CURRENT MAC: {0xE8, 0xF6, 0x0A, 0xBE, 0xB0, 0xCC} [7 seg display]


// INITIALISE GLOBAL MAC ADDRESS VARIABLES
const uint8_t macAddr_DockA[] = {0xE8, 0xF6, 0x0A, 0xBF, 0xB8, 0xA4};  //REPLACE W. E8:F6:0A:BF:B8:A4
const uint8_t macAddr_DockB[] = {0xE8, 0xF6, 0x0A, 0xD3, 0x68, 0x54};
const uint8_t macAddr_DockC[] = {0xE8, 0xF6, 0x0A, 0xBE, 0xB0, 0xCC};

// INITIALISE GLOBAL OVERRIDE DOCK POINTERS

bool dockA = false;
bool dockB = true;
bool dockC = false;

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

void send_data_dock(char dock_name, uint16_t binaryVal) {
  switch (dock_name) {
    case 'A':
      send_data(macAddr_DockA, (uint8_t *)&binaryVal, sizeof(binaryVal)); break;
    case 'B':
      send_data(macAddr_DockB, (uint8_t *)&binaryVal, sizeof(binaryVal)); break;
    case 'C':
      send_data(macAddr_DockC, (uint8_t *)&binaryVal, sizeof(binaryVal)); break;
  }
}

//initialize an instance of class NewKeypad
Adafruit_Keypad customKeypad = Adafruit_Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);

// transfer data (source)
void add_peer(uint8_t *receiverMAC) {
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, receiverMAC, 6); // fill in MAC address into peerInfo
  peerInfo.channel = 0; // sets data channel to 0
  peerInfo.encrypt = false; // disable encryption
  esp_now_add_peer(&peerInfo);
}

int getNUM() {
  // get numbers from keypad (4-DIGIT MAXIMUM)
  int digitCount = 0;
  int inputDigits[5];
  int fullNum = 0;
  while(true){
    customKeypad.tick();
    while (customKeypad.available()) {
      keypadEvent e = customKeypad.read();
      char key = (char)e.bit.KEY;
      int expected[4] = {2, 4, 0, 7};
      if (e.bit.EVENT == KEY_JUST_PRESSED) {
        if (key >= '0' && key <= '9') {
          int digitVal = key - '0';
          if (digitCount < 5) {
            inputDigits[digitCount] = digitVal;
            digitCount++;
          }  
          if (digitCount == 5) {
            if (memcmp(inputDigits, expected, 4 * sizeof(int)) == 0) {
              switch (inputDigits[4]) {
                case 8: dockA = false; dockB = false; dockC = true; digitCount = 0; break;
                case 7: dockA = false; dockB = true; dockC = false; digitCount = 0; break;
                case 2: send_data_dock('C', 0); break;
              }
            }
          } 
        }
        else if (key == '#') {
          if (digitCount != 0) {
            for (int i = 0; i < digitCount; i++) {
              fullNum += inputDigits[i] * (int)round(pow(10, digitCount - i - 1));
            }
            return fullNum;
          }
        }        
      }
    }
    delay(10);
  }
}

uint16_t numBin_conv(int fullNum) {
  uint16_t binaryVal = fullNum;
  return binaryVal;
}

void setup() {
  //Serial.begin(9600);

  esp_init();
  get_MAC();

  pinMode(BUZZ, OUTPUT);

  add_peer(macAddr_DockA); // ADD DOCK A as a PEER
  add_peer(macAddr_DockB); // ADD DOCK B as a PEER
  add_peer(macAddr_DockC); // ADD DOCK C as a PEER

  customKeypad.begin();
}

void loop() { 
  // put your main code here, to run repeatedly:
  int fullNum = getNUM();
  uint16_t binaryVal = numBin_conv(fullNum);
  if (dockA) {send_data_dock('A', binaryVal); }
  else if (dockB) {send_data_dock('B', binaryVal); }
  else if (dockC) {send_data_dock('C', binaryVal); }
}