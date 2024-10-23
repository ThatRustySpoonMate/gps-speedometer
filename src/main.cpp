#include <Arduino.h>
#include "spi.h"
#include <TinyGPS++.h> // For GPS 
#include <RadioLib.h>


#define CS      18     // GPIO18 -- SX1278's CS
#define RST     14     // GPIO14 -- SX1278's RESET
#define DI0     26     // GPIO26 -- SX1278's IRQ(Interrupt Request)
#define BUTTON_PIN 38  // Pin between PWR and RST 

static const int RXPin = 34, TXPin = 12;
static const uint32_t GPSBaud = 9600;


/* Global variable defs */
SX1276 radio = new Module(CS, DI0, RST);


// The TinyGPSPlus object
TinyGPSPlus gps;

int transmissionResult = RADIOLIB_ERR_NONE;
int radioState = 0; // 0 Idle, 1 = busy


/* Private Function Defs */
void transmissionComplete();

void setup() {
  
  Serial.begin(115200);

  Serial2.begin(GPSBaud, SERIAL_8N1, RXPin, TXPin); // GPS Interface OLD: 34, 12

  // initialize SX1278 with default settings
  Serial.print(F("[SX1276] Initializing ... "));
  int state = radio.begin();
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println(F("success!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
    while (true) { delay(10); }
  }

  radio.setPacketSentAction(transmissionComplete);

}

void loop() {
  static uint32_t gps_satellites;
  static float gps_speed, gps_hdop;

  bool dataRead = false;
  while (Serial2.available() > 0){
    Serial.write(Serial2.read());
    //gps.encode(Serial2.read());

    dataRead = true;
    
  }

  if(dataRead) {
    gps_satellites = gps.satellites.value();
    gps_speed = gps.speed.mps();
    gps_hdop = gps.hdop.value();
  }
}



void transmissionComplete() {
  radioState = 0;
}
