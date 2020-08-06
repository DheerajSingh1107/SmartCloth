/*  ********************************************* 
 *  SparkFun_ADXL345_Example
 *  Triple Axis Accelerometer Breakout - ADXL345 
 *  Hook Up Guide Example 
 *  
 *  Utilizing Sparkfun's ADXL345 Library
 *  Bildr ADXL345 source file modified to support 
 *  both I2C and SPI Communication
 *  
 *  E.Robert @ SparkFun Electronics
 *  Created: Jul 13, 2016
 *  Updated: Sep 06, 2016
 *  
 *  Development Environment Specifics:
 *  Arduino 1.6.11
 *  
 *  Hardware Specifications:
 *  SparkFun ADXL345
 *  Arduino Uno
 *  *********************************************/

#include <SparkFun_ADXL345.h>         // SparkFun ADXL345 Library
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
String value;
int i=0;
String angles[6];
BLECharacteristic *pCharacteristic;
bool deviceConnected = false;
int txValue = 0;
int txValue1=0;
//const int readPin0 = 15; // Use GPIO number. See ESP32 board pinouts
//const int readPin1=4;

const int LED = 2; // Could be different depending on the dev board. I used the DOIT ESP32 dev board.

//std::string rxValue; // Could also make this a global var to access it in loop()

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/

#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E" // UART service UUID
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
    }
};

class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string rxValue = pCharacteristic->getValue();

      if (rxValue.length() > 0) {
        value="";
//        Serial.println("*********");
//        Serial.print("Received Value: ");

        for (int i = 0; i < rxValue.length(); i++) {
          Serial.print(rxValue[i]);
          value=value+rxValue[i];
        }
//        Serial.print("Recived Value=");
//        Serial.print(value);
//
        Serial.println();

        // Do stuff based on the command received from the app
        if (rxValue.find("A") != -1) { 
          Serial.print("Turning ON!");
          digitalWrite(LED, HIGH);
        }
        else if (rxValue.find("B") != -1) {
          Serial.print("Turning OFF!");
          digitalWrite(LED, LOW);
        }

        Serial.println();
        Serial.println("*********");
      }
    }
};
/*********** COMMUNICATION SELECTION ***********/
/*    Comment Out The One You Are Not Using    */
//ADXL345 ADXL345(i) = ADXL345(i);           // USE FOR SPI COMMUNICATION, ADXL345(CS_PIN);
//ADXL345 ADXL345(i) = ADXL345();             // USE FOR I2C COMMUNICATION

/****************** INTERRUPT ******************/
/*      Uncomment If Attaching Interrupt       */
//int interruptPin = 2;                 // Setup pin 2 to be the interrupt pin (for most Arduino Boards)


const int nADXL345=1;                   //number of ADXL345 connected to the board
//int i=0;
float roll,pitch;
float rollF[nADXL345],pitchF[nADXL345];
float preRoll[nADXL345],prePitch[nADXL345];
const int adxlPin[]={22,17,4,5,16};
/******************** SETUP ********************/
/*          Configure ADXL345 Settings         */
void setup(){
  
  Serial.begin(115200);                 // Start the serial terminal
  pinMode(LED, OUTPUT);
  Serial.println("SparkFun ADXL345 Accelerometer Hook Up Guide Example");
  Serial.println();
  
  init_ADXLs();
 init_BLE();
 
  
//attachInterrupt(digitalPinToInterrupt(interruptPin), ADXL_ISR, RISING);   // Attach Interrupt

}

/****************** MAIN CODE ******************/
/*     Accelerometer Readings and Interrupt    */
void loop(){
  
  // Accelerometer Readings
   
  for(i=0;i<nADXL345;i++)
    {
     int x,y,z;
     float X_out,Y_out,Z_out; 
    
  ADXL345(adxlPin[i]).readAccel(&x, &y, &z);         // Read the accelerometer values and store them in variables declared above x,y,z

  // Output Results to Serial
  /* UNCOMMENT TO VIEW X Y Z ACCELEROMETER VALUES */  

  X_out=x;
  Y_out=y;
  Z_out=z;
  X_out=X_out/256;
  Y_out=Y_out/256;
  Z_out=Z_out/256;
//    Serial.print(X_out);
//  Serial.print(", ");
//  Serial.print(Y_out);
//  Serial.print(", ");
//  Serial.println(Z_out); 
 // ADXL_ISR();
  // You may also choose to avoid using interrupts and simply run the functions within ADXL_ISR(); 
  //  and place it within the loop instead.  
  // This may come in handy when it doesn't matter when the action occurs.
 
  roll = atan(Y_out / sqrt(pow(X_out, 2) + pow(Z_out, 2))) * 180 / PI;
  pitch = atan(-1 * X_out / sqrt(pow(Y_out, 2) + pow(Z_out, 2))) * 180 / PI;

  // Low-pass filter
  rollF[i] = 0.94 * rollF[i] + 0.06 * roll;
  pitchF[i] = 0.94 * pitchF[i] + 0.06 * pitch;
  Serial.print(i);
  Serial.print("/");
  Serial.print(rollF[i]);
  Serial.print("/");
  Serial.print(pitchF[i]);
  Serial.println("/");
      char dataString [20];
   sprintf(dataString,"%d,%.1f,%.1f",i,rollF[i],pitchF[i]);
  //  sprintf(dataString,"%d,%d",txValue,txValue1);
//    pCharacteristic->setValue(&txValue, 1); // To send the integer value
//    pCharacteristic->setValue("Hello!"); // Sending a test message
    pCharacteristic->setValue(dataString);
    
    pCharacteristic->notify(); // Send the value to the app!
    Serial.print("*** Sent Value: ");
    Serial.print(dataString);
    Serial.println(" ***");
  
     delay(10);
     preRoll[i]=rollF[i];
     prePitch[i]=prePitch[i];
  }

}

/********************* ISR *********************/
/* Look for Interrupts and Triggered Action    */
//void ADXL_ISR() {
//  
//  // getInterruptSource clears all triggered actions after returning value
//  // Do not call again until you need to recheck for triggered actions
//  byte interrupts = ADXL345(i).getInterruptSource();
//  
//  // Free Fall Detection
//  if(ADXL345(i).triggered(interrupts, ADXL345_FREE_FALL)){
//    Serial.println("*** FREE FALL ***");
//    //add code here to do when free fall is sensed
//  } 
//  
//  // Inactivity
//  if(ADXL345(i).triggered(interrupts, ADXL345_INACTIVITY)){
//    Serial.println("*** INACTIVITY ***");
//     //add code here to do when inactivity is sensed
//  }
//  
//  // Activity
//  if(ADXL345(i).triggered(interrupts, ADXL345_ACTIVITY)){
//    Serial.println("*** ACTIVITY ***"); 
//     //add code here to do when activity is sensed
//  }
//  
//  // Double Tap Detection
//  if(ADXL345(i).triggered(interrupts, ADXL345_DOUBLE_TAP)){
//    Serial.println("*** DOUBLE TAP ***");
//     //add code here to do when a 2X tap is sensed
//  }
//  
//  // Tap Detection
//  if(ADXL345(i).triggered(interrupts, ADXL345_SINGLE_TAP)){
//    Serial.println("*** TAP ***");
//     //add code here to do when a tap is sensed
//  } 
//}
void init_BLE()
{
  
  // Create the BLE Device
  BLEDevice::init("ESP32 UART Test"); // Give it a name

  // Create the BLE Server
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic
  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID_TX,
                      BLECharacteristic::PROPERTY_NOTIFY
                    );
                      
  pCharacteristic->addDescriptor(new BLE2902());

  BLECharacteristic *pCharacteristic = pService->createCharacteristic(
                                         CHARACTERISTIC_UUID_RX,
                                         BLECharacteristic::PROPERTY_WRITE
                                       );

  pCharacteristic->setCallbacks(new MyCallbacks());

  // Start the service
  pService->start();

  // Start advertising
  pServer->getAdvertising()->start();
  Serial.println("Waiting a client connection to notify...");
  
  
  
  
  }
  void init_ADXLs()
  {
     for(i=0;i<nADXL345;i++)
  {

  ADXL345(adxlPin[i]).powerOn();                     // Power on the ADXL345

  ADXL345(adxlPin[i]).setRangeSetting(2);           // Give the range settings
                                      // Accepted values are 2g, 4g, 8g or 16g
                                      // Higher Values = Wider Measurement Range
                                      // Lower Values = Greater Sensitivity

  ADXL345(adxlPin[i]).setSpiBit(0);                  // Configure the device to be in 4 wire SPI mode when set to '0' or 3 wire SPI mode when set to 1
                                      // Default: Set to 1
                                      // SPI pins on the ATMega328: 11, 12 and 13 as reference in SPI Library 
   
  ADXL345(adxlPin[i]).setActivityXYZ(1, 0, 0);       // Set to activate movement detection in the axes "ADXL345(i).setActivityXYZ(X, Y, Z);" (1 == ON, 0 == OFF)
  ADXL345(adxlPin[i]).setActivityThreshold(75);      // 62.5mg per increment   // Set activity   // Inactivity thresholds (0-255)
 
  ADXL345(adxlPin[i]).setInactivityXYZ(1, 0, 0);     // Set to detect inactivity in all the axes "ADXL345(i).setInactivityXYZ(X, Y, Z);" (1 == ON, 0 == OFF)
  ADXL345(adxlPin[i]).setInactivityThreshold(75);    // 62.5mg per increment   // Set inactivity // Inactivity thresholds (0-255)
  ADXL345(adxlPin[i]).setTimeInactivity(10);         // How many seconds of no activity is inactive?

  ADXL345(adxlPin[i]).setTapDetectionOnXYZ(0, 0, 1); // Detect taps in the directions turned ON "ADXL345(i).setTapDetectionOnX(X, Y, Z);" (1 == ON, 0 == OFF)
 
  // Set values for what is considered a TAP and what is a DOUBLE TAP (0-255)
  ADXL345(adxlPin[i]).setTapThreshold(50);           // 62.5 mg per increment
  ADXL345(adxlPin[i]).setTapDuration(15);            // 625 μs per increment
  ADXL345(adxlPin[i]).setDoubleTapLatency(80);       // 1.25 ms per increment
  ADXL345(adxlPin[i]).setDoubleTapWindow(200);       // 1.25 ms per increment
 
  // Set values for what is considered FREE FALL (0-255)
  ADXL345(adxlPin[i]).setFreeFallThreshold(7);       // (5 - 9) recommended - 62.5mg per increment
  ADXL345(adxlPin[i]).setFreeFallDuration(30);       // (20 - 70) recommended - 5ms per increment
 
  // Setting all interupts to take place on INT1 pin
  //ADXL345(i).setImportantInterruptMapping(1, 1, 1, 1, 1);     // Sets "ADXL345(i).setEveryInterruptMapping(single tap, double tap, free fall, activity, inactivity);" 
                                                        // Accepts only 1 or 2 values for pins INT1 and INT2. This chooses the pin on the ADXL345 to use for Interrupts.
                                                        // This library may have a problem using INT2 pin. Default to INT1 pin.
  
  // Turn on Interrupts for each mode (1 == ON, 0 == OFF)
  ADXL345(adxlPin[i]).InactivityINT(1);
  ADXL345(adxlPin[i]).ActivityINT(1);
  ADXL345(adxlPin[i]).FreeFallINT(1);
  ADXL345(adxlPin[i]).doubleTapINT(1);
  ADXL345(adxlPin[i]).singleTapINT(1);
 

  }
    
    
    }
