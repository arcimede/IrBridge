/*
  BLE-IrReceiver
  created 2024 by
  Joerg Schnyder
  www.systech-gmbh.ch
  
  based on the following sources...

  Based on Neil Kolban example for IDF: https://github.com/nkolban/esp32-snippets/blob/master/cpp_utils/tests/BLE%20Tests/SampleNotify.cpp
  Ported to Arduino ESP32 by Evandro Copercini
    

  IRremote library
  from
  Armin Joachimsmeyer 
  armin.arduino@gmail.com

  Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files.
  The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software. 

  EN
  How it works:

  Create a BLE server that, once we receive a connection, will send notifications if a valid IrCode is detected.
  The service advertises itself as: 6E400001-B5A3-F393-E0A9-E50E24DCCA9E
  Has a characteristic of: 6E400003-B5A3-F393-E0A9-E50E24DCCA9E - used to send data with  "NOTIFY"

  The design of creating the BLE server is:
  1. Create a BLE Server
  2. Create a BLE Service
  3. Create a BLE Characteristic on the Service
  4. Create a BLE Descriptor on the characteristic
  5. Start the service.
  6. Start advertising.

  7. If the IRemote functions have detected a valid IrCode it is sent via notification
     to the connected IrBridge client.

  If the connection is lost, the server is advertising again until he found a client again...
  The ACTIV-LED only shows the detection of valid IrCodes!


  DE
  So funktionierts:

  Es wird ein BLE-Server kreiert, der wenn mit einem IrBridge-Client verbunden, die IrCodes mittels
  Notification übermittelt.

  Der Service meldet sich als: 6E400001-B5A3-F393-E0A9-E50E24DCCA9E
  und hat folgende Characteristics: 6E400003-B5A3-F393-E0A9-E50E24DCCA9E - benützt um Daten zu senden mit  "NOTIFY"

  Der Ablauf zum Erstellen des BLE-Servers ist:
  1. Erstelle den BLE Server
  2. Erstelle den BLE Service
  3. Erstelle die BLE Characteristic für Service
  4. Erstelle den BLE Descriptor für characteristic
  5. Starten des Service.
  6. Starten Ankündigungen.

  7. Wenn die IRemote-Funktionen einen gültigen IrCode erkannt haben, so sendet der Server diesen via Notification
     zum verbundenen IrBridge-Client.

  Wird die Verbindung unterbrochen, so sendet der Server erneut Ankündigungen des IrCode-Services bis er wiederum einen
  IrBridge-Client gefunden hat.

  Die ACTIV-LED zeigt nur das Erkennen von gültigen IrCodes!

*/


#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <IRremote.hpp>

#define IR_RECEIVE_PIN 4

BLEServer *pServer = NULL;
BLECharacteristic * pTxCharacteristic;
bool deviceConnected = false;
bool oldDeviceConnected = false;

uint8_t txValue[21];
bool NewCommand = false;

uint8_t ACTIV_LED = 16;              // LED to indicate an valid Ir-Code / LED zur Signalisierung gültiger IrCodes

uint8_t CONNECTED_LED = 25;          // LED to show if a server is connected / LED zur Anzeige wenn ein Klient verbunden ist

uint8_t LEDPWM_SF = 128;             // brightness if a servers is found
uint8_t LEDPWM_ON = 16;              // dimmed brightness if connected to a advertising server 
                                     // Helligkeit beim Finden eines anzeigenden (advertising) Servers
uint8_t LEDPWM_OFF = 0;              // Led OFF / Led AUS

String version = "1.0";              // version of this software / Version dieser Software

// Storage for the recorded IrCodes / Speicherplatz für aufgezeichnete IrCodes
struct storedIRDataStruct {
    IRData receivedIRData;
    // extensions for sendRaw / Erweiterung für senRAW (rohdaten, gegenwärtig nicht verwendet!)
    uint8_t rawCode[RAW_BUFFER_LENGTH]; // The durations if raw / Die Zeiten für Rohdaten
    uint8_t rawCodeLength;          // The length of the code / Länge des Codes
} sStoredIRData;

String StrStoredIRData;             // String for stored IrCodes / String für gespeicherte IrCodes

long IrCodeTime = millis();         // time of the last IrCode / Zeit des letzten IrCodes
const long IrCodeSupressTime = 250; // Supress time for non detected "repeats" [ms] / Unterdrückungs-Zeit für nicht detektierte "Repeats" [ms]


// See the following for generating UUIDs: / für die Erstellung von eigenen UUIDs:
// https://www.uuidgenerator.net/

#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E" // service UUID
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"


class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {               // on Connect / bei Verbindungs-Aufbau
      deviceConnected = true;
      analogWrite(CONNECTED_LED,LEDPWM_ON);             // Brightness defined by "LEDPWM_ON"
                                                        // Helligkeit definiert mit "LEDPWM_ON"
      Serial.println("IrBridge BLE Client connected");
    };

    void onDisconnect(BLEServer* pServer) {             // on Disconnect / bei Verbindungs-Abbruch
      deviceConnected = false;
      analogWrite(CONNECTED_LED,LEDPWM_OFF);            // LED off / LED aus
      Serial.println("IrBridge BLE Client disconnected");
    }
};


// Stores the IrCode for later playback in sStoredIRData
// Most of this code is just logging
// Speichert IrCodes für das spätere Abspielen in sStoredIRData
// Das meiste des Codes ist für das Anzeigen des Fortschritts

bool storeCode() {
    if (IrReceiver.decodedIRData.rawDataPtr->rawlen < 4) {
        Serial.print(F("Ignore data with rawlen="));
        Serial.println(IrReceiver.decodedIRData.rawDataPtr->rawlen);
        return false;
    }
    if (IrReceiver.decodedIRData.flags & IRDATA_FLAGS_IS_REPEAT) {
        Serial.println(F("Ignore repeat"));
        return false;                          // if repeats are needed comment this line
                                               // wenn Repeats benötigt werden muss die obige Zeile mit "//" versehen werden
    }
    if (IrReceiver.decodedIRData.flags & IRDATA_FLAGS_IS_AUTO_REPEAT) {
        Serial.println(F("Ignore autorepeat"));
        return false;
    }
    if (IrReceiver.decodedIRData.flags & IRDATA_FLAGS_PARITY_FAILED) {
        Serial.println(F("Ignore parity error"));
        return false;
    }
    /*
      Copy decoded data
      Kopieren der dekodierten Daten
     */
    sStoredIRData.receivedIRData = IrReceiver.decodedIRData;

    if (sStoredIRData.receivedIRData.protocol == UNKNOWN) {
        Serial.print(F("Received unknown code and store "));
        Serial.print(IrReceiver.decodedIRData.rawDataPtr->rawlen - 1);
        Serial.println(F(" timing entries as raw "));
        IrReceiver.printIRResultRawFormatted(&Serial, true);    // Output the results in RAW format / Ausgabe der Resultate im RAW-Format
        sStoredIRData.rawCodeLength = IrReceiver.decodedIRData.rawDataPtr->rawlen - 1;
        /*
         Store the current raw data in a dedicated array for later usage
         Speichern der Raw-Daten in einem vorgesehenen Array für den späteren Gebrauch
         */
        IrReceiver.compensateAndStoreIRResultInArray(sStoredIRData.rawCode);
    } 
    else {
        IrReceiver.printIRResultShort(&Serial);
        IrReceiver.printIRSendUsage(&Serial);
        sStoredIRData.receivedIRData.flags = 0;        // clear flags -esp. repeat- for later sending
                                                       // löschen von Flags
        Serial.print("Time: ");
        Serial.print(millis());
        Serial.println();                                               
    }
    return true;
}



void setup() {
  Serial.begin(115200);

  Serial.println("Starting IrBridge BLE Server application ");
  Serial.print("Version : ");
  Serial.println(version);

  IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK); 

  // Create the BLE Device
  // Erzeugen der BLE-Umgebung
  BLEDevice::init("IRE");

  // Create the BLE Server
  // Erzeugen des BLE-Servers
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  // Erzeugen des BLE-Services
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic
  // Erzeugen der BLE-Characteristics
  pTxCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID_TX, BLECharacteristic::PROPERTY_NOTIFY);                  
  pTxCharacteristic->addDescriptor(new BLE2902());

  // Start the service
  // Starten des Services
  pService->start();

  // Start advertising
  // Starten der Ankündigungen

  BLEAdvertising *pAdvertising = pServer->getAdvertising();

  // UsingiInstead of pAdvertising->addServiceUUID(SERVICE_UUID);
  // the following three lines are supressing showing double serviceUUIDs on advertising
  
  // Statt pAdvertising->addServiceUUID(SERVICE_UUID); 
  // die untenstehenden drei Zeilen um doppelte serviceUUID-Nennung zu unterdrücken
  BLEAdvertisementData BLEAdvData;
  BLEAdvData.setCompleteServices(BLEUUID(SERVICE_UUID));
  pAdvertising->setAdvertisementData(BLEAdvData);

  pAdvertising->start();

  Serial.println("Waiting a client connection to notify...");

  pinMode(ACTIV_LED, OUTPUT);
  digitalWrite(ACTIV_LED,LOW);
  analogWrite(CONNECTED_LED,LEDPWM_OFF);
}

void loop() {

  if (IrReceiver.decode()) {
    IrReceiver.resume();
    if (millis() - IrCodeTime > IrCodeSupressTime) {     // to supress repeated IrCodes / zur Unterdrückung von wiederholten IrCodes
      IrCodeTime = millis();
      if (storeCode()) {
        if (sStoredIRData.receivedIRData.protocol != 0) {
          digitalWrite(ACTIV_LED,HIGH);
          StrStoredIRData = String(sStoredIRData.receivedIRData.protocol,HEX);
          StrStoredIRData = StrStoredIRData + "-";
          StrStoredIRData = StrStoredIRData + String(sStoredIRData.receivedIRData.address,HEX);
          StrStoredIRData = StrStoredIRData + "-";
          StrStoredIRData = StrStoredIRData + String(sStoredIRData.receivedIRData.command,HEX);
          StrStoredIRData = StrStoredIRData + "-";
          StrStoredIRData = StrStoredIRData + String(sStoredIRData.receivedIRData.numberOfBits,HEX);
          Serial.print(StrStoredIRData.length());
          Serial.print(" ");
          Serial.println(StrStoredIRData);
          for (int i = 0; i < StrStoredIRData.length(); i++) {
            txValue[i] = StrStoredIRData[i];
            Serial.print(char(txValue[i]));
          }
          Serial.println();
          NewCommand = true;
          delay(100);
          digitalWrite(ACTIV_LED,LOW);
        }
        else {
        NewCommand = false;
        }
      }
    }
  }
  if (deviceConnected) {
    if (NewCommand) { 
      pTxCharacteristic->setValue(&txValue[0],StrStoredIRData.length());
      pTxCharacteristic->notify();
	    delay(10);           // bluetooth stack will go into congestion, if too many packets are sent
                           // Der BLE-Stack würde überfüllt, wenn zuviele Pakete gesendet werden
      NewCommand = false;
    }
	}

  // disconnecting
  if (!deviceConnected && oldDeviceConnected) {
      delay(500); // give the bluetooth stack the chance to get things ready
                  // Zeitverzögerung um der BLE-Software zu ermöglichen Ordnung zu schaffen...
      pServer->startAdvertising();   // restart advertising ( Neustart der Ankündigungen)
      Serial.println("start advertising");
      oldDeviceConnected = deviceConnected;
  }
  // connecting / am Verbinden
  if (deviceConnected && !oldDeviceConnected) {
      oldDeviceConnected = deviceConnected;
  }
}
