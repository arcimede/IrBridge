/* BLE-ClientMultiserver for the IrBridge
 * cereated by Joerg Schnyder based on:
 *
 * A BLE client example that is rich in capabilities.
 * There is a lot new capabilities implemented.
 * Copyright © 2022 Arduino SA
 * author unknown
 * updated by chegewara
 * modified for 3 connections by Sylvain Lareau
 *  usefull exemple https://www.youtube.com/watch?v=iCKIIMrphtg

 * IMPORTANT: might not work with older version
   Arduino Version: 2.0.1-nightly-20221024
   ESP32 2.0.5

  and the
  IRremote library
  from
  Armin Joachimsmeyer 
  armin.arduino@gmail.com

  created 2024 by
  Joerg Schnyder
  www.systech-gmbh.ch

  Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files.
  The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
  
 */

/*
EN
IR-Transmiter for the IR-Bridge
This software version is a BLE client (Bluetooth Low Energy client) for sending the IR codes detected by severs (max. 3).
This version is connecting to a serves advertising the desired Service and Characteristic.
The bmeServiceUUID and IrCodeCharacteristicUUID must match!!

In case of missing all servers or loosing their connections this version is RESETING the software.

DE
IR-Sender für die IR-Bridge (IR-Brücke)
Diese Software ist ein BLE-Client (Bluetooth-Low-Energy-Client) der die von Servern (max. 3)empfangenen und an den Client übermittelten IR-Codes aussendet.
Diese Version verbindet sich mit bis zu 3 BLE-Servern die den benötigten Service und die benötigte Characteristic melden.
Die bmeServiceUUID und IrCodeCharacteristicUUID müssen übereinstimmen!!

Wird kein Server verbunden oder die Verbindungen unterbrochen so wird nach einer Pause ein NEUSTART der Software durchgeführt.

****************************************************

The following ir protocols are transmited:
Es werden folgende IR-Protokolle übertagen:

NEC, SAMSUNG, SONY, PANASONIC, DENON, SHARP, JVC

if there is a need for other protocols , the software has to be modyfied...
für weitere Protokolle muss die Software angepasst werden....

*/


#include "BLEDevice.h"

#include "PinDefinitionsESP32.h"  // Define macros for input and output pin etc. / Definition von Macros für IRemote und den ESP32 
#include <IRremote.hpp>

#define DELAY_AFTER_SEND 2000
#define DELAY_AFTER_LOOP 5000

// Common Declarations / Allgemeine Deklarationen

String version = "1.0";            // The version of this software / Die Version dieser Software

uint8_t SERVER0_LED = 25;          // LED to show if server 0 is connected / LED zur Anzeige wenn Server 0 verbunden ist
uint8_t SERVER1_LED = 26;          // LED to show if server 1 is connected / LED zur Anzeige wenn Server 1 verbunden ist
uint8_t SERVER2_LED = 27;          // LED to show if server 2 is connected / LED zur Anzeige wenn Server 2 verbunden ist

uint8_t LEDPWM_SF = 128;           // brightness if a server is found (brigth) / Helligkeit der LED wenn ein Server gefunden wurde (Hell)
uint8_t LEDPWM_ON = 16;            // dimmed brightness if connected to a server / reduzierte Helligkeit der LED wenn der Server verbunden ist 
uint8_t LEDPWM_OFF = 0;            // Led OFF / LEd AUS

long FreeHeap;                              // actula minimum of HEAP /  aktuelle minimale Grösse des HEAP 
const long MinimumHeap = 50000;             // minimum required HEAP / minimale Grösse des HEAP
                                            // a value below causes a RESTART / ein Unterschreiten des Werts verursacht einen NEUSTART
const long checkHeapIntervall = 30000;      // intervall to check the HEAP / Interval zur Überprüfung des HEAPs
long HeapChecked = millis();                // timestamp of checking the HEAP / Zeitstempel der HEAP-Überprüfung 


// IrRemote Declarations / IRemote Deklarationen
// Variables to store IrCode / Variable zum Speichern der IrCodes
char* IrCodeChar;                  // the Ircode to send / der zu sendende IrCode 
bool newIrCode;                    // flag to indicate that a new code has arrived / Flag zur Signalisierung eines neuen IrCodes

String strIrCode[16];              // ring buffer for IrCodes /Ringpuffer für IrCodes
int ptrWriteStrIrCode = 0;         // pointer to the strIrCode ring buffer for writing / Zeiger auf strIrCodes-Puffer zum Schreiben
int ptrReadStrIrCode = 0;          // pointer to the strIrCode ring buffer for reading / Zeiger auf strIrCodes-Puffer zum Lesen

// BLE Declarations / BLE Deklarationen

// The remote service we wish to connect to / Unser gewünschter Remote-Servise
// BLEUUIDS

static BLEUUID serviceUUID("6E400001-B5A3-F393-E0A9-E50E24DCCA9E"); //IrCodeService
static BLEUUID charUUID("6E400003-B5A3-F393-E0A9-E50E24DCCA9E");    //IrCodeCharacteristic

static boolean doScan = true;        // a scan will be done if true / Ausführen eines Suchlaufs wenn wahr

static boolean doConnect0 = false;   // try to conect server 0 / versuche Server 0 zu verbinden
static boolean connected0 = false;   // server 0 is connected / Server 0 ist verbunden

static boolean doConnect1 = false;   // try to conect server 1 / versuche Server 1 zu verbinden
static boolean connected1 = false;   // server 1 is connected / Server 1 ist verbunden

static boolean doConnect2 = false;   // try to conect server 2 / versuche Server 2 zu verbinden
static boolean connected2 = false;   // server 2 is connected / Server 2 ist verbunden

static boolean okS0 = false;         // server0 has connected and sends the desired notification! / Server 0 ist verbunden und sendet die gewünschte Meldung
static boolean okS1 = false;         // server1 has connected and sends the desired notification! / Server 1 ist verbunden und sendet die gewünschte Meldung
static boolean okS2 = false;         // server2 has connected and sends the desired notification! / Server 2 ist verbunden und sendet die gewünschte Meldung

uint8_t maxRetries = 4;              // max number of reconnecting retrais / maximale Versuche zum wiederherstellen einer Verbimndung
uint8_t retriesS0 = 0;               // retrais for reconect for server0 / Zähler für die Wiederverbindungs-Versuche von Server 0
uint8_t retriesS1 = 0;               // retrais for reconect for server1 / Zähler für die Wiederverbindungs-Versuche von Server 1
uint8_t retriesS2 = 0;               // retrais for reconect for server2 / Zähler für die Wiederverbindungs-Versuche von Server 2

static boolean notification0 = false;   // flag if server 0 can notify the desired value / Flag wenn Server 0 den gewünschten Wert anzeigen kann
static boolean notification1 = false;   // flag if server 1 can notify the desired value / Flag wenn Server 1 den gewünschten Wert anzeigen kann
static boolean notification2 = false;   // flag if server 2 can notify the desired value / Flag wenn Server 2 den gewünschten Wert anzeigen kann

static BLERemoteCharacteristic* pRemoteCharacteristic;  // pointer to the desired characteristics / Zeiger auf die gewünschte Characteristics

static BLEAdvertisedDevice* myDevice0;  // pointer to the device 0 / Zeiger auf das Gerät 0
static BLEAdvertisedDevice* myDevice1;  // pointer to the device 1 / Zeiger auf das Gerät 1
static BLEAdvertisedDevice* myDevice2;  // pointer to the device 2 / Zeiger auf das Gerät 2

static BLEAddress *pServerAddress;      // pointer to the  BLEv address / Zeiger auf die BLE-Adresse

String strBleAddress;                   // string of a BLE address / Zeichenkette einer BLE-Adresse
String strBleAddress0 = "";             // string of the BLE adress of server 0 / Zeichenkette der BLE-Adresse vvon Server 0
String strBleAddress1 = "";             // string of the BLE adress of server 1 / Zeichenkette der BLE-Adresse vvon Server 1
String strBleAddress2 = "";             // string of the BLE adress of server 2 / Zeichenkette der BLE-Adresse vvon Server 2

int numberOfScans = 0;              // Scan number counter / Zähler für die Suchlauf-Durchgänge
const int maxNumberOfScans = 8;     // max number of scans / maximale Anzahl der Suchläufe


static void notifyCallback0(        // callback for sent Values of server 0 / Callback für gesendete Werte von Server 0
  BLERemoteCharacteristic* pBLERemoteCharacteristic,
  uint8_t* pData,
  size_t length,
  bool isNotify) {
    IrCodeChar = (char*)pData;
    IrCodeChar[length] = 0;   // to terminate the IrCodeChar array (string) / zum Abschliessen der Zeichenkette
    strIrCode[ptrWriteStrIrCode]=(String(IrCodeChar));  // write the IrCode into the ring buffer / schtreiben des IrCodes in den Ring-Puffer
    ptrWriteStrIrCode++;                                // prepeare ring buffer for the next IrCode / Vorbereiten des Ring-Puffers für den nächsteb IrCode
    if (ptrWriteStrIrCode > 15) {
      ptrWriteStrIrCode = 0;   
    }
    Serial.print("Server0 IrCode: ");
    Serial.println(IrCodeChar);
  }

static void notifyCallback1(      // callback for sent Values of server 1 / Callback für gesendete Werte von Server 1
  BLERemoteCharacteristic* pBLERemoteCharacteristic,
  uint8_t* pData,
  size_t length,
  bool isNotify) {
    IrCodeChar = (char*)pData;
    IrCodeChar[length] = 0;   // to terminate the IrCodeChar array (string) / zum Abschliessen der Zeichenkette
    strIrCode[ptrWriteStrIrCode]=(String(IrCodeChar));  // write the IrCode into the ring buffer / schtreiben des IrCodes in den Ring-Puffer
    ptrWriteStrIrCode++;                                // prepeare ring buffer for the next IrCode / Vorbereiten des Ring-Puffers für den nächsteb IrCode
    if (ptrWriteStrIrCode > 15) {
      ptrWriteStrIrCode = 0;   
    }
    Serial.print("Server1 IrCode: ");
    Serial.println(IrCodeChar);
  }

static void notifyCallback2(       // callback for sent Values of server 2 / Callback für gesendete Werte von Server 2
  BLERemoteCharacteristic* pBLERemoteCharacteristic,
  uint8_t* pData,
  size_t length,
  bool isNotify) {
    IrCodeChar = (char*)pData;
    IrCodeChar[length] = 0;   // to terminate the IrCodeChar array (string) / zum Abschliessen der Zeichenkette
    strIrCode[ptrWriteStrIrCode]=(String(IrCodeChar));  // write the IrCode into the ring buffer / schtreiben des IrCodes in den Ring-Puffer
    ptrWriteStrIrCode++;                                // prepeare ring buffer for the next IrCode / Vorbereiten des Ring-Puffers für den nächsteb IrCode
    if (ptrWriteStrIrCode > 15) {
      ptrWriteStrIrCode = 0;   
    }
    Serial.print("Server2 IrCode: ");
    Serial.println(IrCodeChar);
  }


class MyClientCallback0 : public BLEClientCallbacks {  // callbacks foe sever 0 concerning connecting and disconnecting 
                                                       // callbacks für Server 0 bezüglich Verbindungsaubau (verbinden / trennen)
   void onConnect(BLEClient* pClient) {
    retriesS0=0;
    Serial.println(" - Connected to server 0");
  }
  void onDisconnect(BLEClient* pClient) {
    pClient->disconnect();                            // clear the created elements in HEAP / löschen der erzeugten Elemente im HEAP
    connected0 = false;
    notification0 = false;
    analogWrite(SERVER0_LED, LEDPWM_OFF);             // LED Server 0 OFF / AUS
    Serial.println("onDisconnect 0 ");
  }
};

class MyClientCallback1 : public BLEClientCallbacks {  // callbacks foe sever 1 concerning connecting and disconnecting 
                                                       // callbacks für Server 1 bezüglich Verbindungsaubau (verbinden / trennen)
   void onConnect(BLEClient* pClient) {
  }
  void onDisconnect(BLEClient* pClient) {
    pClient->disconnect();                            // clear the created elements in HEAP / löschen der erzeugten Elemente im HEAP
    connected1 = false;
    notification1 = false;
    analogWrite(SERVER1_LED, LEDPWM_OFF);             // LED Server 1 OFF / AUS
    Serial.println("onDisconnect 1 ");
  }
};

class MyClientCallback2 : public BLEClientCallbacks {  // callbacks foe sever 2 concerning connecting and disconnecting 
                                                       // callbacks für Server 2 bezüglich Verbindungsaubau (verbinden / trennen)
   void onConnect(BLEClient* pClient) {
  }
  void onDisconnect(BLEClient* pClient) {
    pClient->disconnect();                            // clear the created elements in HEAP / löschen der erzeugten Elemente im HEAP
    connected2 = false;
    notification2 = false;
    analogWrite(SERVER2_LED, LEDPWM_OFF);             // LED Server 2 OFF / AUS
    Serial.println("onDisconnect 2");
  }
};

bool connectToServer(int server, BLEUUID serviceUUID, BLEUUID charUUID) {  // connect the server / verbinden des Servers
    Serial.print("Forming a connection to ");
    Serial.print("Server ");
    Serial.print(server);
    Serial.print(" : ");

    BLEClient*  pClient  = BLEDevice::createClient();

    if (server == 0)
      {
        pClient->setClientCallbacks(new MyClientCallback0());
        Serial.println(myDevice0->getAddress().toString().c_str());
      }     
    else if (server == 1)
      {
        pClient->setClientCallbacks(new MyClientCallback1());
        Serial.println(myDevice1->getAddress().toString().c_str());
      }
    else if (server == 2)
    { 
       pClient->setClientCallbacks(new MyClientCallback2());
       Serial.println(myDevice2->getAddress().toString().c_str());
    }

    Serial.println(" - Created client"); 

    // Connect to the remove BLE Server.
    if (server == 0)
    {
      if (pClient->connect(myDevice0)) {;  // if you pass BLEAdvertisedDevice instead of address, it will be recognized type of peer device address (public or private)
        // retriesS0++;
      }
      else {
        Serial.println("Connecting to server 0 failed");
        retriesS0++;
        pClient->disconnect();
        return false;      
      }
    }
    else if (server == 1)
    {
      if (pClient->connect(myDevice1)) {;  // if you pass BLEAdvertisedDevice instead of address, it will be recognized type of peer device address (public or private)
        // retriesS1++;
      }
      else {
        Serial.println("Connecting failedto server 1");
        retriesS1++;
        pClient->disconnect();
        return false;      
      }
    }
    else if (server == 2)
    {
      if (pClient->connect(myDevice2)) {;  // if you pass BLEAdvertisedDevice instead of address, it will be recognized type of peer device address (public or private)
        // retriesS2++;
      }
      else {
        Serial.println("Connecting failed to server 2");
        retriesS2++;
        pClient->disconnect();
        return false;      
      }
    }

    pClient->setMTU(64); //set client to request 64 bytes of MTU from server (default is 23 otherwise, maximum 517)
                         // setzen der Paket-Grösse auf 32 Bytes (Standard wäre 23 Bytes, Maximum 517)

    // Obtain a reference to the service we are after in the remote BLE server / kreierten der Referenz zum Service auf dem verbundenen Server
    BLERemoteService* pRemoteService = pClient->getService(serviceUUID);
    if (pRemoteService == nullptr) {
      Serial.print("Failed to find our service UUID sensor ");
      Serial.print (server);
      Serial.print (" : ");
      Serial.println(serviceUUID.toString().c_str());
      pClient->disconnect();
      return false;
    }
    Serial.print(" - Found our service on ");       // that worked! / das hat geklappt!
    Serial.println (server);


    // Obtain a reference to the characteristic in the service of the remote BLE server / kreieren der Referenz zur Charakteristic
    pRemoteCharacteristic = pRemoteService->getCharacteristic(charUUID);
    if (pRemoteCharacteristic == nullptr) {
      Serial.print("Failed to find our characteristic UUID: ");
      Serial.println(charUUID.toString().c_str());
      pClient->disconnect();
      return false;
    }

    Serial.print(" - Found our characteristic on ");       // that worked! / das hat geklappt!
    Serial.println (server);

    if (server == 0) {
      if(pRemoteCharacteristic->canNotify())
        pRemoteCharacteristic->registerForNotify(notifyCallback0);
      connected0 = true;
    return true;
    }
    else  if (server == 1) {
      if(pRemoteCharacteristic->canNotify())
        pRemoteCharacteristic->registerForNotify(notifyCallback1);
      connected1 = true;
    return true;
    }
    else  if (server == 2) {
      if(pRemoteCharacteristic->canNotify())
      pRemoteCharacteristic->registerForNotify(notifyCallback2);
      connected2 = true;
    return true;
    }
}

/**
 * Scan for BLE servers and find the one that advertises the service we are looking for
 * Suchlauf für einen geeigneten BLE Server der den gewünschten Service anbietet
 */
class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
 /**
   * Called for each advertising BLE server.
   * aufgerufen für jeden meldenden BLE-Server
   */
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    Serial.print("BLE Advertised Device found: ");
    Serial.println(advertisedDevice.toString().c_str());

    // We have found a device, let us now see if it contains the service we are looking for.
    // Wir haben ein Gerät gefunden und prüfen, ob der gewünsche Service angeboten wird

    if (advertisedDevice.haveServiceUUID() && advertisedDevice.isAdvertisingService(serviceUUID)) {

      Serial.println("Server with the desired ServiceUUID found");
      Serial.println(advertisedDevice.toString().c_str());

      pServerAddress = new BLEAddress(advertisedDevice.getAddress()); //Address of advertiser
      strBleAddress = pServerAddress->toString().c_str();

      /* for Debbuging / zur Fehlersuche
      Serial.println(strBleAddress);
      Serial.print("Adr0: ");
      Serial.print(strBleAddress0);
      Serial.print("  Adr1: ");
      Serial.print(strBleAddress1);
      Serial.print("  Adr2: ");
      Serial.println(strBleAddress2);
      */           

      // if the found server was not connected before / falls der Server noch nicht verbunden wurde
      if (!strBleAddress.equals(strBleAddress0) && !strBleAddress.equals(strBleAddress1) && !strBleAddress.equals(strBleAddress2))

      Serial.println("New Server found!");
      /*
      Serial.print("connected0 ");
      Serial.print(connected0);
      Serial.print(" doConnect0 ");
      Serial.println(doConnect0);
      Serial.print("connected1 ");
      Serial.print(connected1);
      Serial.print(" doConnect1 ");
      Serial.println(doConnect1);
      Serial.print("connected2 ");
      Serial.print(connected2);
      Serial.print(" doConnect2 ");
      Serial.println(doConnect2);
      */

      {
        if (!(connected0 || doConnect0))
        {
          strBleAddress0 = strBleAddress;
          myDevice0 = new BLEAdvertisedDevice(advertisedDevice);
          Serial.println(myDevice0->toString().c_str());
          doConnect0 = true;
          analogWrite(SERVER0_LED, LEDPWM_SF);  // LED brigth on / LED hell EIN
          Serial.println("Found Server 0");
        }
        else if (!(connected1 || doConnect1))
        {
          strBleAddress1 = strBleAddress;
          myDevice1 = new BLEAdvertisedDevice(advertisedDevice);  // not used?
          doConnect1 = true;
          analogWrite(SERVER1_LED, LEDPWM_SF);  // LED brigth on / LED hell EIN
          Serial.println("Found Server 1");
        }
        else if (!(connected2 || doConnect2))
        {
          strBleAddress2 = strBleAddress;
          myDevice2 = new BLEAdvertisedDevice(advertisedDevice);  // not used?
          doConnect2 = true;
          analogWrite(SERVER2_LED, LEDPWM_SF);  // LED brigth on / LED hell EIN
          Serial.println("Found Server 2");
        }
      }
    } // Found our a server

  } // onResult
}; // MyAdvertisedDeviceCallbacks

void SendIrCode(String IrCode) {       // to decode and send an IrCode / zum Decodieren und Senden eines IrCodes
  int n=0;
  int i=0;
  unsigned int CharValue;
  unsigned int Number=0;
  unsigned int IrCodeValue[4]; // 0=Protocoll, 1=Address, 2=Code, 3=NumberOfBits...

  Serial.print(IrCode);
  Serial.print(" length: "); 
  Serial.println(IrCode.length());

  while (n <= IrCode.length()) {      // creating the needed Values of Protocoll, Address, Code, ..  and converting ASCII -> Value

                                      // Erzeugen der benötigten Werte von Protokoll, Adresse, Code, .. and umwandeln ASCII -> Wert
      CharValue = IrCode[n];
      if (CharValue != 45){  // is it "-"?
        if ((CharValue > 47) && (CharValue < 58)) {
          CharValue = CharValue - 48;
        }
        else if ((CharValue > 64) && (CharValue < 71)) {
          CharValue = CharValue - 55;
        }
        else if ((CharValue > 96) && (CharValue < 103)) {
          CharValue = CharValue - 87;
        } 
        else {
          if (n != IrCode.length()) {
            IrCodeValue[0] = 0; // unknown Code..
            Serial.println("Return");
            return;   // invalid IrCode!
          }
        }
        if (n != IrCode.length()) {
          Number = (Number * 16) + CharValue;
        }
        n++;
      }
    else {
         IrCodeValue[i]=Number;
         Number = 0;
         n++;
         i++;
      } 
  }

  // showing the values / Anzeigen der Werte
   
  IrCodeValue[i]=Number;

  Serial.print("Protocol: ");
  Serial.print(IrCodeValue[0]);
  Serial.print(" Address: ");
  Serial.print(IrCodeValue[1]);
  Serial.print(" Command: ");
  Serial.print(IrCodeValue[2]);
  Serial.print(" Number of Bits: ");
  Serial.println(IrCodeValue[3]);

  // sending the IrCode / Senden des IrCodes

    switch (IrCodeValue[0]) { 
    case NEC:
        Serial.println("Send NEC");
        IrSender.sendNEC(IrCodeValue[1], IrCodeValue[2], 3);
        break;
    case SAMSUNG:
        Serial.println("Send SAMSUNG");
        IrSender.sendSamsung(IrCodeValue[1], IrCodeValue[2], 3);
        break;
    case SONY:
        Serial.println("Send SONY");
        IrSender.sendSony(IrCodeValue[1], IrCodeValue[2], 2, IrCodeValue[3]);
        break;
    case PANASONIC:
        Serial.println("Send PANASONIC");
        IrSender.sendPanasonic(IrCodeValue[1], IrCodeValue[2], 2);
        break;
    case DENON:
        Serial.println("Send DENON");
        IrSender.sendDenon(IrCodeValue[1], IrCodeValue[2], 3);
        break;
    case SHARP:
        IrSender.sendSharp(IrCodeValue[1], IrCodeValue[2], 3);
        break;
    case JVC:
        Serial.println("Send JVC");
        IrSender.sendJVC((uint8_t) IrCodeValue[1], (uint8_t) IrCodeValue[2], 3); // casts are required to specify the right function
        break;
    case RC5:
        // Serial.println("Send RC5");
        // IrSender.sendRC5(IrCodeValue[1], IrCodeValue[2], 3, !tSendRepeat); // No toggle for repeats
        break;
    case RC6:
        // No toggle for repeats//        IrSender.sendRC6(tAddress, tCommand, aNumberOfRepeats, !tSendRepeat); // No toggle for repeats
        break;
    default:
        break;
  }

}

//*********************************  SETUP 
void setup() {
  Serial.begin(115200);
  Serial.println("Starting Arduino BLE Client application.");
  Serial.print("Version: ");
  Serial.println(version);

  analogWrite(SERVER0_LED, LEDPWM_OFF);
  analogWrite(SERVER1_LED, LEDPWM_OFF);
  analogWrite(SERVER2_LED, LEDPWM_OFF);

  BLEDevice::init("");

  /*
  Retrieve a Scanner and set the callback we want to use to be informed when we
  have detected a new device.  Specify that we want active scanning and start the
  scan to run for 5 seconds.

  Vorbereiten des Suchlaufs und Erzeugen des callbacks das die gefundenen BLE-Server untersucht.
  Spezifizieren, dass wir einen aktiven Suchlauf für 5 Sekunden benötigen.
  Start des Suchlaufs.

  */

  BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setWindow(400);  // millisec
  pBLEScan->setActiveScan(true);
  if (doScan) {
    pBLEScan->start(5, false); // 5 sec
    numberOfScans++;    // this is the first scan / dies ist der erste Suchlauf
  }

} // End of setup.


// This is the Arduino main loop function / Hauptschleife
void loop() {
 
  if ( millis() - HeapChecked >= checkHeapIntervall) {
    FreeHeap = esp_get_minimum_free_heap_size();
    Serial.print("Free HEAP is: ");
    Serial.println(FreeHeap);
    HeapChecked = millis();
    if (FreeHeap < MinimumHeap) {
      Serial.print("Restarting due of lack of HEAP!!!");
      delay(5000);
      ESP.restart();      
    }

  }

/* 
If the flag "doConnect" is true then we have scanned for and found the desired
BLE Server with which we wish to connect.  Now we connect to it.  Once we are 
connected we set the connected flag to be true.

Wenn das Flag "doConnect" gesetzt ist kann ein Server verbunden werden.
Ist der Server verbunden, so wird das "connected" Flag gesetzt und das "doConnect" gelöscht.

*/

/*
Serial.print("doConnect0 ");
Serial.println(doConnect0);
Serial.print("doConnect1 ");
Serial.println(doConnect1);
Serial.print("doConnect2 ");
Serial.println(doConnect2);
*/

  // connecting the servers / Verbinden der Server

  if (doConnect0 == true) {
    if (connectToServer(0, serviceUUID, charUUID)) {
      Serial.println("We are now connected to the BLE Server 0");
      } else {
      Serial.println("We have failed to connect to the server 0");
    }
    doConnect0 = false;
    }

  if (doConnect1 == true) {
    if (connectToServer(1, serviceUUID, charUUID)) {
      Serial.println("We are now connected to the BLE Server 1");
    } else {
      Serial.println("We have failed to connect to the server 1");
    }
    doConnect1 = false;
  }

  if (doConnect2 == true) {
     if (connectToServer(2, serviceUUID, charUUID)) {
      Serial.println("We are now connected to the BLE Server 2");
    } else {
      Serial.println("We have failed to connect to the server 2");
    }
    doConnect2 = false;
  }
/*
Serial.print("connected0 "); Serial.println(connected0);
Serial.print("connected1 ");Serial.println(connected1);
Serial.print("connected2 ");Serial.println(connected2);
*/

/*
Serial.print("notification0 ");Serial.println(notification0);
Serial.print("notification1 ");Serial.println(notification1);
Serial.print("notification2 ");Serial.println(notification2);
*/

/*Serial.print("doScan0 ");
Serial.println(doScan0);
Serial.print("doScan1 ");
Serial.println(doScan1);
Serial.print("doScan1 ");
Serial.println(doScan1);*/

  //if new IrCode readings are available, send them
  // wenn neu IrCodes vorhanden sind, sende diese

  if (ptrReadStrIrCode != ptrWriteStrIrCode) {
    SendIrCode(strIrCode[ptrReadStrIrCode]);
    Serial.print("ptrRead: ");
    Serial.println(ptrReadStrIrCode);
    Serial.print("strIrCode: ");
    Serial.println(strIrCode[ptrReadStrIrCode]);    
    Serial.print("ptrWrite: ");
    Serial.println(ptrWriteStrIrCode);
    ptrReadStrIrCode++;
    if (ptrReadStrIrCode > 15) {
      ptrReadStrIrCode = 0;
    }
  }

//  Need to repeat  getScan()->start   or it wont connect to the second+ server
//  Die Suchläufen mussen widerholt werden, damit genug (max .3) Server gefunden werden 

if (doScan) {
  if (connected0 && connected1 && connected2) {
    BLEDevice::getScan()->stop();
    doScan = false;
    Serial.println("Scan complete");
  }
  else {
    numberOfScans++;
    if (numberOfScans >= maxNumberOfScans) {
      doScan = false;
      BLEDevice::getScan()->stop();
      Serial.println("Scan aborted due of reaching the maximum numbers of scans ");
    }
    else { 
      Serial.print("Scan number ");
      Serial.println(numberOfScans);
      BLEDevice::getScan()->start(5, false);
    } 
  }
}
else {      
  // if there were no servers found or all the found servers have lost connection for a longer time, the client ist restartet
  // werden keine Server gefunden oder alle gefundenen Server haben die Verbindung länger unterbrochen wird der Client neu gestartet

  if (strBleAddress0 == "" && strBleAddress1 == "" && strBleAddress2 == "") {
    delay(5000);
    ESP.restart();
  }
}



  // Turn notification on / Notification aktivieren

  if (connected0) {
    if (notification0 == false) {
      Serial.println(F("Turning Notification On for server 0"));
      const uint8_t onPacket[] = {0x1, 0x0};
      pRemoteCharacteristic->getDescriptor(BLEUUID((uint16_t)0x2902))->writeValue((uint8_t*)onPacket, 2, true);
      notification0 = true;
      Serial.println("Server 0 Notification OK");
      analogWrite(SERVER0_LED, LEDPWM_ON);
      okS0 = true;      // server0 is sending the notifications / server 0 sendet die gewünschten Werte
    }  
  }
  else{
    if (okS0) {        // the server0 was found and connected so we know it / server 0 ist verbunden und funktioniert
      if (connectToServer(0, serviceUUID, charUUID)) {
        Serial.println("We are now reconnected to the BLE Server 0");
      } 
      else {
      Serial.println("We have failed to reconnect to the server 0");
      if (retriesS0 > maxRetries) {
        strBleAddress0 = "";
        okS0 = false;      // server 0 does not exist anymore!!
      }
      }
    }      
  }
  
  if (connected1) {
    if (notification1 == false) {
      Serial.println(F("Turning Notification On for server 1"));
      const uint8_t onPacket[] = {0x1, 0x0};
      pRemoteCharacteristic->getDescriptor(BLEUUID((uint16_t)0x2902))->writeValue((uint8_t*)onPacket, 2, true);
      notification1 = true;
      Serial.println("Server 1 Notification OK");
      analogWrite(SERVER1_LED, LEDPWM_ON);
      okS1 = true;      // server1 is sending the notifications / server 1 sendet die gewünschten Werte
    } 
  }
  else{
    if (okS1) {        // the server0 was found and connected so we know it / server 1 ist verbunden und funktioniert
      if (connectToServer(1, serviceUUID, charUUID)) {
        Serial.println("We are now reconnected to the BLE Server 1");
      } 
      else {
      Serial.println("We have failed to reconnect to the server 1");
      if (retriesS1 > maxRetries) {
        strBleAddress1 = "";
        okS1 = false;     // server 1 does not exist anymore!!      
      }
      }
    }      
  }

  if (connected2) {
    if (notification2 == false) {
      Serial.println(F("Turning Notification On for server 2"));
      const uint8_t onPacket[] = {0x1, 0x0}; 
      pRemoteCharacteristic->getDescriptor(BLEUUID((uint16_t)0x2902))->writeValue((uint8_t*)onPacket, 2, true);
      notification2 = true;
      Serial.println("Server 2 Notification OK");
      analogWrite(SERVER2_LED, LEDPWM_ON);
      okS2 = true;      // server2 is sending the notifications / server 2 sendet die gewünschten Werte
    }
  }
  else{
    if (okS2) {        // the server0 was found and connected so we know it / server 2 ist verbunden und funktioniert
      if (connectToServer(2, serviceUUID, charUUID)) {
        Serial.println("We are now reconnected to the BLE Server 2");
      } 
      else {
      Serial.println("We have failed to reconnect to the server 2");
      if (retriesS2 > maxRetries) {
        strBleAddress2 = "";
        okS2 = false;      // server 2 does not exist anymore!!
      }
      }
    }      
  }

  /*
  Serial.print("Retraies (S0, S1, S2) : ");
  Serial.print(retriesS0);
  Serial.print(", ");
  Serial.print(retriesS1);
  Serial.print(", ");
  Serial.println(retriesS2);
  */

  delay(100); // Delay 100 ms between loops / 100m Pause zwischen den Durchläufen
} // End of loop
