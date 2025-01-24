/*********
  based on:
  BLE_client
  by
  Rui Santos
  https://RandomNerdTutorials.com

  and the
  IRremote library
  from
  Armin Joachimsmeyer 
  armin.arduino@gmail.com

  created for the IrBridge
  2024 by
  Joerg Schnyder
  www.systech-gmbh.ch

  Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files.
  The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
*********/

/*
EN
IR-Transmiter for the IR-Bridge
This software version is a BLE client (Bluetooth Low Energy client) for sending the IR codes detected by the sever.
This version is connecting to a server named "bleServerName".
The bmeServiceUUID and IrCodeCharacteristicUUID must match!!

In case of missing the server or loosing the connection this version is RESETING the software.

DE
IR-Sender für die IR-Bridge (IR-Brücke)
Diese Software ist ein BLE-Client (Bluetooth-Low-Energy-Client) der die vom Server empfangenen IR-Codes und an den Client übermittelten aussendet.
Diese Version verbindet sich mit EINEM BLE-Server der den Namen "bleServerName" hat.
Die bmeServiceUUID und IrCodeCharacteristicUUID müssen übereinstimmen!!

Wird kein Server verbunden oder die Verbindung unterbrochen so wird nach einer Pause ein NEUSTART der Software durchgeführt.

****************************************************

The following ir protocols are transmited:
Es werden folgende IR-Protokolle übertagen:

NEC, SAMSUNG, SONY, PANASONIC, DENON, SHARP, JVC

if there is a need for other protocols , the software has to be modyfied...
für weitere Protokolle muss die Software angepasst werden....

*/


#include "BLEDevice.h"

#define DISABLE_CODE_FOR_RECEIVER // Disables restarting receiver after each send. Saves 450 bytes program memory and 269 bytes RAM if receiving functions are not used.
                                  // Sperrt den Start des IR-Empfängers nach jedem senden. Spart 450 Byte Program-Speicher und 269 Byte RAM da diese Funktion nicht benutzt wird.
#define EXCLUDE_EXOTIC_PROTOCOLS  // Saves around 240 bytes program memory if IrSender.write is used
                                  // Spart ungefähr 240 Byte Programm-Speicher

#include "PinDefinitionsESP32.h"  // Define macros for input and output pin etc.
                                  // Makros für Pin-Belegungen etc. (angepasst auf den ESP32!)

#include <IRremote.hpp>

#define DELAY_AFTER_SEND 2000
#define DELAY_AFTER_LOOP 5000

String version = "1.0";            // version of this software / Version dieser Software

uint8_t CONNECTED_LED = 25;        // LED to show if a server is connected / LED zur Anzeige wenn ein Klient verbunden iszt

uint8_t LEDPWM_SF = 128;           // brightness if a servers is found
uint8_t LEDPWM_ON = 16;            // dimmed brightness if connected to a advertising server 
                                   // Helligkeit beim Finden eines anzeigenden (advertising) Servers
uint8_t LEDPWM_OFF = 0;            // Led OFF / Led AUS

long restartDelay = 30000;         // delay after an non succesful scan or disconnecting in [ms]
                                   // Verzögerung nach einem erfolglosen Suchlauf oder einem Verbindungsunterbruch in [ms]

long timeout = 10000;              // Timeout after the scan if no client was found!
                                   // Abbruch-Zeit nach dem Suchlauf wenn kein Client gefunden wurde!

long scanEndTime;                  // Time when the scan finished / Zeit nach dem Edne des Suchlaufs             


// BLE Server name (the other ESP32 detecting the IRCodes)
// BLE Servername (der ander ESP32 der die IR-Signale empfängt)
#define bleServerName "IRE"

// UUID's of the service, characteristic that we want to read / UUID´s des Services und der Characteristik die benötigt wird

// BLE Service
static BLEUUID bmeServiceUUID("6E400001-B5A3-F393-E0A9-E50E24DCCA9E");

// BLE Characteristics
static BLEUUID IrCodeCharacteristicUUID("6E400003-B5A3-F393-E0A9-E50E24DCCA9E");

// Flags stating if should begin connecting and if the connection is up
// Flags zur Anzeige ob ein Server verbunden werden soll oder ob er verbunden ist
static boolean doConnect = false;
static boolean connected = false;

// Variable to store IrCode
// Variable zum speichern des IrCodes
char* IrCodeChar;

// Flag to check whether a new IrCode readings are available
// Flag zur Signalisation eines neuen Codes
boolean newIrCode = false;

// Address of the peripheral device. Address will be found during scanning...
// Adresse des gefundenen Servers
static BLEAddress *pServerAddress;
 
// Characteristic that we want to read
// Characteristic die benötigt wird
static BLERemoteCharacteristic* IrCodeCharacteristic;

// Activate IrCode notification
// Aktivirung der IrCode Notification
const uint8_t notificationOn[] = {0x1, 0x0};
const uint8_t notificationOff[] = {0x0, 0x0};

// Callback for signaling Connecting and Disconnecting of the server
// Callback zur Signalisierung von Verbindungs-Aufbau und -Trennung mit dem Server

class MyClientCallback : public BLEClientCallbacks {   // on Connect / beim Verbinden
   void onConnect(BLEClient* pclient) {
    Serial.println(" - Connected to server");
    analogWrite(CONNECTED_LED,LEDPWM_SF);
  }
  void onDisconnect(BLEClient* pclient) {              // on Disconnect / beim Trennen der Verbindung
    pclient->disconnect();
    analogWrite(CONNECTED_LED, LEDPWM_OFF);
    Serial.println("onDisconnect -> the client will restart!");
    delay(restartDelay);
    ESP.restart();                          // restart the client / neustart des Clients
  }
};

// Connect to the BLE server that will notify the desired Service and Characteristics and has the correct name
// Verbinden mit dem BLE Server welcher den benötigten Service und Characteristic anbietet und den geforderten Namen hat 
bool connectToServer(BLEAddress pAddress) {
   BLEClient* pClient = BLEDevice::createClient();
 
  // Connect to the BLE Server
  // Verbinden mit dem BLE-Server
  pClient->connect(pAddress);
  Serial.println(" - Connected to server");

  pClient->setClientCallbacks(new MyClientCallback());
 
  // Obtain a reference to the service we are after in the remote BLE server.
  // Referenzierung des benötigten Services des BLE-Servers
  BLERemoteService* pRemoteService = pClient->getService(bmeServiceUUID);
  if (pRemoteService == nullptr) {
    Serial.print("Failed to find our service UUID: ");
    Serial.println(bmeServiceUUID.toString().c_str());
    return (false);
  }
 
  // Obtain a reference to the characteristics in the service of the remote BLE server.
  // Referenzierung der Characteristic des Services des BLE-Servers
  IrCodeCharacteristic = pRemoteService->getCharacteristic(IrCodeCharacteristicUUID);

  if (IrCodeCharacteristic == nullptr) {
    Serial.print("Failed to find our characteristic UUID");
    return false;
  }
  Serial.print(bmeServiceUUID.toString().c_str());
  Serial.println(" - Found our characteristics");
 
  // Assign callback functions for the Characteristics
  // Verweisen auf die Callback-Funktionen für die benötigte Characteristic
  IrCodeCharacteristic->registerForNotify(IrCodeIrCodeCallback);
  analogWrite(CONNECTED_LED,LEDPWM_ON);
  return true;
}



// Callback function that gets called, when another device's advertisement has been received
// Callback-Funktion die aufgerufen wird, wenn ein Server dem benötigten Service gefunden wurde
class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {


  void onResult(BLEAdvertisedDevice advertisedDevice) {
    Serial.println(advertisedDevice.toString().c_str());
    if (advertisedDevice.getName() == bleServerName) { //Check if the name of the advertiser matches
      Serial.println(advertisedDevice.toString().c_str());
      advertisedDevice.getScan()->stop(); //Scan can be stopped, we found what we are looking for
      pServerAddress = new BLEAddress(advertisedDevice.getAddress()); //Address of advertiser is the one we need
      doConnect = true; //Set indicator, stating that we are ready to connect
      Serial.println("Device found. Connecting!");
    }
  }
};
 

// When the BLE Server sends a new IrCode 
// Wenn der BLE-Server einen neuen IrCode sendet 
static void IrCodeIrCodeCallback(BLERemoteCharacteristic* pBLERemoteCharacteristic, 
                                    uint8_t* pData, size_t length, bool isIrCode) {
  // store IrCode / speichere IrCode
  IrCodeChar = (char*)pData;
  newIrCode = true;
  IrCodeChar[length] = 0;   // to terminate the IrCodeChar array (string) / abschliessen des IrCodeChar-Arrays (String)
  Serial.print(newIrCode);
  Serial.print(": ");
  Serial.print(IrCodeChar);
  Serial.print(" length: ");
  Serial.println(length);
}


// Decode IrCode and sent it via IRemote functions
// Entschlüsseln des IrCodes und senden mit IRemote-Funktionen
void SendIrCode(char* Code) {
  String IrCode;
  int n=0;
  int i=0;
  unsigned int CharValue;
  unsigned int Number=0;
  unsigned int IrCodeValue[4]; // 0=Protocoll, 1=Address, 2=Code, 3=NumberOfBits...

  IrCode = String(Code);
  Serial.print(IrCode);
  Serial.print(" length: "); 
  Serial.println(IrCode.length());

  while (n <= IrCode.length()) {
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
            IrCodeValue[0] = 0; // unknown Code.. / unbekannter Code
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

  IrCodeValue[i]=Number;

  // the values for sending are now ready
  // die Sende-Werte sind nun bereit

  Serial.print("Protocol: ");
  Serial.print(IrCodeValue[0]);
  Serial.print(" Address: ");
  Serial.print(IrCodeValue[1]);
  Serial.print(" Command: ");
  Serial.print(IrCodeValue[2]);
  Serial.print(" Number of Bits: ");
  Serial.println(IrCodeValue[3]);

    // sending the IrCode in function of the protocol number
    // Senden des IrCodes in Funktion der Protokoll-Nummer

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
        IrSender.sendPanasonic(IrCodeValue[1], IrCodeValue[2], 3);
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

void setup() {
  
  // Start serial communication
  // Starten der seriellen Kommunikation
  Serial.begin(115200);
  Serial.println("Starting Arduino BLE Client application ");
  Serial.print("Version : ");
  Serial.println(version);
  


  // Init BLE device
  // Initialisierung des BLE-Objekts
  BLEDevice::init("");
 
  /* Retrieve a Scanner and set the callback we want to use to be informed when we
     have detected a new device.  Specify that we want active scanning and start the
     scan to run for 30 seconds.

    Aufrufen eines Suchlaufs und setzen des Callbacks zum Auffinden des benötigten Geräts
    Spezifizieren, dass ein aktiver Suchlauf für 30 Sekunden durchgeführt werden soll.


  */
  BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(true);
  pBLEScan->start(30);
  Serial.println("Scan done!");

  scanEndTime = millis();         // for the timeout if no device was found
                                  // für den zeitgesteuerten Neustart im Falle eines vergeblichen Suchlaufs 

  analogWrite(CONNECTED_LED,LEDPWM_OFF);           // LED off / LED aus
}

void loop() {
  /* If the flag "doConnect" is true then we have scanned for and found the desired
     BLE Server with which we wish to connect.  Now we connect to it.  Once we are
     connected we set the connected flag to be true and the "doConnect" flag is cleared.
     If after "timeout" miliseconds no connection is established, the server is restarting.
     If a new IrCode is detected it will be sent by the IRemote functions.

     Wenn das Flag "doConnect" gesetzt ist, wurde ein (der) Server mit dem benötigten Service und Characteristc gefunden.
     Der Server kann nun verbunden werden. Ist dieser Vorgang erfolgreich, so wird das Flag "connected" gesetzt und
     das Flag "doConnect" gelöscht.
     Wird innerhalb "timeout" Millisekunden keine Verbindung zum Server aufgebaut wird der Server neu gestartet.
     Beim Erkennen eines neuen IrCodes wird dieser mittels IRemote-Funktionen ausgestrahlt.
  */

  if (doConnect == true) {
    if (connectToServer(*pServerAddress)) {
      Serial.println("We are now connected to the BLE Server.");
      // Activate the IrCode characteristic
      // Aktivieren der IrCode-Characteristic
      IrCodeCharacteristic->getDescriptor(BLEUUID((uint16_t)0x2902))->writeValue((uint8_t*)notificationOn, 2, true);
      connected = true;
    } 
    else {
      // in case of no connection / im Falle eines fehlgeschlagenen Verbindungs-Aufbaus
      Serial.println("We have failed to connect to the server; The device will be restarted!");
      analogWrite(CONNECTED_LED,LEDPWM_OFF);
      delay(restartDelay);
      ESP.restart();
    }
    doConnect = false;
  }

  if (!connected) {
    // Waiting for a connection or restart / Warten auf den Verbindungs-Aufbau or Neustart
    if ( millis() - scanEndTime >= timeout) {
          Serial.println("Restarting Server!");
          ESP.restart();
    }
  }

  // if new IrCode readings are available, send it!
  // Wenn eine neuer IrCode erkannt wurde, sende ihn!

  if (newIrCode){
    newIrCode = false;
    SendIrCode(IrCodeChar);

  }
  delay(10); // Delay a second between loops / etwas Verzögerung zwischen den Durchläufen
}
