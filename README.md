# IrBridge
Bridge to Transfer IR-Remote-Signals via BLE in an other Room

Since I needed to control my adio system in the living roon from the kitchen and the commercial 433Mhz product did not work as it should i created this units... 

The sistem consist of one PCB capable to act as a IR receiver or as IR transmitter.

Since a single IR LED for sending is not always able to transmit enougth power I have forseen six "normal" IR LEDs and two Power LEDs on the PCB.

The heart of the IrBridge is an ESP32-WROOM-32 CPU. The software is written for the ARDUINO IDE so it is easy to use.

The programming is done via a ESP-Link. The ESP-Link is not anymore used after programming.


The PCB and the documentation ca be found on http://www.systech-gmbh.ch/IrBridge.htm#section6
