#include "BluetoothSerial.h"
 
#define USE_NAME // Comment this to use MAC address instead of a slaveName
const char *pin = "1234"; // Change this to reflect the pin expected by the real slave BT device
 
#if !defined(CONFIG_BT_SPP_ENABLED)
#error Serial Bluetooth not available or not enabled. It is only available for the ESP32 chip.
#endif
 
#define LED_BUILTIN 25
#define LED_BUILTIN2 19

#define PIN_RED    33 // GPIO23
#define PIN_GREEN  32 // GPIO22
#define PIN_BLUE   27 // GPIO21

BluetoothSerial SerialBT;
 
#ifdef USE_NAME
  String slaveName = "ESP32-BT-Slave"; // Change this to reflect the real name of your slave BT device
#else
  String MACadd = "cc:7b:5c:35:b5:bc"; // This only for printing
  uint8_t address[6]  = {0xcc, 0x7b, 0x5c, 0x35, 0xb5, 0xbc}; // Change this to reflect real MAC address of your slave BT device
#endif
 
String myName = "ESP32-BT-Master";
bool connected; 

void setup() {
  Serial.begin(115200);
 
  SerialBT.begin(myName, true);
  Serial.printf("The device \"%s\" started in master mode, make sure slave BT device is on!\n", myName.c_str());
 
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(LED_BUILTIN2, OUTPUT);

  pinMode(PIN_RED,   OUTPUT);
  pinMode(PIN_GREEN, OUTPUT);
  pinMode(PIN_BLUE,  OUTPUT);

  #ifndef USE_NAME
    SerialBT.setPin(pin);
    Serial.println("Using PIN");
  #endif
 
  // connect(address) is fast (up to 10 secs max), connect(slaveName) is slow (up to 30 secs max) as it needs
  // to resolve slaveName to address first, but it allows to connect to different devices with the same name.
  // Set CoreDebugLevel to Info to view devices Bluetooth address and device names
  #ifdef USE_NAME
    connected = SerialBT.connect(slaveName);
    Serial.printf("Connecting to slave BT device named \"%s\"\n", slaveName.c_str());
  #else
    connected = SerialBT.connect(address);
    Serial.print("Connecting to slave BT device with MAC "); Serial.println(MACadd);
  #endif
 
  if(connected) {
    Serial.println("Connected Successfully!");
  } else {
    while(!SerialBT.connected(10000)) {
      Serial.println("Failed to connect. Make sure remote device is available and in range, then restart app.");
    }
  }
  // Disconnect() may take up to 10 secs max
  if (SerialBT.disconnect()) {
    Serial.println("Disconnected Successfully!");
  }
  // This would reconnect to the slaveName(will use address, if resolved) or address used with connect(slaveName/address).
  SerialBT.connect();
  if(connected) {
    Serial.println("Reconnected Successfully!");
  } else {
    while(!SerialBT.connected(10000)) {
      Serial.println("Failed to reconnect. Make sure remote device is available and in range, then restart app.");
    }
  }
}
 
void loop() {
// This would reconnect to the slaveName(will use address, if resolved) or address used with connect(slaveName/address).
  
  if(!connected) {
    SerialBT.connect();
    connected = true;
    Serial.println("Reconnected Successfully!");
  } else {
    while(!SerialBT.connected(10000)) {
      Serial.println("Failed to reconnect. Make sure remote device is available and in range, then restart app.");
    }
  }
  if (connected) {
    String receivedData = SerialBT.readStringUntil('\n');
    Serial.println(receivedData);
    int commaIndex = receivedData.indexOf(' ');
    int commaIndex1 = receivedData.indexOf(',');
  if (receivedData.length() > 0) {
    //int value = receivedData.toInt(); // Convert received string to integer
      
      String angleZString = receivedData.substring(0, commaIndex);
      angleZString.trim(); // Use trim() directly on the string object
      String elapsedTimeString = receivedData.substring(commaIndex + 1);
      elapsedTimeString.trim(); // Use trim() directly on the string object
      String zAccelerationString = receivedData.substring(commaIndex1 + 2);
      //zAccelerationString.trim();

      int value = angleZString.toInt();
      int elapsedTime = elapsedTimeString.toInt();
      float zAcceleration = zAccelerationString.toFloat();

      Serial.print("AngleZ: "); Serial.println(value);
      Serial.print("Elapsed Time: "); Serial.println(elapsedTime);
      Serial.print("zAcceleration: "); Serial.println(zAcceleration);

    if (value > 98) { // Check threshold for right rotation
      digitalWrite(LED_BUILTIN, HIGH);  // Turn on LED for right rotation
      //digitalWrite(LED_BUILTIN2, LOW);  // Ensure the other LED is off
    } else if (value < 84) { // Check threshold for left rotation
      digitalWrite(LED_BUILTIN, LOW); // Turn on LED for left rotation
      if (elapsedTime < 160) {
        digitalWrite(LED_BUILTIN2, LOW);
      } 
      if (elapsedTime == 160)  {
        digitalWrite(LED_BUILTIN2, HIGH); // Turn on LED for left rotation
      }
    } 
    
    if(value < 98 && value > 84){
      digitalWrite(LED_BUILTIN, LOW);   // Turn off both LEDs if in between thresholds
      digitalWrite(LED_BUILTIN2, LOW);
    }

    if (zAcceleration >= 2.40) {
      setColor(0, 255, 0); // Turn on LED
    } else if (zAcceleration <= 2.40 && zAcceleration > 1.80){
      setColor(0, 0, 0);// Turn off LED
    } else {
      setColor(255, 0 ,0);
    }

  } else {
    connected = false;
    Serial.println("No data received.");
  }
  }

  delay(10);
}

void setColor(int R, int G, int B) {
  analogWrite(PIN_RED,   R);
  analogWrite(PIN_GREEN, G);
  analogWrite(PIN_BLUE,  B);
}