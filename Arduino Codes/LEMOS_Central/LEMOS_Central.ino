#include <ArduinoBLE.h>

// Constants
const char* deviceServiceUUID = "19b10000-e8f2-537e-4f6c-d104768a1214";
const char* deviceServiceCharacteristicUUID = "19b10001-e8f2-537e-4f6c-d104768a1214";
const int LED_PIN = LED_BUILTIN; //Built-in LED pin for Arduino Nano 33 IoT
bool ledState = 0;

// Variables
String lastSent = "00000000000000000000000000000000";

void setup() {
  Serial.begin(9600);

  pinMode(LED_PIN, OUTPUT);

  while (!BLE.begin()) {
    Serial.println("Error: Could not start bluetooth module.");
    handleBlinkError(1);
  } 

  BLE.setLocalName("Central Arduino Nano IOT 33");
  Serial.println("Bluetooth module started...");
}

void loop() {
  Serial.println("Looking for peripheral (well-plate) Arduino...");
  BLEDevice peripheral;

  do{
    BLE.scanForUuid(deviceServiceUUID);
    peripheral = BLE.available();
    delay(500);
    ledState = !ledState;
    digitalWrite(LED_PIN, ledState ? LOW : HIGH);
    Serial.print("LED state: ");
    Serial.println(ledState);
  } while(!peripheral);

  digitalWrite(LED_PIN, LOW);
  BLE.stopScan();
  
  if (!peripheral.connect()) {
    Serial.println("Connection failed!");
    return;
  }

  Serial.println("Connection successful!");
  peripheral.discoverAttributes();

  BLECharacteristic ledChar = peripheral.characteristic(deviceServiceCharacteristicUUID);
  BLECharacteristic statusChar = peripheral.characteristic("2A57");

  if (!statusChar || !statusChar.canSubscribe() || !statusChar.subscribe()) {
    Serial.println("Warning: Status characteristic not available for subscription.");
  }

  while (peripheral.connected()) {
    String toSend = getData();
    
    if (!toSend.equals(lastSent)) {
      lastSent = toSend;

      // Send LED command
      ledChar.writeValue((const unsigned char*)lastSent.c_str(), lastSent.length());
      Serial.print("Writing LED command: ");
      Serial.println(lastSent);
    }

    // Check for acknowledgment
    if (statusChar && statusChar.valueUpdated()) {
      byte deviceStatus;
      statusChar.readValue(deviceStatus);
      Serial.println("DEVICE: COMMAND RECEIPT CONFIRMED");
    }
  }

  Serial.println("Peripheral (well-plate) device disconnected.");
}

String getData(){
  if(Serial.available()){
    String ledVal = Serial.readStringUntil('\n');
    return ledVal;
  }
  return lastSent;
}

void handleBlinkError(int a) {
  for (int i = 0; i < 3*a; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(100);
    digitalWrite(LED_PIN, LOW);
    delay(100);
  }
  delay(700);
}
