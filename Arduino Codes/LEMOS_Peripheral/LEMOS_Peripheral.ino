#include <ArduinoBLE.h>
#include <Adafruit_NeoPixel.h>

#define NUM_LEDS 32
#define PIN_LED  5 // Ensure that this is the PIN that the Digital input on the LED strip is connected to

// Command characters
#define CMD_RED   'r'
#define CMD_GREEN 'g'
#define CMD_BLUE  'b'
#define CMD_ON    '1'
#define CMD_OFF   '0'

// NeoPixel strip
Adafruit_NeoPixel leds(NUM_LEDS, PIN_LED, NEO_GRB + NEO_KHZ800);

char ledStatus[NUM_LEDS];

// BLE
const char* deviceServiceUUID               = "19b10000-e8f2-537e-4f6c-d104768a1214";
const char* deviceServiceCharacteristicUUID = "19b10001-e8f2-537e-4f6c-d104768a1214";
unsigned long blinkInterval = 600; // ms
bool greenLedState = LOW;

BLEService ledService(deviceServiceUUID);
BLECharacteristic ledCharacteristic(deviceServiceCharacteristicUUID, BLERead | BLEWrite, NUM_LEDS);
BLEByteCharacteristic statusCharacteristic("2A57", BLERead | BLEWrite | BLENotify);

void ledOff() {
  for(int i = 0; i < NUM_LEDS; i++) leds.setPixelColor(i, leds.Color(0,     0,   0));
  leds.show();
}

uint32_t decodeColor(char cmd) {
  switch (cmd) {
    case CMD_RED:    return leds.Color(1,  0,  0); break;
    case CMD_GREEN:  return leds.Color(0,   1,   0); break;
    case CMD_BLUE:  return leds.Color(0,   0,   1); break;
    case CMD_ON:    return leds.Color(0,   1,   0); break;
    case CMD_OFF:   return leds.Color(0,   0,   0); break;
    default:        return leds.Color(0,   0,   0); 
  }
}

void setup() {
  pinMode(LEDB, OUTPUT);
  pinMode(LEDR, OUTPUT);
  pinMode(LEDG, OUTPUT);
  digitalWrite(LEDR, HIGH);
  digitalWrite(LEDG, HIGH);
  digitalWrite(LEDB, HIGH);

  leds.begin();
  ledOff();

  // "Alive" indicator
  leds.setPixelColor(0, leds.Color(0,  1, 0));
  leds.show();

  // BLE
  while (!BLE.begin()) {
    ledOff();
    
    // Flash onboard LED red while BLE attempts initialization
    digitalWrite(LEDR, LOW);   // Red ON
    delay(500);
    digitalWrite(LEDR, HIGH);  // Red OFF
    delay(500); 
  }

  BLE.setLocalName("Well-Plate Device Arduino Nano 33 BLE Rev2");
  BLE.setAdvertisedService(ledService);
  ledService.addCharacteristic(ledCharacteristic);
  ledService.addCharacteristic(statusCharacteristic);
  BLE.addService(ledService);
  ledCharacteristic.writeValue(ledStatus);
  BLE.advertise();
}

void loop() {
  BLEDevice central = BLE.central();
  delay(50);

  if(central){
    while (central.connected()) {
      if (ledCharacteristic.written()) {
        ledCharacteristic.readValue(ledStatus, NUM_LEDS);

        for (int i = 0; i < NUM_LEDS; i++) {
          leds.setPixelColor(i, decodeColor(ledStatus[i]));
        }
        leds.show();

        statusCharacteristic.writeValue(1);
      }
    }
  } 
  // else {
  //   unsigned long lastBlinkTime = millis();
  //   unsigned long currentTime = millis();
  //   if (currentTime - lastBlinkTime > blinkInterval) {
  //     greenLedState = !greenLedState; // Toggle state
  //     digitalWrite(LEDG, greenLedState ? LOW : HIGH); // LOW=ON, HIGH=OFF (? is conditional operator)
  //     lastBlinkTime = currentTime;
  //   }
  // }

}
