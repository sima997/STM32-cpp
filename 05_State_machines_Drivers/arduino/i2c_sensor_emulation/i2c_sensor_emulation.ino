#include <Wire.h>

const uint8_t SLAVE_ADDR = 0x53;

// Example “sensor” registers
uint8_t registers[3] = {0x42, 0x64, 0x19};
uint8_t currentReg = 0;

void setup() {
  Wire.begin(SLAVE_ADDR);      // Join I2C bus as slave
  Wire.onReceive(receiveEvent); // Callback for master write
  Wire.onRequest(requestEvent); // Callback for master read

  Serial.begin(115200);
  Serial.println("I2C slave ready");
}

void loop() {
  // Optionally update sensor registers
  registers[1] = (millis() / 100) & 0xFF; // fake sensor value changing
  delay(10);
}

// Master wrote some bytes
void receiveEvent(int numBytes) {
  if (numBytes < 1) return;
  currentReg = Wire.read(); // first byte = register address
  while (Wire.available()) Wire.read(); // ignore remaining bytes
}

// Master wants to read
void requestEvent() {
  if (currentReg < sizeof(registers)) {
    Wire.write(registers[currentReg]);
  } else {
    Wire.write((uint8_t)0x00);
  }
}
