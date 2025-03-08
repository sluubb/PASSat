#include <RadioLib.h>

const int retryDelay = 1000;

// LoRa module
const float radioFrequency = 434.0; // MHz
const float radioBandwidth = 125.0; // kHz
SX1278 radio = new Module(15, 17, 16, 18);
volatile bool packetReceivedFlag = false;

void onPacketReceived(void) {
  packetReceivedFlag = true;
}

void setup() {
  Serial.begin(9600);
  while (!Serial);

  int state;
  while ((state = radio.begin(radioFrequency, radioBandwidth)) != RADIOLIB_ERR_NONE) {
    Serial.println("ERROR :: LoRa module initialization failed (code: " + String(state) + ").");
    delay(retryDelay);
  }

  radio.setPacketReceivedAction(onPacketReceived);

  Serial.println("INFO :: LoRa module initialized.");

  while ((state = radio.startReceive()) != RADIOLIB_ERR_NONE) {
    Serial.println("ERROR :: LoRa module failed to start listening (code: " + String(state) + ").");
    delay(retryDelay);
  }

  Serial.println("INFO :: LoRa module now listening...");
}

void loop() {
  if (packetReceivedFlag) {
    packetReceivedFlag = false;

    String data;
    int state = radio.readData(data);

    if (state == RADIOLIB_ERR_NONE) {
      Serial.println("RECEIVED :: " + data);
    } else if (state == RADIOLIB_ERR_CRC_MISMATCH) {
      Serial.println("WARN :: CRC mismatch in data received.");
    } else {
      Serial.println("WARN :: Failed to receive data (code: " + String(state) + ").");
    }
  }
}