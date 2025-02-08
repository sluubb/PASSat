#include <SPI.h>
#include <SD.h>

#include <DFRobot_ICP10111.h>
#include <DFRobot_AHT20.h>

#define LOG_FILE "log.txt"
#define RETRY_DELAY 1000

// 0->don't log to serial | 1->log only info & errors | 2->log all
#define SERIAL_LOG_LEVEL 2

int statusPin = LED_BUILTIN;

DFRobot_ICP10111 pressureSensor;
DFRobot_AHT20 tempHumSensor;

void setup() {
#if SERIAL_LOG_LEVEL > 0
  // initialize serial logging
  Serial.begin(9600);
  while (!Serial);
#endif

  pinMode(statusPin, OUTPUT);

  initPressure();
  initTempHum();
  initSD();  
}

void loop() {
  float pressure;
  if (readPressure(&pressure)) return;

  float temp, hum;
  if (readTempHum(&temp, &hum)) return;

  log(
    String(pressure) +
    ", " +
    String(temp) +
    ", " +
    String(hum)
  );
}


//--- PRESSURE SENSOR ---//

void initPressure() {
  uint8_t status;
  while ((status = pressureSensor.begin()) != 0) {
    error("Pressure sensor initialization failed (status: " + String(status) + ").");
    delay(RETRY_DELAY);
  }

  pressureSensor.setWorkPattern(pressureSensor.eNormal);
  info("Pressure sensor initialized.");
}

bool readPressure(float *p) {
  *p = pressureSensor.getAirPressure();
  return false;
}


//--- TEMPERATURE & HUMIDITY SENSOR ---//

void initTempHum() {
  uint8_t status;
  while ((status = tempHumSensor.begin()) != 0) {
    error("Temperature/humidity sensor initialization failed (status: " + String(status) + ").");
    delay(RETRY_DELAY);
  }

  info("Temperature/humidity sensor initialized.");  
}

bool readTempHum(float *t, float *h) {
  if (!tempHumSensor.startMeasurementReady(true)) return true;
  *t = tempHumSensor.getTemperature_C();
  *h = tempHumSensor.getHumidity_RH();
  return false;
}


//--- SD CARD ---//

void initSD() {
  while (!SD.begin(SDCARD_SS_PIN)) {
    error("SD card initialization failed.");
    delay(RETRY_DELAY);
  }
  info("SD card initialized.");

  // clear previous logs
  if (SD.exists(LOG_FILE)) {
    SD.remove(LOG_FILE);
    info("Cleared previous logs.");
  }
}


//--- LOGGING FUNCTIONS ---//

void log(String s) {
  File log = SD.open(LOG_FILE, FILE_WRITE);
  if (log) {
#if SERIAL_LOG_LEVEL > 1
    Serial.print("LOG :: ");
    Serial.println(s);
#endif
    log.print(millis());
    log.print(" :: ");
    log.println(s);
    log.close();
  } else {
    error("Couldn't open log file.");
  }
}

void error(String s) {
  digitalWrite(statusPin, HIGH);
  delay(100);
  digitalWrite(statusPin, LOW);

#if SERIAL_LOG_LEVEL > 0
  Serial.print("ERROR :: ");
  Serial.println(s);
#endif
}

void info(String s) {
#if SERIAL_LOG_LEVEL > 0
  Serial.print("INFO :: ");
  Serial.println(s);
#endif
}