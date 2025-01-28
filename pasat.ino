#include <SPI.h>
#include <SD.h>

#include <DFRobot_ICP10111.h>
#include <DFRobot_AHT20.h>

#define LOG_FILE "log.txt"
#define RETRY_DELAY 1000

// 0->don't log to serial | 1->log only info & errors | 2->log all
#define SERIAL_LOG_LEVEL 2

DFRobot_ICP10111 pressureSensor;
DFRobot_AHT20 tempHumSensor;

void setup() {
#if SERIAL_LOG_LEVEL > 0
  // initialize serial logging
  Serial.begin(9600);
  while (!Serial);
#endif

  // initialize sensors
  initPressure();
  initTempHum();

  // initialize SD card
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

void loop() {
  if (!tempHumSensor.startMeasurementReady(true)) return;

  log(
    String(pressureSensor.getAirPressure()) +
    ", " +
    String(tempHumSensor.getTemperature_C()) +
    ", " +
    String(tempHumSensor.getHumidity_RH())
  );
}


//--- PRESSURE SENSOR ---//

void initPressure() {
  while (pressureSensor.begin() != 0) {
    error("Pressure sensor initialization failed.");
    delay(RETRY_DELAY);
  }

  pressureSensor.setWorkPattern(pressureSensor.eNormal);
  info("Pressure sensor initialized.");
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


//--- LOGGING FUNCTIONS ---//

void log(String s) {
  File log = SD.open(LOG_FILE, FILE_WRITE);
  if (log) {
#if SERIAL_LOG_LEVEL > 1
    Serial.println("LOG :: ")
    Serial.print(s);
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
  // TODO: physically indicate
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