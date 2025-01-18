#include <SPI.h>
#include <SD.h>

#include <DFRobot_ICP10111.h>

#define LOG_FILE "log.txt"

// 0->don't log to serial | 1->log only info & errors | 2->log all
#define SERIAL_LOG_LEVEL 2

DFRobot_ICP10111 pressureSensor;

void setup() {
#if SERIAL_LOG_LEVEL > 0
  // initialize serial logging
  Serial.begin(9600);
  while (!Serial);
#endif

  if (pressureSensor.begin() != 0) {
    error("Pressure sensor initialization failed.");
  }
  pressureSensor.setWorkPattern(pressureSensor.eNormal);
  info("Pressure sensor initialized.");

  // initialize SD card
  if (!SD.begin(SDCARD_SS_PIN)) {
    error("SD card initialization failed.");
  }
  info("SD card initialized.");

  // clear previous logs
  if (SD.exists(LOG_FILE)) {
    SD.remove(LOG_FILE);
    info("Cleared previous logs.");
  }
}

void loop() {
  log(String(pressureSensor.getAirPressure()));
}


//--- LOGGING FUNCTIONS ---//

void log(String s) {
  File log = SD.open(LOG_FILE, FILE_WRITE);
  if (log) {
#if SERIAL_LOG_LEVEL > 1
    Serial.println("LOG :: " + s);
#endif
    log.println(s);
    log.close();
  } else {
    error("Couldn't open log file.");
  }
}

void error(String s) {
  // TODO: physically indicate
#if SERIAL_LOG_LEVEL > 0
  Serial.println("ERROR :: " + s);
#endif
  
  // errors should end program for safety
  while (true);
}

void info(String s) {
#if SERIAL_LOG_LEVEL > 0
  Serial.println("INFO :: " + s);
#endif
}