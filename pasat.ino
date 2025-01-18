#include <SPI.h>
#include <SD.h>

#define LOG_FILE "log.txt"

// 0->don't log to serial | 1->log only info & errors | 2->log all
#define SERIAL_LOG_LEVEL 1

void setup() {
#if SERIAL_LOG_LEVEL > 0
  // initialize serial logging
  Serial.begin(9600);
  while (!Serial);
#endif

  // initialize SD card
  if (!SD.begin(SDCARD_SS_PIN)) {
    error("SD card initialization failed.");
    while (true);
  }
  info("SD card initialized.");

  // clear previous logs
  if (SD.exists(LOG_FILE)) {
    SD.remove(LOG_FILE);
    info("Cleared previous logs.");
  }
}

void loop() {
  log("the end is never");
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
}

void info(String s) {
#if SERIAL_LOG_LEVEL > 0
  Serial.println("INFO :: " + s);
#endif
}