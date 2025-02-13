#include <SPI.h>
#include <SD.h>

#include <DFRobot_ICP10111.h>
#include <DFRobot_AHT20.h>

// 0->don't log to serial | 1->log only info & errors | 2->log all
#define SERIAL_LOG_LEVEL 2

const String logFile = "log.txt";
const int retryDelay = 1000;

// LED status indicator
const int statusPin = LED_BUILTIN;

// ultrasonic distance sensor
const int triggerPin = 4;
const int echoPin = 5;

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
  initDistance();
  initSD();  
}

void loop() {
  float pressure, temp, distance;
  if (readPressure(&pressure) || readTemp(&temp) || readDistance(&distance)) return;

  log(
    String(pressure) +
    ", " +
    String(temp) +
    ", " +
    String(distance)
  );
}


//--- ULTRASONIC DISTANCE SENSOR ---//

void initDistance() {
  pinMode(triggerPin, OUTPUT);
  pinMode(echoPin, INPUT);

  info("Ultrasonic distance sensor initialized.");
}

bool readDistance(float *d) {
  digitalWrite(triggerPin, LOW);
  delayMicroseconds(2);
  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPin, LOW);

  float duration = pulseIn(echoPin, HIGH);
  float distance = duration * 0.0001458;

  *d = distance;
  return false;
}


//--- PRESSURE SENSOR ---//

void initPressure() {
  uint8_t status;
  while ((status = pressureSensor.begin()) != 0) {
    error("Pressure sensor initialization failed (status: " + String(status) + ").");
    delay(retryDelay);
  }

  pressureSensor.setWorkPattern(pressureSensor.eNormal);
  info("Pressure sensor initialized.");
}

bool readPressure(float *p) {
  *p = pressureSensor.getAirPressure();
  return false;
}

bool readTemp(float *t) {
  *t = pressureSensor.getTemperature();
  return false;
}

//--- TEMPERATURE & HUMIDITY SENSOR ---//
/*
void initTempHum() {
  uint8_t status;
  while ((status = tempHumSensor.begin()) != 0) {
    error("Temperature/humidity sensor initialization failed (status: " + String(status) + ").");
    delay(retryDelay);
  }

  info("Temperature/humidity sensor initialized.");  
}

bool readTempHum(float *t, float *h) {
  if (!tempHumSensor.startMeasurementReady(true)) return true;
  *t = tempHumSensor.getTemperature_C();
  *h = tempHumSensor.getHumidity_RH();
  return false;
}
*/

//--- SD CARD ---//

void initSD() {
  while (!SD.begin(SDCARD_SS_PIN)) {
    error("SD card initialization failed.");
    delay(retryDelay);
  }
  info("SD card initialized.");

  // clear previous logs
  if (SD.exists(logFile)) {
    SD.remove(logFile);
    info("Cleared previous logs.");
  }
}


//--- LOGGING FUNCTIONS ---//

void log(String s) {
  File log = SD.open(logFile, FILE_WRITE);
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