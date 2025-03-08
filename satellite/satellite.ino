#include <SPI.h>
#include <SD.h>
#include <DFRobot_ICP10111.h>
#include <TinyGPS++.h>
#include <RadioLib.h>
//#include <DFRobot_AHT20.h>
//#include <AccelStepper.h>

// 0->don't log to serial | 1->log only info & errors | 2->log all
#define SERIAL_LOG_LEVEL 2

const String logFile = "log.csv";
const int retryDelay = 1000;

// LED status indicator
const int statusPin = LED_BUILTIN;

// ultrasonic distance sensor
//const int triggerPin = 4;
//const int echoPin = 5;

// stepper motor
//AccelStepper stepper(AccelStepper::FULL4WIRE, 0, 2, 1, 3, true);

// leg release mechanism
//const int timeThreshold = 1000;
//const float altitudeThreshold = 100.0;
//int timerStart = -1;
//bool hasLeftGround = true;
//bool legsReleased = false;

// GPS module
TinyGPSPlus gpsParser;

// LoRa module
const float radioFrequency = 434.0; // MHz
const float radioBandwidth = 125.0; // kHz
SX1278 radio = new Module(15, 17, 16, 18);
int transmissionState = RADIOLIB_ERR_NONE;
volatile bool packetSentFlag = false;

DFRobot_ICP10111 pressureSensor;
//DFRobot_AHT20 tempHumSensor;

bool hasShouted = false;

void setup() {
#if SERIAL_LOG_LEVEL > 0
  Serial.begin(9600);
  while (!Serial);
#endif

  pinMode(statusPin, OUTPUT);

  initSD();
  initLoRa();
  initPressure();
  initGPS();
}

void loop() {
  updateGPS();

  float pressure, elevation, temperature;
  double latitude, longitude, altitudeGPS, course, speed;

  String logStr = String(millis()) + ",";
  if (readPressure(&pressure)) logStr += String(pressure) + ","; else logStr += ",";
  if (readTemperature(&temperature)) logStr += String(temperature) + ","; else logStr += ",";
  if (readElevation(&elevation)) logStr += String(elevation) + ","; else logStr += ",";
  if (readGPSLocation(&latitude, &longitude)) logStr += String(latitude, 6) + "," + String(longitude, 6) + ",";
  else logStr += ",,";
  if (readGPSAltitude(&altitudeGPS)) logStr += String(altitudeGPS) + ","; else logStr += ",";
  if (readGPSCourse(&course)) logStr += String(course) + ","; else logStr += ",";
  if (readGPSSpeed(&speed)) logStr += String(speed);

  log(logStr);

  // send data through LoRa radio to ground station
  if (packetSentFlag) {
    packetSentFlag = false;

    if (transmissionState != RADIOLIB_ERR_NONE) {
      error("Radio transmission failed (code: " + String(transmissionState) + ").");
    }

    radio.finishTransmit();

    if (!hasShouted && elevation > 500) {
      hasShouted = true;
      logStr += ", msg: You all look so small from up here!";
    }

    transmissionState = radio.startTransmit(logStr);
  }

  // we have left ground if altitude has been above threshold for long enough
  /*if (!hasLeftGround && alt > altitudeThreshold) {
    if (timerStart < 0) {
      timerStart = millis();
    } else if (millis() - timerStart > timeThreshold) {
      hasLeftGround = true;
    }
  } else if (!hasLeftGround) {
    timerStart = -1;
  }

  // release legs if altitude has indicated being near ground for long enough (and previously left ground)
  if (!legsReleased && hasLeftGround && alt < altitudeThreshold) {
    if (timerStart < 0) {
      timerStart = millis();
    } else if (millis() - timerStart > timeThreshold) {
      releaseLegs();
    }
  } else if (!legsReleased && hasLeftGround) {
    timerStart = -1;
  }*/
}


//--- GPS MODULE ---//

void initGPS() {
  Serial1.begin(9600);
  info("GPS module initialized.");
}

void updateGPS() {
  while (Serial1.available() > 0) {
    char packet = Serial1.read();
    gpsParser.encode(packet);
    //Serial.print(packet);
  }
}

bool readGPSLocation(double *lat, double *lng) {
  if (!gpsParser.location.isValid()) return false;

  *lat = gpsParser.location.lat();
  *lng = gpsParser.location.lng();
  return true;
}

bool readGPSAltitude(double *alt) {
  if (!gpsParser.altitude.isValid()) return false;

  *alt = gpsParser.altitude.meters();
  return true;
}

bool readGPSCourse(double *crs) {
  if (!gpsParser.course.isValid()) return false;

  *crs = gpsParser.course.deg();
  return true;
}

bool readGPSSpeed(double *spd) {
  if (!gpsParser.speed.isValid()) return false;

  *spd = gpsParser.speed.mps();
  return true;
}


//--- LORA MODULE ---//

void onPacketSent(void) {
  packetSentFlag = true;
}

void initLoRa() {
  int state;
  while ((state = radio.begin(radioFrequency, radioBandwidth)) != RADIOLIB_ERR_NONE) {
    error("LoRa module initialization failed (code: " + String(state) + ").");
    delay(retryDelay);
  }

  radio.setPacketSentAction(onPacketSent);

  info("LoRa module initialized.");

  transmissionState = radio.startTransmit("Transmission has begun.");
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
  return true;
}

bool readElevation(float *alt) {
  *alt = pressureSensor.getElevation();
  return true;
}

bool readTemperature(float *t) {
  *t = pressureSensor.getTemperature();
  return true;
}


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


//--- STEPPER MOTOR ---//
/*
void releaseLegs() {
  legsReleased = true;

  log("Releasing legs...");

  stepper.setMaxSpeed(1000);
  stepper.setAcceleration(1000);
  stepper.runToNewPosition(200);
  stepper.disableOutputs();

  log("Legs released.");
}
*/

//--- ULTRASONIC DISTANCE SENSOR ---//
/*
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
*/

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

//--- LOGGING FUNCTIONS ---//

void log(String s) {
  File log = SD.open(logFile, FILE_WRITE);
  if (log) {
#if SERIAL_LOG_LEVEL > 1
    Serial.print("LOG :: ");
    Serial.println(s);
#endif
    log.println(s);
    log.close();
  } else {
    error("Couldn't open log file.");
  }
}

void error(String s) {
  digitalWrite(statusPin, HIGH);
  delay(40);
  digitalWrite(statusPin, LOW);
  delay(120);
  digitalWrite(statusPin, HIGH);
  delay(40);
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