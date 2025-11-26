#include <SevSeg.h>

SevSeg sevseg;

const int trigPin = 2;
const int echoPin = 3;

float farDist = 60.0;   // cm
float nearDist = 30.0;  // cm
float gateDistance = (farDist - nearDist) / 100.0;  // meters

// Simulated GPS base location
double baseLat = 26.8845;
double baseLon = 80.9549;
double totalDistance = 0.0;

enum State { WAITING, TIMING, SHOW };
State state = WAITING;

unsigned long startTime;

float lastSpeedMS = 0.0;
float lastAccel  = 0.0;

float maxSpeedMS = 0.0;
float maxAccel   = 0.0;

// for noise filtering: object must be in gate N times in a row
int gateHitCount = 0;
const int gateHitNeeded = 3;  // increase if still too sensitive

float measureDist() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  unsigned long duration = pulseIn(echoPin, HIGH, 30000);
  if (duration == 0) return -1;
  return duration / 58.0; // cm
}

void showSpeedOnDisplay(float speedKmh) {
  int value = (int)(speedKmh + 0.5);    // round to nearest int
  if (value < 0) value = 0;
  if (value > 9999) value = 9999;
  sevseg.setNumber(value, 0);           // no decimal places
}

void setup() {
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // SevSeg setup
  byte numDigits = 4;
  byte digitPins[]   = {6, 7, 8, 9};                   // D1..D4
  byte segmentPins[] = {4, 5, 10, 11, 12, 13, A0, A1}; // A,B,C,D,E,F,G,DP
  bool resistorsOnSegments = false; // we (preferably) have resistors on digits
  byte hardwareConfig = COMMON_CATHODE;

  sevseg.begin(hardwareConfig, numDigits, digitPins, segmentPins, resistorsOnSegments);
  sevseg.setBrightness(90);

  // start with 0 on the display
  showSpeedOnDisplay(0);

  Serial.begin(9600);
  Serial.println(">>> SPEED + ACCEL + GPS + DISPLAY <<<");
  Serial.println("Move object from FAR (60cm) to NEAR (30cm)");
}

void loop() {
  float d = measureDist();
  if (d < 0) {
    sevseg.refreshDisplay();
    return;
  }

  switch (state) {
    case WAITING:
      // just show distance in serial, DON'T touch display (keeps last value)
      Serial.print("Distance: ");
      Serial.print(d);
      Serial.println(" cm");

      // object inside the gate?
      if (d < farDist && d > nearDist) {
        gateHitCount++;
        if (gateHitCount >= gateHitNeeded) {
          startTime = millis();
          Serial.println("Tracking object...");
          gateHitCount = 0;
          state = TIMING;
        }
      } else {
        gateHitCount = 0;  // lost the object, reset
      }
      break;

    case TIMING:
      if (d <= nearDist) {
        unsigned long dtMs = millis() - startTime;
        float timeSec = dtMs / 1000.0;

        float speedMS  = gateDistance / timeSec;
        float speedKMH = speedMS * 3.6;
        float accel    = speedMS / timeSec;

        lastSpeedMS = speedMS;
        lastAccel   = accel;

        if (speedMS > maxSpeedMS) maxSpeedMS = speedMS;
        if (accel   > maxAccel)   maxAccel   = accel;

        totalDistance += gateDistance;
        double lat = baseLat + (totalDistance / 111000.0);
        double lon = baseLon;

        Serial.println("----- CURRENT RUN -----");
        Serial.print("Time in gate: ");
        Serial.print(timeSec, 3);
        Serial.println(" s");

        Serial.print("Speed: ");
        Serial.print(speedMS, 2);
        Serial.print(" m/s  (");
        Serial.print(speedKMH, 2);
        Serial.println(" km/h)");

        Serial.print("Acceleration: ");
        Serial.print(accel, 2);
        Serial.println(" m/s^2");

        Serial.print("Sim GPS Lat: ");
        Serial.println(lat, 6);
        Serial.print("Sim GPS Lon: ");
        Serial.println(lon, 6);

        Serial.println("----- FASTEST RUN SO FAR -----");
        Serial.print("Max Speed: ");
        Serial.print(maxSpeedMS, 2);
        Serial.print(" m/s  (");
        Serial.print(maxSpeedMS * 3.6, 2);
        Serial.println(" km/h)");
        Serial.print("Max Accel: ");
        Serial.print(maxAccel, 2);
        Serial.println(" m/s^2");
        Serial.println("------------------------------");

        // update display ONLY when we have a new measured speed
        showSpeedOnDisplay(speedKMH);
        state = SHOW;
      }
      break;

    case SHOW:
      // keep showing last speed until object moves away,
      // then reset display to 0 and go back to WAITING
      if (d > farDist + 10) {
        Serial.println("READY FOR NEXT OBJECT");
        showSpeedOnDisplay(0);   // reset display just once here
        state = WAITING;
      }
      break;
  }

  // multiplex display
  sevseg.refreshDisplay();
  delay(60);   // small delay to slow down serial spam + avoid flicker
}
