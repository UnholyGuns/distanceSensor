#define NUMSENSORS 1

const float soundSpeed = 1125.0; // ft/s, 343 m/s
const unsigned long sensorTimeout = 38000; // sensor timeout is 38ms per datasheet

class HCSR04 {
  public:
    unsigned long startTime, endTime;
    float distance;
    int echoPin, trigPin;
    bool echoRising = true; 
    bool pingGood = true;

    //HCSR04() : startTime(0), endTime(0), distance(0), echoPin(-1), trigPin(-1), echoRising(true), pingGood(true) {}

    HCSR04(int echoPinIn, int trigPinIn) : startTime(0), endTime(0), distance(0), echoPin(echoPinIn), trigPin(trigPinIn), echoRising(true), pingGood(true) {}

    float calcDistance() {
      pingGood = true; // measurement is done, we are good to ping again

      if (endTime != 0) { // arbitrary out of range condition set in the ISR
        distance = (static_cast<float>(endTime) - static_cast<float>(startTime)) * soundSpeed / 2 / 1000000.0; // Convert to seconds
        endTime = 0; // reset endTime after calculation
        return distance;
      } else {
        return 0;
      }
    }

    void ping() {
      if (pingGood) {
        pingGood = false; // prevent pinging again until the measurement is complete
        digitalWrite(trigPin, HIGH);
        delayMicroseconds(10); // doesn't block for long, we don't care for now
        digitalWrite(trigPin, LOW);
      }  
    }
};

// global variables
HCSR04 sensors[NUMSENSORS] = { HCSR04(3, 2) }; // Initialize with pins

void setup() {
  // initialize hardware pins
  for (int i = 0; i < NUMSENSORS; i++) {
    pinMode(sensors[i].echoPin, INPUT);
    pinMode(sensors[i].trigPin, OUTPUT);

    sensors[i].distance = 0;
    sensors[i].startTime = 0;
    sensors[i].endTime = 0;
  }

  // attach interrupts
  attachInterrupt(digitalPinToInterrupt(sensors[0].echoPin), updateSensor<0>, CHANGE);

  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  sensors[0].ping();
  if (sensors[0].endTime != 0) {
    sensors[0].calcDistance();
  }
  Serial.println(sensors[0].distance);
  delay(100); // Adding delay to avoid flooding Serial Monitor
}

//did all this so that it was non-blocking. we hate blocking
template<int i>
void updateSensor() {
  unsigned long currentTime = micros();
  if (sensors[i].echoRising == true) {
    sensors[i].startTime = currentTime;
    sensors[i].echoRising = false;
  } else {
    sensors[i].endTime = currentTime;
    if ((sensors[i].endTime - sensors[i].startTime) >= sensorTimeout) {
      sensors[i].endTime = 0;
    }
    sensors[i].echoRising = true;
  }
}