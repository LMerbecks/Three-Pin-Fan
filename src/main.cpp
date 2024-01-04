#include <Arduino.h>
#include <ThreePinFan.h>
#include <math.h>

#define FAN_CONTROL_PIN 11
#define FAN_SENSOR_PIN 2
#define FAN_SENSOR_DIVISIONS 4.0

ThreePinFan myTestFan(FAN_CONTROL_PIN, FAN_SENSOR_PIN, FAN_SENSOR_DIVISIONS);

// double input, output, target;
// double Kp = 1.0;
// double Ki = 1.0;
// double Kd = 1.0;

// PID myPID(&input, &output, &target, Kp, Ki, Kd, DIRECT);

bool printedThisPeriod = false;
unsigned long timePeriod = 250;

void setup() {
  // put your setup code here, to run once:
  myTestFan.begin();
  Serial.begin(115200);
  myTestFan.setRPM(2500);
  // myPID.SetMode(AUTOMATIC);
  // target = 512;
}

void loop() {
  // put your main code here, to run repeatedly:
  myTestFan.update();

  if((millis() % timePeriod <= 1) & !printedThisPeriod){
    printedThisPeriod = true;
    // Serial.println("Current RPM: " + String(myTestFan.currentRPM));
    // Serial.println("Current PWM: " + String(myTestFan.controlSignal));
    
  }
  if(millis() % timePeriod > 1){
    printedThisPeriod = false;
  }
  // Serial.println(50*sin(2*PI / 30.0 * millis()/1000.0)+120);
  // analogWrite(FAN_CONTROL_PIN,50*sin(2*PI / 30.0 * millis()/1000.0)+120);
}