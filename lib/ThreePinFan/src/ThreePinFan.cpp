#include "ThreePinFan.h"

ThreePinFan::ThreePinFan(uint8_t _controlSignalPin, uint8_t _sensorPin, float _divisions){
    sensorPin = _sensorPin;
    divisions = _divisions;
    controlSignalPin = _controlSignalPin;
}

bool ThreePinFan::detectChange(){
    previousSensorValue = currentSensorValue;
    currentSensorValue = digitalRead(sensorPin);
    // Serial.println("curr State: " + String(currentSensorValue));
    return previousSensorValue != currentSensorValue;
}

unsigned long ThreePinFan::getTimeDelta(){
    previousTime_us = currentTime_us;
    currentTime_us = micros();
    return currentTime_us - previousTime_us;
}

float ThreePinFan::calculateRPM(){
    float timeDelta_us = (float) getTimeDelta();
    float timeDelta_min = timeDelta_us / (1e6 * 60.0);
    return 1/(divisions*timeDelta_min);
}

void ThreePinFan::getRPM(){
    if(detectChange()){
        currentRPM = calculateRPM();
    }
}

void ThreePinFan::controlRPM(){
    speedControlPID.Compute();
    analogWrite(controlSignalPin, controlSignal);
}

void ThreePinFan::setRPM(double _targetRPM){
    targetRPM = _targetRPM;
}

void ThreePinFan::update(){
    getRPM();
    controlRPM();
    // Serial.println("updating...");
}

void ThreePinFan::begin(){
    pinMode(sensorPin, INPUT_PULLUP);
    pinMode(controlSignalPin, OUTPUT);
    currentRPM = 0.0;
    currentSensorValue = LOW;
    currentTime_us = 0;
    previousSensorValue = LOW;
    previousTime_us = 0;
    speedControlPID.SetOutputLimits(200, 255);
    speedControlPID.SetMode(AUTOMATIC);
}