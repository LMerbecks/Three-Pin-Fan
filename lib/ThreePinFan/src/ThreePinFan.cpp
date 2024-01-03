#include "ThreePinFan.h"

ThreePinFan::ThreePinFan(uint8_t _controlSignalPin, uint8_t _sensorPin, float _divisions){
    sensorPin = _sensorPin;
    divisions = _divisions;
    controlSignalPin = _controlSignalPin;
}

void ThreePinFan::getSample(){
    if(checkSampleTime()){
        double currentSample = (double) digitalRead(sensorPin);
        shiftSamples(1);
        addNew(currentSample);
    }
    resetRanThisPeriodFlag();
}

void ThreePinFan::shiftSamples(int shift){
    for(int i = NUM_SAMPLES-1; i > (shift - 1); i--){
        // iterate backwards because shift is to the front;
        samples[i - shift] = samples[i];
    }

    for(uint16_t i = NUM_SAMPLES-1; i > ((NUM_SAMPLES - 1) - shift); i++){
        samples[i] = 0.0;
    }
}

void ThreePinFan::addNew(double newValue){
    samples[NUM_SAMPLES - 1] = newValue;
}

bool ThreePinFan::checkSampleTime(){
    return (micros() % SAMPLING_TIME_US <= SAMPLING_TIME_TOL_US) && !ranThisPeriod;
}

void ThreePinFan::resetRanThisPeriodFlag(){
    ranThisPeriod = !(micros() % SAMPLING_TIME_US >= SAMPLING_TIME_TOL_US);
}

void ThreePinFan::copyArrays(double *targetArray, double *sourceArray, uint16_t arraySize){
    for(uint16_t i = 0; i < arraySize; i++){
        targetArray[i] = sourceArray[i];
    }
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
    getSample();
    updateFFT();
    speedFFT.Windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
    speedFFT.Compute(FFT_FORWARD);
    speedFFT.ComplexToMagnitude();
    
    double mechanicalFrequency = speedFFT.MajorPeak() * 0.5;

    return freqToRPM(mechanicalFrequency);
}

double ThreePinFan::freqToRPM(double freq){
    return freq * 0.01666667;
}

void ThreePinFan::initializeArray(double *array, uint16_t arrayLength){
    for(uint16_t i = 0; i < arrayLength; i++){
        array[i] = 0.0;
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
    calculateRPM();
    controlRPM();
    // Serial.println("updating...");
}

void ThreePinFan::updateFFT(){
    copyArrays(fftReal, samples, NUM_SAMPLES);
    copyArrays(fftImag, samplesImag, NUM_SAMPLES);
    speedFFT = arduinoFFT(fftReal, fftImag, NUM_SAMPLES, SAMPLING_FREQ);
}

void ThreePinFan::begin(){
    pinMode(sensorPin, INPUT_PULLUP);
    pinMode(controlSignalPin, OUTPUT);
    initializeArray(samples, NUM_SAMPLES);
    initializeArray(samplesImag, NUM_SAMPLES);
    initializeArray(fftReal, NUM_SAMPLES);
    initializeArray(fftImag, NUM_SAMPLES);
    currentRPM = 0.0;
    currentSensorValue = LOW;
    currentTime_us = 0;
    previousSensorValue = LOW;
    previousTime_us = 0;
    speedControlPID.SetOutputLimits(200, 255);
    speedControlPID.SetMode(AUTOMATIC);
}

void ThreePinFan::PrintVector(double *vData, uint16_t bufferSize, uint8_t scaleType)
{
  for (uint16_t i = 0; i < bufferSize; i++)
  {
    double abscissa;
    /* Print abscissa value */
    switch (scaleType)
    {
      case SCL_INDEX:
        abscissa = (i * 1.0);
	break;
      case SCL_TIME:
        abscissa = ((i * 1.0) / SAMPLING_FREQ);
	break;
      case SCL_FREQUENCY:
        abscissa = ((i * 1.0 * SAMPLING_FREQ) / NUM_SAMPLES);
	break;
    }
    Serial.print(abscissa, 6);
    if(scaleType==SCL_FREQUENCY)
      Serial.print("Hz");
    Serial.print(" ");
    Serial.println(vData[i], 4);
  }
  Serial.println();
}