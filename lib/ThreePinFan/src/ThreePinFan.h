#ifndef THREEPINFAN_HEADER_GUARD
#define THREEPINFAN_HEADER_GUARD
#include <Arduino.h>
#include <PID_v1.h>
#include <arduinoFFT.h>

#define NUM_SAMPLES 256
#define SAMPLING_FREQ 150
#define SAMPLING_TIME_US 6667

class ThreePinFan{
    public:
    /// @brief Constructor for fan object
    /// @param _controlSigalPin 
    /// @param _sensorPin 
    /// @param _divisions 
    ThreePinFan(uint8_t _controlSigalPin, uint8_t _sensorPin, float _divisions);

    /// @brief Number of divisions the rpm sensor can recognize
    float divisions; 
    /// @brief current speed of the fan in rpm
    double currentRPM;
    /// @brief time of latest event
    unsigned long currentTime_us;
    /// @brief time of previous event
    unsigned long previousTime_us;
    /// @brief previous state of sensor
    int previousSensorValue;
    /// @brief latest sensor state
    int currentSensorValue;
    /// @brief pin number the sensor is connected to
    uint8_t sensorPin;
    /// @brief pin number of the control pin (connected to the power of the fan)
    uint8_t controlSignalPin;

    double proportionalConstant = 0.1;
    double differentialConstant = 0.0;
    double integralConstant = 0.1;

    double targetRPM;
    double controlSignal;

    float samples[NUM_SAMPLES];
    /// @brief imaginary part of samples, this will be zero. 
    float samplesImag[NUM_SAMPLES];

    PID speedControlPID = PID(&currentRPM, &controlSignal, &targetRPM, proportionalConstant, integralConstant, differentialConstant, DIRECT);
    arduinoFFT speedFFT; 

    /// @brief initializer function for runtime initialization
    void begin();
    
    /// @brief function to call in each loop
    void update();
    /// @brief calculate the currentRPM using an FFT
    /// @return current RPM
    float calculateRPM();
    /// @brief wrapper function to execute gathering of one sample of the RPM signal
    void getSample();
    /// @brief wrapper function to recondition the FFT object
    void updateFFT();
    /// @brief /// @brief converts between frequency and RPM
    /// @param freq to be converted
    /// @return RPM
    float freqToRPM(float freq);
    /// @brief check wether a sample should be acquired based on the sampling time
    /// @return true when a sample should be acquired
    bool checkSampleTime();
    /// @brief read value from signal pin
    void readValue();
    /// @brief shift all samples towards the end of the array
    void shiftSamples();
    /// @brief Add new element to the END of the sample array
    void addNew();
    /// @brief to be called in @ref begin to setup sample array. Basically fill it with zeros 
    void initializeSampleArray();
    unsigned long getTimeDelta();
    bool detectChange();

    void setRPM(double _targetRPM);
    void getRPM();
    void controlRPM();
};

#endif