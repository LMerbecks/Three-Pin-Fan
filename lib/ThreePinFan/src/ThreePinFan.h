#ifndef THREEPINFAN_HEADER_GUARD
#define THREEPINFAN_HEADER_GUARD
#include <Arduino.h>
#include <PID_v1.h>
#include <StateMachine.h>

#define DUTY_CYCLE 0.2
#define TIME_PERIOD_US 100000

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

    PID speedControlPID = PID(&currentRPM, &controlSignal, &targetRPM, proportionalConstant, integralConstant, differentialConstant, DIRECT);

    bool dutyCycleFunction(unsigned long period_us, double dutyCylce);

    void begin();
    
    void update();
    float calculateRPM();
    unsigned long getTimeDelta();
    bool detectChange();

    void setRPM(double _targetRPM);
    void getRPM();
    void controlRPM();
};

#endif