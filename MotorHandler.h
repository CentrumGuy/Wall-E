#ifndef MotorHandler_h
#define MotorHandler_h

#include "Arduino.h"
#include "Motor.h"
#include "MotorDirection.h"
#include "MotorRecording.h"

#include <myDuino.h>

class MotorHandler {
  public:
    MotorHandler(Motor motor);
    void setRobot(myDuino robot);
    void update();
    void move(float speed, MotorDirection direction, bool record);
    void timedMove(float speed, MotorDirection direction, unsigned long duration, bool record);
    void distanceMove(float speed, MotorDirection direction, float distance, bool record);
    void calibrate(float distance, float calibrationSpeed);
    void stopRecording();
    void playback();
    void stop();
    Motor getMotor();
    MotorDirection getDirection();
    float getSpeed();
    bool isMoving();
    bool isRecording();
    bool isInPlayback();
    bool isCalibrating();
    
  private:
    Motor _motor;
    myDuino *_robot;
    MotorDirection _direction;
    unsigned long _deadline;
    float _speedScalar;
    float _speed;
    float _calibrationDistance;
    bool _isRecording;
    bool _isInPlayback;
    bool _isCalibrating;
    int _recordingIndex;
    MotorRecording _calibrationRecording;
    MotorRecording _recordings[7];

    myDuino robot();
    void startRecording();
    void stopCalibrating();
    void calibrateSpeedScalar(double distance, MotorRecording recording);
    void saveSpeedScalar();
    void loadSpeedScalar();
    unsigned long travelTime(double distance, float speed);
    float ratedSpeed();
    float effectiveRadius();
    int directionNumber(MotorDirection direction);
};

#endif
