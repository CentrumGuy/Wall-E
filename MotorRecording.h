#ifndef MotorRecording_h
#define MotorRecording_h

#include "MotorRecording.h"
#include "MotorDirection.h"

struct MotorRecording {
  public:
    MotorRecording();
    MotorRecording(float speed, MotorDirection direction);
    void endRecording();
    bool isComplete();
    float getSpeed();
    unsigned long getBeginTime();
    unsigned long getEndTime();
    unsigned long getDuration();
    MotorDirection getDirection();
    MotorDirection getOppositeDirection();

  private:
    MotorDirection _direction;
    bool _isComplete;
    float _speed;
    unsigned long _beginTime;
    unsigned long _endTime;
};

#endif
