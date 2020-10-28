#include "Arduino.h"
#include "MotorRecording.h"
#include "MotorDirection.h"

MotorRecording::MotorRecording() {
  _speed = 0;
  _direction = FORWARD;
  _beginTime = millis();
  _endTime = millis();
  _isComplete = false;
}

MotorRecording::MotorRecording(float speed, MotorDirection direction) {
  _speed = speed;
  _direction = direction;
  _beginTime = millis();
  _endTime = millis();
  _isComplete = false;
}

void MotorRecording::endRecording() {
  _endTime = millis();
  _isComplete = true;
}

bool MotorRecording::isComplete() {
  return _isComplete;
}

float MotorRecording::getSpeed() {
  return _speed;
}

unsigned long MotorRecording::getBeginTime() {
  return _beginTime;
}

unsigned long MotorRecording::getEndTime() {
  return _endTime;
}

unsigned long MotorRecording::getDuration() {
  return _endTime - _beginTime;
}

MotorDirection MotorRecording::getDirection() {
  return _direction;
}

MotorDirection MotorRecording::getOppositeDirection() {
  switch(_direction) {
    case FORWARD:
      return BACKWARD;
    case BACKWARD:
      return FORWARD;
  }
}
