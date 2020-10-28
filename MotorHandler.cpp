#include "Arduino.h"
#include "MotorHandler.h"
#include "Motor.h"
#include "MotorDirection.h"
#include "MotorRecording.h"

#include <myDuino.h>
#include <math.h>
#include <EEPROM.h>

#define BIG_MOTOR_RATED_SPEED 57
#define SMALL_MOTOR_RATED_SPEED 43
#define SMALL_GEAR_RADIUS 0.5
#define BIG_GEAR_RADIUS 0.71875
#define SMALL_GEAR_BIG_GEAR_OFFSET_X 0.25
#define SMALL_GEAR_BIG_GEAR_OFFSET_Y 1.2
#define WHELL_RADIUS 1.25
#define DRIVE_FORWARD_DIRECTION 2
#define DRIVE_BACKWARD_DIRECTION 1
#define DROPPER_FORWARD_DIRECTION 1
#define DROPPER_BACKWARD_DIRECTION 2
#define DRIVE_SPEED_SCALAR_SAVE_ADRESS 0



MotorHandler::MotorHandler(Motor motor) {
  _motor = motor;
  _direction = FORWARD;
  _deadline = 0;
  _speedScalar = 1;
  _speed = 0;
  _calibrationDistance = 12;
  _isRecording = false;
  _recordingIndex = 0;
  _isInPlayback = false;
  _isCalibrating = false;
}

void MotorHandler::setRobot(myDuino robot) {
  _robot = &robot;
  loadSpeedScalar();
}





void MotorHandler::update() {
  if (!isMoving() || _deadline == 0) return;
  if (millis() >= _deadline) {
    if (isInPlayback()) {
      Serial.print("Next recording... Current index: ");
      Serial.println(_recordingIndex);
      playback();
    } else {
      stop();
      Serial.println("Stopped");
    }
  }
}

void MotorHandler::move(float speed, MotorDirection direction, bool record) {
  if (_speed == speed && _direction == direction && _isRecording == record) return;
  _deadline = 0;
  _direction = direction;
  _speed = speed;
  if (record) startRecording();
  int motorSpeed = (int) (speed * 255);
  robot().moveMotor(_motor, directionNumber(direction), motorSpeed);
  Serial.print("Started Moving Motor: ");
  Serial.print(_motor);
  Serial.print(", Motor Speed: ");
  Serial.print(motorSpeed);
  Serial.print(", Direction: ");
  Serial.print(directionNumber(direction));
  Serial.print(", Speed: ");
  Serial.print(_speed);
  Serial.print(", Recording: ");
  Serial.println(record);
}

void MotorHandler::timedMove(float speed, MotorDirection direction, unsigned long duration, bool record) {
  move(speed, direction, record);
  _deadline = millis() + duration;
}

void MotorHandler::distanceMove(float speed, MotorDirection direction, float distance, bool record) {
  unsigned long duration = travelTime(distance, speed);
  timedMove(speed, direction, duration, record);
  Serial.print("Started moving distance with time: ");
  Serial.println(duration);
}

void MotorHandler::calibrate(float calibrationDistance, float calibrationSpeed) {
  stop();
  _isCalibrating = true;
  _calibrationDistance = calibrationDistance;
  MotorRecording calibrationRecording(calibrationSpeed, FORWARD);
  move(calibrationSpeed, FORWARD, false);
  _calibrationRecording = calibrationRecording;
}

void MotorHandler::stopRecording() {
  if (!isRecording()) return;
  MotorRecording recording = _recordings[_recordingIndex];
  recording.endRecording();
  _recordings[_recordingIndex] = recording;
  Serial.print("Stopped recording. Duration: ");
  Serial.print(recording.getDuration());
  Serial.print(", Speed: ");
  Serial.print(recording.getSpeed());
  Serial.print(", Direction: ");
  Serial.print(directionNumber(recording.getDirection()));
  Serial.print(", Index: ");
  Serial.println(_recordingIndex);
  _recordingIndex++;
  _isRecording = false;
}

void MotorHandler::playback() {
  stopRecording();
  _isInPlayback = true;
  _recordingIndex--;
  if (_recordingIndex < 0) {
    _recordingIndex = 0;
    _isInPlayback = false;
    stop();
    return;
  }

  MotorRecording recording = _recordings[_recordingIndex];
  Serial.print("Playing back at index ");
  Serial.print(_recordingIndex);
  Serial.print(", duration: ");
  Serial.println(recording.getDuration());
  timedMove(recording.getSpeed(), recording.getOppositeDirection(), recording.getDuration(), false);
}

void MotorHandler::stop() {
  stopRecording();
  stopCalibrating();
  _speed = 0;
  _deadline = 0;
  _isInPlayback = false;
  robot().moveMotor(_motor, directionNumber(_direction), 0);
}

Motor MotorHandler::getMotor() {
  return _motor;
}

MotorDirection MotorHandler::getDirection() {
  return _direction;
}

float MotorHandler::getSpeed() {
  return _speed;
}

bool MotorHandler::isMoving() {
  return getSpeed() != 0;
}

bool MotorHandler::isRecording() {
  return _isRecording;
}

bool MotorHandler::isInPlayback() {
  return _isInPlayback;
}

bool MotorHandler::isCalibrating() {
  return _isCalibrating;
}







// BEGIN PRIVATE METHODS

myDuino MotorHandler::robot() {
  return *_robot;
}

void MotorHandler::startRecording() {
  if (isRecording()) stopRecording();
  MotorRecording recording(_speed, _direction);
  _recordings[_recordingIndex] = recording;
  Serial.print("Started recording. Speed: ");
  Serial.print(recording.getSpeed());
  Serial.print(", direction: ");
  Serial.println(directionNumber(recording.getDirection()));
  _isRecording = true;
}

void MotorHandler::stopCalibrating() {
  if (!isCalibrating()) return;
  float calibrationDistance = _calibrationDistance;
  MotorRecording calibrationRecording = _calibrationRecording;
  calibrationRecording.endRecording();
  calibrateSpeedScalar(calibrationDistance, calibrationRecording);
  _isCalibrating = false;
}

void MotorHandler::calibrateSpeedScalar(double distance, MotorRecording recording) {
  float speed = recording.getSpeed();
  unsigned long duration = recording.getDuration();
  float radPerSec = (ratedSpeed() * 2 * M_PI * speed) / 60;
  float radPerMillisecond = radPerSec / 1000;
  float idealVelocity = radPerMillisecond * effectiveRadius();
  float observedVelocity = distance/duration;
  _speedScalar = observedVelocity/idealVelocity;
  saveSpeedScalar();
}

void MotorHandler::saveSpeedScalar() {
  int addr = _motor == DRIVE ? DRIVE_SPEED_SCALAR_SAVE_ADRESS : (DRIVE_SPEED_SCALAR_SAVE_ADRESS + sizeof(float));
  EEPROM.put(addr, _speedScalar);
  Serial.print("Saving speed scalar for motor: ");
  Serial.print(_motor);
  Serial.print(", scalar: ");
  Serial.println(_speedScalar);
}

void MotorHandler::loadSpeedScalar() {
  int addr = _motor == DRIVE ? DRIVE_SPEED_SCALAR_SAVE_ADRESS : (DRIVE_SPEED_SCALAR_SAVE_ADRESS + sizeof(float));
  float scalar = 1;
  EEPROM.get(addr, scalar);
  if (scalar == NAN) scalar = 1;
  _speedScalar = scalar;
  Serial.print("Loaded speed scalar for motor: ");
  Serial.print(_motor);
  Serial.print(", scalar: ");
  Serial.println(scalar);
}

// The time it takes to travel a certain distance with a certain motor speed (0 ... 1)
unsigned long MotorHandler::travelTime(double distance, float speed) {
  float radPerSec = (ratedSpeed() * 2 * M_PI * speed) / 60;
  float radPerMillisecond = radPerSec / 1000;
  float velocity = radPerMillisecond * effectiveRadius() * _speedScalar;
  return (unsigned long) distance/velocity;
}

float MotorHandler::ratedSpeed() {
  switch (_motor) {
  case DRIVE: return BIG_MOTOR_RATED_SPEED;
  case DROPPER: return SMALL_MOTOR_RATED_SPEED;
  }
}

float MotorHandler::effectiveRadius() {
switch (_motor) {
  case DRIVE: {
      float smallGearBigGearDistance = sqrt(pow(SMALL_GEAR_BIG_GEAR_OFFSET_X, 2) + pow(SMALL_GEAR_BIG_GEAR_OFFSET_Y, 2));
      float gearSpacing = smallGearBigGearDistance - (SMALL_GEAR_RADIUS + BIG_GEAR_RADIUS);
      float smallGearRadius = SMALL_GEAR_RADIUS + (gearSpacing/2);
      float bigGearRadius = BIG_GEAR_RADIUS + (gearSpacing/2);
      return abs((smallGearRadius/bigGearRadius) * WHELL_RADIUS);
    }
  case DROPPER:
    return SMALL_GEAR_RADIUS;
  }
}

int MotorHandler::directionNumber(MotorDirection direction) {
  switch (direction) {
    case FORWARD:
      return _motor == DRIVE ? DRIVE_FORWARD_DIRECTION : DROPPER_FORWARD_DIRECTION;
     case BACKWARD:
      return _motor == DRIVE ? DRIVE_BACKWARD_DIRECTION : DROPPER_BACKWARD_DIRECTION;
  }
}
