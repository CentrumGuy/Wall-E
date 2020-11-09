#include <myDuino.h>
#include "MotorHandler.h"
#include "Motor.h"
#include "MotorDirection.h"
#include "Task.h"
#include "CalibrationTask.h"
#include "Mode.h"



// BEGIN CONSTANTS
#define START_DELAY 5
#define CANNON_DELAY 3
#define DROP_DELAY 5
#define CALIBRATION_DELAY 3

#define IR_TARGET_FAR 100
#define IR_TARGET_CLOSE 190

#define DROPPER_MONEYBALL_EXTENSION 7
#define DROPPER_CUPS_EXTENSION 3
#define DROPPER_DROP_EXTENSION 2.9
#define MONEYBALL_TO_CUPS_DISTANCE 21.5

#define CALIBRATION_SPEED 0.5
#define DROPPER_CALIBRATION_DISTANCE 12.85
#define DRIVE_CALIBRATION_DISTANCE 36

Task tasks[] = {
  TASK_ZERO_DROPPER,
  TASK_START,
  TASK_FIRE_CANNON,
  TASK_MOVE_TO_MONEYBALL,
  TASK_CAPTURE_MONEYBALL,
  TASK_DRIVE_TO_CUPS,
  TASK_DROP_BALLS,
  TASK_RETURN,
  TASK_STOP
};

CalibrationTask calibrationTasks[] = {
  CALIBRATION_ZERO_DROPPER,
  CALIBRATION_DELAY_TASK,
  CALIBRATION_CALIBRATE_DROPPER,
  CALIBRATION_ZERO_DROPPER,
  CALIBRATION_DELAY_TASK,
  CALIBRATION_CALIBRATE_DRIVE,
  CALIBRATION_STOP
};

#define START_STOP_BUTTON 1
#define ENDSTOP_BUTTON 2
#define CALIBRATION_BUTTON 3
#define TRACK_LED 1
#define CALIBRATION_LED 2
#define LASER_PIN 1
// END CONSTANTS



// BEGIN INSTANCE VARIABLES
myDuino robot(1);
Mode mode = IDLE;
bool isInitialTaskRun = true;
unsigned long deadline = millis();
unsigned short taskIndex = 0;
MotorHandler driveHandler(DRIVE);
MotorHandler dropperHandler(DROPPER);
// END INSTANCE VARIABLES







void setup() {
  Serial.begin(9600);
  driveHandler.setRobot(robot);
  dropperHandler.setRobot(robot);
}

void loop() {
  if (robot.readButton(START_STOP_BUTTON)) {
    driveHandler.stop();
    dropperHandler.stop();
    mode = IDLE;
    taskIndex = 0;
    isInitialTaskRun = true;
    robot.LED(TRACK_LED, 0);
    robot.LED(CALIBRATION_LED, 0);
    robot.digital(LASER_PIN, 1);
    return;
  }

  driveHandler.update();
  dropperHandler.update();
  robot.digital(LASER_PIN, 0);
  switch (mode) {
    case IDLE: {
      mode = robot.readButton(CALIBRATION_BUTTON) ? CALIBRATION : TRACK;
      robot.LED(mode == TRACK ? TRACK_LED : CALIBRATION_LED, 1);
      break;
    } case TRACK: {
      runTask();
      break;
    } case CALIBRATION: {
      runCalibrationTask();
      break;
    }
  }
}

void runTask() {
  bool shouldContinue = true;
  switch (tasks[taskIndex]) {
    case TASK_ZERO_DROPPER: {
      shouldContinue = zeroDropper();
      break;
    } case TASK_START: {
      shouldContinue = start();
      break;
    } case TASK_FIRE_CANNON: {
      shouldContinue = fireCannons();
      break;
    } case TASK_MOVE_TO_MONEYBALL: {
      shouldContinue = moveToMoneyball();
      break;
    } case TASK_CAPTURE_MONEYBALL: {
      shouldContinue = captureMoneyball();
      break;
    } case TASK_DRIVE_TO_CUPS: {
      shouldContinue = driveToCups();
      break;
    } case TASK_DROP_BALLS: {
      shouldContinue = dropBalls();
      break;
    } case TASK_RETURN: {
      shouldContinue = returnRobot();
      break;
    } case TASK_STOP: {
      shouldContinue = stop();
      break;
    }
  }

  if (shouldContinue) nextTask();
  isInitialTaskRun = false;
}

void runCalibrationTask() {
  bool shouldContinue = true;
  switch (calibrationTasks[taskIndex]) {
    case CALIBRATION_ZERO_DROPPER: {
      shouldContinue = zeroDropper();
      break;
    } case CALIBRATION_DELAY_TASK: {
      shouldContinue = calibrationDelay();
      break;
    } case CALIBRATION_CALIBRATE_DROPPER: {
      shouldContinue = calibrateDropper();
      break;
    } case CALIBRATION_CALIBRATE_DRIVE: {
      shouldContinue = calibrateDrive();
      break;
    } case CALIBRATION_STOP: {
      shouldContinue = stop();
      break;
    }
  }

  if (shouldContinue) nextTask();
  isInitialTaskRun = false;
}





// BEGIN TASKS

/*
 * ZERO DROPPER
 * Returns the dropper to its initial position
 */
bool zeroDropper() {
  if (isInitialTaskRun) {
    dropperHandler.move(0.8, BACKWARD, false);
  }

  if (robot.readButton(ENDSTOP_BUTTON)) {
    dropperHandler.move(0.3, FORWARD, false);
  } else if (dropperHandler.getDirection() == FORWARD) {
    dropperHandler.stop();
    return true;
  }

  return false;
}

/*
 * START
 * Prepare the robot. This task will delay the robot by the set START_DELAY
 */
bool start() {
  if (isInitialTaskRun) deadline = millis() + START_DELAY * 1000;
  return millis() >= deadline;
}

/*
 * FIRE CANNONS
 * Fires the cannons. This task will fire the cannons and then delay the robot by the set CANNON_DELAY
 */
bool fireCannons() {
  if (isInitialTaskRun) {
    deadline = millis() + CANNON_DELAY * 1000;
    // TODO: Fire Cannons
  }
  return millis() >= deadline;
}

/*
 * MOVE TO MONEYBALL
 * Moves the robot to the moneyball. This task will terminate once the moneyball is
 * within close reach of the robot
 */
bool moveToMoneyball() {
  if (isInitialTaskRun) {
    driveHandler.move(0.6, FORWARD, true);
  }

  if (robot.readIR() > IR_TARGET_CLOSE) {
    driveHandler.stop();
    return true;
  } else if (robot.readIR() > IR_TARGET_FAR) {
    driveHandler.move(0.3, FORWARD, true);
  }

  return false;
}

/*
 * CAPTURE MONEYBALL
 * Extends the drawer slides to capture the moneyball. This task will terminate once the drawer slides have
 * extended the required distance
 */
bool captureMoneyball() {
  if (isInitialTaskRun) {
    dropperHandler.distanceMove(0.5, FORWARD, DROPPER_MONEYBALL_EXTENSION, false);
  }
  return !dropperHandler.isMoving();
}

/*
 * DRIVE TO CUPS
 * Drives the robot to the cups, extending the drawer slides along the way. This task will terminate
 * once the robot has extended the drawer slides the required distance and moved forward the required distance
 */
bool driveToCups() {
  if (isInitialTaskRun) {
    driveHandler.distanceMove(1, FORWARD, MONEYBALL_TO_CUPS_DISTANCE, true);
    dropperHandler.distanceMove(0.6, FORWARD, DROPPER_CUPS_EXTENSION, false);
  }

  return !driveHandler.isMoving() && !dropperHandler.isMoving();
}

/*
 * DROP BALLS
 * Drops the balls in the cups, extending the drawer slides just enough to release the trap door. This task
 * will terminate once the drawer slides are fully extended and at least DROP_DELAY seconds have passed
 * since the beginning of the event.
 */
bool dropBalls() {
  if (isInitialTaskRun) {
    dropperHandler.distanceMove(0.3, FORWARD, DROPPER_DROP_EXTENSION, false);
    deadline = millis() + DROP_DELAY * 1000;
  }

  return !dropperHandler.isMoving() && millis() >= deadline;
}

/*
 * RETURN ROBOT
 * Returns the robot to the original location. This task will terminate when the robot is no longer tracing
 * back its original steps.
 */
bool returnRobot() {
  if (isInitialTaskRun) {
    driveHandler.playback();
  }

  return zeroDropper() && !driveHandler.isInPlayback();
}

/*
 * STOP
 * Stops the robot. This task doesn't terminate.
 */
bool stop() {
  driveHandler.stop();
  dropperHandler.stop();
  return false;
}




// CALIBRATION TASKS

/*
 * CALIBRATION DELAY
 * Delays the next task by the specified CALIBRATION_DELAY amount of seconds
 */
bool calibrationDelay() {
  if (isInitialTaskRun) deadline = millis() + CALIBRATION_DELAY * 1000;
  return millis() >= deadline;
}

/*
 * CALIBRATE DROPPER
 * Calibrates the dropper motor. Task will terminate when the calibration button is pressed.
 * The calibration button should be pressed when the dropper has traveled the DROPPER_CALIBRATION_DISTANCE in MotorHandler.cpp
 */
bool calibrateDropper() {
  if (isInitialTaskRun) dropperHandler.calibrate(DROPPER_CALIBRATION_DISTANCE, CALIBRATION_SPEED);
  else if (robot.readButton(CALIBRATION_BUTTON)) {
    dropperHandler.stop();
    return true;
  }
  return false;
}

/*
 * CALIBRATE DRIVE
 * Calibrates the drive motor. Task will terminate when the calibration button is pressed.
 * The calibration button should be pressed when the dropper has traveled the DRIVE_CALIBRATION_DISTANCE in MotorHandler.cpp
 */
bool calibrateDrive() {
  if (isInitialTaskRun) driveHandler.calibrate(DRIVE_CALIBRATION_DISTANCE, CALIBRATION_SPEED);
  else if (robot.readButton(CALIBRATION_BUTTON)) {
    driveHandler.stop();
    return true;
  }
  return false;
}






// BEGIN HELPER METHODS
void nextTask() {
  taskIndex++;
  isInitialTaskRun = true;
  mode == TRACK ? runTask() : runCalibrationTask();
}
