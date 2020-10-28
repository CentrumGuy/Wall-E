#ifndef Task_h
#define Task_h

#include "Arduino.h"

enum Task {
  TASK_ZERO_DROPPER = 0,
  TASK_START = 1,
  TASK_FIRE_CANNON = 2,
  TASK_MOVE_TO_MONEYBALL = 3,
  TASK_CAPTURE_MONEYBALL = 4,
  TASK_DRIVE_TO_CUPS = 5,
  TASK_DROP_BALLS = 6,
  TASK_RETURN = 7,
  TASK_STOP = 8
};

#endif
