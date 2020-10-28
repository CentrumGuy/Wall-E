#ifndef CalibrationTask_h
#define CalibrationTask_h

#include "Arduino.h"

enum CalibrationTask {
  CALIBRATION_ZERO_DROPPER,
  CALIBRATION_DELAY_TASK,
  CALIBRATION_CALIBRATE_DROPPER,
  CALIBRATION_CALIBRATE_DRIVE,
  CALIBRATION_STOP
};

#endif
