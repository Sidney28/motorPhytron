/*
FILENAME... phytronAxisMotor.h
USAGE...    Motor record  support for Phytron Axis controller.

Tom Slejko & Bor Marolt
Cosylab d.d. 2014

*/

#include "asynMotorController.h"
#include "asynMotorAxis.h"
#include <interpolation.h>


//Number of controller specific parameters
#define NUM_PHYTRON_PARAMS 42

#define MAX_VELOCITY      40000 //steps/s
#define MIN_VELOCITY      1     //steps/s

#define MAX_ACCELERATION  40000  // steps/s^2
#define MIN_ACCELERATION  1      // steps/s^2

//Controller parameters
#define controllerStatusString      "CONTROLLER_STATUS"
#define controllerStatusResetString "CONTROLLER_STATUS_RESET"
#define resetControllerString       "CONTROLLER_RESET"

//Axis parameters
#define axisStatusString            "AXIS_STATUS"
#define homingProcedureString       "HOMING_PROCEDURE"
#define axisModeString              "AXIS_MODE"
#define mopOffsetPosString          "MOP_POS"
#define mopOffsetNegString          "MOP_NEG"
#define stepResolutionString        "STEP_RES"
#define stopCurrentString           "STOP_CURRENT"
#define runCurrentString            "RUN_CURRENT"
#define boostCurrentString          "BOOST_CURRENT"
#define encoderTypeString           "ENCODER_TYP"
#define initRecoveryTimeString      "INIT_TIME"
#define positionRecoveryTimeString  "POSITION_TIME"
#define boostConditionString        "BOOST"
#define encoderRateString           "ENC_RATE"
#define switchTypString             "SWITCH_TYP"
#define pwrStageModeString          "PWR_STAGE_MODE"
#define encoderResolutionString     "ENC_RESOLUTION"
#define encoderFunctionString       "ENC_FUNCTION"
#define encoderSFIWidthString       "ENC_SFI_WIDTH"
#define encoderDirectionString      "ENC_DIRECTION"
#define powerStageTempString        "PS_TEMPERATURE"
#define powerStagetMonitorString    "PS_MONITOR"
#define motorTempString             "MOTOR_TEMP"
#define currentDelayTimeString      "CURRENT_DELAY_TIME"
#define axisResetString             "AXIS_RESET"
#define axisStatusResetString       "AXIS_STATUS_RESET"

#define enableEncRevCounterStirng    "ENABLE_ENC_REV_COUNTER"
#define encoderRevolutionCountString "ENCODER_REVOLUTION_COUNT"

#define motorRecEncoderResolutionString "MOTOR_REC_ENCODER_RESOLUTION"
#define profileCurrentFollowingErrorString "PROFILE_CUR_FOL_ERR"
#define profileControlFrequencyString "PROFILE_CONTROL_FREQUENCY"
#define profileControlVeloString "PROFILE_CONTROL_VELO"

#define profileCorrectionString "PROFILE_CORRECTION"
#define profileCorrectionEnableString "PROFILE_CORRECTION_ENABLE"
#define profileCorrectionMaximumString "PROFILE_CORRECTION_MAX_ALLOWABLE"

//JVEL and JAR
#define motorJogVeloString "MOTOR_JOGVELO"
#define motorJogAccString "MOTOR_JOGACC"

// max allowable profile following error
#define profileFollowingErrorAllowableString "PROFILE_FOLLOWING_ERROR_ALLOWABLE"
#define profileFollowingErrorExceedAllowableValueString "PROFILE_FOLLOWING_ERROR_EXCEED_ALLOWABLE_VALUE"

typedef enum {
  phytronSuccess,
  phytronTimeout,
  phytronOverflow,
  phytronError,
  phytronDisconnected,
  phytronDisabled,
  phytronInvalidReturn,
  phytronInvalidCommand
} phytronStatus;

enum movementType{
  stdMove,
  homeMove,
  stopMove
};

enum homingType{
  limit,
  center,
  encoder,
  limitEncoder,
  centerEncoder,
  referenceCenter,
  referenceCenterEncoder,
};


class phytronAxis : public asynMotorAxis
{
public:
  /* These are the methods we override from the base class */
  phytronAxis(class phytronController *pC, int axis);
  void report(FILE *fp, int level);
  asynStatus move(double position, int relative, double min_velocity, double max_velocity, double acceleration);
  asynStatus moveVelocity(double min_velocity, double max_velocity, double acceleration);
  asynStatus home(double min_velocity, double max_velocity, double acceleration, int forwards);
  asynStatus stop(double acceleration);
  asynStatus poll(bool *moving);
  asynStatus setPosition(double position);

  asynStatus setEncoderRatio(double ratio);
  asynStatus setEncoderPosition(double position);

  virtual asynStatus setPGain(double pGain);
  virtual asynStatus setIGain(double iGain);
  virtual asynStatus setDGain(double dGain);

  virtual asynStatus initializeProfile(size_t maxPoints);
  virtual asynStatus defineProfile(double *positions, size_t numPoints);
  virtual asynStatus buildProfile();
  virtual asynStatus executeProfile();
  virtual asynStatus abortProfile();
  virtual asynStatus readbackProfile();
  
  asynStatus axisProfileControl();
  
  float axisModuleNo_; //Used by sprintf to form commands
protected:
  
  double pGain;
  double iGain;
  double dGain;
 
  double profileErrorIntegral;
 
  int axisMovingDirection;  
  
  int encoderRevolutionCountEnabled;
  int encoderRevolutionCount;
  int lastPoolEncoderPosition;

  // corrections for profile with double loopback
  double profileCorrection;
  int profileCorrectionEnable;
  double profileCorrectionMaximum;

  // max allowable profile following error
  double profileFollowingErrorAllowable;
  int profileFollowingErrorExceedAllowableValue;
  // used for profile move
  alglib::spline1dinterpolant profileSpline;

private:
  phytronController *pC_;          /**< Pointer to the asynMotorController to which this axis belongs.
                                   *   Abbreviated because it is used very frequently */

  phytronStatus setVelocity(double minVelocity, double maxVelocity, int moveType);
  phytronStatus setAcceleration(double acceleration, int movementType);
  size_t response_len;
friend class phytronController;
};

class phytronController : public asynMotorController {
public:
  phytronController(const char *portName, const char *phytronPortName, double movingPollPeriod, double idlePollPeriod, double timeout, int profilemaxpoints);
  asynStatus readInt32(asynUser *pasynUser, epicsInt32 *value);
  asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
  asynStatus readFloat64(asynUser *pasynUser, epicsFloat64 *value);
  virtual asynStatus writeFloat64(asynUser *pasynUser, epicsFloat64 value);
  
  asynStatus writeFloat64Array(asynUser *pasynUser, epicsFloat64 *value, size_t nElements);
  asynStatus readFloat64Array(asynUser *pasynUser, epicsFloat64 *value, size_t nElements, size_t *nRead);

  virtual asynStatus initializeProfile(size_t maxPoints);
  virtual asynStatus buildProfile();
  virtual asynStatus executeProfile();
  virtual asynStatus abortProfile();
  virtual asynStatus readbackProfile();
  
  void profileExecuter();

  void report(FILE *fp, int level);
  phytronAxis* getAxis(asynUser *pasynUser);
  phytronAxis* getAxis(int axisNo);

  phytronStatus sendPhytronCommand(const char *command, char *response_buffer, size_t response_max_len, size_t *nread);

  void resetAxisEncoderRatio();

  //casts phytronStatus to asynStatus
  asynStatus    phyToAsyn(phytronStatus phyStatus);

  char * controllerName_;
  std::vector<phytronAxis*> axes;

protected:

  //Additional parameters used by additional records
  int axisStatus_;
  int controllerStatus_;
  int homingProcedure_;
  int axisMode_;
  int mopOffsetPos_;
  int mopOffsetNeg_;
  int stepResolution_;
  int stopCurrent_;
  int runCurrent_;
  int boostCurrent_;
  int encoderType_;
  int initRecoveryTime_;
  int positionRecoveryTime_;
  int boost_;
  int encoderRate_;
  int switchTyp_;
  int pwrStageMode_;
  int encoderRes_;
  int encoderFunc_;
  int encoderSFIWidth_;
  int encoderDirection_;
  int powerStageTemp_;
  int powerStageMonitor_;
  int motorTemp_;
  int currentDelayTime_;
  int resetController_;
  int axisReset_;
  int axisStatusReset_;
  int controllerStatusReset_;
  int enableEncRevCounter_;
  int encoderRevolutionCount_;
  
  // profile move
  int motorRecEncoderResolution_;
  int profileCurrentFollowingError_;
  int profileControlFrequency_;
  int profileControlVelo_;

  // corrections for profile with double loopback
  int profileCorrection_;
  int profileCorrectionEnable_;
  int profileCorrectionMaximum_;

  // max allowable profile following error
  int profileFollowingErrorAllowable_;
  int profileFollowingErrorExceedAllowableValue_;
  
  int motorJogVelo_;
  int motorJogAcc_;
private:
  double timeout_;
  epicsEventId profileControlEventId_;
  

  //parameters used for profile move
  int profileExecuterIsRunning;
  int profileExecuterAbort;
  
friend class phytronAxis;
};
