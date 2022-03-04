/*
FILENAME... phytronAxisMotor.cpp
USAGE...    Motor driver support for Phytron Axis controller.

Tom Slejko & Bor Marolt
Cosylab d.d. 2014
 
*/

#include <stdio.h>
#include <stdint.h>
#include <algorithm>
#include <iostream>
#include <string.h>
#include <stdlib.h>
#include <vector>
#include <math.h>
#ifndef _WIN32
#include <unistd.h>
#endif

#include <drvAsynIPPort.h>
#include <iocsh.h>
#include <epicsThread.h>
#include <cantProceed.h>

#include <asynOctetSyncIO.h>

#include "phytronAxisMotor.h"
#include <epicsExport.h>

#include <interpolation.h> // alglib


using namespace std;

#ifndef ASYN_TRACE_WARNING
#define ASYN_TRACE_WARNING ASYN_TRACE_ERROR
#endif

//Used for casting position doubles to integers
#define NINT(f) (int)((f)>0 ? (f)+0.5 : (f)-0.5)

/*
 * Contains phytronController instances, phytronCreateAxis uses it to find and
 * bind axis object to the correct controller object.
 */
static vector<phytronController*> controllers;

/** Creates a new phytronController object.
  * \param[in] portName          The name of the asyn port that will be created for this driver
  * \param[in] phytronPortName   The name of the drvAsynIPPort that was created previously to connect to the phytron controller
  * \param[in] movingPollPeriod  The time between polls when any axis is moving
  * \param[in] idlePollPeriod    The time between polls when no axis is moving
  */
phytronController::phytronController(const char *phytronPortName, const char *asynPortName,
                                 double movingPollPeriod, double idlePollPeriod, double timeout,
                                 int profileMaxPointsNumber)
  :  asynMotorController(phytronPortName,
                         0xFF,
                         NUM_PHYTRON_PARAMS,
                         0, //No additional interfaces beyond those in base class
                         0, //No additional callback interfaces beyond those in base class
                         ASYN_CANBLOCK | ASYN_MULTIDEVICE,
                         1, // autoconnect
                         0, 0)// Default priority and stack size
{
  asynStatus status;
  size_t response_len;
  phytronStatus phyStatus;
  static const char *functionName = "phytronController::phytronController";

  //Timeout is defined in milliseconds, but sendPhytronCommand expects seconds
  timeout_ = timeout/1000;

  //pyhtronCreateAxis uses portName to identify the controller
  this->controllerName_ = (char *) mallocMustSucceed(sizeof(char)*(strlen(portName)+1),
      "phytronController::phytronController: Controller name memory allocation failed.\n");

  strcpy(this->controllerName_, portName);

  //Create Controller parameters
  createParam(controllerStatusString,     asynParamInt32, &this->controllerStatus_);
  createParam(controllerStatusResetString,asynParamInt32, &this->controllerStatusReset_);
  createParam(resetControllerString,      asynParamInt32, &this->resetController_);

  //Create Axis parameters
  createParam(axisStatusResetString,      asynParamInt32, &this->axisStatusReset_);
  createParam(axisResetString,            asynParamInt32, &this->axisReset_);
  createParam(axisStatusString,           asynParamInt32, &this->axisStatus_);
  createParam(homingProcedureString,      asynParamInt32, &this->homingProcedure_);
  createParam(axisModeString,             asynParamInt32, &this->axisMode_);
  createParam(mopOffsetPosString,         asynParamInt32, &this->mopOffsetPos_);
  createParam(mopOffsetNegString,         asynParamInt32, &this->mopOffsetNeg_);
  createParam(stepResolutionString,       asynParamInt32, &this->stepResolution_);
  createParam(stopCurrentString,          asynParamInt32, &this->stopCurrent_);
  createParam(runCurrentString,           asynParamInt32, &this->runCurrent_);
  createParam(boostCurrentString,         asynParamInt32, &this->boostCurrent_);
  createParam(encoderTypeString,          asynParamInt32, &this->encoderType_);
  createParam(initRecoveryTimeString,     asynParamInt32, &this->initRecoveryTime_);
  createParam(positionRecoveryTimeString, asynParamInt32, &this->positionRecoveryTime_);
  createParam(boostConditionString,       asynParamInt32, &this->boost_);
  createParam(encoderRateString,          asynParamInt32, &this->encoderRate_);
  createParam(switchTypString,            asynParamInt32, &this->switchTyp_);
  createParam(pwrStageModeString,         asynParamInt32, &this->pwrStageMode_);
  createParam(encoderResolutionString,    asynParamInt32, &this->encoderRes_);
  createParam(encoderFunctionString,      asynParamInt32, &this->encoderFunc_);
  createParam(encoderSFIWidthString,      asynParamInt32, &this->encoderSFIWidth_);
  createParam(encoderDirectionString,     asynParamInt32, &this->encoderDirection_);
  createParam(powerStagetMonitorString,   asynParamInt32, &this->powerStageMonitor_);
  createParam(currentDelayTimeString,     asynParamInt32, &this->currentDelayTime_);
  createParam(powerStageTempString,       asynParamFloat64, &this->powerStageTemp_);
  createParam(motorTempString,            asynParamFloat64, &this->motorTemp_);
  createParam(enableEncRevCounterStirng,  asynParamInt32, &this->enableEncRevCounter_);
  createParam(encoderRevolutionCountString,asynParamInt32, &this->encoderRevolutionCount_);

  createParam(motorRecEncoderResolutionString,    asynParamFloat64, &this->motorRecEncoderResolution_);
  createParam(profileCurrentFollowingErrorString, asynParamFloat64, &this->profileCurrentFollowingError_);
  createParam(profileControlVeloString, asynParamFloat64, &this->profileControlVelo_);
  createParam(profileControlFrequencyString, asynParamFloat64, &this->profileControlFrequency_);
 
  createParam(profileCorrectionString, asynParamFloat64, &this->profileCorrection_);
  createParam(profileCorrectionEnableString, asynParamInt32, &this->profileCorrectionEnable_); 
  createParam(profileCorrectionMaximumString, asynParamFloat64, &this->profileCorrectionMaximum_);

  createParam(profileFollowingErrorAllowableString, asynParamFloat64, &this->profileFollowingErrorAllowable_);
  createParam(profileFollowingErrorExceedAllowableValueString, asynParamInt32, &this->profileFollowingErrorExceedAllowableValue_);

  createParam(motorJogVeloString, asynParamFloat64, &this->motorJogVelo_);
  createParam(motorJogAccString, asynParamFloat64, &this->motorJogAcc_);
  
  /* for profile move control */
  profileControlEventId_ = epicsEventMustCreate(epicsEventEmpty);
  profileExecuterIsRunning = 0;
  
  /* Connect to phytron controller */
  status = pasynOctetSyncIO->connect(asynPortName, 0, &pasynUserController_, NULL);
  if (status) {
    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
      "%s: cannot connect to phytron controller\n",
      functionName);
  } else {
    //phytronCreateAxis will search for the controller for axis registration
    controllers.push_back(this);

    //RESET THE CONTROLLER
    sprintf(this->outString_, "CR");
    phyStatus = sendPhytronCommand(this->outString_, this->inString_, MAX_CONTROLLER_STRING_SIZE, &response_len);
    if(phyStatus){
      asynPrint(this->pasynUserSelf, ASYN_TRACE_WARNING,
            "phytronController::phytronController: Could not reset controller %s\n", this->controllerName_);
    }
    initializeProfile(profileMaxPointsNumber);
    
    //Wait for reset to finish
    epicsThreadSleep(10.0);

    startPoller(movingPollPeriod, idlePollPeriod, 5);
  }

}

/** Creates a new phytronController object.
  * Configuration command, called directly or from iocsh
  * \param[in] portName          The name of the asyn port that will be created for this driver
  * \param[in] phytronPortName   The name of the drvAsynIPPPort that was created previously to connect to the phytron controller
  * \param[in] numController     number of axes that this controller supports is numController*AXES_PER_CONTROLLER
  * \param[in] movingPollPeriod  The time in ms between polls when any axis is moving
  * \param[in] idlePollPeriod    The time in ms between polls when no axis is moving
  */
extern "C" int phytronCreateController(const char *phytronPortName, const char *asynPortName,
                                   int movingPollPeriod, int idlePollPeriod, double timeout,
                                   int profileMaxPointsNumber)
{
  new phytronController(phytronPortName, asynPortName, movingPollPeriod/1000., idlePollPeriod/1000., timeout, profileMaxPointsNumber);
  
  return asynSuccess;
}

/** asynUsers use this to read integer parameters
 * \param[in] pasynUser   asynUser structure containing the reason
 * \param[out] value      Parameter value
 */
asynStatus phytronController::readInt32(asynUser *pasynUser, epicsInt32 *value)
{
  phytronAxis   *pAxis;
  phytronStatus phyStatus;

  //Call base implementation first
  asynPortDriver::readInt32(pasynUser, value);

  //Check if this is a call to read a controller parameter
  if(pasynUser->reason == resetController_ || pasynUser->reason == controllerStatusReset_){
    //Called only on initialization of bo records RESET and RESET-STATUS
    return asynSuccess;
  } else if (pasynUser->reason == controllerStatus_){
    size_t response_len;
    sprintf(this->outString_, "ST");
    phyStatus = sendPhytronCommand(this->outString_, this->inString_, MAX_CONTROLLER_STRING_SIZE, &response_len);
    if(phyStatus){
      asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
          "phytronAxis::readInt32: Reading controller %s status failed with error "
          "code: %d\n", this->controllerName_, phyStatus);
      return phyToAsyn(phyStatus);
    }

    *value = atoi(this->inString_);
    return asynSuccess;
  }

  // controller parameters for profile moves
  if (pasynUser->reason == profileNumAxes_){
     return getIntegerParam(profileNumAxes_,value);
  }else if (pasynUser->reason == profileNumPoints_){
     return getIntegerParam(profileNumPoints_,value);
  }else if (pasynUser->reason == profileCurrentPoint_){
     return getIntegerParam(profileCurrentPoint_,value);
  }else if (pasynUser->reason == profileNumPulses_){
     return getIntegerParam(profileNumPulses_,value);
  }else if (pasynUser->reason == profileStartPulses_){
     return getIntegerParam(profileStartPulses_,value);
  }else if (pasynUser->reason == profileEndPulses_){
     return getIntegerParam(profileEndPulses_,value);
  }else if (pasynUser->reason == profileActualPulses_){
     return getIntegerParam(profileActualPulses_,value);
  }else if (pasynUser->reason == profileNumReadbacks_){
     return getIntegerParam(profileNumReadbacks_,value);
  }else if (pasynUser->reason == profileTimeMode_){
     return getIntegerParam(profileTimeMode_,value);
  }else if (pasynUser->reason == profileMoveMode_){
     return getIntegerParam(profileMoveMode_,value);
  }else if (pasynUser->reason == profileBuildState_){
     return getIntegerParam(profileBuildState_,value);
  }else if (pasynUser->reason == profileBuildStatus_){
     return getIntegerParam(profileBuildStatus_,value);
  }else if (pasynUser->reason == profileExecuteState_){
     return getIntegerParam(profileExecuteState_,value);
  }else if (pasynUser->reason == profileExecuteStatus_){
     return getIntegerParam(profileExecuteStatus_,value);
  }else if (pasynUser->reason == profileReadbackState_){
     return getIntegerParam(profileReadbackState_,value);
  }else if (pasynUser->reason == profileReadbackStatus_){
     return getIntegerParam(profileReadbackStatus_,value);
  }else if (pasynUser->reason == profileBuild_){
     return getIntegerParam(profileBuild_,value);
  }else if (pasynUser->reason == profileExecute_){
     return getIntegerParam(profileExecute_,value);
  }else if (pasynUser->reason == profileAbort_){
     return getIntegerParam(profileAbort_,value);
  }else if (pasynUser->reason == profileReadback_){
     return getIntegerParam(profileReadback_,value);
  }

  //This is an axis request, find the axis
  pAxis = getAxis(pasynUser);
  if(!pAxis){
    int axisNo;                                                                                                 
    getAddress(pasynUser, &axisNo);                                                                             
    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
       "phytronAxis::readInt32: Axis %d not found on the controller %s reason %d\n", axisNo, this->controllerName_, pasynUser->reason);
    return asynError;
  }

  if(pasynUser->reason == homingProcedure_){
    getIntegerParam(pAxis->axisNo_, homingProcedure_, value);
    return asynSuccess;
  } else if (pasynUser->reason == axisReset_ || pasynUser->reason == axisStatusReset_){
    //Called only on initialization of AXIS-RESET and AXIS-STATUS-RESET bo records
    return asynSuccess;
  } else if (pasynUser->reason == axisMode_){
    sprintf(this->outString_, "M%.1fP01R", pAxis->axisModuleNo_);
  } else if(pasynUser->reason == mopOffsetPos_){
    sprintf(this->outString_, "M%.1fP11R", pAxis->axisModuleNo_);
  } else if(pasynUser->reason == mopOffsetNeg_){
    sprintf(this->outString_, "M%.1fP12R", pAxis->axisModuleNo_);
  } else if (pasynUser->reason == stepResolution_){
    sprintf(this->outString_, "M%.1fP45R", pAxis->axisModuleNo_);
  } else if (pasynUser->reason == stopCurrent_){
    sprintf(this->outString_, "M%.1fP40R", pAxis->axisModuleNo_);
  } else if (pasynUser->reason == runCurrent_){
    sprintf(this->outString_, "M%.1fP41R", pAxis->axisModuleNo_);
  } else if (pasynUser->reason == boostCurrent_){
    sprintf(this->outString_, "M%.1fP42R", pAxis->axisModuleNo_);
  } else if (pasynUser->reason == encoderType_){
    sprintf(this->outString_, "M%.1fP34R", pAxis->axisModuleNo_);
  } else if(pasynUser->reason == initRecoveryTime_){
    sprintf(this->outString_, "M%.1fP13R", pAxis->axisModuleNo_);
  } else if(pasynUser->reason == positionRecoveryTime_){
    sprintf(this->outString_, "M%.1fP16R", pAxis->axisModuleNo_);
  } else if(pasynUser->reason == boost_){
    sprintf(this->outString_, "M%.1fP17R", pAxis->axisModuleNo_);
  } else if(pasynUser->reason == encoderRate_){
    sprintf(this->outString_, "M%.1fP26R", pAxis->axisModuleNo_);
  } else if(pasynUser->reason == switchTyp_){
    sprintf(this->outString_, "M%.1fP27R", pAxis->axisModuleNo_);
  } else if(pasynUser->reason == pwrStageMode_){
    sprintf(this->outString_, "M%.1fP28R", pAxis->axisModuleNo_);
  } else if(pasynUser->reason == encoderRes_){
    sprintf(this->outString_, "M%.1fP35R", pAxis->axisModuleNo_);
  } else if(pasynUser->reason == encoderFunc_){
    sprintf(this->outString_, "M%.1fP36R", pAxis->axisModuleNo_);
  } else if(pasynUser->reason == encoderSFIWidth_){
    sprintf(this->outString_, "M%.1fP37R", pAxis->axisModuleNo_);
  } else if(pasynUser->reason == encoderDirection_){
    sprintf(this->outString_, "M%.1fP38R", pAxis->axisModuleNo_);
  } else if(pasynUser->reason == currentDelayTime_){
    sprintf(this->outString_, "M%.1fP43R", pAxis->axisModuleNo_);
  } else if(pasynUser->reason == powerStageMonitor_){
    sprintf(this->outString_, "M%.1fP53R", pAxis->axisModuleNo_);
  } else if(pasynUser->reason == enableEncRevCounter_){
    *value = pAxis->encoderRevolutionCountEnabled;
    return asynSuccess;
  } else if(pasynUser->reason == encoderRevolutionCount_){
    *value = pAxis->encoderRevolutionCount;
    return asynSuccess;
  }

  if(pasynUser->reason == this->profileCorrectionEnable_){                                                    
    *value = pAxis->profileCorrectionEnable;                                                                   
    return asynSuccess;                                                                                       
  } else if(pasynUser->reason == this->profileFollowingErrorExceedAllowableValue_){                                      
    *value = pAxis->profileFollowingErrorExceedAllowableValue;                                                           
    return asynSuccess;                                                                                       
  }


  phyStatus = sendPhytronCommand(this->outString_, this->inString_, MAX_CONTROLLER_STRING_SIZE, &pAxis->response_len);
  if(phyStatus){
    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
              "phytronAxis::readInt32: Failed with status %d for reason %d\n", phyStatus, pasynUser->reason);
    return phyToAsyn(phyStatus);
  }

  *value = atoi(this->inString_);

  //{STOP,RUN,BOOST} current records have EGU set to mA, but device returns 10mA
  if(pasynUser->reason == stopCurrent_ || pasynUser->reason == runCurrent_ ||
      pasynUser->reason == boostCurrent_)
  {
    *value *= 10;
  } // else if


  return asynSuccess;
}

/** asynUsers use this to write integer parameters
 * \param[in] pasynUser   asynUser structure containing the reason
 * \param[in] value       Parameter value to be written
 */
asynStatus phytronController::writeInt32(asynUser *pasynUser, epicsInt32 value)
{
  phytronAxis   *pAxis;
  phytronStatus phyStatus;  
  asynStatus status;

  //Call base implementation first
  asynMotorController::writeInt32(pasynUser, value);

  // controller parameters for profile moves
  if (pasynUser->reason == profileNumAxes_){
     return setIntegerParam(profileNumAxes_,value);
  }else if (pasynUser->reason == profileNumPoints_){
     return setIntegerParam(profileNumPoints_,value);
  }else if (pasynUser->reason == profileCurrentPoint_){
     return setIntegerParam(profileCurrentPoint_,value);
  }else if (pasynUser->reason == profileNumPulses_){
     return setIntegerParam(profileNumPulses_,value);
  }else if (pasynUser->reason == profileStartPulses_){
     return setIntegerParam(profileStartPulses_,value);
  }else if (pasynUser->reason == profileEndPulses_){
     return setIntegerParam(profileEndPulses_,value);
  }else if (pasynUser->reason == profileActualPulses_){
     return setIntegerParam(profileActualPulses_,value);
  }else if (pasynUser->reason == profileNumReadbacks_){
     return setIntegerParam(profileNumReadbacks_,value);
  }else if (pasynUser->reason == profileTimeMode_){
     return setIntegerParam(profileTimeMode_,value);
  }else if (pasynUser->reason == profileMoveMode_){
     return setIntegerParam(profileMoveMode_,value);
  }else if (pasynUser->reason == profileBuildState_){
     return setIntegerParam(profileBuildState_,value);
  }else if (pasynUser->reason == profileBuildStatus_){
     return setIntegerParam(profileBuildStatus_,value);
  }else if (pasynUser->reason == profileExecuteState_){
     return setIntegerParam(profileExecuteState_,value);
  }else if (pasynUser->reason == profileExecuteStatus_){
     return setIntegerParam(profileExecuteStatus_,value);
  }else if (pasynUser->reason == profileReadbackState_){
     return setIntegerParam(profileReadbackState_,value);
  }else if (pasynUser->reason == profileReadbackStatus_){
     return setIntegerParam(profileReadbackStatus_,value);
  }


  if (pasynUser->reason == profileBuild_){
     status = setIntegerParam(profileBuild_,value);
     callParamCallbacks();
     buildProfile();
     return status;
  }else if (pasynUser->reason == profileExecute_){
     status = setIntegerParam(profileExecute_,value);
     callParamCallbacks();
     executeProfile();
     return status;
  }else if (pasynUser->reason == profileAbort_){
     status = setIntegerParam(profileAbort_,value);
     asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
          "phytronAxis::writeInt32: Abort profile %s \n", this->controllerName_);
     abortProfile();
     return status;
  }else if (pasynUser->reason == profileReadback_){
     status = setIntegerParam(profileReadback_,value);
     readbackProfile();
     return status;
  }


  /*
   * Check if this is a call to reset the controller, else it is an axis request
   */
  if(pasynUser->reason == resetController_){
    size_t response_len;
    sprintf(this->outString_, "CR");
    phyStatus = sendPhytronCommand(this->outString_, this->inString_, MAX_CONTROLLER_STRING_SIZE, &response_len);
    if(phyStatus){
      asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
          "phytronAxis::writeInt32: Reseting controller %s failed with error code: %d\n", this->controllerName_, phyStatus);
    }
    resetAxisEncoderRatio();
    return phyToAsyn(phyStatus);
  } else if(pasynUser->reason == controllerStatusReset_){
    size_t response_len;
    sprintf(this->outString_, "STC");
    phyStatus = sendPhytronCommand(this->outString_, this->inString_, MAX_CONTROLLER_STRING_SIZE, &response_len);
    if(phyStatus){
     asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
         "phytronAxis::writeInt32: Reseting controller %s failed with error code: %d\n", this->controllerName_, phyStatus);
    }
    return phyToAsyn(phyStatus);
  }

  /*
   * This is an axis request, find the axis
   */
  pAxis = getAxis(pasynUser);
  if(!pAxis){
    int axisNo;                                                                                               
    getAddress(pasynUser, &axisNo);                                                                           
    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,                                                          
       "phytronAxis::readInt32: Axis %d not found on the controller %s reason %d\n", axisNo, this->controllerName_, pasynUser->reason);
    return asynError;
  }

  // profile move axis params
  if(pasynUser->reason == profileUseAxis_){
     return pAxis->setIntegerParam(profileUseAxis_, value);
  }else

  // phytron controller comands and params
  if(pasynUser->reason == homingProcedure_){
    setIntegerParam(pAxis->axisNo_, pasynUser->reason, value);
    callParamCallbacks();
    return asynSuccess;
  } else if(pasynUser->reason == axisReset_){
    sprintf(this->outString_, "M%.1fC", pAxis->axisModuleNo_);
  } else if(pasynUser->reason == axisStatusReset_){
    sprintf(this->outString_, "SEC%.1f", pAxis->axisModuleNo_);
  } else if(pasynUser->reason == axisMode_){
    sprintf(this->outString_, "M%.1fP01=%d", pAxis->axisModuleNo_,value);
  } else if(pasynUser->reason == mopOffsetPos_){
    sprintf(this->outString_, "M%.1fP11=%d", pAxis->axisModuleNo_,value);
  } else if(pasynUser->reason == mopOffsetNeg_){
    sprintf(this->outString_, "M%.1fP12=%d", pAxis->axisModuleNo_,value);
  } else if (pasynUser->reason == stepResolution_){
    sprintf(this->outString_, "M%.1fP45=%d", pAxis->axisModuleNo_,value);
  }  else if (pasynUser->reason == stopCurrent_){
    value /= 10; //STOP_CURRENT record has EGU mA, device expects 10mA
    sprintf(this->outString_, "M%.1fP40=%d", pAxis->axisModuleNo_,value);
  } else if (pasynUser->reason == runCurrent_){
    value /= 10; //RUN_CURRENT record has EGU mA, device expects 10mA
    sprintf(this->outString_, "M%.1fP41=%d", pAxis->axisModuleNo_,value);
  } else if (pasynUser->reason == boostCurrent_){
    value /= 10; //BOOST_CURRENT record has EGU mA, device expects 10mA
    sprintf(this->outString_, "M%.1fP42=%d", pAxis->axisModuleNo_,value);
  } else if (pasynUser->reason == encoderType_){
    sprintf(this->outString_, "M%.1fP34=%d", pAxis->axisModuleNo_, value);
  } else if (pasynUser->reason == initRecoveryTime_){
    sprintf(this->outString_, "M%.1fP13=%d", pAxis->axisModuleNo_, value);
  } else if (pasynUser->reason == positionRecoveryTime_){
    sprintf(this->outString_, "M%.1fP16=%d", pAxis->axisModuleNo_, value);
  } else if (pasynUser->reason == boost_){
    sprintf(this->outString_, "M%.1fP17=%d", pAxis->axisModuleNo_, value);
  } else if (pasynUser->reason == encoderRate_){
    sprintf(this->outString_, "M%.1fP26=%d", pAxis->axisModuleNo_, value);
  } else if (pasynUser->reason == switchTyp_){
    sprintf(this->outString_, "M%.1fP27=%d", pAxis->axisModuleNo_, value);
  } else if (pasynUser->reason == pwrStageMode_){
    sprintf(this->outString_, "M%.1fP28=%d", pAxis->axisModuleNo_, value);
  } else if (pasynUser->reason == encoderRes_){
    sprintf(this->outString_, "M%.1fP35=%d", pAxis->axisModuleNo_, value);
  } else if (pasynUser->reason == encoderFunc_){
    //Value is VAL field of parameter P37 record. If P37 is positive P36 is set to 1, else 0
    sprintf(this->outString_, "M%.1fP36=%d", pAxis->axisModuleNo_, value > 0 ? 1 : 0);
  } else if(pasynUser->reason == encoderSFIWidth_){
    sprintf(this->outString_, "M%.1fP37=%d", pAxis->axisModuleNo_, value);
  } else if(pasynUser->reason == encoderSFIWidth_){
    sprintf(this->outString_, "M%.1fP38=%d", pAxis->axisModuleNo_, value);
  } else if(pasynUser->reason == powerStageMonitor_){
    sprintf(this->outString_, "M%.1fP53=%d", pAxis->axisModuleNo_, value);
  } else if(pasynUser->reason == currentDelayTime_){
    sprintf(this->outString_, "M%.1fP43=%d", pAxis->axisModuleNo_, value);
  } else if(pasynUser->reason == encoderDirection_){
    sprintf(this->outString_, "M%.1fP38=%d", pAxis->axisModuleNo_, value);
  } else if(pasynUser->reason == enableEncRevCounter_){
    pAxis->encoderRevolutionCountEnabled = value;
    return asynSuccess;
  } else if(pasynUser->reason == encoderRevolutionCount_){
    pAxis->encoderRevolutionCount = value;
    return asynSuccess;
  }

  if(pasynUser->reason == this->profileCorrectionEnable_){                                                    
    pAxis->profileCorrectionEnable = value;   
    return setIntegerParam(pAxis->axisNo_, pasynUser->reason, value);
  } else if(pasynUser->reason == this->profileFollowingErrorExceedAllowableValue_){                                      
    pAxis->profileFollowingErrorExceedAllowableValue = value;                                                            
    return setIntegerParam(pAxis->axisNo_, pasynUser->reason, value);                                         
  }

  phyStatus = sendPhytronCommand(this->outString_, this->inString_, MAX_CONTROLLER_STRING_SIZE, &pAxis->response_len);
  if(phyStatus){
    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
              "phytronAxis::writeInt32: Failed with status %d for reason %d\n", phyStatus, pasynUser->reason);
    return phyToAsyn(phyStatus);
  }

  return asynSuccess;
}

/** asynUsers use this to read float parameters
 * \param[in] pasynUser   asynUser structure containing the reason
 * \param[out] value      Parameter value
 */
asynStatus phytronController::readFloat64(asynUser *pasynUser, epicsFloat64 *value){
  phytronStatus phyStatus;
  phytronAxis   *pAxis;
  
  if(pasynUser->reason == profileFixedTime_){
    return getDoubleParam(profileFixedTime_, value);
  }else if(pasynUser->reason == profileAcceleration_){
    return getDoubleParam(profileAcceleration_, value);
  }
  
  pAxis = getAxis(pasynUser);
  if(!pAxis){
    int axisNo;                                                                                               
    getAddress(pasynUser, &axisNo);                                                                           
    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,                                                          
       "phytronAxis::readInt32: Axis %d not found on the controller %s reason %d\n", axisNo, this->controllerName_, pasynUser->reason);
    return asynError;
  }
  
  //Call base implementation first
  asynPortDriver::readFloat64(pasynUser, value);

  if(pasynUser->reason == this->profileCorrection_){                                                                   
    *value = pAxis->profileCorrection;
    return asynSuccess;
  } else if(pasynUser->reason == this->profileCorrectionMaximum_){                                               
    *value = pAxis->profileCorrectionMaximum;                                                                    
    return asynSuccess; 
  } else if(pasynUser->reason == this->profileFollowingErrorAllowable_){                                                    
    *value = pAxis->profileFollowingErrorAllowable;                                                                 
    return asynSuccess;                                                                                       
  }


  if(pasynUser->reason == powerStageTemp_){
    sprintf(this->outString_, "M%.1fP49R", pAxis->axisModuleNo_);
  } else if(pasynUser->reason == motorTemp_){
    sprintf(this->outString_, "M%.1fP54R", pAxis->axisModuleNo_);
  }else if(pasynUser->reason == profileCurrentFollowingError_){
    return getDoubleParam(pAxis->axisNo_, profileCurrentFollowingError_, value);
  } else if (pasynUser->reason == motorJogVelo_){
    return getDoubleParam(pAxis->axisNo_, motorJogVelo_,value);
  }else if (pasynUser->reason == motorJogAcc_){
     return getDoubleParam(pAxis->axisNo_, motorJogAcc_,value);
  }

  phyStatus = sendPhytronCommand(this->outString_, this->inString_, MAX_CONTROLLER_STRING_SIZE, &pAxis->response_len);
  if(phyStatus){
    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
              "phytronAxis::readFloat64: Failed with status %d for reason %d\n", phyStatus, pasynUser->reason);
    return phyToAsyn(phyStatus);
  }

  *value = atof(this->inString_);

  //Power stage and motor temperature records have EGU °C, but device returns 0.1 °C
  *value /= 10;

  return phyToAsyn(phyStatus);

}

asynStatus phytronController::writeFloat64(asynUser *pasynUser, epicsFloat64 value){
  phytronAxis   *pAxis;
  
  asynMotorController::writeFloat64(pasynUser,value);

  if(pasynUser->reason == profileFixedTime_){
    return setDoubleParam(profileFixedTime_,value);
  }else if(pasynUser->reason == profileAcceleration_){
    return setDoubleParam(profileAcceleration_,value);
  }if(pasynUser->reason == profileControlFrequency_){
   return setDoubleParam(profileControlFrequency_, value);
  }

  pAxis = getAxis(pasynUser);
  if(!pAxis){
    int axisNo;                                                                                               
    getAddress(pasynUser, &axisNo);                                                                           
    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,                                                          
       "phytronAxis::readInt32: Axis %d not found on the controller %s reason %d\n", axisNo, this->controllerName_, pasynUser->reason);
    return asynError;
  }
  
  if(pasynUser->reason == motorRecEncoderResolution_){
    return setDoubleParam(pAxis->axisNo_, motorRecEncoderResolution_, value);
  }

  if(pasynUser->reason == this->profileCorrection_){                                                                   
    pAxis->profileCorrection = value;
    return setDoubleParam(pAxis->axisNo_, pasynUser->reason, value);
  } else if(pasynUser->reason == this->profileCorrectionMaximum_){                                               
    pAxis->profileCorrectionMaximum = value;
    return setDoubleParam(pAxis->axisNo_, pasynUser->reason, value); 
  } else if(pasynUser->reason == this->profileFollowingErrorAllowable_){                                      
    pAxis->profileFollowingErrorAllowable = value;                                                          
    return setDoubleParam(pAxis->axisNo_, pasynUser->reason, value);
  } 

  if(pasynUser->reason == this->motorJogVelo_){
    return setDoubleParam(pAxis->axisNo_, pasynUser->reason, value);
  } 
  if(pasynUser->reason == this->motorJogAcc_){
    return setDoubleParam(pAxis->axisNo_, pasynUser->reason, value);
  }

  return asynSuccess;
}

asynStatus phytronController::writeFloat64Array(asynUser *pasynUser, epicsFloat64 *value, size_t nElements){
  return asynMotorController::writeFloat64Array(pasynUser, value, nElements);
}


asynStatus phytronController::readFloat64Array(asynUser *pasynUser, epicsFloat64 *value, size_t nElements, size_t *nRead){
  return asynMotorController::readFloat64Array(pasynUser, value, nElements, nRead);
}


/*
 * Reset the motorEncoderRatio to 1 after the reset of MCM unit
 */
void phytronController::resetAxisEncoderRatio(){

  for(uint32_t i = 0; i < axes.size(); i++){
    setDoubleParam(axes[i]->axisNo_, motorEncoderRatio_, 1);
  }
}


/** Reports on status of the driver
  * \param[in] fp The file pointer on which report information will be written
  * \param[in] level The level of report detail desired
  *
  * If details > 0 then information is printed about each axis.
  * After printing controller-specific information it calls asynMotorController::report()
  */
void phytronController::report(FILE *fp, int level)
{
  fprintf(fp, "MCB-4B motor driver %s, numAxes=%d, moving poll period=%f, idle poll period=%f\n",
    this->portName, numAxes_, movingPollPeriod_, idlePollPeriod_);

  // Call the base class method
  asynMotorController::report(fp, level);
}

/** Returns a pointer to an phytronAxis object.
  * Returns NULL if the axis number encoded in pasynUser is invalid.
  * \param[in] pasynUser asynUser structure that encodes the axis index number.
  */
phytronAxis* phytronController::getAxis(asynUser *pasynUser)
{
  return static_cast<phytronAxis*>(asynMotorController::getAxis(pasynUser));
}

/** Returns a pointer to an phytronAxis object.
  * Returns NULL if the axis number encoded in pasynUser is invalid.
  * \param[in] axisNo Axis index number.
  */
phytronAxis* phytronController::getAxis(int axisNo)
{
  return static_cast<phytronAxis*>(asynMotorController::getAxis(axisNo));
}

/**
 * @brief implements phytron specific data fromat
 * @param output
 * @param input
 * @param maxChars
 * @param nread
 * @param timeout
 * @return
 */
phytronStatus phytronController::sendPhytronCommand(const char *command, char *response_buffer, size_t response_max_len, size_t *nread)
{
    char buffer[255];
    char* buffer_end=buffer;
    static const char *functionName = "phytronController::sendPhytronCommand";

    *(buffer_end++)=0x02;                               //STX
    *(buffer_end++)='0';                                //Module address TODO: add class member
    buffer_end += sprintf(buffer_end,"%s",command);     //Append command
    *(buffer_end++)=0x3a;                               //Append separator

    buffer_end += sprintf(buffer_end,"%c%c",'X','X');   //XX disables checksum
    *(buffer_end++)=0x03;                               //Append ETX
    *(buffer_end)=0x0;                                  //Null terminate message for saftey

    phytronStatus status = (phytronStatus) writeReadController(buffer,buffer,255,nread, timeout_);
    if(status){
        return status;
    }

    char* nack_ack = strchr(buffer,0x02); //Find STX
    if(!nack_ack){
        nread=0;
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
          "%s: Communication failed\n",
          functionName);
        return phytronInvalidReturn;
    }
    nack_ack++; //NACK/ACK is one
    //ACK, extract response
    if(*nack_ack==0x06){
        char* separator = strchr(nack_ack,0x3a);          //find separator

        /* Copy data from nack_ack to
         * separator into buffer */
        uint32_t len = separator-nack_ack-1;              //calculate length of message
        if(len > response_max_len) len=response_max_len;

        memcpy(response_buffer,nack_ack+1,len);           //copy payload to destination
        response_buffer[separator-nack_ack-1]=0;          //Add NULL terminator

        *nread=strlen(response_buffer);
    }
    //NAK return error
    else if(*nack_ack==0x15){
        nread=0;
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
          "%s: Nack sent by the controller\n",
          functionName);
        return phytronInvalidCommand;
    }

    return status;

}

/** Castst phytronStatus to asynStatus enumeration
 * \param[in] phyStatus
 */
asynStatus phytronController::phyToAsyn(phytronStatus phyStatus){
  if(phyStatus == phytronInvalidReturn || phyStatus == phytronInvalidCommand) return asynError;
  return (asynStatus) phyStatus;
}

/* These are the functions for profile moves */
/** Initialize a profile move of multiple axes. */
asynStatus phytronController::initializeProfile(size_t maxProfilePoints)
{
  int axis;
  asynMotorAxis *pAxis;
  // static const char *functionName = "initializeProfile";
  
  maxProfilePoints_ = maxProfilePoints;
  if (profileTimes_) free(profileTimes_);
  profileTimes_ = (double *)calloc(maxProfilePoints, sizeof(double));
  for (axis=0; axis<numAxes_; axis++) {
    pAxis = getAxis(axis);
    if (!pAxis) continue;
    pAxis->initializeProfile(maxProfilePoints);
  }
  return asynSuccess;
}
  
/** Build a profile move of multiple axes. */
asynStatus phytronController::buildProfile()
{
  //static const char *functionName = "buildProfile";
  asynMotorAxis *pAxis;
  int i;
  asynStatus status;

  setIntegerParam(profileBuildState_,1);
  setIntegerParam(profileBuildStatus_,0);
  setStringParam(profileBuildMessage_,"");
  callParamCallbacks();

  for (i=0; i<numAxes_; i++) {
    pAxis = getAxis(i);
    if (!pAxis) continue;
    status = pAxis->buildProfile();
    if (status) break;
  }
  
  if(status == asynSuccess){
    setIntegerParam(profileBuildStatus_,1);
    setStringParam(profileBuildMessage_, "Ok"); 
  }

  setIntegerParam(profileBuildState_,0);
  setIntegerParam(profileBuild_,0);
  setIntegerParam(profileCurrentPoint_,0);
  printf("all is ok\n");
  callParamCallbacks();
  return status;
}

/** run executer **/
static void phytronProfileExecuterC(void *phytronController_ptr)
{
  phytronController *pController = (phytronController*)phytronController_ptr;
  pController->profileExecuter();
}

/** Execute a profile move of multiple axes. */
asynStatus phytronController::executeProfile()
{
  phytronAxis *pAxis;
  for (int i=0; i<numAxes_; i++) {
    pAxis = getAxis(i);
    if (!pAxis) continue;
    pAxis->executeProfile();
  }  
   
  epicsThreadCreate("phytronControllerProfileExecuter", 
                    epicsThreadPriorityLow,
                    epicsThreadGetStackSize(epicsThreadStackMedium),
                    (EPICSTHREADFUNC)phytronProfileExecuterC, (void *)this);
  
  return asynSuccess;
}

/** Aborts a profile move. */
asynStatus phytronController::abortProfile()
{
  // static const char *functionName = "abortProfile";
  int axis;
  asynMotorAxis *pAxis;
  
  for (axis=0; axis<numAxes_; axis++) {
    pAxis = getAxis(axis);
    if (!pAxis) continue;
    pAxis->abortProfile();
  }

  // stop the profile executer thread;
  if(profileExecuterIsRunning){
    profileExecuterAbort=1;
    epicsEventSignal(profileControlEventId_);
  }

  return asynSuccess;
}

/** Readback the actual motor positions from a profile move of multiple axes. */
asynStatus phytronController::readbackProfile()
{
  // static const char *functionName = "readbackProfile";
  int axis;
  asynMotorAxis *pAxis;
  
  for (axis=0; axis<numAxes_; axis++) {
    pAxis = getAxis(axis);
    if (!pAxis) continue;
    pAxis->readbackProfile();
  }
  return asynSuccess;
}

/**
* Profile executer.
* This function runs in separated thread and maneges the pereod of the control routines calls. 
*
*
**/
void phytronController::profileExecuter(){
   double controlPeriod = 1; // TODO: manage control freq
   int status;
   double acceleration = 1;
   int stopTriesNumber = 3;
   phytronAxis *pAxis = NULL; 
   int current_point=0;
   int profileNumPoints=0;
   epicsTimeStamp current_time_ts;
   double current_time;
   int axisControlStatus=0;
   int axisControlTry=0;
   int axisControlTriesMax=3;

   if (profileExecuterIsRunning){
     printf("profile executer thread alrady running...\n");
     return;
   }
   profileExecuterAbort=0;
   profileExecuterIsRunning = 1;
      
   controlPeriod=1./controlPeriod;
 
   setIntegerParam(profileExecuteStatus_,1);
   callParamCallbacks();

   epicsEventSignal(profileControlEventId_);
   while(1){
     if (controlPeriod != 0.) status = epicsEventWaitWithTimeout(profileControlEventId_, controlPeriod);
     else status = epicsEventWaitWithTimeout(profileControlEventId_,1.);
     
     lock();
     if(profileExecuterAbort){
       asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                "phytronController::profileExecuter: Aborting porfile \n"); 
       for(int axis=0; axis<numAxes_; axis++) {
          pAxis = getAxis(axis);
          if (!pAxis) continue;
          status = getDoubleParam(pAxis->axisNo_, motorAccel_, &acceleration); 
          for(int tryNumb=0; tryNumb < stopTriesNumber; tryNumb++){
             status = pAxis->stop(acceleration);
             if(!status) break;
             epicsEventWaitWithTimeout(profileControlEventId_, controlPeriod);
          }
          if(status){
             asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                "phytronController::profileExecuter: Stop axis %d failed \n",
                pAxis->axisNo_);
          }        
       }
       
       setIntegerParam(profileExecuteState_, 0);
       setIntegerParam(profileExecute_, 0);
       callParamCallbacks();

       profileExecuterAbort=0;
       profileExecuterIsRunning=0;
       unlock();
       break;
     }
     
     getIntegerParam(profileNumPoints_, &profileNumPoints);
     
     epicsTimeGetCurrent(&current_time_ts);
     current_time = current_time_ts.secPastEpoch + double(current_time_ts.nsec)/1000000000.;

     while(current_point < profileNumPoints){
        if(profileTimes_[current_point]>current_time) break;
        current_point++;
     }
     
     setIntegerParam(profileCurrentPoint_, current_point);
     setIntegerParam(profileExecuteState_, 2);
     callParamCallbacks();

     if(current_point >= profileNumPoints){
        profileExecuterAbort=1;
        epicsEventSignal(profileControlEventId_);
        // profile move done
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                "phytronController::profileExecuter: Profile move done (current_point >= profileNumPoints) \n");
        setIntegerParam(profileExecuteState_, 3);
        callParamCallbacks();
        
        unlock(); 
        continue;
     }

     axisControlStatus=asynSuccess;
     for(int axis=0; axis<numAxes_; axis++) {
       pAxis = getAxis(axis);
       if (!pAxis) continue;
       axisControlStatus|=pAxis->axisProfileControl();
     }
     
     if (axisControlStatus != asynSuccess){
         axisControlTry++;
         for(int axis=0; axis<numAxes_; axis++) {
           pAxis = getAxis(axis);
           if (!pAxis) continue;
           if (axisControlTry >= axisControlTriesMax){
             this->setIntegerParam(pAxis->axisNo_, this->profileFollowingErrorExceedAllowableValue_, 1);
             callParamCallbacks();
             pAxis->stop(1.);
           }
         }
     }else{
         axisControlTry=0;
     }

     getDoubleParam(profileControlFrequency_, &controlPeriod);
     if (controlPeriod == 0.){
        controlPeriod = 1.;
     }else{
        controlPeriod = 1. / controlPeriod;
     }

     unlock();
   }

}

//******************************************************************************
//                   PHYTRON AXIS IMPLEMENTATION
//******************************************************************************

/** Creates a new phytronAxis object.
  * Configuration command, called directly or from iocsh
  * \param[in] controllerName    Name of the asyn port created by calling phytronCreateController from st.cmd
  * \param[in] module            Index of the I1AM01 module controlling this axis
  * \param[in] axis              Axis index
  */
extern "C" int phytronCreateAxis(const char* controllerName, int module, int axis){

  phytronAxis *pAxis;

  //Find the controller
  uint32_t i;
  for(i = 0; i < controllers.size(); i++){
    if(!strcmp(controllers[i]->controllerName_, controllerName)) {
      controllers[i]->lock();
      pAxis = new phytronAxis(controllers[i], module*10 + axis);
      controllers[i]->axes.push_back(pAxis);
      controllers[i]->unlock();
      break;
    }
  }

  //If controller is not found, report error
  if(i == controllers.size()){
    printf("ERROR: phytronCreateAxis: Controller %s is not registered\n", controllerName);
    return asynError;
  }

  return asynSuccess;
}

/** Creates a new phytronAxis object.
  * \param[in] pC Pointer to the phytronController to which this axis belongs.
  * \param[in] axisNo Index number of this axis, range 0 to pC->numAxes_-1.
  *
  * Initializes register numbers, etc.
  */
phytronAxis::phytronAxis(phytronController *pC, int axisNo)
  : asynMotorAxis(pC, axisNo),
    axisModuleNo_((float)axisNo/10),
    pC_(pC),
    response_len(0),
    encoderRevolutionCountEnabled(1),
    encoderRevolutionCount(0),
    lastPoolEncoderPosition(0)
{
  //Controller always supports encoder. Encoder enable/disable is set through UEIP
  setIntegerParam(pC_->motorStatusHasEncoder_, 1);

  setDoubleParam(pC_->motorEncoderRatio_, 1);
  
  setDoubleParam(pC_->profileCurrentFollowingError_, 0.);

  // TODO: AUTOSAVE  
  profileCorrection = 0.;                              
  setDoubleParam(pC_->profileCorrection_, 0.);                                          

  profileCorrectionMaximum = 1.;
  setDoubleParam(pC_->profileCorrectionMaximum_, 1.);
 
  profileFollowingErrorAllowable = 2.;
  setDoubleParam(pC_->profileFollowingErrorAllowable_, 2.);
 
  initializeProfile(pC->maxProfilePoints_);
}



/** Reports on status of the axis
  * \param[in] fp The file pointer on which report information will be written
  * \param[in] level The level of report detail desired
  *
  * After printing device-specific information calls asynMotorAxis::report()
  */
void phytronAxis::report(FILE *fp, int level)
{
  if (level > 0) {
    fprintf(fp, "  axis %d\n",
            axisNo_);
  }

  // Call the base class method
  asynMotorAxis::report(fp, level);
}

/** Sets velocity parameters before the move is executed. Controller produces a
 * trapezoidal speed profile defined by these parmeters.
 * \param[in] minVelocity   Start velocity
 * \param[in] maxVelocity   Maximum velocity
 * \param[in] moveType      Type of movement determines which controller speed parameters are set
 */
phytronStatus phytronAxis::setVelocity(double minVelocity, double maxVelocity, int moveType)
{

  phytronStatus maxStatus = phytronSuccess;
  phytronStatus minStatus = phytronSuccess;
  maxVelocity = fabs(maxVelocity);
  minVelocity = fabs(minVelocity);

  if(maxVelocity > MAX_VELOCITY){
    maxVelocity = MAX_VELOCITY;
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_WARNING,
              "phytronAxis::setVelocity: Failed for axis %d - Velocity %f is to high, setting to"
              "maximum velocity: %d!\n", axisNo_, maxVelocity, MAX_VELOCITY);
  } else if (maxVelocity < MIN_VELOCITY){
    maxVelocity = MIN_VELOCITY;
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_WARNING,
              "phytronAxis::setVelocity: Failed for axis %d - Velocity %f is to low, setting to"
              "minimum velocity: %d!\n", axisNo_, maxVelocity, MIN_VELOCITY);
  }

  if(minVelocity > MAX_VELOCITY){
    minVelocity = MAX_VELOCITY;
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_WARNING,
              "phytronAxis::setVelocity: Failed for axis %d - Velocity %f is to high, setting to"
              "maximum velocity: %d!\n", axisNo_, maxVelocity, MAX_VELOCITY);
  } else if (minVelocity < MIN_VELOCITY){
    minVelocity = MIN_VELOCITY;
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_WARNING,
              "phytronAxis::setVelocity: Failed for axis %d - Velocity %f is to low, setting to"
              "minimum velocity: %d!\n", axisNo_, minVelocity, MIN_VELOCITY);
  }


  if(moveType == stdMove){
    //Set maximum velocity (P14)
    sprintf(pC_->outString_, "M%.1fP14=%f", axisModuleNo_, maxVelocity);
    maxStatus = pC_->sendPhytronCommand(pC_->outString_, pC_->inString_, MAX_CONTROLLER_STRING_SIZE, &this->response_len);

    //Set minimum velocity (P04)
    sprintf(pC_->outString_, "M%.1fP04=%f", axisModuleNo_, minVelocity);
    minStatus = pC_->sendPhytronCommand(pC_->outString_, pC_->inString_, MAX_CONTROLLER_STRING_SIZE, &this->response_len);
  } else if (moveType == homeMove){
    //Set maximum velocity (P08)
    sprintf(pC_->outString_, "M%.1fP08=%f", axisModuleNo_, maxVelocity);
    maxStatus = pC_->sendPhytronCommand(pC_->outString_, pC_->inString_, MAX_CONTROLLER_STRING_SIZE, &this->response_len);

    //Set minimum velocity (P10)
    sprintf(pC_->outString_, "M%.1fP10=%f", axisModuleNo_, minVelocity);
    minStatus = pC_->sendPhytronCommand(pC_->outString_, pC_->inString_, MAX_CONTROLLER_STRING_SIZE, &this->response_len);
  }

  return (maxStatus > minStatus) ? maxStatus : minStatus;
}

/** Sets acceleration parameters before the move is executed.
 * \param[in] acceleration  Acceleration to be used in the move
 * \param[in] moveType      Type of movement determines which controller acceleration parameters is set
 */
phytronStatus phytronAxis::setAcceleration(double acceleration, int moveType)
{
  if(acceleration > MAX_ACCELERATION){
    acceleration = MAX_ACCELERATION;
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_WARNING,
              "phytronAxis::setAcceleration: Failed for axis %d - Acceleration %f is to high, "
              "setting to maximum acceleration: %d!\n", axisNo_, acceleration, MAX_ACCELERATION);
  } else if(acceleration < MIN_ACCELERATION){
    acceleration = MIN_ACCELERATION;
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_WARNING,
              "phytronAxis::setAcceleration: Failed for axis %d - Acceleration %f is to low, "
              "setting to minimum acceleration: %d!\n", axisNo_, acceleration, MIN_ACCELERATION);
  }

  if (moveType == stdMove){
    sprintf(pC_->outString_, "M%.1fP15=%f", axisModuleNo_, acceleration);
  } else if(moveType == homeMove){
    sprintf(pC_->outString_, "M%.1fP09=%f", axisModuleNo_, acceleration);
  } else if (moveType == stopMove){
    sprintf(pC_->outString_, "M%.1fP07=%f", axisModuleNo_, acceleration);
  }

  return pC_->sendPhytronCommand(pC_->outString_, pC_->inString_, MAX_CONTROLLER_STRING_SIZE, &this->response_len);
}

/** Execute the move.
 * \param[in] position      Target position (relative or absolute).
 * \param[in] relative      Is the move absolute or relative
 * \param[in] minVelocity   Lowest velocity of the trapezoidal speed profile.
 * \param[in] maxVelocity   Highest velocity of the trapezoidal speed profile
 * \param[in] acceleration  Acceleration to be used
 */
asynStatus phytronAxis::move(double position, int relative, double minVelocity, double maxVelocity, double acceleration)
{
  phytronStatus phyStatus;

  //NOTE: Check if velocity is different, before setting it.
  phyStatus = setVelocity(minVelocity, maxVelocity, stdMove);
  if(phyStatus){
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
              "phytronAxis::move: Setting the velocity for axis %d to %f failed with error "
              "code: %d!\n", axisNo_, maxVelocity, phyStatus);
    return pC_->phyToAsyn(phyStatus);
  }

  //NOTE: Check if velocity is different, before setting it.
  phyStatus = setAcceleration(acceleration, stdMove);
  if(phyStatus){
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
              "phytronAxis::move: Setting the acceleration for axis %d to %f failed with "
              "error code: %d!\n", axisNo_, acceleration, phyStatus);
    return pC_->phyToAsyn(phyStatus);
  }

  if (relative) {
    sprintf(pC_->outString_, "M%.1f%c%d", axisModuleNo_, position>0 ? '+':'-', abs(NINT(position)));
  } else {
    sprintf(pC_->outString_, "M%.1fA%d", axisModuleNo_, NINT(position));
  }

  phyStatus = pC_->sendPhytronCommand(pC_->outString_, pC_->inString_, MAX_CONTROLLER_STRING_SIZE, &this->response_len);
  if(phyStatus){
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
              "phytronAxis::move: Moving axis %d failed with error code: %d!\n", axisNo_, phyStatus);
    return pC_->phyToAsyn(phyStatus);
  }

  return asynSuccess;
}

/** Execute the homing procedure
 * \param[in] minVelocity   Lowest velocity of the trapezoidal speed profile.
 * \param[in] maxVelocity   Highest velocity of the trapezoidal speed profile
 * \param[in] acceleration  Acceleration to be used
 * \param[in] forwards      Direction of homing move
 */
asynStatus phytronAxis::home(double minVelocity, double maxVelocity, double acceleration, int forwards)
{
  phytronStatus phyStatus;
  int           homingType;

  pC_->getIntegerParam(axisNo_, pC_->homingProcedure_, &homingType);

  phyStatus =  setVelocity(minVelocity, maxVelocity, homeMove);
  if(phyStatus){
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
        "phytronAxis::home: Setting the velocity for axis %d to %f failed with error "
        "code: %d!\n", axisNo_, maxVelocity, phyStatus);
    return pC_->phyToAsyn(phyStatus);
  }

  phyStatus =  setAcceleration(acceleration, homeMove);
  if(phyStatus){
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
        "phytronAxis::home: Setting the acceleration for axis %d to %f failed with "
        "error code: %d!\n", axisNo_, acceleration, phyStatus);
    return pC_->phyToAsyn(phyStatus);
  }

  if(forwards){
    if(homingType == limit) sprintf(pC_->outString_, "M%.1fR+", axisModuleNo_);
    else if(homingType == center) sprintf(pC_->outString_, "M%.1fR+C", axisModuleNo_);
    else if(homingType == encoder) sprintf(pC_->outString_, "M%.1fR+I", axisModuleNo_);
    else if(homingType == limitEncoder) sprintf(pC_->outString_, "M%.1fR+^I", axisModuleNo_);
    else if(homingType == centerEncoder) sprintf(pC_->outString_, "M%.1fR+C^I", axisModuleNo_);
    //Homing procedures for rotational movements (no hardware limit switches)
    else if(homingType == referenceCenter) sprintf(pC_->outString_, "M%.1fRC+", axisModuleNo_);
    else if(homingType == referenceCenterEncoder) sprintf(pC_->outString_, "M%.1fRC+^I", axisModuleNo_);
  } else {
    if(homingType == limit) sprintf(pC_->outString_, "M%.1fR-", axisModuleNo_);
    else if(homingType == center) sprintf(pC_->outString_, "M%.1fR-C", axisModuleNo_);
    else if(homingType == encoder) sprintf(pC_->outString_, "M%.1fR-I", axisModuleNo_);
    else if(homingType == limitEncoder) sprintf(pC_->outString_, "M%.1fR-^I", axisModuleNo_);
    else if(homingType == centerEncoder) sprintf(pC_->outString_, "M%.1fR-C^I", axisModuleNo_);
    //Homing procedures for rotational movements (no hardware limit switches)
    else if(homingType == referenceCenter) sprintf(pC_->outString_, "M%.1fRC-", axisModuleNo_);
    else if(homingType == referenceCenterEncoder) sprintf(pC_->outString_, "M%.1fRC-^I", axisModuleNo_);
  }

  phyStatus = pC_->sendPhytronCommand(pC_->outString_, pC_->inString_, MAX_CONTROLLER_STRING_SIZE, &this->response_len);
  if(phyStatus){
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
              "phytronAxis::home: Moving axis %d failed with error code: %d!\n", axisNo_, phyStatus);
    return pC_->phyToAsyn(phyStatus);
  }

  return asynSuccess;
}

/** Jog the motor. Direction is determined by sign of the maxVelocity profile
 * \param[in] minVelocity   Lowest velocity of the trapezoidal speed profile.
 * \param[in] maxVelocity   Highest velocity of the trapezoidal speed profile
 * \param[in] acceleration  Acceleration to be used
 */
asynStatus phytronAxis::moveVelocity(double minVelocity, double maxVelocity, double acceleration)
{
  phytronStatus phyStatus;

  phyStatus = setVelocity(minVelocity, maxVelocity, stdMove);
  if(phyStatus){

    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
             "phytronAxis::moveVelocity: Setting the velocity for axis %d to %f failed with error "
             "code: %d!\n", axisNo_, maxVelocity, phyStatus);
  }

  phyStatus = setAcceleration(acceleration, stdMove);
  if(phyStatus){
   asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
             "phytronAxis::moveVelocity: Setting the acceleration for axis %d to %f failed with "
             "error code: %d!\n", axisNo_, acceleration, phyStatus);
  }
  
  if(maxVelocity * axisMovingDirection < 0){
      sprintf(pC_->outString_, "M%.1fS", axisModuleNo_);
      phyStatus = pC_->sendPhytronCommand(pC_->outString_, pC_->inString_, MAX_CONTROLLER_STRING_SIZE, &this->response_len);
      if(phyStatus){
         setIntegerParam(pC_->motorStatusProblem_, 1);
         callParamCallbacks();
         asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR, 
             "phytronAxis::moveVelocity: Stop axis for direction changing failed for axis: %d!\n", axisNo_);
      }
  }

  sprintf(pC_->outString_, "M%.1f==H", axisModuleNo_);
  phyStatus = pC_->sendPhytronCommand(pC_->outString_, pC_->inString_, MAX_CONTROLLER_STRING_SIZE, &this->response_len);
  if(phyStatus){
    setIntegerParam(pC_->motorStatusProblem_, 1);
    callParamCallbacks();
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
             "phytronAxis::moveVelocity: Reading axis standstill status failed for axis: %d!\n", axisNo_);
    return pC_->phyToAsyn(phyStatus);
  }
  
  if(pC_->inString_[0]=='E'){
    if(maxVelocity < 0) {
      sprintf(pC_->outString_, "M%.1fL-", axisModuleNo_);
    } else {
      sprintf(pC_->outString_, "M%.1fL+", axisModuleNo_);
    }
    phyStatus = pC_->sendPhytronCommand(pC_->outString_, pC_->inString_, MAX_CONTROLLER_STRING_SIZE, &this->response_len);
  }

  if(phyStatus){
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
              "phytronAxis::moveVelocity: Moving axis %d failed with error code: %d!\n", axisNo_, phyStatus);
    return pC_->phyToAsyn(phyStatus);
  }

  return asynSuccess;
}

/** Stop the motor
 * \param[in] acceleration  Deceleration to be used
 */
asynStatus phytronAxis::stop(double acceleration)
{
  phytronStatus phyStatus;

  phyStatus = setAcceleration(acceleration, stopMove);

  if(phyStatus){
  asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
            "phytronAxis::stop: Setting the acceleration for axis %d to %f failed with "
            "error code: %d!\n", axisNo_, acceleration, phyStatus);
  }
  
  if(pC_->profileExecuterIsRunning){
    pC_->profileExecuterAbort=1;
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR, "phytronAxis::stop: profile abort, axis %d\n", axisNo_);
    epicsEventSignal(pC_->profileControlEventId_);
  }

  sprintf(pC_->outString_, "M%.1fS", axisModuleNo_);
  phyStatus = pC_->sendPhytronCommand(pC_->outString_, pC_->inString_, MAX_CONTROLLER_STRING_SIZE, &this->response_len);
  if(phyStatus){
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
              "phytronAxis::stop: Stopping axis %d failed with error code: %d!\n", axisNo_, phyStatus);
    return pC_->phyToAsyn(phyStatus);
  }

  return asynSuccess;
}

//NOTE: Use this for step-slip check?
asynStatus phytronAxis::setEncoderRatio(double ratio){

  phytronStatus phyStatus;

  sprintf(pC_->outString_, "M%.1fP39=%f", axisModuleNo_, 1/ratio);
  phyStatus = pC_->sendPhytronCommand(pC_->outString_, pC_->inString_, MAX_CONTROLLER_STRING_SIZE, &this->response_len);
  if(phyStatus){
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
              "phytronAxis::setEncoderRatio: Failed for axis %d with status %d!\n", axisNo_, phyStatus);
    return pC_->phyToAsyn(phyStatus);
  }


  return asynSuccess;
}

//NOTE: Keep this for step-slip check?
asynStatus phytronAxis::setEncoderPosition(double position){


  return asynError;
}

/** Set the new position of the motor on the controller
 * \param[in] position  New absolute motor position
 */
asynStatus phytronAxis::setPosition(double position)
{
  phytronStatus phyStatus = phytronSuccess;

  sprintf(pC_->outString_, "M%.1fP20=%f", axisModuleNo_, position);
  phyStatus = pC_->sendPhytronCommand(pC_->outString_, pC_->inString_, MAX_CONTROLLER_STRING_SIZE, &this->response_len);
  if(phyStatus){
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
              "phytronAxis::setPosition: Setting position %f on axis %d failed with error code: %d!\n", position, axisNo_, phyStatus);
    return pC_->phyToAsyn(phyStatus);
  }

  return asynSuccess;
}

/* These are the functions for profile moves */
asynStatus phytronAxis::initializeProfile(size_t maxProfilePoints)
{
  return asynMotorAxis::initializeProfile(maxProfilePoints);
}

/** Function to define the motor positions for a profile move. 
  * This base class function converts the positions from user units
  * to controller units, using the profileMotorOffset_, profileMotorDirection_,
  * and profileMotorResolution_ parameters. 
  * \param[in] positions Array of profile positions for this axis in user units.
  * \param[in] numPoints The number of positions in the array.
  */
asynStatus phytronAxis::defineProfile(double *positions, size_t numPoints)
{ 
  size_t i;
  double resolution;
  double offset;
  int direction;
  double scale;
  int status=0;
  static const char *functionName = "defineProfile";
  
  asynPrint(pasynUser_, ASYN_TRACE_FLOW,
            "phytronAxis:%s: axis=%d, numPoints=%d, positions[0]=%f\n",
            functionName, axisNo_, (int)numPoints, positions[0]);

  if (numPoints > pC_->maxProfilePoints_){
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
            "phytronAxis::defineProfile: numPoints > maxProfilePoints !!! numPoints = %d maxProfilePoints = %d\n",
             numPoints,pC_->maxProfilePoints_);
    return asynError;
  }

  status |= pC_->getDoubleParam(axisNo_, pC_->motorRecEncoderResolution_, &resolution);
  status |= pC_->getDoubleParam(axisNo_, pC_->motorRecOffset_, &offset);
  status |= pC_->getIntegerParam(axisNo_, pC_->motorRecDirection_, &direction);

  asynPrint(pasynUser_, ASYN_TRACE_FLOW,
            "phytronAxis:%s: axis=%d, status=%d, offset=%f direction=%d, resolution=%f\n",
            functionName, axisNo_, status, offset, direction, resolution);

  if (status){
   asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
            "phytronAxis::defineProfile: get params status %d !\n",status);
   return asynError;
  }
  if (resolution == 0.0){
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
       "phytronAxis::defineProfile: resolution is 0.0!!!!\n");
    return asynError;
  }
  // Convert to controller units
  scale = 1.0/resolution;
  if (direction != 0) scale = -scale;
  for (i=0; i<numPoints; i++) {
    profilePositions_[i] = (positions[i] - offset)*scale;
  }
  asynPrint(pasynUser_, ASYN_TRACE_FLOW,
            "phytronAxis:%s: axis=%d, scale=%f, offset=%f positions[0]=%f, profilePositions_[0]=%f\n",
            functionName, axisNo_, scale, offset, positions[0], profilePositions_[0]);

  return asynSuccess;
}


/** Function to build a coordinated move of multiple axes. */
asynStatus phytronAxis::buildProfile()
{
  alglib::real_1d_array x, y;
  int numPoints = 0;
  int times_is_correct = 1;
  pC_->getIntegerParam(pC_->profileNumPoints_, &numPoints);
  
  if (numPoints > pC_->maxProfilePoints_){
     asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
            "phytronAxis::buildProfile: numPoints > maxProfilePoints !!! numPoints = %d maxProfilePoints = %d\n",
             numPoints,pC_->maxProfilePoints_);
     pC_->setIntegerParam(pC_->profileBuildStatus_,2); // Failure 
     pC_->setStringParam(pC_->profileBuildMessage_,  "Error: numPoints > maxProfilePoints"); 
     return asynError;
  }   
  if (numPoints < 5){
     asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
            "phytronAxis::buildProfile: numPoints < 5 !!! ",
            numPoints,pC_->maxProfilePoints_);
     pC_->setIntegerParam(pC_->profileBuildStatus_,2); // Failure 
     pC_->setStringParam(pC_->profileBuildMessage_, "Error: numPoints < 5"); 
     
     return asynError;
  }
  
  for(int i=0; i < numPoints-1; i++){
    if(pC_->profileTimes_[i] >= pC_->profileTimes_[i+1]){
      asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
            "phytronAxis::buildProfile: Times are not ordered! i=%d, prprofileTimes_[i]=%f, profileTimes_[i+1]=%f",
             i, pC_->profileTimes_[i], pC_->profileTimes_[i+1]);
      pC_->setIntegerParam(pC_->profileBuildStatus_,2); // Failure
      pC_->setStringParam(pC_->profileBuildMessage_, "Error: times are not ordered !!!");
      return asynError;
    }
  }
  
  x.setcontent(numPoints, &(pC_->profileTimes_[0]));
  y.setcontent(numPoints, &(profilePositions_[0]));
  printf("building spline\n"); 
  alglib::spline1dbuildakima(x, y, profileSpline);
  printf("building spline done\n"); 
  
  return asynSuccess;
}



/** Function to execute a coordinated move of multiple axes. */
asynStatus phytronAxis::executeProfile()
{
  profileErrorIntegral = 0; 
  return asynSuccess;
}



/** Function to abort a profile. */
asynStatus phytronAxis::abortProfile()
{
  static const char *functionName = "abortProfile";

  asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
              "phytronAxis::abortProfile:  Axis %d \n", axisNo_);

  return asynSuccess;
}



/** Function to readback the actual motor positions from a coordinated move of multiple axes.
  * This base class function converts the readbacks and following errors from controller units 
  * to user units and does callbacks on the arrays.
  * Caution: this function modifies the readbacks in place, so it must only be called
  * once per readback operation.
 */
asynStatus phytronAxis::readbackProfile()
{
  int i;
  double resolution;
  double offset;
  int direction;
  int numReadbacks;
  int status=0;
  //static const char *functionName = "readbackProfile";

  status |= pC_->getDoubleParam(axisNo_, pC_->motorRecEncoderResolution_, &resolution);
  status |= pC_->getDoubleParam(axisNo_, pC_->motorRecOffset_, &offset);
  status |= pC_->getIntegerParam(axisNo_, pC_->motorRecDirection_, &direction);
  status |= pC_->getIntegerParam(0, pC_->profileCurrentPoint_, &numReadbacks);
  if (status) return asynError;
  
  // Convert to user units
  if (direction != 0) resolution = -resolution;
  for (i=0; i<numReadbacks; i++) {
    profileReadbacks_[i] = profileReadbacks_[i] * resolution + offset;
    profileFollowingErrors_[i] = profileFollowingErrors_[i] * resolution;
  }
  status  = pC_->doCallbacksFloat64Array(profileReadbacks_,       numReadbacks, pC_->profileReadbacks_, axisNo_);
  status |= pC_->doCallbacksFloat64Array(profileFollowingErrors_, numReadbacks, pC_->profileFollowingErrors_, axisNo_);
  callParamCallbacks();
  return asynSuccess;
}

asynStatus  phytronAxis::setPGain(double pGain){
   this->pGain = pGain;
   return asynSuccess;
}

asynStatus  phytronAxis::setIGain(double iGain){
   this->iGain = iGain;
   profileErrorIntegral = 0;
   return asynSuccess;
}

asynStatus  phytronAxis::setDGain(double dGain){
   this->dGain = dGain;
   return asynSuccess;
}

asynStatus phytronAxis::axisProfileControl(){
  int currentPoint = 0;
  int status = 0;

  double motorReslution;
  double encoderResolution;
  double encoderOffset;
  int encoderDirection;

  double minVelo=1;
  double maxVelo=1000;
  double acceleration=10000;  
    
  int maxProfilePoints;
  int currentProfilePoint;
  
  epicsTimeStamp currentTimeStamp;
  double currentTime; // second number past 1990-01-01 00:00:00

  double profileAcc;
  double profileVelo;
  double profilePosition;
  double currentEncoderPosition;
  double currentError;
  double currentErrorDerivative;

  bool isMoving;
  
  double controlVelo;
 
  status  = pC_->getIntegerParam(pC_->profileCurrentPoint_,&currentProfilePoint);
  status |= pC_->getDoubleParam(axisNo_, pC_->motorRecResolution_,&motorReslution);
  status |= pC_->getDoubleParam(axisNo_, pC_->motorRecEncoderResolution_,&encoderResolution);
  status |= pC_->getDoubleParam(axisNo_, pC_->motorRecOffset_, &encoderOffset);
  status |= pC_->getIntegerParam(axisNo_, pC_->motorRecDirection_, &encoderDirection);
    
  status |= pC_->getDoubleParam(axisNo_, pC_->motorJogVelo_, &maxVelo);
  status |= pC_->getDoubleParam(axisNo_, pC_->motorJogAcc_, &acceleration);

  maxVelo = maxVelo / motorReslution;
  acceleration = acceleration / motorReslution;
  
  poll(&isMoving); 
  epicsTimeGetCurrent(&currentTimeStamp);
  currentTime = currentTimeStamp.secPastEpoch + double(currentTimeStamp.nsec)/1000000000.; 
  callParamCallbacks();
  
  pC_->getDoubleParam(axisNo_, pC_->motorEncoderPosition_, &currentEncoderPosition);
  
  alglib::spline1ddiff(profileSpline, currentTime, profilePosition, profileVelo, profileAcc);

  // Profile corrections
  if (this->profileCorrectionEnable != 0) {
    if( fabs(this->profileCorrection) > this->profileCorrectionMaximum && this->profileCorrectionMaximum > 0.0){
      if ( fabs(this->profileCorrection) > 0) {
        profilePosition += fabs(this->profileCorrection)/this->profileCorrection*this->profileCorrectionMaximum/encoderResolution;
      }
    } else {
      profilePosition += this->profileCorrection/encoderResolution;
    }
  }
 
  currentError = currentEncoderPosition - profilePosition;

  if ( (fabs( currentError )*encoderResolution > this->profileFollowingErrorAllowable) && ( this->profileFollowingErrorAllowable > double(0.0)) ) {
         this->profileFollowingErrorExceedAllowableValue = 1;
         asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,                                                        
						 "phytronAxis::axisProfileControl: profile following error exceed allowed value. fabs( currentError ) == %f ; profileFollowingErrorAllowable == %e ; %f \n", fabs( currentError ), this->profileFollowingErrorAllowable,this->profileFollowingErrorAllowable);
         //pC_->setIntegerParam(axisNo_, pC_->profileFollowingErrorExceedAllowableValue_, 1);
         pC_->setDoubleParam(axisNo_, pC_->profileCurrentFollowingError_, currentError*encoderResolution);
         callParamCallbacks();
         // TODO: several tryes
         //this->stop(1);
         return asynError;
  }

  profileReadbacks_[currentProfilePoint] = currentEncoderPosition;
  profileFollowingErrors_[currentProfilePoint] = currentError;

  profileErrorIntegral+=currentError;
  currentErrorDerivative = 0;

  // TODO: if point is not int the profile
  if (currentProfilePoint > 0){
    currentErrorDerivative = profileFollowingErrors_[currentProfilePoint]
                           - profileFollowingErrors_[currentProfilePoint-1];
  }

  controlVelo = profileVelo - pGain*currentError - iGain*profileErrorIntegral - dGain*currentErrorDerivative;
  controlVelo = controlVelo * encoderResolution / motorReslution;

  controlVelo = (controlVelo > maxVelo)? maxVelo : controlVelo;
  controlVelo = (controlVelo < -maxVelo)? -maxVelo : controlVelo; 

  pC_->setDoubleParam(axisNo_, pC_->profileControlVelo_, controlVelo*motorReslution);  
  pC_->setDoubleParam(axisNo_, pC_->profileCurrentFollowingError_, currentError*encoderResolution);
  callParamCallbacks();
  
  moveVelocity(minVelo, controlVelo, acceleration);
  
  return asynSuccess;
}

/** Polls the axis.
  * This function reads the motor position, the limit status, the home status, the moving status,
  * and the drive power-on status.
  * It calls setIntegerParam() and setDoubleParam() for each item that it polls,
  * and then calls callParamCallbacks() at the end.
  * \param[out] moving A flag that is set indicating that the axis is moving (true) or done (false).
  */
asynStatus phytronAxis::poll(bool *moving)
{
  int axisStatus;
  double position;
  double encoderPosition;
  double encoderRatio;
  int encoderResolutin;
  int encoderPeriod;
  
  double lastMotorPosition;

  phytronStatus phyStatus;
  
  pC_->getDoubleParam(axisNo_,pC_->motorPosition_, &lastMotorPosition);

  // Read the current motor position
  sprintf(pC_->outString_, "M%.1fP20R", axisModuleNo_);
  phyStatus = pC_->sendPhytronCommand(pC_->outString_, pC_->inString_, MAX_CONTROLLER_STRING_SIZE, &this->response_len);
  if(phyStatus){
    setIntegerParam(pC_->motorStatusProblem_, 1);
    callParamCallbacks();
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
             "phytronAxis::poll: Reading axis position failed for axis: %d!\n", axisNo_);
    return pC_->phyToAsyn(phyStatus);
  }
  position = atof(pC_->inString_);
  setDoubleParam(pC_->motorPosition_, position);

  if( NINT(lastMotorPosition) != NINT(position) ) {
    axisMovingDirection = ((NINT(position) - NINT(lastMotorPosition) < 0)? -1: 1);
  }
  
  // Read the moving status of this motor
  sprintf(pC_->outString_, "M%.1f==H", axisModuleNo_);
  phyStatus = pC_->sendPhytronCommand(pC_->outString_, pC_->inString_, MAX_CONTROLLER_STRING_SIZE, &this->response_len);
  if(phyStatus){
    setIntegerParam(pC_->motorStatusProblem_, 1);
    callParamCallbacks();
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
              "phytronAxis::poll: Reading axis moving status failed for axis: %d!\n", axisNo_);
    return pC_->phyToAsyn(phyStatus);
  }
  *moving = (pC_->inString_[0] == 'E') ? 0:1;
  setIntegerParam(pC_->motorStatusDone_, !*moving);

  sprintf(pC_->outString_, "M%.1fSE", axisModuleNo_);
  phyStatus = pC_->sendPhytronCommand(pC_->outString_, pC_->inString_, MAX_CONTROLLER_STRING_SIZE, &this->response_len);
  if(phyStatus){
    setIntegerParam(pC_->motorStatusProblem_, 1);
    callParamCallbacks();
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
             "phytronAxis::poll: Reading axis status failed for axis: %d!\n", axisNo_);
    return pC_->phyToAsyn(phyStatus);
  }
  axisStatus = atoi(pC_->inString_);
  setIntegerParam(pC_->motorStatusHighLimit_, (axisStatus & 0x10)/0x10);
  setIntegerParam(pC_->motorStatusLowLimit_, (axisStatus & 0x20)/0x20);
  setIntegerParam(pC_->motorStatusAtHome_, (axisStatus & 0x40)/0x40);

  setIntegerParam(pC_->motorStatusHomed_, (axisStatus & 0x08)/0x08);
  setIntegerParam(pC_->motorStatusHome_, (axisStatus & 0x08)/0x08);

  setIntegerParam(pC_->motorStatusSlip_, (axisStatus & 0x4000)/0x4000);

  //Update the axis status record ($(P)$(M)_STATUS)
  setIntegerParam(pC_->axisStatus_, axisStatus);

  // Read the current encoder value
  sprintf(pC_->outString_, "M%.1fP22R", axisModuleNo_);
  phyStatus = pC_->sendPhytronCommand(pC_->outString_, pC_->inString_, MAX_CONTROLLER_STRING_SIZE, &this->response_len);
  if(phyStatus){
    setIntegerParam(pC_->motorStatusProblem_, 1);
    callParamCallbacks();
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
             "phytronAxis::poll: Reading encoder value failed for axis: %d!\n", axisNo_);
    return pC_->phyToAsyn(phyStatus);
  }
  encoderPosition = atof(pC_->inString_);  
  if (encoderPosition == 0) {
    setIntegerParam(pC_->motorStatusProblem_, 1);
    callParamCallbacks();
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
                "phytronAxis::poll: Raw encoder value is 0 for axis %d! The value is ignored!\n", axisNo_);
    return asynError;
  }

  pC_->getDoubleParam(axisNo_, pC_->motorEncoderRatio_, &encoderRatio);
  encoderPosition = encoderPosition*encoderRatio;
  
  // Read the current encoder resolution
  sprintf(pC_->outString_, "M%.1fP35R", axisModuleNo_);
  phyStatus = pC_->sendPhytronCommand(pC_->outString_, pC_->inString_, MAX_CONTROLLER_STRING_SIZE, &this->response_len);
  if(phyStatus){
    setIntegerParam(pC_->motorStatusProblem_, 1);
    callParamCallbacks();
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
    		"phytronAxis::poll: Reading encoder resolution failed for axis: %d!\n", axisNo_);
    return pC_->phyToAsyn(phyStatus);
  }
  encoderResolutin = atof(pC_->inString_);
  encoderPeriod = 1 << encoderResolutin;

  if( fabs(encoderPosition - lastPoolEncoderPosition) > encoderPeriod*0.5 ){
    encoderRevolutionCount = encoderRevolutionCount + ((encoderPosition - lastPoolEncoderPosition < 0)? 1:-1);
  }
  lastPoolEncoderPosition = encoderPosition;
  
  // apply revolution count
  if(this->encoderRevolutionCountEnabled != 0){
    encoderPosition = encoderPosition + encoderRevolutionCount*encoderPeriod; 
  }
  
   setIntegerParam(pC_->encoderRevolutionCount_,encoderRevolutionCount);   
  
  /*
   * The encoder position returned by the controller is weighted by the controller
   * resolutio. To get absolute encoder position, the received position must be
   * multiplied by the encoder resolution.
   */
  setDoubleParam(pC_->motorEncoderPosition_, encoderPosition);

  //No problem occurred
  setIntegerParam(pC_->motorStatusProblem_, 0);

  callParamCallbacks();
  return asynSuccess;
}

/** Parameters for iocsh phytron axis registration*/
static const iocshArg phytronCreateAxisArg0 = {"Controller Name", iocshArgString};
static const iocshArg phytronCreateAxisArg1 = {"Module index", iocshArgInt};
static const iocshArg phytronCreateAxisArg2 = {"Axis index", iocshArgInt};
static const iocshArg* const phytronCreateAxisArgs[] = {&phytronCreateAxisArg0,
                                                      &phytronCreateAxisArg1,
                                                      &phytronCreateAxisArg2};

/** Parameters for iocsh phytron controller registration */
static const iocshArg phytronCreateControllerArg0 = {"Port name", iocshArgString};
static const iocshArg phytronCreateControllerArg1 = {"PhytronAxis port name", iocshArgString};
static const iocshArg phytronCreateControllerArg2 = {"Moving poll period (ms)", iocshArgInt};
static const iocshArg phytronCreateControllerArg3 = {"Idle poll period (ms)", iocshArgInt};
static const iocshArg phytronCreateControllerArg4 = {"Idle poll period (ms)", iocshArgDouble};
static const iocshArg phytronCreateControllerArg5 = {"max profile points number", iocshArgInt};
static const iocshArg * const phytronCreateControllerArgs[] = {&phytronCreateControllerArg0,
                                                             &phytronCreateControllerArg1,
                                                             &phytronCreateControllerArg2,
                                                             &phytronCreateControllerArg3,
                                                             &phytronCreateControllerArg4,
                                                             &phytronCreateControllerArg5};

static const iocshFuncDef phytronCreateAxisDef = {"phytronCreateAxis", 3, phytronCreateAxisArgs};
static const iocshFuncDef phytronCreateControllerDef = {"phytronCreateController", 6, phytronCreateControllerArgs};

static void phytronCreateControllerCallFunc(const iocshArgBuf *args)
{
  phytronCreateController(args[0].sval, args[1].sval, args[2].ival, args[3].ival, args[4].dval, args[5].ival);
}

static void phytronCreateAxisCallFunc(const iocshArgBuf *args)
{
  phytronCreateAxis(args[0].sval, args[1].ival, args[2].ival);
}

static void phytronRegister(void)
{
  iocshRegister(&phytronCreateControllerDef, phytronCreateControllerCallFunc);
  iocshRegister(&phytronCreateAxisDef, phytronCreateAxisCallFunc);
}

extern "C" {
epicsExportRegistrar(phytronRegister);
}
