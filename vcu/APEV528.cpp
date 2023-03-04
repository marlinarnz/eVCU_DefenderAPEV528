#include "APEV528.h"


/** Start operation.
 *  Start threads, subscribe Parameters, set outgoing messages
 */
void APEV528::begin()
{
  // Start the threads
  this->startTasks(4000);

  // Register for parameter changes
  this->registerForValueChanged(104);
  this->registerForValueChanged(105);
  this->registerForValueChanged(114);
  this->registerForValueChanged(106);
  this->registerForValueChanged(108);
  this->registerForValueChanged(107);
  this->registerForValueChanged(120);
  this->registerForValueChanged(221);
  this->registerForValueChanged(113);
  this->registerForValueChanged(223);

  // Initialise MCU Parameters
  this->setIntegerValue(&vcuMotorOperationMode, 0);
}


/** End operation.
 */
void APEV528::shutdown()
{
  this->unregisterForValueChanged(104);
  this->unregisterForValueChanged(105);
  this->unregisterForValueChanged(114);
  this->unregisterForValueChanged(106);
  this->unregisterForValueChanged(108);
  this->unregisterForValueChanged(107);
  this->unregisterForValueChanged(120);
  this->unregisterForValueChanged(113);
  this->unregisterForValueChanged(221);
  this->unregisterForValueChanged(223);
}


/** React to Parameter changes
 */
void APEV528::onValueChanged(Parameter* pParamWithNewValue)
{
  if(pParamWithNewValue) {
    switch(pParamWithNewValue->getId()) {
      case 104:
        // Switch motor off if not authenticated
        if (!authenticationValid.getVal()) {
          this->setIntegerValue(&vcuMotorOperationMode, 0);
        }
        break;
      case 105:
        // Switch motor off if not ready
        if (!vehicleReady.getVal()) {
          this->setIntegerValue(&vcuMotorOperationMode, 0);
        }
        break;
      case 114:
        // Switch motor off, if not on
        if (keyPosition.getVal() < 2) {
          this->setIntegerValue(&vcuMotorOperationMode, 0);
        }
        // or start the motor
        else if (keyPosition.getVal() == 3) {
          setMotorRunning();
        }
        break;
      case 106:
        // Switch motor to regen, if brake pressed
        if (brakePositionMCU.getVal() > BRAKE_THRESHOLD_PERCENT) {
          if (vcuMotorOperationMode.getVal() != 2) {
            setMotorRegen();
          }
        }
        // or back to running
        else if (brakePositionMCU.getVal() == 0) {
          setMotorRunning();
        }
        break;
      case 108:
        // Switch motor off, if not in forward or backward gear
        if (gearLeverPosition.getVal() != 1 && gearLeverPosition.getVal() != 3) {
          this->setIntegerValue(&vcuMotorOperationMode, 0);
        }
        break;
      case 107:
        // Switch motor off if main contactor not closed
        if (!mainRelayConnected.getVal()) {
          this->setIntegerValue(&vcuMotorOperationMode, 0);
        }
        // or switch it on, if it was waiting for the contactor
        else if (m_waitingForContactor) {
          m_waitingForContactor = false;
          setMotorRunning();
        }
        break;
      case 120:
        // Switch motor to regen, if brake is pressed and regen just switched on
        if (switchRecuOn.getVal() && brakePositionMCU.getVal() > BRAKE_THRESHOLD_PERCENT) {
          setMotorRegen();
        }
        // or switch regen off, if it was activated
        else if (!switchRecuOn.getVal() && vcuMotorOperationMode.getVal() == 2) {
          setMotorRunning();
        }
        break;
      case 113:
        // Switch motor off if high VCU warning level
        if (vehicleWarningLevel.getVal() == 3) {
          this->setIntegerValue(&vcuMotorOperationMode, 0);
        }
        break;
      case 221:
        // Switch motor off if high MCU warning level
        if (motorWarningLevel.getVal() == 3) {
          this->setIntegerValue(&vcuMotorOperationMode, 0);
        }
        break;
      case 223:
        // Switch motor off if stalled
        if (motorStall.getVal()) {
          this->setIntegerValue(&vcuMotorOperationMode, 0);
        }
        break;
      default:
        break;
    }
  }
}


/** Set the motor to run, if reasonable.
 *  Based on vehicle readiness, VCU and MCU warning levels below 3,
 *  and stall status. Stars only if main contactor is closed.
 *  Otherwise, sets the waiting status and starts when it is closed.
 */
void APEV528::setMotorRunning()
{
  if (vehicleReady.getVal()
      && vehicleWarningLevel.getVal() < 3
      && motorWarningLevel.getVal() < 3
      && !motorStall.getVal())
  {
    if (mainRelayConnected.getVal()) {
      this->setIntegerValue(&vcuMotorOperationMode, 1);
    }
    else {
      m_waitingForContactor = true;
    }
  }
}


/** Set the motor to regenerative breaking, if reasonable.
 *  Only if the motor was running before and regen is not deactivated.
 */
void APEV528::setMotorRegen()
{
  if (vcuMotorOperationMode.getVal() == 1
      && switchRecuOn.getVal())
  {
    this->setIntegerValue(&vcuMotorOperationMode, 2);
  }
}
