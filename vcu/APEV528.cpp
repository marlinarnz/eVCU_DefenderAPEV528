#include "APEV528.h"


/** Constructor.
 */
APEV528::APEV528(VehicleController* vc, uint8_t enablePin)
    : Device(vc), m_waitingForContactor(false), m_enablePin(enablePin)
{
  // Set the MCU relay pin
  pinMode(enablePin, OUTPUT);
}


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

  // Start the MCU
  //digitalWrite(m_enablePin, LOW);   // TODO: Set LOW to enable relay
}


/** End operation.
 */
void APEV528::shutdown()
{
  digitalWrite(m_enablePin, HIGH);
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
        // Switch motor off, if KL15 not on
        // TODO: KL15 is key pos. 1
        if (keyPosition.getVal() < 2) {
          this->setIntegerValue(&vcuMotorOperationMode, 0);
          digitalWrite(m_enablePin, HIGH); // shutdown MCU
        }
        // start the MCU with KL15
        else if (keyPosition.getVal() == 2) {
          digitalWrite(m_enablePin, LOW);
        }
        // or start the motor
        else if (keyPosition.getVal() == 3) {
          setMotorRunning();
        }
        break;
      case 106:
        // Switch motor to regen, if brake pressed
        if (brakePositionMCU.getVal() > BRAKE_THRESHOLD_PERCENT) {
          if (vcuMotorOperationMode.getVal() == 1) {
            setMotorRegen();
          }
        }
        // or back to running
        else if (brakePositionMCU.getVal() == 0) {
          if (vcuMotorOperationMode.getVal() == 2) {
            setMotorRunning();
          }
        }
        break;
      case 108:
        // Switch motor off, if not in any gear
        if (gearLeverPosition.getVal() == 0) {
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
          //setMotorRunning(); // TODO
        }
        break;
      case 120:
        // Switch motor to regen, if brake is pressed and regen just switched on
        if (switchRecuOn.getVal()
            && vcuMotorOperationMode.getVal() == 1
            && brakePositionMCU.getVal() > BRAKE_THRESHOLD_PERCENT) {
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
 *  BMS status, and stall status. Starts only if main contactor is closed.
 *  Otherwise, sets the waiting status and starts when it is closed.
 */
void APEV528::setMotorRunning()
{
  if (vehicleReady.getVal()
      && vehicleWarningLevel.getVal() < 3
      && motorWarningLevel.getVal() < 3
      && !motorStall.getVal()
      && !batteryDischargeInhibit.getVal())
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
      && switchRecuOn.getVal()
      && !batteryChargeInhibit.getVal())
  {
    this->setIntegerValue(&vcuMotorOperationMode, 2);
  }
}
