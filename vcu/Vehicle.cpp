#include "Vehicle.h"


/** Start operation.
 *  Start threads, subscribe Parameters, set outgoing messages
 */
void Vehicle::begin()
{
  // Start the threads
  this->startTasks(4000);

  // Register for parameter changes
  this->registerForValueChanged(104);
  this->registerForValueChanged(113);
  this->registerForValueChanged(114);
  this->registerForValueChanged(121);
  this->registerForValueChanged(122);
  this->registerForValueChanged(204);
  this->registerForValueChanged(205);
  this->registerForValueChanged(206);
  this->registerForValueChanged(207);
  this->registerForValueChanged(208);
  this->registerForValueChanged(209);
  this->registerForValueChanged(210);
  this->registerForValueChanged(211);
  this->registerForValueChanged(212);
  this->registerForValueChanged(213);
  this->registerForValueChanged(214);
  this->registerForValueChanged(215);
  this->registerForValueChanged(216);
  this->registerForValueChanged(217);
  this->registerForValueChanged(218);
  this->registerForValueChanged(219);
  this->registerForValueChanged(220);
  this->registerForValueChanged(221);
  this->registerForValueChanged(222);

  // Initialise vehicle Parameters
  this->setIntegerValue(&vehicleWarningLevel, 0);
  this->setBooleanValue(&authenticationValid, true);
}


/** End operation.
 */
void Vehicle::shutdown()
{
  this->unregisterForValueChanged(104);
  this->unregisterForValueChanged(113);
  this->unregisterForValueChanged(114);
  this->unregisterForValueChanged(121);
  this->unregisterForValueChanged(122);
  this->unregisterForValueChanged(204);
  this->unregisterForValueChanged(205);
  this->unregisterForValueChanged(206);
  this->unregisterForValueChanged(207);
  this->unregisterForValueChanged(208);
  this->unregisterForValueChanged(209);
  this->unregisterForValueChanged(210);
  this->unregisterForValueChanged(211);
  this->unregisterForValueChanged(212);
  this->unregisterForValueChanged(213);
  this->unregisterForValueChanged(214);
  this->unregisterForValueChanged(215);
  this->unregisterForValueChanged(216);
  this->unregisterForValueChanged(217);
  this->unregisterForValueChanged(218);
  this->unregisterForValueChanged(219);
  this->unregisterForValueChanged(220);
  this->unregisterForValueChanged(221);
  this->unregisterForValueChanged(222);
}


/** React to Parameter changes
 */
void Vehicle::onValueChanged(Parameter* pParamWithNewValue)
{
  if(pParamWithNewValue) {
    switch(pParamWithNewValue->getId()) {
      case 104:
        updateVehicleReadiness();
        break;
      case 113:
        updateVehicleReadiness();
        break;
      case 114:
        updateVehicleReadiness();
        break;
      case 121:
        updateWarningLevel();
        break;
      case 122:
        updateWarningLevel();
        break;
      case 204:
        updateWarningLevel();
        break;
      case 205:
        updateWarningLevel();
        break;
      case 206:
        updateWarningLevel();
        break;
      case 207:
        updateWarningLevel();
        break;
      case 208:
        updateWarningLevel();
        break;
      case 209:
        updateWarningLevel();
        break;
      case 210:
        updateWarningLevel();
        break;
      case 211:
        updateWarningLevel();
        break;
      case 212:
        updateWarningLevel();
        break;
      case 213:
        updateWarningLevel();
        break;
      case 214:
        updateWarningLevel();
        break;
      case 215:
        updateWarningLevel();
        break;
      case 216:
        updateWarningLevel();
        break;
      case 217:
        updateWarningLevel();
        break;
      case 218:
        updateWarningLevel();
        break;
      case 219:
        updateWarningLevel();
        break;
      case 220:
        updateWarningLevel();
        break;
      case 221:
        updateWarningLevel();
        break;
      case 222:
        updateWarningLevel();
        break;
      default:
        break;
    }
  }
}


/** Set the vehicle readiness for MCU.
 *  Based on authentication status, warning level, key position.
 */
void Vehicle::updateVehicleReadiness()
{
  if (authenticationValid.getVal()
      && vehicleWarningLevel.getVal() < 3
      && keyPosition.getVal() >= 1)
  {
    this->setBooleanValue(&vehicleReady, true);
  } else {
    this->setBooleanValue(&vehicleReady, false);
  }
}


/** Set the vehicle warning level.
 *  Based on the consideration of all relevant factors.
 *  The first warning level denotes a harmless failure or warning,
 *  the second an event that causes derating, and the third
 *  causes the vehicle to stop because safe operation is not
 *  guaranteed anymore.
 */
void Vehicle::updateWarningLevel()
{
  if (motorBodyTemp.getVal() > WARN_MOTOR_OVERTEMP_C
      || motorControllerTemp.getVal() > WARN_MOTOR_OVERTEMP_C
      || motorPhaseCurrentFault.getVal()
      || motorRotationTransformFault.getVal()
      || motorPhaseCurrentSensorFault.getVal()
      || motorOverSpeedFault.getVal()
      || motorDCOverVoltageFault.getVal()
      || motorBodyUnderTempFault.getVal()
      || motorSystemFault.getVal()
      || motorTempSensorFault.getVal()
      || motorBodyTempSensorFault.getVal()
      || motorDCVoltageSensorFault.getVal()
      || motorDCUnderVoltageWarning.getVal()
      || motorLVUnderVoltageWarning.getVal()
      || motorOpenPhaseFault.getVal()
      || motorWarningLevel.getVal() == 1
     ) // Conditions for warning level "Warning"
  { this->setIntegerValue(&vehicleWarningLevel, 1); }
  else if (motorBodyTemp.getVal() > DERATE_MOTOR_OVERTEMP_C
           || motorControllerTemp.getVal() > DERATE_MOTOR_OVERTEMP_C
           || motorBodyOverTempFault.getVal()
           || motorWarningLevel.getVal() == 2
           || auxRelayFault.getVal()
          ) // Conditions for warning level "Derating"
  { this->setIntegerValue(&vehicleWarningLevel, 2); }
  else if (motorDCOverCurrentFault.getVal()
           || motorOverTempFault.getVal()
           || motorWarningLevel.getVal() == 3
           || mainRelayFault.getVal()
          ) // Conditions for warning level "Emergency stop"
  { this->setIntegerValue(&vehicleWarningLevel, 3); }
  else // All good
  { this->setIntegerValue(&vehicleWarningLevel, 0); }
}
