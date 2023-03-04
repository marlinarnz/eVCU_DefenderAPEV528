#include "DebugLogger.h"


/** Start operation.
 *  Start threads, subscribe Parameters
 */
void DebugLogger::begin()
{
  // Start the threads
  this->startTasks(4000);

  // Register for parameter changes
  this->registerForValueChanged(104);
  this->registerForValueChanged(105);
  this->registerForValueChanged(107);
  this->registerForValueChanged(108);
  this->registerForValueChanged(112);
  this->registerForValueChanged(113);
  this->registerForValueChanged(114);
  this->registerForValueChanged(115);
  this->registerForValueChanged(201);
  this->registerForValueChanged(204);
  this->registerForValueChanged(205);
  this->registerForValueChanged(221);
  this->registerForValueChanged(227);
  this->registerForValueChanged(228);
  this->registerForValueChanged(229);
  this->registerForValueChanged(230);
  this->registerForValueChanged(231);
}


/** End operation.
 */
void DebugLogger::shutdown()
{
  this->unregisterForValueChanged(104);
  this->unregisterForValueChanged(105);
  this->unregisterForValueChanged(107);
  this->unregisterForValueChanged(108);
  this->unregisterForValueChanged(112);
  this->unregisterForValueChanged(113);
  this->unregisterForValueChanged(114);
  this->unregisterForValueChanged(115);
  this->unregisterForValueChanged(201);
  this->unregisterForValueChanged(204);
  this->unregisterForValueChanged(205);
  this->unregisterForValueChanged(221);
  this->unregisterForValueChanged(227);
  this->unregisterForValueChanged(228);
  this->unregisterForValueChanged(229);
  this->unregisterForValueChanged(230);
  this->unregisterForValueChanged(231);
}


/** React to Parameter changes
 */
void DebugLogger::onValueChanged(Parameter* pParamWithNewValue)
{
  if(pParamWithNewValue) {
    switch(pParamWithNewValue->getId()) {
      case 104:
        PRINT("Vehicle authentication: " + String(authenticationValid.getVal()))
        break;
      case 105:
        PRINT("Vehicle readiness: " + String(vehicleReady.getVal()))
        break;
      case 107:
        PRINT("Main contactor closed: " + String(mainRelayConnected.getVal()))
        break;
      case 108:
        PRINT("Gear lever position (0-4: Default, R, N, D, P): " + String(gearLeverPosition.getVal()))
        break;
      case 112:
        PRINT("Operation command (0-2: Standby, Drive, Regen): " + String(vcuMotorOperationMode.getVal()))
        break;
      case 113:
        PRINT("VCU warning level (0-3): " + String(vehicleWarningLevel.getVal()))
        break;
      case 114:
        PRINT("Key position (off, aux, on, cranck): " + String(keyPosition.getVal()))
        break;
      case 115:
        PRINT("Aux. contactor closed: " + String(auxiliaryRelayConnected.getVal()))
        break;
      case 201:
        PRINT("Pack voltage at MCU: " + String(motorDCVoltage.getVal()))
        break;
      case 204:
        PRINT("Motor temperature: " + String(motorBodyTemp.getVal()))
        break;
      case 205:
        PRINT("Controller temperature: " + String(motorControllerTemp.getVal()))
        break;
      case 221:
        PRINT("MCU warning level (0-3): " + String(motorWarningLevel.getVal()))
        break;
      case 227:
        PRINT("Maximum motor torque: " + String(motorMaxTorque.getVal()))
        break;
      case 228:
        PRINT("Maximum motor negative torque: " + String(motorMaxNegativeTorque.getVal()))
        break;
      case 229:
        PRINT("Motor direction (0-3: standby, forward, backward, error): " + String(motorDirection.getVal()))
        break;
      case 230:
        PRINT("Motor state (0-4: standby, precharge, ready, running, shutdown): " + String(motorState.getVal()))
        break;
      case 231:
        PRINT("Motor operation mode (0-2: standby, torque, speed): " + String(motorMode.getVal()))
        break;
      default:
        break;
    }
  }
}