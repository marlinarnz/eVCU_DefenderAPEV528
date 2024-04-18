#include "DebugLogger.h"


/** Start operation.
 *  Start threads, subscribe Parameters
 */
void DebugLogger::begin()
{
  // Start the threads
  this->startTasks(24000);

  // Register for parameter changes
  this->registerForValueChanged(101);
  this->registerForValueChanged(104);
  this->registerForValueChanged(105);
  this->registerForValueChanged(106);
  this->registerForValueChanged(107);
  this->registerForValueChanged(108);
  this->registerForValueChanged(112);
  this->registerForValueChanged(113);
  this->registerForValueChanged(114);
  this->registerForValueChanged(115);
  this->registerForValueChanged(120);
  this->registerForValueChanged(201);
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
  this->registerForValueChanged(223);
  this->registerForValueChanged(225);
  this->registerForValueChanged(227);
  this->registerForValueChanged(228);
  this->registerForValueChanged(229);
  this->registerForValueChanged(230);
  this->registerForValueChanged(231);
  this->registerForValueChanged(302);
  this->registerForValueChanged(303);
  this->registerForValueChanged(304);
  this->registerForValueChanged(306);
  this->registerForValueChanged(308);
  this->registerForValueChanged(309);
  this->registerForValueChanged(310);
}


/** End operation.
 */
void DebugLogger::shutdown()
{
  this->unregisterForValueChanged(101);
  this->unregisterForValueChanged(104);
  this->unregisterForValueChanged(105);
  this->unregisterForValueChanged(106);
  this->unregisterForValueChanged(107);
  this->unregisterForValueChanged(108);
  this->unregisterForValueChanged(112);
  this->unregisterForValueChanged(113);
  this->unregisterForValueChanged(114);
  this->unregisterForValueChanged(115);
  this->unregisterForValueChanged(120);
  this->unregisterForValueChanged(201);
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
  this->unregisterForValueChanged(223);
  this->unregisterForValueChanged(225);
  this->unregisterForValueChanged(227);
  this->unregisterForValueChanged(228);
  this->unregisterForValueChanged(229);
  this->unregisterForValueChanged(230);
  this->unregisterForValueChanged(231);
  this->unregisterForValueChanged(302);
  this->unregisterForValueChanged(303);
  this->unregisterForValueChanged(304);
  this->unregisterForValueChanged(306);
  this->unregisterForValueChanged(308);
  this->unregisterForValueChanged(309);
  this->unregisterForValueChanged(310);
}


/** React to Parameter changes
 */
void DebugLogger::onValueChanged(Parameter* pParamWithNewValue)
{
  if(pParamWithNewValue) {
    switch(pParamWithNewValue->getId()) {
      case 101:
        if (throttlePosition.getVal() > 50 && !m_throttleStatePressed) {
          PRINT("Throttle pressed halve way")
          m_throttleStatePressed = true;
        }
        else if (throttlePosition.getVal() < 30 && m_throttleStatePressed) {
          PRINT("Throttle released")
          m_throttleStatePressed = false;
        }
        break;
      case 104:
        PRINT("Vehicle authentication: " + String(authenticationValid.getVal()))
        break;
      case 105:
        PRINT("Vehicle readiness: " + String(vehicleReady.getVal()))
        break;
      case 106:
        if (brakePositionMCU.getVal() > BRAKE_THRESHOLD_PERCENT && !m_brakeStatePressed) {
          PRINT("Brake pressed")
          m_brakeStatePressed = true;
        }
        else if (brakePositionMCU.getVal() < BRAKE_THRESHOLD_PERCENT && m_brakeStatePressed) {
          PRINT("Brake released")
          m_brakeStatePressed = false;
        }
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
      case 120:
        PRINT("Regenerative breaking active: " + String(switchRecuOn.getVal()))
        break;
      case 201:
        //PRINT("Pack voltage at MCU: " + String(motorDCVoltage.getVal()))
        break;
      case 204:
        PRINT("Motor temperature: " + String(motorBodyTemp.getVal()))
        break;
      case 205:
        PRINT("Controller temperature: " + String(motorControllerTemp.getVal()))
        break;
      case 206:
        PRINT("motorDCOverCurrentFault: " + String(motorDCOverCurrentFault.getVal()))
        break;
      case 207:
        PRINT("motorPhaseCurrentFault: " + String(motorPhaseCurrentFault.getVal()))
        break;
      case 208:
        PRINT("motorOverTempFault: " + String(motorOverTempFault.getVal()))
        break;
      case 209:
        PRINT("motorRotationTransformFault: " + String(motorRotationTransformFault.getVal()))
        break;
      case 210:
        PRINT("motorPhaseCurrentSensorFault: " + String(motorPhaseCurrentSensorFault.getVal()))
        break;
      case 211:
        PRINT("motorOverSpeedFault: " + String(motorOverSpeedFault.getVal()))
        break;
      case 212:
        PRINT("motorBodyOverTempFault: " + String(motorBodyOverTempFault.getVal()))
        break;
      case 213:
        PRINT("motorDCOverVoltageFault: " + String(motorDCOverVoltageFault.getVal()))
        break;
      case 214:
        PRINT("motorBodyUnderTempFault: " + String(motorBodyUnderTempFault.getVal()))
        break;
      case 215:
        PRINT("motorSystemFault: " + String(motorSystemFault.getVal()))
        break;
      case 216:
        PRINT("motorTempSensorFault: " + String(motorTempSensorFault.getVal()))
        break;
      case 217:
        PRINT("motorBodyTempSensorFault: " + String(motorBodyTempSensorFault.getVal()))
        break;
      case 218:
        PRINT("motorDCVoltageSensorFault: " + String(motorDCVoltageSensorFault.getVal()))
        break;
      case 219:
        PRINT("motorDCUnderVoltageWarning: " + String(motorDCUnderVoltageWarning.getVal()))
        break;
      case 220:
        PRINT("motorLVUnderVoltageWarning: " + String(motorLVUnderVoltageWarning.getVal()))
        break;
      case 221:
        PRINT("MCU warning level (0-3): " + String(motorWarningLevel.getVal()))
        break;
      case 222:
        PRINT("motorOpenPhaseFault: " + String(motorOpenPhaseFault.getVal()))
        break;
      case 223:
        PRINT("motorStall: " + String(motorStall.getVal()))
        break;
      case 225:
        if (motorSpeed.getVal() > 10 && !m_motorRotating) {
          m_motorRotating = true;
          PRINT("Motor is spinning")
        } else if (motorSpeed.getVal() == 0 && m_motorRotating) {
          m_motorRotating = false;
        }
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
      case 302:
        PRINT("BMS avg. temperature: " + String(batteryAverageTemp.getVal()))
        break;
      case 303:
        PRINT("BMS max. temperature: " + String(batteryHighestTemp.getVal()))
        break;
      case 304:
        PRINT("BMS min. temperature: " + String(batteryLowestTemp.getVal()))
        break;
      case 306:
        PRINT("BMS state (0-5: INIT, STANDBY, PRE_CHARGE, CHARGING, POST_CHARGE, RUN): " + String(batteryState.getVal()))
        break;
      case 308:
        PRINT("BMS is faulted: " + String(batteryIsStickyFaulted.getVal()))
        break;
      case 309:
        PRINT("BMS charge inhibit: " + String(batteryChargeInhibit.getVal()))
        break;
      case 310:
        PRINT("BMS discharge inhibit: " + String(batteryDischargeInhibit.getVal()))
        break;
      default:
        break;
    }
  }
}
