/* ======================================================================
 * An electric vehicle conversion with a Alpha Electrics APEV528 motor
 * and inverter and Tesla Battery packs.
*/

#include "CANManager.h"

// Instantiate the VehicleController and the vehicle's Parameters
VehicleController vc;
ParameterInt    throttlePosition(101);             // 0-100%
ParameterBool   authentication(104);
ParameterBool   vehicleReady(105);
ParameterInt    breakPosition(106);
ParameterBool   mainRelayConnected(107);
ParameterInt    gearLeverPosition(108);            // see CAN bus Excel spec
ParameterInt    vcuMotorOperationMode(112);        // see CAN bus Excel spec
ParameterInt    vehicleWarningLevel(113);          // 0 (normal) to 3 (high)
ParameterInt    keyPosition(114);                  // see CAN bus Excel spec
ParameterBool   auxiliaryRelayConnected(115);
// The MCU's Parameters
// Message ID 0x107
ParameterDouble motorDCVoltage(201);
ParameterDouble motorDCCurrent(202);
ParameterDouble motorPhaseCurrent(203);
// Message ID 0x106
ParameterInt    motorBodyTemp(204);
ParameterInt    motorControllerTemp(205);
ParameterBool   motorDCOverCurrentFault(206);
ParameterBool   motorPhaseCurrentFault(207);
ParameterBool   motorOverTempFault(208);
ParameterBool   motorRotationTransformFault(209);
ParameterBool   motorPhaseCurrentSensorFault(210);
ParameterBool   motorOverSpeedFault(211);
ParameterBool   motorBodyOverTempFault(212);
ParameterBool   motorDCOverVoltageFault(213);
ParameterBool   motorBodyUnderTempFault(214);
ParameterBool   motorSystemFault(215);
ParameterBool   motorTempSensorFault(216);
ParameterBool   motorBodyTempSensorFault(217);
ParameterBool   motorDCVoltageSensorFault(218);
ParameterBool   motorDCUnderVoltageWarning(219);
ParameterBool   motorLVUnderVoltageWarning(220);
Parameterint    motorWarningLevel(221);
ParameterBool   motorOpenPhaseFault(222);
ParameterBool   motorStall(223);
ParameterInt    motor106RollingCounter(224);
// Message ID 0x105
ParameterDouble motorSpeed(225);
ParameterDouble motorTorque(226);
ParameterDouble motorMaxTorque(227);
ParameterDouble motorMaxNegativeTorque(228);
ParameterInt    motorDirection(229);
ParameterInt    motorState(230);
ParameterInt    motorMode(231);
ParameterInt    motor105RollingCounter(232);


// Instantiate Devices
CANManager can(&vc);


void setup() {
  // Preparations
  Serial.begin(9600); // Start the Serial monitor
  
  // Register the parameters
  vc.registerParameter(&current);
  
  // Start the devices

  while(1) {
    vTaskDelay(100); // Run indefinitely
  }
}


void loop()
{
  vTaskDelete(NULL); // We don't need that loop
}
