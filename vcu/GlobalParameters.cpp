#include "GlobalParameters.h"


// Define the vehicle controller and the vehicle's Parameters
VehicleController vc;
// VCU1 message
// Message ID 0x101
ParameterDouble throttlePosition(101);             // 0-100%
ParameterBool   authenticationValid(104);
ParameterBool   vehicleReady(105);
ParameterDouble brakePositionMCU(106);
ParameterBool   mainRelayConnected(107);
ParameterInt    gearLeverPosition(108);            // Default, R, N, D, P
ParameterInt    vcuMotorOperationMode(112);        // Standby, Drive, Regen
ParameterInt    vehicleWarningLevel(113);          // 0 (normal) to 3 (high)
ParameterInt    keyPosition(114);                  // off, acc, on, crank
ParameterBool   auxiliaryRelayConnected(115);
// Other VCU Parameters
ParameterBool   switchRecuOn(120);
ParameterBool   mainRelayFault(121);
ParameterBool   auxRelayFault(122);
ParameterBool   throttleInhibit(123);

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
ParameterInt    motorWarningLevel(221);
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

// BMS Parameters
// Message ID 0x103
ParameterDouble batteryPackVoltage(301);
ParameterInt    batteryAverageTemp(302);
ParameterInt    batteryHighestTemp(303);
ParameterInt    batteryLowestTemp(304);
ParameterDouble batteryLVVoltage(305);
ParameterInt    batteryState(306);
ParameterBool   batteryIsFaulted(307);
ParameterBool   batteryIsStickyFaulted(308);
ParameterBool   batteryChargeInhibit(309);
ParameterBool   batteryDischargeInhibit(310);
ParameterBool   batteryPackCommFault(311);
ParameterBool   batteryOverVoltageFault(312);
ParameterBool   batteryUnderVoltageFault(313);
ParameterBool   batteryOverTempFault(314);
ParameterBool   batteryUnderTempFault(315);
ParameterBool   batteryLVVoltageFault(316);
ParameterBool   batteryWaterFault(317);
ParameterBool   batteryHeatLoopFault(318);
