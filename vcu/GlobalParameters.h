#ifndef GLOBALPARAMETERS_H
#define GLOBALPARAMETERS_H

#include <VehicleController.h>
#include <Parameter.h>


/** Global parameters of this vehicle.
 *  Must be available to all devices. See cpp file for IDs and definition.
 */

// Declare the VehicleController and the vehicle's Parameters
extern VehicleController vc;
// VCU1 message
// Message ID 0x101
extern ParameterDouble throttlePosition;               // 0-100%
extern ParameterBool   authenticationValid;
extern ParameterBool   vehicleReady;
extern ParameterDouble brakePositionMCU;
extern ParameterBool   mainRelayConnected;
extern ParameterInt    gearLeverPosition;              // Default, R, N, D, P
extern ParameterInt    vcuMotorOperationMode;          // Standby, Drive, Regen
extern ParameterInt    vehicleWarningLevel;            // 0 (normal) to 3 (high)
extern ParameterInt    keyPosition;                    // off, acc, on, crank
extern ParameterBool   auxiliaryRelayConnected;
// Other VCU Parameters
extern ParameterBool   switchRecuOn;
extern ParameterBool   mainRelayFault;
extern ParameterBool   auxRelayFault;
// The MCU's Parameters
// Message ID 0x107
extern ParameterDouble motorDCVoltage;
extern ParameterDouble motorDCCurrent;
extern ParameterDouble motorPhaseCurrent;
// Message ID 0x106
extern ParameterInt    motorBodyTemp;
extern ParameterInt    motorControllerTemp;
extern ParameterBool   motorDCOverCurrentFault;
extern ParameterBool   motorPhaseCurrentFault;
extern ParameterBool   motorOverTempFault;
extern ParameterBool   motorRotationTransformFault;
extern ParameterBool   motorPhaseCurrentSensorFault;
extern ParameterBool   motorOverSpeedFault;
extern ParameterBool   motorBodyOverTempFault;
extern ParameterBool   motorDCOverVoltageFault;
extern ParameterBool   motorBodyUnderTempFault;
extern ParameterBool   motorSystemFault;
extern ParameterBool   motorTempSensorFault;
extern ParameterBool   motorBodyTempSensorFault;
extern ParameterBool   motorDCVoltageSensorFault;
extern ParameterBool   motorDCUnderVoltageWarning;
extern ParameterBool   motorLVUnderVoltageWarning;
extern ParameterInt    motorWarningLevel;
extern ParameterBool   motorOpenPhaseFault;
extern ParameterBool   motorStall;
extern ParameterInt    motor106RollingCounter;
// Message ID 0x105
extern ParameterDouble motorSpeed;
extern ParameterDouble motorTorque;
extern ParameterDouble motorMaxTorque;
extern ParameterDouble motorMaxNegativeTorque;
extern ParameterInt    motorDirection;
extern ParameterInt    motorState;
extern ParameterInt    motorMode;
extern ParameterInt    motor105RollingCounter;


// Define thresholds
#define WARN_MOTOR_OVERTEMP_C 60
#define DERATE_MOTOR_OVERTEMP_C 70
#define WARN_BATTERY_UNDERVOLTAGE_V 340
#define DERATE_BATTERY_UNDERVOLTAGE_V 330
#define BRAKE_THRESHOLD_PERCENT 0.1f


#endif
