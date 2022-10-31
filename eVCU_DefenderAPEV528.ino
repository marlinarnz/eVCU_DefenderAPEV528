/* ======================================================================
 * An electric vehicle conversion with a Alpha Electrics APEV528 motor
 * and inverter and Tesla Battery packs.
*/

#include "CANManager.h"

// Instantiate the VehicleController and the vehicle's Parameters
VehicleController vc;
ParameterInt throttlePosition(101);             // 0-100%
ParameterBool authentication(104);
ParameterBool vehicleReady(105);
ParameterInt breakPosition(106);
ParameterBool mainRelayConnected(107);
ParameterInt gearLeverPosition(108);            // see CAN bus Excel spec
ParameterInt motorOperationMode(112);           // see CAN bus Excel spec
ParameterInt vehicleWarningLevel(113);          // 0 (normal) to 3 (high)
ParameterInt keyPosition(114);                  // see CAN bus Excel spec
ParameterBool auxiliaryRelayConnected(115);


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
