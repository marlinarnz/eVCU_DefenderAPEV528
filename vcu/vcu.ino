/* ======================================================================
 * An electric vehicle conversion with an Alpha Electrics APEV528 motor
 * and inverter and Tesla Battery packs with Mitsubishi Outlander OBC+DCDC
*/

#include "GlobalParameters.h"
#include "DebugLogger.h"
#include "CANManager.h"
#include "Vehicle.h"
#include "APEV528.h"
#include <Contactors.h>
#include <IgnitionSwitch.h>
#include <Switch.h>

// Instantiate Devices
DebugLogger     logger(&vc);
CANManager      can(&vc);
Vehicle         vehicle(&vc);
APEV528         motor(&vc);
uint8_t ignitionPins[3] = {13, 12, 14};
uint8_t ignitionModes[3] = {INPUT_PULLDOWN, INPUT_PULLDOWN, INPUT_PULLDOWN};
//IgnitionSwitch  ignition(&vc, ignitionPins, ignitionModes, &keyPosition);
Contactors      contactors(&vc, 27, 26, 1000, &mainRelayConnected, &auxiliaryRelayConnected, &keyPosition, &vehicleReady);


void setup() {
  // Preparations
  Serial.begin(115200); // Start the Serial monitor
  
  // Start the devices
  logger.begin();
  can.begin();
  vehicle.begin();
  motor.begin();
  //ignition.begin();
  contactors.begin();

  while(1) {
    vTaskDelay(100); // Run indefinitely
  }
}


void loop()
{
  vTaskDelete(NULL); // We don't need that loop
}
