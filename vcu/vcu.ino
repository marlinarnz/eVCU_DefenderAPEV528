/* ======================================================================
 * An electric vehicle conversion with an Alpha Electrics APEV528 motor
 * and inverter and Tesla Battery packs with Mitsubishi Outlander OBC+DCDC
*/

#include "GlobalParameters.h"
#include "CANManager.h"
#include "Vehicle.h"

// Instantiate Devices
CANManager      can(&vc);
Vehicle         vehicle(&vc);


void setup() {
  // Preparations
  Serial.begin(9600); // Start the Serial monitor
  
  // Start the devices
  can.begin();
  vehicle.begin();

  while(1) {
    vTaskDelay(100); // Run indefinitely
  }
}


void loop()
{
  vTaskDelete(NULL); // We don't need that loop
}
