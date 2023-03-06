/* ======================================================================
 * An electric vehicle conversion with an Alpha Electrics APEV528 motor
 * and inverter and Tesla Battery packs with Mitsubishi Outlander OBC+DCDC
*/


// General tools (use a lot of program storage space)
//#include "esp_wifi.h"
//#include "esp_bt.h"
// Custom devices
#include "GlobalParameters.h"
#include "DebugLogger.h"
#include "CANManager.h"
#include "Vehicle.h"
#include "APEV528.h"
// Standard devices
#include <Contactors.h>
#include <IgnitionSwitch.h>
#include <Switch.h>
#include <Pedal.h>


// Instantiate Devices
DebugLogger     logger(&vc);
CANManager      can(&vc, 22, 21);
Vehicle         vehicle(&vc);
APEV528         motor(&vc);
uint8_t ignitionPins[3] = {13, 35, 34};
int ignitionModes[3] = {INPUT_PULLDOWN, INPUT, INPUT};
IgnitionSwitch  ignition(&vc, ignitionPins, ignitionModes, &keyPosition, 50);
Contactors      contactors(&vc, 27, 26, 1000, &mainRelayConnected, &auxiliaryRelayConnected, &keyPosition, &vehicleReady);
Switch          recuSwitch(&vc, 25, INPUT, &switchRecuOn, 50);
Pedal           throttle(&vc, 32, 10, &throttlePosition);
Pedal           brake(&vc, 33, 10, &brakePositionMCU);


void setup() {
  // Preparations
  // Start the Serial monitor
  Serial.begin(115200);
  vTaskDelay(100);
  /*// Disable WiFi
  esp_wifi_stop();
  esp_wifi_deinit();
  // Disable Bluetooth
  esp_bt_controller_disable();
  esp_bt_controller_deinit();
  esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);*/
  
  // Start the devices in reasonable order
  logger.begin();
  can.begin(500000);
  throttle.begin();
  brake.begin();
  motor.begin();
  contactors.begin();
  vehicle.begin();
  recuSwitch.begin();
  ignition.begin();       // Should start at last to notify everone

  vTaskDelete(NULL); // exit
}


void loop()
{
  vTaskDelete(NULL); // We don't need that loop
}
