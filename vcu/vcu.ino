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
#include "GearLever.h"
// Standard devices
#include <Contactors.h>
#include <IgnitionSwitch.h>
#include <Switch.h>
#include <Pedal.h>

// Define pins
#define PIN_CANRx             22
#define PIN_CANTx             21
#define PIN_MotorEnable       14
#define PIN_RecuSwitch        25
uint8_t PINS_GearLever[4] =   {15, 2, 25, 5};
int     MODES_GearLever[4] =  {INPUT_PULLDOWN, INPUT_PULLDOWN, INPUT, INPUT_PULLDOWN};
uint8_t PINS_Ignition[3] =    {13, 35, 34};
int     MODES_Ignition[3] =   {INPUT_PULLDOWN, INPUT, INPUT};
#define PIN_AuxContactor      26
#define PIN_MainContactor     27
#define PIN_Throttle          32
#define PIN_Brake             33


// Instantiate Devices
DebugLogger     logger(&vc);
CANManager      can(&vc, PIN_CANRx, PIN_CANTx);
Vehicle         vehicle(&vc);
APEV528         motor(&vc, PIN_MotorEnable);
//Switch          recuSwitch(&vc, PIN_RecuSwitch, INPUT, &switchRecuOn, 50);
GearLever       gear(&vc, PINS_GearLever, MODES_GearLever, &gearLeverPosition);
IgnitionSwitch  ignition(&vc, PINS_Ignition, MODES_Ignition, &keyPosition, 50);
Contactors      contactors(&vc, PIN_MainContactor, PIN_AuxContactor, 1000, &mainRelayConnected, &auxiliaryRelayConnected, &keyPosition, &vehicleReady, 2, false);
Pedal           throttle(&vc, PIN_Throttle, 10, &throttlePosition); // TODO: include discharge inhibit and BMS state = RUN (concatenate both BMS parameters to one ParameterBool ThrottleInhibit)
Pedal           brake(&vc, PIN_Brake, 10, &brakePositionMCU); // TODO: include charge inhibit and BMS state = RUN


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
  can.begin(500000);      // Should start first to get all starting values
  throttle.begin();
  brake.begin();
  motor.begin();
  contactors.begin();
  vehicle.begin();        // Sets vehicle authentication to true
  //recuSwitch.begin();
  gear.begin();
  ignition.begin();       // Should start at last to notify everyone

  while (1) {
    vTaskDelay(pdMS_TO_TICKS(4));
  }
  vTaskDelete(NULL); // exit this task
}


void loop()
{
  vTaskDelete(NULL); // We don't need that loop
}
