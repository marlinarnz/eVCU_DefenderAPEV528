#ifndef VEHICLE_H
#define VEHICLE_H

#include <Arduino.h>
#include <Device.h>
#include "GlobalParameters.h"


/** Manages the vehicle stati.
 *  Reads Parameter changes of internal and external Devices and
 *  sets top-level Parameters.
 */
class Vehicle : public Device
{
public:
  Vehicle(VehicleController* vc) : Device(vc) {};
  ~Vehicle() {};
  void begin();
  void shutdown();

private:
  void onValueChanged(Parameter* pParam);
  void updateVehicleReadiness();
  void updateWarningLevel();
};

#endif
