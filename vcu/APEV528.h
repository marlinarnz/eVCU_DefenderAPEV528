#ifndef APEV528_H
#define APEV528_H

#include <Arduino.h>
#include <Device.h>
#include "GlobalParameters.h"


/** Manages the APEV528 MCU.
 *  Starts/stops the motor/regenerative braking, depending on vehicle
 *  parameters.
 */
class APEV528 : public Device
{
public:
  APEV528(VehicleController* vc, uint8_t enablePin);
  ~APEV528() {};
  void begin();
  void shutdown();

private:
  void onValueChanged(Parameter* pParam);
  void setMotorRunning();
  void setMotorRegen();
  bool m_waitingForContactor;
  uint8_t m_enablePin;
};

#endif
