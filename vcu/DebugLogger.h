#ifndef DEBUGLOGGER_H
#define DEBUGLOGGER_H

#include <Arduino.h>
#include <Device.h>
#include "GlobalParameters.h"


/** Logs the vehicle status.
 *  Prints informative messages on the console.
 */
class DebugLogger : public Device
{
public:
  DebugLogger(VehicleController* vc) : Device(vc), m_throttleStatePressed(false), m_brakeStatePressed(false), m_motorRotating(false) {};
  ~DebugLogger() {};
  void begin();
  void shutdown();

private:
  void onValueChanged(Parameter* pParam);
  bool m_throttleStatePressed;
  bool m_brakeStatePressed;
  bool m_motorRotating;
};

#endif
