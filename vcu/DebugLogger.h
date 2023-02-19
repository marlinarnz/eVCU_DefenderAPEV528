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
  DebugLogger(VehicleController* vc) : Device(vc) {};
  ~DebugLogger() {};
  void begin();
  void shutdown();

private:
  void onValueChanged(Parameter* pParam);
};

#endif
