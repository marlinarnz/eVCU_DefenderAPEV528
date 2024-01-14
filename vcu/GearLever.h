#ifndef GEARLEVER_H
#define GEARLEVER_H

#include <Arduino.h>
#include <Constants.h>
#include <DevicePinMulti.h>


/** Class for gear levers with R, N, D, P.
 *  Observes 4 pins to write the gear lever position into a
 *  `ParameterInt` (`0`: default, `1`: R, `2`: N, `3`: D, `4`: P).
 */
class GearLever : public DevicePinMulti<4>
{
public:
  GearLever(VehicleController* vc, uint8_t pins[4], int inputModes[4], ParameterInt* pParam, int debounce=SWITCH_DEBOUNCE_MS);
  ~GearLever();
  void begin();
  void shutdown();
  
private:
  void onValueChanged(Parameter* pParamWithNewValue);
  void onPinInterrupt();
  int getLeverPosition();
  ParameterInt* m_pParam;
};

#endif