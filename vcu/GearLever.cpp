#include "GearLever.h"


/** The constructor saves all relevant information as members.
 *  @param vc pointer to the VehicleController instance
 *  @param pins array of 4 GPIO pins to use as input in this order:
 *              reverse, neutral, drive, park
 *  @param inputModes array of 4 modes for the Arduino function `pinMode()`.
 *                    Can be `INPUT`; `INPUT_PULLUP`; `INPUT_PULLDOWN`
 *  @param pParam pointer to the ParameterInt instance that
 *                shall inform other Devices about gear lever position
 *  @param debounce minimum time between two pin interactions in ms.
 *                  Defaults to macro `SWITCH_DEBOUNCE_MS`
 */
GearLever::GearLever(VehicleController* vc, uint8_t pins[4], int inputModes[4], ParameterInt* pParam, int debounce)
  : DevicePinMulti<4>(vc, pins, debounce, inputModes, CHANGE),
    m_pParam(pParam)
{}


/** The destructor does nothing.
 */
GearLever::~GearLever()
{}


/** Start tasks.
 *  Pin mode parameters were already handled in the parent class
 *  constructor, but the interrupt must be attached at begin.
 *  Init the gear lever position to the physical value.
 */
void GearLever::begin()
{
  // Attach the interrupt on the pins given in the constructor
  DevicePinMulti<4>::attachISR();
  // Start tasks
  this->startTasks(4096, 8192);
  // Init the switch position
  this->setIntegerValue(m_pParam, getLeverPosition());
}


/** At shutdown, nothing happens.
 */
void GearLever::shutdown()
{}


/** Notify other Devices about the switch status change.
 */
void GearLever::onPinInterrupt()
{
  this->setIntegerValue(m_pParam, getLeverPosition());
}


/** Figure out the position of the gear lever.
 *  Check the pins, in order: P, D, N, R.
 *  @return integer position of the gear lever:
 *          `0`: default, `1`: R, `2`: N, `3`: D, `4`: P
 */
/*int GearLever::getLeverPosition()
{
  if (digitalRead(m_pins[3]) == HIGH) {
    return 4;
  }
  else if (digitalRead(m_pins[2]) == HIGH) {
    return 3;
  }
  else if (digitalRead(m_pins[1]) == HIGH) {
    return 2;
  }
  else if (digitalRead(m_pins[0]) == HIGH) {
    return 1;
  }
  return 0;
}*/
int GearLever::getLeverPosition()
{
  if (digitalRead(m_pins[2]) == HIGH) {
    return 3;
  } else {
    return 4;
  }
}


/** The GearLever Device is not interested in other Devices.
 */
void GearLever::onValueChanged(Parameter* pParamWithNewValue)
{}
