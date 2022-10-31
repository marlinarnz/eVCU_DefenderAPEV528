#ifndef CANMANAGER_H
#define CANMANAGER_H

#include <Arduino.h>
#include <DeviceCAN.h>


/** Class to manage the CAN bus on this specific vehicle.
 *  Reads the CAN bus and updates external parameters. Also writes
 *  messages to other CAN nodes given other parameters from the
 *  vehicle.
 */
class CANManager : public DeviceCAN
{
public:
  CANManager(VehicleController* vc);

private:
  void onMsgRcv(twai_message_t* pMsg);
  void onRemoteFrameRcv(twai_message_t* pMsg);
  void setBits(uint8_t* pByte, uint8_t lsb, uint8_t len, uint8_t val);
  // CAN messages
  twai_message_t* m_pMsgVCU1;
};

#endif
