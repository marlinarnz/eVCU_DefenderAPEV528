#include "CANManager.h"


/** Constructor
 *  Instantiates all known CAN bus messages.
 */
CANManager::CANManager(VehicleController* vc)
  : DeviceCAN(vc), m_pMsgVCU1(NULL)
{
  // Instantiate messages
  m_pMsgVCU1 = new twai_message_t;
  m_pMsgVCU1->data_length_code = 8;
  m_pMsgVCU1->identifier = 0x101;
  m_pMsgVCU1->data = {0,0,0,0,0,0,0,0x00^0xFF};
}


/** Destructor
 *  Deletes messages on the heap
 */
CANManager::~CANManager()
{
  delete m_pMsgVCU1;
}


/** Start operation.
 *  Start threads, install CAN driver, set outgoing messages
 */
void CANManager::begin()
{
  // Start the threads and install the driver
  this->startTasks(8000, 8000);
  configCAN_t config;
  config.speed = 500000;
  this->initSerialProtocol(config);

  // Register for parameter changes
  this->registerForValueChanged(101);
  this->registerForValueChanged(104);
  this->registerForValueChanged(105);
  this->registerForValueChanged(106);
  this->registerForValueChanged(107);
  this->registerForValueChanged(108);
  this->registerForValueChanged(112);
  this->registerForValueChanged(113);
  this->registerForValueChanged(114);
  this->registerForValueChanged(115);
  
  // Define message(s) to send
  this->setTransactionPeriodic(m_pMsgVCU1, 10);
}


/** End operation.
 *  Uninstall the CAN driver.
 */
void CANManager::shutdown()
{
  this->endSerialProtocol();
}


/** React to changes in vehicle parameters.
 *  Update the vehicle's CAN messages.
 */
void CANManager::onValueChanged(Parameter* pParam)
{
  // Switch through parameter IDs
  switch (pParam->getId()) {
    case 101:
      m_pMsgVCU1->data[0] = (uint8_t)(pParam.getVal() / 0.3923);
      break;
    case 104:
      setBits(m_pMsgVCU1->data[3], 29%8, 2, (uint8_t)(pParam.getVal()));
      break;
    case 105:
      setBits(m_pMsgVCU1->data[4], 32%8, 1, (uint8_t)(pParam.getVal()));
      break;
    case 106:
      setBits(m_pMsgVCU1->data[4], 33%8, 2, (uint8_t)(pParam.getVal()));
      break;
    case 107:
      setBits(m_pMsgVCU1->data[4], 35%8, 1, (uint8_t)(pParam.getVal()));
      break;
    case 108:
      setBits(m_pMsgVCU1->data[4], 36%8, 3, (uint8_t)(pParam.getVal()));
      break;
    case 112:
      setBits(m_pMsgVCU1->data[5], 42%8, 2, (uint8_t)(pParam.getVal()));
      break;
    case 113:
      setBits(m_pMsgVCU1->data[5], 44%8, 2, (uint8_t)(pParam.getVal()));
      break;
    case 114:
      setBits(m_pMsgVCU1->data[5], 46%8, 2, (uint8_t)(pParam.getVal()));
      break;
    case 115:
      setBits(m_pMsgVCU1->data[6], 52%8, 1, (uint8_t)(pParam.getVal()));
      break;
    default:
      break;
  }
  // Set the checksum
  m_pMsgVCU1->data[7] = 
    (m_pMsgVCU1->data[0] + m_pMsgVCU1->data[1]
     + m_pMsgVCU1->data[2] + m_pMsgVCU1->data[3]
     + m_pMsgVCU1->data[4] + m_pMsgVCU1->data[5]
     + m_pMsgVCU1->data[6]) ^ 0xFF;
}


/** Handle incoming messages.
 *  Sets the corresponding parameters.
 *  @param pMsg: pointer to a twai_message_t message
 */
void CANManager::onMsgRcv(twai_message_t* pMsg)
{
  // Switch through known message IDs
  switch (pMsg->identifier) {
    case 0x105:
      // MCU1 with checksum
      if (pMsg->data[7] ==
          (pMsg->data[0] + pMsg->data[1]
           + pMsg->data[2] + pMsg->data[3]
           + pMsg->data[4] + pMsg->data[5]
           + pMsg->data[6]) ^ 0xFF) {
        float speed = (pMsg->data[0]*256 + pMsg->data[1]) * 0.25;
        this->setDoubleValue(&motorSpeed, speed);
        this->setDoubleValue(&motorTorque, pMsg->data[2] * 0.392);
        this->setDoubleValue(&motorMaxTorque, pMsg->data[3] * 0.392);
        this->setDoubleValue(&motorMaxNegativeTorque, pMsg->data[4] * 0.392);
        this->setIntegerValue(&motorDirection, (pMsg->data[5] & 0b00000011));
        this->setIntegerValue(&motorState, (pMsg->data[5] & 0b00011100) >> 2);
        this->setIntegerValue(&motorMode, (pMsg->data[6] & 0b00000011));
      }
      break;
    case 0x106:
      // MCU2 with checksum
      if (pMsg->data[7] ==
          (pMsg->data[0] + pMsg->data[1]
           + pMsg->data[2] + pMsg->data[3]
           + pMsg->data[4] + pMsg->data[5]
           + pMsg->data[6]) ^ 0xFF) {
        this->setIntegerValue(&motorBodyTemp, pMsg->data[0] - 40);
        this->setIntegerValue(&motorControllerTemp, pMsg->data[1] - 40);
        this->setBooleanValue(&motorDCOverCurrentFault, getBit(pMsg->data[2], 16));
        this->setBooleanValue(&motorPhaseCurrentFault, getBit(pMsg->data[2], 17));
        this->setBooleanValue(&motorOverTempFault, getBit(pMsg->data[2], 18));
        this->setBooleanValue(&motorRotationTransformFault, getBit(pMsg->data[2], 19));
        this->setBooleanValue(&motorPhaseCurrentSensorFault, getBit(pMsg->data[2], 20));
        this->setBooleanValue(&motorOverSpeedFault, getBit(pMsg->data[2], 21));
        this->setBooleanValue(&motorBodyOverTempFault, getBit(pMsg->data[2], 22));
        this->setBooleanValue(&motorDCOverVoltageFault, getBit(pMsg->data[2], 23));
        this->setBooleanValue(&motorBodyUnderTempFault, getBit(pMsg->data[3], 25));
        this->setBooleanValue(&motorSystemFault, getBit(pMsg->data[3], 26));
        this->setBooleanValue(&motorTempSensorFault, getBit(pMsg->data[3], 27));
        this->setBooleanValue(&motorBodyTempSensorFault, getBit(pMsg->data[3], 28));
        this->setBooleanValue(&motorDCVoltageSensorFault, getBit(pMsg->data[3], 29));
        this->setBooleanValue(&motorDCUnderVoltageWarning, getBit(pMsg->data[3], 30));
        this->setBooleanValue(&motorLVUnderVoltageWarning, getBit(pMsg->data[3], 31));
        this->setIntegerValue(&motorWarningLevel, (pMsg->data[5] & 0b00001100) >> 2);
        this->setBooleanValue(&motorOpenPhaseFault, getBit(pMsg->data[6], 54));
        this->setBooleanValue(&motorStall, getBit(pMsg->data[6], 55));
      }
      break;
    case 0x107:
      // MCU3
      float voltage = (pMsg->data[0]*256 + pMsg->data[1]) * 0.01;
      this->setDoubleValue(&motorDCVoltage, voltage);
      float current = (pMsg->data[2]*256 + pMsg->data[3]) * 0.01;
      this->setDoubleValue(&motorDCCurrent, current);
      float phaseCurr = (pMsg->data[4]*256 + pMsg->data[5]) * 0.01;
      this->setDoubleValue(&motorPhaseCurrent, phaseCurr);
      break;
    default:
      break;
  }
}


/** Handle incoming request / remote frames.
 *  Remote frames can trigger config changes.
 *  @param pMsg: pointer to a twai_message_t message
 */
void CANManager::onRemoteFrameRcv(twai_message_t* pMsg)
{
  // Switch through known message IDs
  switch (pMsg->identifier) {
    default:
      break;
  }
}


/** Helper function to set bits within one byte.
 *  Does not reset values of other bits. Starts at the lsb
 *  https://stackoverflow.com/questions/47981/how-do-i-set-clear-and-toggle-a-single-bit
 * @param pByte:  pointer to the byte to override
 * @param lsb:    start of the bit sequence
 * @param len:    lenght of the bit sequence
 * @param val:    value to be set
 */
void CANManager::setBits(uint8_t* pByte, uint8_t lsb, uint8_t len, uint8_t val)
{
  for (uint8_t pos=0; pos<len; pos++) {
    uint8_t one = 1;
    uint8_t valBit = ((val >> lsb+len-1-pos) & one) << lsb+len-pos;
    *pByte ^= (-valBit ^ *pByte) & (one << (lsb + pos));
  }
}


/** Helper function to get one bit from a byte.
 * @param pByte:  pointer to the byte to read
 * @param bitNum: bit position
 */
bool CANManager::getBit(uint8_t* pByte, uint8_t bitNum)
{
  return (bool)((*pByte << bitNum%8) >> 7);
}
