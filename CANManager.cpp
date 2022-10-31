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
  m_pMsgVCU1->data = {0,0,0,0,0,0,0,0^0xFF};
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
      m_pMsgVCU1->data[3] = ((uint8_t)(pParam.getVal()) << 5);
      break;
    case 104:
      m_pMsgVCU1->data[0] = (uint8_t)();
      break;
    case 104:
      m_pMsgVCU1->data[0] = (uint8_t)();
      break;
    case 104:
      m_pMsgVCU1->data[0] = (uint8_t)();
      break;
    case 104:
      m_pMsgVCU1->data[0] = (uint8_t)();
      break;
    case 104:
      m_pMsgVCU1->data[0] = (uint8_t)();
      break;
    case 104:
      m_pMsgVCU1->data[0] = (uint8_t)();
      break;
    case 104:
      m_pMsgVCU1->data[0] = (uint8_t)();
      break;
    case 104:
      m_pMsgVCU1->data[0] = (uint8_t)();
      break;
    default:
      break;
  }
}

 
/** Handle incoming messages.
 *  Sets the corresponding parameters.
 *  @param pMsg: pointer to a twai_message_t message
 */
void CANManager::onMsgRcv(twai_message_t* pMsg)
{
  // Switch through known message IDs
  switch (pMsg->identifier) {
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
 * @param pByte:  pointer to the byte to override
 * @param lsb:    start of the bit sequence
 * @param len:    lenght of the bit sequence
 * @param val:    value to be set
 */
void CANManager::setBits(uint8_t* pByte, uint8_t lsb, uint8_t len, uint8_t val)
{
  if (val > 0) {
    pByte |= (uint8_t)();
  }
}
