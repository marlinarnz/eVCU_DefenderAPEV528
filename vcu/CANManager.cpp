#include "CANManager.h"


/** Constructor
 *  Instantiates all known CAN bus messages.
 *  @param vc pointer to the VehicleController instance
 *  @param pinRx RX capable GPIO pin for CAN bus communication
 *  @param pinRx TX capable GPIO pin for CAN bus communication
 */
CANManager::CANManager(VehicleController* vc, uint8_t pinRx, uint8_t pinTx)
  : DeviceCAN(vc), m_pMsgVCU1(NULL), m_msgVCU1UpdateTime(0), m_msgVCU1Counter(0),
    m_pinRx(pinRx), m_pinTx(pinTx)
{
  // Instantiate messages
  m_pMsgVCU1 = new twai_message_t;
  m_pMsgVCU1->extd = 0;
  m_pMsgVCU1->rtr = 0;
  m_pMsgVCU1->ss = 0;
  m_pMsgVCU1->self = 0;
  m_pMsgVCU1->dlc_non_comp = 0;
  m_pMsgVCU1->reserved = 0;
  m_pMsgVCU1->data_length_code = 8;
  m_pMsgVCU1->identifier = 0x101;
  for (int i=0; i<7; i++) {
    m_pMsgVCU1->data[i] = 0;    
  }
  m_pMsgVCU1->data[7] = 0^0xFF; // checksum
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
 *  @param speed CAN bus bps `{125000, 250000, 500000, 1000000}`
 */
void CANManager::begin(uint32_t speed)
{
  // Start the threads and install the driver
  this->startTasks(8000, 8000);
  configCAN_t config;
  config.speed = speed;
  config.txPin = static_cast<gpio_num_t>(m_pinTx);
  config.rxPin = static_cast<gpio_num_t>(m_pinRx);
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
/** Start operation.
 *  Start threads, install CAN driver, set outgoing messages
 */
void CANManager::begin()
{
  CANManager::begin(500000);
}


/** End operation.
 *  Uninstall the CAN driver.
 */
void CANManager::shutdown()
{
  this->endSerialProtocol();
  this->unregisterForValueChanged(101);
  this->unregisterForValueChanged(104);
  this->unregisterForValueChanged(105);
  this->unregisterForValueChanged(106);
  this->unregisterForValueChanged(107);
  this->unregisterForValueChanged(108);
  this->unregisterForValueChanged(112);
  this->unregisterForValueChanged(113);
  this->unregisterForValueChanged(114);
  this->unregisterForValueChanged(115);
}


/** React to changes in vehicle parameters.
 *  Update the vehicle's CAN messages.
 */
void CANManager::onValueChanged(Parameter* pParam)
{
  if (pParam) {
    // Switch through parameter IDs
    switch (pParam->getId()) {
      case 101:
        m_pMsgVCU1->data[0] = (uint8_t)(throttlePosition.getVal() / 0.3923);
        break;
      case 104:
        setBits(&(m_pMsgVCU1->data[3]), 29%8, 2, (uint8_t)(authenticationValid.getVal()));
        break;
      case 105:
        setBits(&(m_pMsgVCU1->data[4]), 32%8, 1, (uint8_t)(vehicleReady.getVal()));
        break;
      case 106:
        {
          bool pressed = brakePositionMCU.getVal() > BRAKE_THRESHOLD_PERCENT;
          setBits(&(m_pMsgVCU1->data[4]), 33%8, 2, (uint8_t)(pressed));
        }
        break;
      case 107:
        setBits(&(m_pMsgVCU1->data[4]), 35%8, 1, (uint8_t)(mainRelayConnected.getVal()));
        break;
      case 108:
        setBits(&(m_pMsgVCU1->data[4]), 36%8, 3, (uint8_t)(gearLeverPosition.getVal()));
        break;
      case 112:
        setBits(&(m_pMsgVCU1->data[5]), 42%8, 2, (uint8_t)(vcuMotorOperationMode.getVal()));
        break;
      case 113:
        setBits(&(m_pMsgVCU1->data[5]), 44%8, 2, (uint8_t)(vehicleWarningLevel.getVal()));
        break;
      case 114:
        setBits(&(m_pMsgVCU1->data[5]), 46%8, 2, (uint8_t)(keyPosition.getVal()));
        break;
      case 115:
        setBits(&(m_pMsgVCU1->data[6]), 52%8, 1, (uint8_t)(auxiliaryRelayConnected.getVal()));
        break;
      default:
        break;
    }
    // Update the rolling counter
    if (millis() - m_msgVCU1UpdateTime > 10) m_msgVCU1Counter ++;
    if (m_msgVCU1Counter > 0xf) m_msgVCU1Counter = 0;
    setBits(&(m_pMsgVCU1->data[6]), 48%8, 4, m_msgVCU1Counter);
    m_msgVCU1UpdateTime = millis();
    // Set the checksum
    m_pMsgVCU1->data[7] = 
      (uint8_t)(m_pMsgVCU1->data[0] + m_pMsgVCU1->data[1]
      + m_pMsgVCU1->data[2] + m_pMsgVCU1->data[3]
      + m_pMsgVCU1->data[4] + m_pMsgVCU1->data[5]
      + m_pMsgVCU1->data[6]) ^ 0xFF;
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
    case 0x105:
      // MCU1 with checksum
      if (pMsg->data[7] ==
          (uint8_t)(pMsg->data[0] + pMsg->data[1]
          + pMsg->data[2] + pMsg->data[3]
          + pMsg->data[4] + pMsg->data[5]
          + pMsg->data[6]) ^ 0xFF) {
        float speed = (pMsg->data[0]*256 + pMsg->data[1]) * 0.25;
        this->setDoubleValue(&motorSpeed, speed);
        this->setDoubleValue(&motorTorque, pMsg->data[2] * 0.392);
        this->setDoubleValue(&motorMaxTorque, pMsg->data[3] * 0.392);
        this->setDoubleValue(&motorMaxNegativeTorque, pMsg->data[4] * 0.392);
        this->setIntegerValue(&motorDirection, (pMsg->data[5] & 0b11000000) >> 6);
        this->setIntegerValue(&motorState, (pMsg->data[5] & 0b00111000) >> 3);
        this->setIntegerValue(&motorMode, (pMsg->data[6] & 0b00110000) >> 4);
      }
      break;
    case 0x106:
      // MCU2 with checksum
      if (pMsg->data[7] ==
          (uint8_t)(pMsg->data[0] + pMsg->data[1]
          + pMsg->data[2] + pMsg->data[3]
          + pMsg->data[4] + pMsg->data[5]
          + pMsg->data[6]) ^ 0xFF) {
        this->setIntegerValue(&motorBodyTemp, pMsg->data[0] - 40);
        this->setIntegerValue(&motorControllerTemp, pMsg->data[1] - 40);
        this->setBooleanValue(&motorDCOverCurrentFault, getBit(&(pMsg->data[2]), 16));
        this->setBooleanValue(&motorPhaseCurrentFault, getBit(&(pMsg->data[2]), 17));
        this->setBooleanValue(&motorOverTempFault, getBit(&(pMsg->data[2]), 18));
        this->setBooleanValue(&motorRotationTransformFault, getBit(&(pMsg->data[2]), 19));
        this->setBooleanValue(&motorPhaseCurrentSensorFault, getBit(&(pMsg->data[2]), 20));
        this->setBooleanValue(&motorOverSpeedFault, getBit(&(pMsg->data[2]), 21));
        this->setBooleanValue(&motorBodyOverTempFault, getBit(&(pMsg->data[2]), 22));
        this->setBooleanValue(&motorDCOverVoltageFault, getBit(&(pMsg->data[2]), 23));
        this->setBooleanValue(&motorBodyUnderTempFault, getBit(&(pMsg->data[3]), 25));
        this->setBooleanValue(&motorSystemFault, getBit(&(pMsg->data[3]), 26));
        this->setBooleanValue(&motorTempSensorFault, getBit(&(pMsg->data[3]), 27));
        this->setBooleanValue(&motorBodyTempSensorFault, getBit(&(pMsg->data[3]), 28));
        this->setBooleanValue(&motorDCVoltageSensorFault, getBit(&(pMsg->data[3]), 29));
        this->setBooleanValue(&motorDCUnderVoltageWarning, getBit(&(pMsg->data[3]), 30));
        this->setBooleanValue(&motorLVUnderVoltageWarning, getBit(&(pMsg->data[3]), 31));
        this->setIntegerValue(&motorWarningLevel, (pMsg->data[6] & 0b00110000) >> 4);
        this->setBooleanValue(&motorOpenPhaseFault, getBit(&(pMsg->data[6]), 54));
        this->setBooleanValue(&motorStall, getBit(&(pMsg->data[6]), 55));
      }
      break;
    case 0x107:
      // MCU3
      // round currents and voltages to one decimal to reduce traffic
      this->setDoubleValue(&motorDCVoltage, this->round((pMsg->data[0]*256 + pMsg->data[1]) * 0.01));
      this->setDoubleValue(&motorDCCurrent, this->round((pMsg->data[2]*256 + pMsg->data[3]) * 0.01));
      this->setDoubleValue(&motorPhaseCurrent, this->round((pMsg->data[4]*256 + pMsg->data[5]) * 0.01));
      break;
    case 0x103:
      // BMS1
      this->setDoubleValue(&batteryPackVoltage, (pMsg->data[0]*256 + pMsg->data[1]) * 0.1);
      this->setIntegerValue(&batteryAverageTemp, pMsg->data[2] - 40);
      this->setIntegerValue(&batteryHighestTemp, pMsg->data[3] - 40);
      this->setIntegerValue(&batteryLowestTemp, pMsg->data[4] - 40);
      this->setDoubleValue(&batteryLVVoltage, pMsg->data[5] * 0.1);
      this->setIntegerValue(&batteryState, pMsg->data[6] >> 4);
      this->setBooleanValue(&batteryIsFaulted, getBit(&(pMsg->data[6]), 4));
      this->setBooleanValue(&batteryIsStickyFaulted, getBit(&(pMsg->data[6]), 5));
      this->setBooleanValue(&batteryChargeInhibit, getBit(&(pMsg->data[6]), 6));
      this->setBooleanValue(&batteryDischargeInhibit, getBit(&(pMsg->data[6]), 7));
      this->setBooleanValue(&batteryPackCommFault, getBit(&(pMsg->data[7]), 0));
      this->setBooleanValue(&batteryOverVoltageFault, getBit(&(pMsg->data[7]), 1));
      this->setBooleanValue(&batteryUnderVoltageFault, getBit(&(pMsg->data[7]), 2));
      this->setBooleanValue(&batteryOverTempFault, getBit(&(pMsg->data[7]), 3));
      this->setBooleanValue(&batteryUnderTempFault, getBit(&(pMsg->data[7]), 4));
      this->setBooleanValue(&batteryLVVoltageFault, getBit(&(pMsg->data[7]), 5));
      this->setBooleanValue(&batteryWaterFault, getBit(&(pMsg->data[7]), 6));
      this->setBooleanValue(&batteryHeatLoopFault, getBit(&(pMsg->data[7]), 7));
      break;
    default:
      // Print the message ID
      //PRINT("Unknown message received: 0x" + String(pMsg->identifier, HEX))
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
      // Print the message ID
      PRINT("Unknown remote frame received: 0x" + String(pMsg->identifier, HEX))
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
  // create a mask with the given length and lsb position
  uint8_t one = 1;
  uint8_t mask = ((one << len) - one) << lsb;
  // clear the bits within the mask from the target byte
  *pByte &= ~mask;
  // set the bits in the target byte based on the given value
  *pByte |= (val << lsb) & mask;
}


/** Helper function to get one bit from a byte.
 * Motorola forward LSB byte order
 * @param pByte:  pointer to the byte to read
 * @param bitNum: bit position
 * @return bool bit value
 */
bool CANManager::getBit(uint8_t* pByte, uint8_t bitNum)
{
  return (bool)( (*pByte << (8 - bitNum%8) ) >> 7);
}


/** Helper function to round voltages and currents.
 * Defaults to one decimal
 * @param val:    double value to be rounded
 * @return double rounded value
 */
double CANManager::round(double val)
{
  return static_cast<double>(static_cast<int>(val * 10 + 0.5)) / 10.0;
}
