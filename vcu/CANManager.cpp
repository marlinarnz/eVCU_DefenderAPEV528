#include "CANManager.h"

// Create variables to store CAN messages
twai_message_t msgBMS1;
twai_message_t msgBMS2;
twai_message_t msgBMS3;
twai_message_t msgBMS4;
twai_message_t msgBMS8;
twai_message_t msgICU1;
twai_message_t msgVCU2;
twai_message_t msgCDU1;
twai_message_t msgCDU2;
twai_message_t msgCDU3;
twai_message_t msgCDU4;
twai_message_t msgSRS;
twai_message_t msgSAS;
twai_message_t msgABS1;
twai_message_t msgABS2;
twai_message_t msgABS3;
int msgBMS1Interval = 20;
int msgBMS2Interval = 20;
int msgBMS3Interval = 20;
int msgBMS4Interval = 20;
int msgBMS8Interval = 20;
int msgICU1Interval = 20;
int msgVCU2Interval = 20;
int msgCDU1Interval = 100;
int msgCDU2Interval = 100;
int msgCDU3Interval = 100;
int msgCDU4Interval = 50;
int msgSRSInterval = 500;
int msgSASInterval = 10;
int msgABS1Interval = 20;
int msgABS2Interval = 20;
int msgABS3Interval = 20;
long msgBMS1Time = 0;
long msgBMS2Time = 0;
long msgBMS3Time = 0;
long msgBMS4Time = 0;
long msgBMS8Time = 0;
long msgICU1Time = 0;
long msgVCU2Time = 0;
long msgCDU1Time = 0;
long msgCDU2Time = 0;
long msgCDU3Time = 0;
long msgCDU4Time = 0;
long msgSRSTime = 0;
long msgSASTime = 0;
long msgABS1Time = 0;
long msgABS2Time = 0;
long msgABS3Time = 0;
int msgBMS1Counter = 0;
int msgBMS2Counter = 0;
int msgBMS3Counter = 0;
int msgBMS4Counter = 0;
int msgBMS8Counter = 0;
int msgICU1Counter = 0;
int msgVCU2Counter = 0;
int msgCDU1Counter = 0;
int msgCDU2Counter = 0;
int msgCDU3Counter = 0;
int msgCDU4Counter = 0;
int msgSRSCounter = 0;
int msgSASCounter = 0;
int msgABS1Counter = 0;
int msgABS2Counter = 0;
int msgABS3Counter = 0;


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

  
  // Init the outgoing messages
  msgBMS1.extd = 0;
  msgBMS1.rtr = 0;
  msgBMS1.ss = 0;
  msgBMS1.self = 0;
  msgBMS1.dlc_non_comp = 0;
  msgBMS1.reserved = 0;
  msgBMS1.data_length_code = 8;
  msgBMS1.identifier = 0x1A0; // BMS
  msgBMS1.data[0] = 0x00; // warning level, status, main contactors
  msgBMS1.data[1] = 0xC6; // SOC
  msgBMS1.data[2] = 0xC6; // SOH
  msgBMS1.data[3] = 0b11000000; // AC, precharge relays, mode
  msgBMS1.data[4] = 0xFF; // max discharge (14 bits)
  msgBMS1.data[5] = 0x0F; // max discharge continued, contactor adherence
  msgBMS1.data[6] = 0x00; // alarms and rolling counter
  msgBMS1.data[7] = 0x00; // checksum

  msgBMS2.extd = 0;
  msgBMS2.rtr = 0;
  msgBMS2.ss = 0;
  msgBMS2.self = 0;
  msgBMS2.dlc_non_comp = 0;
  msgBMS2.reserved = 0;
  msgBMS2.data_length_code = 8;
  msgBMS2.identifier = 0x1A1; // BMS2
  msgBMS2.data[0] = 0b01110011; // V
  msgBMS2.data[1] = 0b10100000; // V and errors
  msgBMS2.data[2] = 0b00000001; // A
  msgBMS2.data[3] = 0b11110101; // A
  msgBMS2.data[4] = 0b00001111; // errors and cell V
  msgBMS2.data[5] = 0b00111100; // cell V
  msgBMS2.data[6] = 0x00; // alarms and rolling counter
  msgBMS2.data[7] = 0x00; // checksum

  msgBMS3.extd = 0;
  msgBMS3.rtr = 0;
  msgBMS3.ss = 0;
  msgBMS3.self = 0;
  msgBMS3.dlc_non_comp = 0;
  msgBMS3.reserved = 0;
  msgBMS3.data_length_code = 8;
  msgBMS3.identifier = 0x1A2; // BMS3
  msgBMS3.data[0] = 0x00;
  msgBMS3.data[1] = 0x00;
  msgBMS3.data[2] = 0x00;
  msgBMS3.data[3] = 0x00;
  msgBMS3.data[4] = 0x00;
  msgBMS3.data[5] = 0x00;
  msgBMS3.data[6] = 0x00;
  msgBMS3.data[7] = 0x00;

  msgBMS4.extd = 0;
  msgBMS4.rtr = 0;
  msgBMS4.ss = 0;
  msgBMS4.self = 0;
  msgBMS4.dlc_non_comp = 0;
  msgBMS4.reserved = 0;
  msgBMS4.data_length_code = 8;
  msgBMS4.identifier = 0x1A3; // BMS4
  msgBMS4.data[0] = 0xF0;
  msgBMS4.data[1] = 0xF0;
  msgBMS4.data[2] = 0xF0;
  msgBMS4.data[3] = 0xF0;
  msgBMS4.data[4] = 0x00;
  msgBMS4.data[5] = 0x0F;
  msgBMS4.data[6] = 0x00;
  msgBMS4.data[7] = 0x00;

  msgBMS3.extd = 0;
  msgBMS3.rtr = 0;
  msgBMS3.ss = 0;
  msgBMS3.self = 0;
  msgBMS3.dlc_non_comp = 0;
  msgBMS3.reserved = 0;
  msgBMS3.data_length_code = 8;
  msgBMS3.identifier = 0x1A7; // BMS8
  msgBMS3.data[0] = 0x00;
  msgBMS3.data[1] = 0xC6;
  msgBMS3.data[2] = 0x3C;
  msgBMS3.data[3] = 0x3C;
  msgBMS3.data[4] = 0x00;
  msgBMS3.data[5] = 0x00;
  msgBMS3.data[6] = 0x00;
  msgBMS3.data[7] = 0x00;

  msgICU1.extd = 0;
  msgICU1.rtr = 0;
  msgICU1.ss = 0;
  msgICU1.self = 0;
  msgICU1.dlc_non_comp = 0;
  msgICU1.reserved = 0;
  msgICU1.data_length_code = 8;
  msgICU1.identifier = 0x431; // park break
  msgICU1.data[0] = 0x00; // park break, counter
  msgICU1.data[1] = 0x63; // SOC
  msgICU1.data[2] = 0x00;
  msgICU1.data[3] = 0x00;
  msgICU1.data[4] = 0x00;
  msgICU1.data[5] = 0x00;
  msgICU1.data[6] = 0x00;
  msgICU1.data[7] = 0x00;

  msgVCU2.extd = 0;
  msgVCU2.rtr = 0;
  msgVCU2.ss = 0;
  msgVCU2.self = 0;
  msgVCU2.dlc_non_comp = 0;
  msgVCU2.reserved = 0;
  msgVCU2.data_length_code = 8;
  msgVCU2.identifier = 0x102; // VCU2
  msgVCU2.data[0] = 0x00;
  msgVCU2.data[1] = 0x00;
  msgVCU2.data[2] = 0x00;
  msgVCU2.data[3] = 0x00;
  msgVCU2.data[4] = 0x00;
  msgVCU2.data[5] = 0xF0; // max torque request
  msgVCU2.data[6] = 0x00; // counter
  msgVCU2.data[7] = 0xF0 ^ 0xFF; // checksum

  msgCDU1.extd = 0;
  msgCDU1.rtr = 0;
  msgCDU1.ss = 0;
  msgCDU1.self = 0;
  msgCDU1.dlc_non_comp = 0;
  msgCDU1.reserved = 0;
  msgCDU1.data_length_code = 8;
  msgCDU1.identifier = 0x373; // CDU1
  msgCDU1.data[0] = 0x00;
  msgCDU1.data[1] = 0x00;
  msgCDU1.data[2] = 0x00;
  msgCDU1.data[3] = 0x3C;
  msgCDU1.data[4] = 0x00;
  msgCDU1.data[5] = 0x00;
  msgCDU1.data[6] = 0x00;
  msgCDU1.data[7] = 0x00;

  msgCDU2.extd = 0;
  msgCDU2.rtr = 0;
  msgCDU2.ss = 0;
  msgCDU2.self = 0;
  msgCDU2.dlc_non_comp = 0;
  msgCDU2.reserved = 0;
  msgCDU2.data_length_code = 8;
  msgCDU2.identifier = 0x374; // CDU2
  msgCDU2.data[0] = 0x00;
  msgCDU2.data[1] = 0x00;
  msgCDU2.data[2] = 0x00;
  msgCDU2.data[3] = 0x00;
  msgCDU2.data[4] = 0x00;
  msgCDU2.data[5] = 0b11100111;
  msgCDU2.data[6] = 0x00;
  msgCDU2.data[7] = 0x00;

  msgCDU3.extd = 0;
  msgCDU3.rtr = 0;
  msgCDU3.ss = 0;
  msgCDU3.self = 0;
  msgCDU3.dlc_non_comp = 0;
  msgCDU3.reserved = 0;
  msgCDU3.data_length_code = 8;
  msgCDU3.identifier = 0x375; // CDU3
  msgCDU3.data[0] = 0x3C;
  msgCDU3.data[1] = 0x00;
  msgCDU3.data[2] = 0x00;
  msgCDU3.data[3] = 0x00;
  msgCDU3.data[4] = 0x00;
  msgCDU3.data[5] = 0x00;
  msgCDU3.data[6] = 0x00;
  msgCDU3.data[7] = 0x00;

  msgCDU4.extd = 0;
  msgCDU4.rtr = 0;
  msgCDU4.ss = 0;
  msgCDU4.self = 0;
  msgCDU4.dlc_non_comp = 0;
  msgCDU4.reserved = 0;
  msgCDU4.data_length_code = 8;
  msgCDU4.identifier = 0x376; // CDU4
  msgCDU4.data[0] = 0x00;
  msgCDU4.data[1] = 0x00;
  msgCDU4.data[2] = 0x00;
  msgCDU4.data[3] = 0x00;
  msgCDU4.data[4] = 0x00;
  msgCDU4.data[5] = 0b11100111;
  msgCDU4.data[6] = 0x00;
  msgCDU4.data[7] = 0x00;

  msgSRS.extd = 0;
  msgSRS.rtr = 0;
  msgSRS.ss = 0;
  msgSRS.self = 0;
  msgSRS.dlc_non_comp = 0;
  msgSRS.reserved = 0;
  msgSRS.data_length_code = 8;
  msgSRS.identifier = 0x31D; // SRS
  msgSRS.data[0] = 0x00;
  msgSRS.data[1] = 0x00;
  msgSRS.data[2] = 0x00;
  msgSRS.data[3] = 0x00;
  msgSRS.data[4] = 0x00;
  msgSRS.data[5] = 0x00;
  msgSRS.data[6] = 0x00;
  msgSRS.data[7] = 0x00;

  msgSAS.extd = 0;
  msgSAS.rtr = 0;
  msgSAS.ss = 0;
  msgSAS.self = 0;
  msgSAS.dlc_non_comp = 0;
  msgSAS.reserved = 0;
  msgSAS.data_length_code = 8;
  msgSAS.identifier = 0x0C4; // SAS
  msgSAS.data[0] = 0x00;
  msgSAS.data[1] = 0x00;
  msgSAS.data[2] = 0x00;
  msgSAS.data[3] = 0b11100000;
  msgSAS.data[4] = 0x00;
  msgSAS.data[5] = 0x00;
  msgSAS.data[6] = 0x00;
  msgSAS.data[7] = 0x00;
  
  msgABS1.extd = 0;
  msgABS1.rtr = 0;
  msgABS1.ss = 0;
  msgABS1.self = 0;
  msgABS1.dlc_non_comp = 0;
  msgABS1.reserved = 0;
  msgABS1.data_length_code = 8;
  msgABS1.identifier = 0x311; // ABS1
  msgABS1.data[0] = 0x00;
  msgABS1.data[1] = 0b00000111;
  msgABS1.data[2] = 0x00;
  msgABS1.data[3] = 0b00000100;
  msgABS1.data[4] = 0x00;
  msgABS1.data[5] = 0x00;
  msgABS1.data[6] = 0x00;
  msgABS1.data[7] = 0x00;

  msgABS2.extd = 0;
  msgABS2.rtr = 0;
  msgABS2.ss = 0;
  msgABS2.self = 0;
  msgABS2.dlc_non_comp = 0;
  msgABS2.reserved = 0;
  msgABS2.data_length_code = 8;
  msgABS2.identifier = 0x2EA; // ABS2
  msgABS2.data[0] = 0x00;
  msgABS2.data[1] = 0b00000110;
  msgABS2.data[2] = 0x00;
  msgABS2.data[3] = 0x00;
  msgABS2.data[4] = 0x00;
  msgABS2.data[5] = 0x00;
  msgABS2.data[6] = 0x00;
  msgABS2.data[7] = 0x00;

  msgABS3.extd = 0;
  msgABS3.rtr = 0;
  msgABS3.ss = 0;
  msgABS3.self = 0;
  msgABS3.dlc_non_comp = 0;
  msgABS3.reserved = 0;
  msgABS3.data_length_code = 8;
  msgABS3.identifier = 0x313; // ABS3
  msgABS3.data[0] = 0b00000011;
  msgABS3.data[1] = 0x00;
  msgABS3.data[2] = 0x00;
  msgABS3.data[3] = 0x00;
  msgABS3.data[4] = 0x00;
  msgABS3.data[5] = 0x00;
  msgABS3.data[6] = 0x00;
  msgABS3.data[7] = 0x00;
  
  // Define message(s) to send
  this->setTransactionPeriodic(m_pMsgVCU1, 20);
  this->setTransactionPeriodic(&msgBMS1, msgBMS1Interval);
  this->setTransactionPeriodic(&msgBMS2, msgBMS2Interval);
  this->setTransactionPeriodic(&msgBMS3, msgBMS2Interval);
  this->setTransactionPeriodic(&msgBMS4, msgBMS2Interval);
  this->setTransactionPeriodic(&msgBMS8, msgBMS2Interval);
  this->setTransactionPeriodic(&msgICU1, msgICU1Interval);
  this->setTransactionPeriodic(&msgVCU2, msgVCU2Interval);
  this->setTransactionPeriodic(&msgCDU1, msgVCU2Interval);
  this->setTransactionPeriodic(&msgCDU2, msgVCU2Interval);
  this->setTransactionPeriodic(&msgCDU3, msgVCU2Interval);
  this->setTransactionPeriodic(&msgCDU4, msgVCU2Interval);
  this->setTransactionPeriodic(&msgSRS, msgSRSInterval);
  this->setTransactionPeriodic(&msgSAS, msgSASInterval);
  this->setTransactionPeriodic(&msgABS1, msgABS1Interval);
  this->setTransactionPeriodic(&msgABS2, msgABS2Interval);
  this->setTransactionPeriodic(&msgABS3, msgABS3Interval);
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
        if (!mainRelayConnected.getVal() && auxiliaryRelayConnected.getVal()) {
          msgBMS1.data[3] = 0x00;
        }
        else if (mainRelayConnected.getVal() && !auxiliaryRelayConnected.getVal()) {
          msgBMS1.data[3] = 0b11010000;
          msgBMS1.data[0] = 0b01110000;
        }
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
        if (keyPosition.getVal() < 2) {
          msgBMS1.data[3] = 0b11000000;
          msgBMS1.data[0] = 0x00;
        }
        break;
      case 115:
        setBits(&(m_pMsgVCU1->data[6]), 52%8, 1, (uint8_t)(auxiliaryRelayConnected.getVal()));
        if (!mainRelayConnected.getVal() && auxiliaryRelayConnected.getVal()) {
          msgBMS1.data[3] = 0x00;
        }
        else if (mainRelayConnected.getVal() && !auxiliaryRelayConnected.getVal()) {
          msgBMS1.data[3] = 0b11010000;
          msgBMS1.data[0] = 0b01110000;
        }
        break;
      default:
        break;
    }
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
      break;
  }
  
    // Update the rolling counter of send message
    if (millis() - m_msgVCU1UpdateTime > 20) {
      m_msgVCU1Counter ++;
      if (m_msgVCU1Counter > 0xf) m_msgVCU1Counter = 0;
      setBits(&(m_pMsgVCU1->data[6]), 48%8, 4, m_msgVCU1Counter);
      m_msgVCU1UpdateTime = millis();
    }
    
    // Set the checksum
    m_pMsgVCU1->data[7] = 
      (uint8_t)(m_pMsgVCU1->data[0] + m_pMsgVCU1->data[1]
      + m_pMsgVCU1->data[2] + m_pMsgVCU1->data[3]
      + m_pMsgVCU1->data[4] + m_pMsgVCU1->data[5]
      + m_pMsgVCU1->data[6]) ^ 0xFF;
    
    // Other messages
    if (millis() - msgBMS1Time > msgBMS1Interval) {
      msgBMS1Counter++;
      msgBMS1.data[6] = msgBMS1.data[6] + 1;
      if (msgBMS1.data[6] >= 0xf) {msgBMS1.data[6] = 0;}
      msgBMS1.data[7] = (uint8_t)(msgBMS1.data[0] + msgBMS1.data[1] + msgBMS1.data[2] + msgBMS1.data[3] + msgBMS1.data[4] + msgBMS1.data[5] + msgBMS1.data[6]) ^ 0xFF;
      msgBMS1Time = millis();
    }
    if (millis() - msgBMS2Time > msgBMS2Interval) {
      msgBMS2Counter++;
      msgBMS2.data[6] = msgBMS2.data[6] + 1;
      if (msgBMS2.data[6] >= 0xf) {msgBMS2.data[6] = 0;}
      msgBMS2.data[7] = (uint8_t)(msgBMS2.data[0] + msgBMS2.data[1] + msgBMS2.data[2] + msgBMS2.data[3] + msgBMS2.data[4] + msgBMS2.data[5] + msgBMS2.data[6]) ^ 0xFF;
      msgBMS2Time = millis();
    }
    if (millis() - msgBMS3Time > msgBMS3Interval) {
      msgBMS3Counter++;
      msgBMS3.data[6] = msgBMS3.data[6] + 1;
      if (msgBMS3.data[6] >= 0xf) {msgBMS3.data[6] = 0;}
      msgBMS3.data[7] = (uint8_t)(msgBMS3.data[0] + msgBMS3.data[1] + msgBMS3.data[2] + msgBMS3.data[3] + msgBMS3.data[4] + msgBMS3.data[5] + msgBMS3.data[6]) ^ 0xFF;
      msgBMS3Time = millis();
    }
    if (millis() - msgBMS4Time > msgBMS4Interval) {
      msgBMS4Counter++;
      msgBMS4.data[6] = msgBMS4.data[6] + 1;
      if (msgBMS4.data[6] >= 0xf) {msgBMS4.data[6] = 0;}
      msgBMS4.data[7] = (uint8_t)(msgBMS4.data[0] + msgBMS4.data[1] + msgBMS4.data[2] + msgBMS4.data[3] + msgBMS4.data[4] + msgBMS4.data[5] + msgBMS4.data[6]) ^ 0xFF;
      msgBMS4Time = millis();
    }
    if (millis() - msgBMS8Time > msgBMS8Interval) {
      msgBMS8Counter++;
      msgBMS8.data[6] = msgBMS8.data[6] + 1;
      if (msgBMS8.data[6] >= 0xf) {msgBMS8.data[6] = 0;}
      msgBMS8.data[7] = (uint8_t)(msgBMS8.data[0] + msgBMS8.data[1] + msgBMS8.data[2] + msgBMS8.data[3] + msgBMS8.data[4] + msgBMS8.data[5] + msgBMS8.data[6]) ^ 0xFF;
      msgBMS8Time = millis();
    }
    if (millis() - msgICU1Time > msgICU1Interval) {
      if (msgICU1Counter) {
        msgICU1.data[0] = 0b00100000;
        msgICU1Counter = 0;
      } else {
        msgICU1.data[0] = 0b00000000;
        msgICU1Counter = 1;
      }
      msgICU1Time = millis();
    }
    if (millis() - msgVCU2Time > msgVCU2Interval) {
      msgVCU2Counter++;
      msgVCU2.data[6] = msgVCU2.data[6] + 1;
      if (msgVCU2.data[6] >= 0xf) {msgVCU2.data[6] = 0;}
      msgVCU2.data[7] = (uint8_t)(msgVCU2.data[0] + msgVCU2.data[1] + msgVCU2.data[2] + msgVCU2.data[3] + msgVCU2.data[4] + msgVCU2.data[5] + msgVCU2.data[6]) ^ 0xFF;
      msgVCU2Time = millis();
    }
    if (millis() - msgCDU1Time > msgCDU1Interval) {
      msgCDU1Counter++;
      msgCDU1.data[6] = msgCDU1.data[6] + 1;
      if (msgCDU1.data[6] >= 0xf) {msgCDU1.data[6] = 0;}
      msgCDU1.data[7] = (uint8_t)(msgCDU1.data[0] + msgCDU1.data[1] + msgCDU1.data[2] + msgCDU1.data[3] + msgCDU1.data[4] + msgCDU1.data[5] + msgCDU1.data[6]) ^ 0xFF;
      msgCDU1Time = millis();
    }
    if (millis() - msgCDU2Time > msgCDU2Interval) {
      msgCDU2Counter++;
      msgCDU2.data[6] = msgCDU2.data[6] + 1;
      if (msgCDU2.data[6] >= 0xf) {msgCDU2.data[6] = 0;}
      msgCDU2.data[7] = (uint8_t)(msgCDU2.data[0] + msgCDU2.data[1] + msgCDU2.data[2] + msgCDU2.data[3] + msgCDU2.data[4] + msgCDU2.data[5] + msgCDU2.data[6]) ^ 0xFF;
      msgCDU2Time = millis();
    }
    if (millis() - msgCDU3Time > msgCDU3Interval) {
      msgCDU3Counter++;
      msgCDU3.data[6] = msgCDU3.data[6] + 1;
      if (msgCDU3.data[6] >= 0xf) {msgCDU3.data[6] = 0;}
      msgCDU3.data[7] = (uint8_t)(msgCDU3.data[0] + msgCDU3.data[1] + msgCDU3.data[2] + msgCDU3.data[3] + msgCDU3.data[4] + msgCDU3.data[5] + msgCDU3.data[6]) ^ 0xFF;
      msgCDU3Time = millis();
    }
    if (millis() - msgCDU4Time > msgCDU4Interval) {
      msgCDU4Counter++;
      msgCDU4.data[6] = msgCDU4.data[6] + 1;
      if (msgCDU4.data[6] >= 0xf) {msgCDU4.data[6] = 0;}
      msgCDU4.data[7] = (uint8_t)(msgCDU4.data[0] + msgCDU4.data[1] + msgCDU4.data[2] + msgCDU4.data[3] + msgCDU4.data[4] + msgCDU4.data[5] + msgCDU4.data[6]) ^ 0xFF;
      msgCDU4Time = millis();
    }
    if (millis() - msgSRSTime > msgSRSInterval) {
      msgSRSCounter++;
      msgSRS.data[6] = ((msgSRS.data[6]>>2 + 1) & 0x0F)<<2;
      uint8_t temp = 0x0 ^ 0x0 ^ 0x0 ^ 0x0 ^ 0x0 ^ 0x0 ^ 0x0;
      msgSRS.data[7] = ((temp>>4) ^ (temp&0x0F) ^ (msgSRS.data[6]>>2)) & 0x0F;
      msgSRSTime = millis();
    }
    if (millis() - msgSASTime > msgSASInterval) {
      msgSASCounter++;
      msgSAS.data[6] = msgSAS.data[6] + 1;
      if (msgSAS.data[6] >= 0xf) {msgSAS.data[6] = 0;}
      msgSAS.data[7] = (uint8_t)(msgSAS.data[0] + msgSAS.data[1] + msgSAS.data[2] + msgSAS.data[3] + msgSAS.data[4] + msgSAS.data[5] + msgSAS.data[6]) ^ 0xFF;
      msgSASTime = millis();
    }
    if (millis() - msgABS1Time > msgABS1Interval) {
      msgABS1Counter++;
      msgABS1.data[6] = msgABS1.data[6] + 1;
      if (msgABS1.data[6] >= 0xf) {msgABS1.data[6] = 0;}
      msgABS1.data[7] = (uint8_t)(msgABS1.data[0] + msgABS1.data[1] + msgABS1.data[2] + msgABS1.data[3] + msgABS1.data[4] + msgABS1.data[5] + msgABS1.data[6]) ^ 0xFF;
      msgABS1Time = millis();
    }
    if (millis() - msgABS2Time > msgABS2Interval) {
      msgABS2Counter++;
      msgABS2.data[6] = msgABS2.data[6] + 1;
      if (msgABS2.data[6] >= 0xf) {msgABS2.data[6] = 0;}
      msgABS2.data[7] = (uint8_t)(msgABS2.data[0] + msgABS2.data[1] + msgABS2.data[2] + msgABS2.data[3] + msgABS2.data[4] + msgABS2.data[5] + msgABS2.data[6]) ^ 0xFF;
      msgABS2Time = millis();
    }
    if (millis() - msgABS3Time > msgABS3Interval) {
      msgABS3Counter++;
      msgABS3.data[6] = msgABS3.data[6] + 1;
      if (msgABS3.data[6] >= 0xf) {msgABS3.data[6] = 0;}
      msgABS3.data[7] = (uint8_t)(msgABS3.data[0] + msgABS3.data[1] + msgABS3.data[2] + msgABS3.data[3] + msgABS3.data[4] + msgABS3.data[5] + msgABS3.data[6]) ^ 0xFF;
      msgABS3Time = millis();
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
