/* ============================== test CAN messages ======================= 
 * Use an Arduino with MCP2515 to read and write CAN bus messages
 */

 
#include <SPI.h>
#include <mcp2515.h>		// MCP2515 driver

// Set up the CAN interface
MCP2515 CAN(10);			// Set SPI Chip Select

// Create variables to store CAN messages
struct can_frame msgIn;
struct can_frame msgOut1;
struct can_frame msgOut2;

// timers for outgoing messages
long msg1Time = 0;
long msg2Time = 0;
int msg1Interval, msg2Interval;
int msg1Counter, msg2Counter;

// Interactive pins
const int PIN_IRQ = 2;

// flags for connection of contactors
bool mainConn = false;
bool auxConn = false;

// Interrupt handler
bool flagIncoming = false;
void irqHandler() {
  flagIncoming = true;
}


// Print a CAN message
void printMessage(uint32_t id, uint8_t len, uint8_t *frame) {
  Serial.print("0x");
	Serial.print(String(id, HEX));
  Serial.print(" ");
	for(int i=0; i<len; i++) {
		Serial.print(String(frame[i], HEX));
    Serial.print(" ");
	}
	Serial.println("");
}


void setup() {
	// Set digital pins
  pinMode(PIN_IRQ, INPUT_PULLUP); // Interrupt from MCP2515

  Serial.begin(115200);
  SPI.begin();

  // Init the interrupt handler
  attachInterrupt(digitalPinToInterrupt(PIN_IRQ), irqHandler, FALLING);
	
	// Start the CAN bus communication
	Serial.println("Starting the MCP2515...");
	CAN.reset();
  CAN.setBitrate(CAN_500KBPS, MCP_8MHZ);
  CAN.setNormalMode();
	Serial.println("Done.");

  // Init the outgoing messages
  msg1Interval = 20;
  msgOut1.can_id  = 0x1A0; // BMS
  msgOut1.can_dlc = 8;
  msgOut1.data[0] = 0x00; // warning level, status, main contactors
  msgOut1.data[1] = 0x00; // SOC
  msgOut1.data[2] = 0x00; // SOH
  msgOut1.data[3] = 0x00; // AC, precharge relays, mode
  msgOut1.data[4] = 0x00; // max discharge (14 bits)
  msgOut1.data[5] = 0x00; // max discharge continued, contactor adherence
  msgOut1.data[6] = 0x00; // alarms and rolling counter
  msgOut1.data[7] = 0x00; // checksum

  msg2Interval = 50;
  msgOut2.can_id  = 0x431; // park break
  msgOut2.can_dlc = 8;
  msgOut2.data[0] = 0x00; // park break
  msgOut2.data[1] = 0x00;
  msgOut2.data[2] = 0x00;
  msgOut2.data[3] = 0x00;
  msgOut2.data[4] = 0x00;
  msgOut2.data[5] = 0x00;
  msgOut2.data[6] = 0x00;
  msgOut2.data[7] = 0x00;
}


// Check the bus in an infinite loop
void loop() {

  // Read message
  // Interrupt-based
  /*if (flagIncoming) {//CAN.checkReceive()) {
    flagIncoming = false;
    uint8_t irq = CAN.getInterrupts();

    if (irq & MCP2515::CANINTF_RX1IF) {
      MCP2515::ERROR err = CAN.readMessage(MCP2515::RXB1, &msgIn);
    	if (MCP2515::ERROR_OK == err) {
        printMessage(msgIn.can_id, msgIn.can_dlc, msgIn.data);
    	}
    }

    if (irq & MCP2515::CANINTF_RX0IF) {
      MCP2515::ERROR err = CAN.readMessage(MCP2515::RXB0, &msgIn);
    	if (MCP2515::ERROR_OK == err) {
        printMessage(msgIn.can_id, msgIn.can_dlc, msgIn.data);
    	}
    }
  }*/
  // Polling
  MCP2515::ERROR err = CAN.readMessage(&msgIn);
  if (MCP2515::ERROR_OK == err) {
    //printMessage(msgIn.can_id, msgIn.can_dlc, msgIn.data);
    if (msgIn.can_id == 0x101) {

      // Check contactor status
      if ((msgIn.data[4] >> 3) & 1 == 1) {
        // main contactor connected
        mainConn = true;
      } else {
        mainConn = false;
      }
      if ((msgIn.data[6] >> 4) & 1 == 1) {
        // precharge contactor connected
        auxConn = true;
      } else {
        auxConn = false;
      }

      // Now set the precharge message accordingly
      if (!mainConn && auxConn) {
        msgOut1.data[3] = 0x00;
        Serial.println("Precharge not finished");
      }
      else if (mainConn && !auxConn) {
        msgOut1.data[3] = 0x01 << 4;
        Serial.println("Precharge finished");
      }

      // Reset precharge message when key position is acc
      if ((msgIn.data[5] >> 6 ) & 0x3 < 2) {
        msgOut1.data[3] = 0x00;
        Serial.println("Precharge not finished");
      }
    }
  }

  // Write
  if (millis() - msg1Time > msg1Interval) {
    msg1Counter++;
    msgOut1.data[6] = msgOut1.data[6] + 1;
    if (msgOut1.data[6] > 0xf) {msgOut1.data[6] = 0;}
    msgOut1.data[7] = (uint8_t)((msgOut1.data[0] + msgOut1.data[1] + msgOut1.data[2] + msgOut1.data[3] + msgOut1.data[4] + msgOut1.data[5] + msgOut1.data[6]) ^ 0xFF );
    CAN.sendMessage(&msgOut1);
    msg1Time = millis();
  }
  if (millis() - msg2Time > msg2Interval) {
    CAN.sendMessage(&msgOut2);
    msg2Time = millis();
  }
}
