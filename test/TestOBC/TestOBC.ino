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

// Interactive pins
const int PIN_PP = 7;   // Proximity pilot
const int PIN_IRQ = 2;

// Interrupt handler
bool flagIncoming = false;
void irqHandler() {
  flagIncoming = true;
}


// Print a CAN message
void printMessage(uint32_t id, uint8_t len, uint8_t *frame) {
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
  pinMode(PIN_PP, INPUT_PULLUP);  // Proximity pilot
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
  msg1Interval = 100;
  msgOut1.can_id  = 0x285;
  msgOut1.can_dlc = 8;
  msgOut2.data[0] = 0x00;
  msgOut2.data[1] = 0x00;
  msgOut2.data[2] = 0xb6; // pulls in EVSE
  msgOut1.data[3] = 0x00;
  msgOut1.data[4] = 0x00; // looks like a heartbeat signal incrementing every second
  msgOut1.data[5] = 0x00;
  msgOut1.data[6] = 0x00;
  msgOut1.data[7] = 0x00;

  msg2Interval = 100;
  msgOut2.can_id  = 0x286;
  msgOut2.can_dlc = 8;
  msgOut1.data[0] = 0x0E;
  msgOut1.data[1] = 0xD8; // 380V (Big Endian e.g. 0x0E 0x74 = 3700 = 370v)
  msgOut1.data[2] = 0x32; // 5A (Amps times 10)
  msgOut2.data[3] = 0x00;
  msgOut2.data[4] = 0x00;
  msgOut2.data[5] = 0x00;
  msgOut2.data[6] = 0x00;
  msgOut2.data[7] = 0x00;
}


// Check the bus in an infinite loop
void loop() {

  // Read message
  if (flagIncoming) {//CAN.checkReceive()) {
    flagIncoming = false;
    uint8_t irq = CAN.getInterrupts();

    if (irq & MCP2515::CANINTF_RX1IF) {
      MCP2515::ERROR err = CAN.readMessage(MCP2515::RXB1, &msgIn);
      
      if (err != MCP2515::ERROR_NOMSG && err != MCP2515::ERROR_OK) {
        Serial.println("CAN bus error: " + String(err));
        delay(400);
      }
      
    	if (MCP2515::ERROR_OK == err) {
        printMessage(msgIn.can_id, msgIn.can_dlc, msgIn.data);
    	}
    }
  }

  // Write messages
  if (millis() - msg1Time > msg1Interval) {
    CAN.sendMessage(&msgOut1);
  }

  if (digitalRead(PIN_PP) == LOW) {
    if (millis() - msg2Time > msg2Interval) {
      CAN.sendMessage(&msgOut2);
    }
  }
}
