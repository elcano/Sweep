/*
 *  CAN commands for remote control
 *  Tyler Folsom    Nov. 10 2019
 *  CAN commands are only used by the Receiver, which transmits data to Drive-By-Wire.
 */
#include <mcp_can_dfs.h>    // <---- Import from another library: Seeed-Studio/CAN-BUS-Shield
#include <mcp_can.h>        //       (install Library from Arduino, search for "Can-Bus-Shield")
#define DEBUG 1  
#include "RadioControl_rf69.h" 
#include "CAN_Remote.h"      

//MCP_CAN CAN(3);            // chip selection pin for CAN. 53 for mega, 49 for our new low level board
MCP_CAN CAN(13); 
// Inital CAN bus with 500KBPS baud rate (CAN_500KBPS is the baud rate)
void CAN_Init()
{
    while (CAN_OK != CAN.begin(CAN_500KBPS))              // init can bus : baudrate = 500k
    {
      if (DEBUG)
      {
        Serial.println("CAN BUS Shield init fail");
        Serial.println(" Init CAN BUS Shield again");
        delay(100);
      }
    }
    if (DEBUG)
    {
     Serial.println("CAN BUS Shield init OK!");
    }
}

// Receive data from the other high/low level board
// NOTE: This function is not tested yet...
void receiveFromCanBus()
{
    unsigned char len = 0;      // message length
    unsigned char msgBuffer[8]; //8 Bytes buffer to store CAN message
    unsigned int canID = 0;

    // Check if received anything
    if (CAN_MSGAVAIL == CAN.checkReceive())
    {
        CAN.readMsgBuf(&len, msgBuffer); // put the data read into buffer and length
        canID = CAN.getCanId();

        if (canID == HiDrive_CANID) 
        /* <--- Change the CAN ID according to where this receiver 
        transmitted the CAN message to. */
        {
            if (DEBUG)
            {
                Serial.print("HiDrive_CANID received: ");
                Serial.println(canID, HEX);
            }
        }
        else if (canID == HiStatus_CANID)
        {
            if (DEBUG)
            {
                Serial.print("HiStatus_CANID received: ");
                Serial.println(canID, HEX);
            }
        }
        else if (canID == RCStatus_CANID)
        {
            if (DEBUG)
            {
                Serial.print("RCStatus_CANID received: ");
                Serial.println(canID, HEX);
            }
        }
        else if (canID == LowStatus_CANID)
        {
            if (DEBUG)
            {
                Serial.print("LowStatus_CANID received: ");
                Serial.println(canID, HEX);
            }
        }
        else if (canID == RCDrive_CANID)
        {
            if (DEBUG)
            {
                Serial.print("RCDrive_CANID received: ");
                Serial.println(canID, HEX);
            }
        }
        else if (canID == Actual_CANID)
        {
            if (DEBUG)
            {
                Serial.print("Actual_CANID received: ");
                Serial.println(canID, HEX);
            }
        }
        else
        {
            if (DEBUG)
            {
                Serial.print("Unexpected CAN ID received: ");
                Serial.println(canID, HEX);
            }
        }

        // Process incoming data (This step should be in the if/else statement above according to each if/else...)
        int resultFromCanBUS = (unsigned int)(msgBuffer[3] << 24) | (msgBuffer[2] << 16) | (msgBuffer[1] << 8) | (msgBuffer[0]);
        if (DEBUG)
        {
            Serial.print("ACK msg from CAN BUS: ");
            Serial.println(resultFromCanBUS, DEC);
            Serial.println("Message received from the CAN BUS! Finished...");
        }
    }
}
// Process the data over the CAN BUS protocal
// NOTE: This function is not tested yet...
void sendToCanBus(DataFromTransmitter message)
{
    DataFromTransmitter buttons; 
    buttons.bytes[0] = 0;
    if (message.ebrake)
        buttons.bytes[0] |= EStop_BIT;
    if (message.autonomous)
        buttons.bytes[0] |= Auto_BIT;  
    if (DEBUG)
    {
        Serial.println("Sending data to CAN BUS...");
    }

    // send CAN message to CAN BUS
    CAN.sendMsgBuf(RCStatus_CANID, 0, 1, (uint8_t *)&buttons);
    
   // discrepancy between CAN definition (6 bytes, Throttle, Brake, Steer)
   // and radio data  (4 bytes, Throttle, Steer)
   
    CAN.sendMsgBuf(RCDrive_CANID, 0, 4, (uint8_t *)&message);

    delay(100); // a proper delay here is necessay, CAN bus need a time to clear the buffer. delay could be 100 minimum

    if (DEBUG)
    {
        Serial.println("Messages SENT!");
    }
}
