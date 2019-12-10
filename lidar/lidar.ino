/*
   Sweep Obstacle Detector
   by B. Johnson

   modified 5/8/2019 JB: removed code not required for testing lidar
    added angle limit functionality
    added scan segment functionality
    

   Runs on Arduino Micro connected to Scanse Sweep;
   For best results, Scanse Sweep should have a 400mA, 5V power source
   connect power, synch, tx, rx, and ground wires before beginning
   Serial1 (connection to Sweep) uses pins 0 and 1

   0 or 360 degrees aligns with the Sweep LED, so (x= -1, y= 0) or 180 degrees is straight forward
*/

/************************** LEO Added **********************/
#include <mcp_can.h>      // <---- Import from another library: Seeed-Studio/CAN-BUS-Shield
#include <mcp_can_dfs.h>  //       (install Library from Arduino, search for "Can-Bus-Shield")
#include <Can_Protocol.h> // <---- Import from path: elcano/Elcano_C2_LowLevel/Can_Protocol.h
/*************************** END *****************************/

#include <Sweep.h> // https://github.com/scanse/sweep-arduino/tree/master/Sweep

const uint8_t POWER_PIN = 2; // Power Enable pin
const uint8_t SYNC_PIN = 3;  // Sync / Device Ready pin

const uint32_t SERIAL_BAUD_RATE = 115200; // Baud rate for Serial. 14,400 bytes/s, 14.4 bytes/ms

const uint8_t MOTOR_SPEED_HZ = 1;                               // Number of rotations per second
const uint8_t (&MOTOR_SPEED_CODE)[2] = MOTOR_SPEED_CODE_1_HZ;   // Number of rotations per second
const uint16_t SAMPLE_RATE_HZ = 750;                            // Number of samples per second
const uint8_t (&SAMPLE_RATE_CODE)[2] = SAMPLE_RATE_CODE_750_HZ; // Number of samples per second

const uint8_t NOISE_FLOOR = 50;                                              // adjust this to filter garbage readings
const uint8_t NUM_FRACTIONAL_BITS = 4;                                       // Number of fractional bits in raw angle
const uint16_t RIGHT_ANGLE_BOUNDARY = 1;                                     // Right boundary angle of scan range
const uint16_t R_ANGLE_FIXEDP = RIGHT_ANGLE_BOUNDARY << NUM_FRACTIONAL_BITS; // use this to test "raw angle" value lower boundary
const uint16_t LEFT_ANGLE_BOUNDARY = 359;                                    // Left boundary angle of scan range
const uint16_t L_ANGLE_FIXEDP = LEFT_ANGLE_BOUNDARY << NUM_FRACTIONAL_BITS;  // use this to test "raw angle" value upper boundary

const uint16_t MAX_DELTA = 15; // max change per scan segment in cm; values greater than this will start a new segment

/*************************** LEO Added *************************/
// 8 bits = 1 byte
// uint16_t = unsigned int or int
// uint8_t = unsigned char or char
typedef union {
    struct
    {
        uint16_t range;
        int bearing;
        uint16_t width;
        uint8_t quality;
        int slant;
    };
    uint8_t bytes[9]; // around 9 or 10 bytes depending on the struct above
} Lidar;

Lidar lidar;

MCP_CAN CAN(49); // chip selection pin for CAN. 53 for mega, 49 for our new low level board
/*************************** LEO END ******************************/

#define DEBUG true // do not use Serial to print any text during scan. Printing too much will cause this code to miss new scans

// Sweep sensor
Sweep sweep(Serial1);

void enablePower(void)
{
    digitalWrite(POWER_PIN, HIGH);
    if (DEBUG)
    {
        Serial.println(F("pwr on"));
    }
}

void disablePower(void)
{
    sweep.stopScanning();
    digitalWrite(POWER_PIN, LOW);
    if (DEBUG)
    {
        Serial.println(F("pwr off"));
    }
}

void prettyPrintRect(uint16_t myAngle, uint16_t myDistance)
{
    int angle = myAngle * pow(2, -(NUM_FRACTIONAL_BITS));
    int x = myDistance * cos(radians(angle));
    int y = myDistance * sin(radians(angle));
    Serial.print(String(x) + ",");
    Serial.println(String(y));
    Serial.flush();
}

void prettyPrint(uint16_t myAngle, uint16_t myDistance)
{
    int tenthsDegree = int(myAngle * pow(2, -(NUM_FRACTIONAL_BITS)) * 10);
    Serial.print(String(tenthsDegree) + ",");
    Serial.println(myDistance);
    Serial.flush();
}

void sendToCanBus(Lidar lidarStruct)
{
    if (DEBUG)
    {
        Serial.println("Sending data to CAN BUS...");
    }

    // send CAN message to CAN BUS
    CAN.sendMsgBuf(Actual_CANID, 0, 8, (uint8_t *)&lidarMSGBuffer);
    delay(1000); // This code might not be needed since it is too long for the buffer to be cleared...

    Serial.println("Messages SENT!");
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

        if (canID = HiDrive_CANID) // <--- Change the CAN ID according to where this receiver transmitted the CAN message to.
        {
            Serial.print("HiDrive_CANID received: ");
            Serial.println(canID, HEX);
        }
        else if (canID == HiStatus_CANID)
        {
            Serial.print("HiStatus_CANID received: ");
            Serial.println(canID, HEX);
        }
        else if (canID == RCStatus_CANID)
        {
            Serial.print("RCStatus_CANID received: ");
            Serial.println(canID, HEX);
        }
        else if (canID == LowStatus_CANID)
        {
            Serial.print("LowStatus_CANID received: ");
            Serial.println(canID, HEX);
        }
        else if (canID == RCDrive_CANID)
        {
            Serial.print("RCDrive_CANID received: ");
            Serial.println(canID, HEX);
        }
        else if (canID == Actual_CANID)
        {
            Serial.print("Actual_CANID received: ");
            Serial.println(canID, HEX);
        }
        else
        {
            Serial.print("Unexpected CAN ID received: ");
            Serial.println(canID, HEX);
        }

        // Process incoming data (This step should be in the if/else statement above according to each if/else...)
        int resultFromCanBUS = (unsigned int)(msgBuffer[3] << 24) | (msgBuffer[2] << 16) | (msgBuffer[1] << 8) | (msgBuffer[0]);
        Serial.print("ACK msg from CAN BUS: ");
        Serial.println(resultFromCanBUS, DEC);
        Serial.println("Message received from the CAN BUS! Finished...");
    }
}

void setup()
{
    long setupStart = millis();
    // Serial
    Serial.begin(SERIAL_BAUD_RATE);
    // Serial1
    Serial1.begin(SERIAL_BAUD_RATE);

    // Leo: Inital CAN bus with 500KBPS baud rate (CAN_500KBPS is the baud rate)
    while (CAN_OK != CAN.begin(CAN_500KBPS))
    {
        if (DEBUG)
        {
            Serial.println("CAN BUS Shield init fail!!!");
            Serial.println("Re-initializing...");
        }
        delay(100);
    }
    if (DEBUG)
    {
        Serial.println("CAN BUS Shield init OK!");
    }
    // Leo: END

    // Power Enable pin
    pinMode(POWER_PIN, OUTPUT);
    enablePower();
    // Sync / Device Ready pin
    pinMode(SYNC_PIN, INPUT);
    sweep.reset();
    if (DEBUG)
    {
        Serial.println(F("reset"));
    }
    // Wait until motor speed has stabilized
    if (!sweep.getMotorReady())
    {
        sweep.waitUntilMotorReady();
    }
    if (DEBUG)
    {
        Serial.println(F("motor on"));
    }
    // Set motor speed
    if (sweep.getMotorSpeed() != MOTOR_SPEED_HZ)
    {
        sweep.setMotorSpeed(MOTOR_SPEED_CODE);
    }
    // Wait until motor speed has stabilized
    if (!sweep.getMotorReady())
    {
        sweep.waitUntilMotorReady();
    }
    if (DEBUG)
    {
        Serial.println(F("motor ready"));
    }
    // Set sample rate
    if (sweep.getSampleRate() != SAMPLE_RATE_HZ)
    {
        sweep.setSampleRate(SAMPLE_RATE_CODE);
    }
    // Begin taking readings
    if (!sweep.isScanning())
    {
        sweep.startScanning();
    }
    if (DEBUG)
    {
        Serial.println(F("laser scanning!"));
    }
    if (DEBUG)
    {
        Serial.println("motorHz:" + String(MOTOR_SPEED_HZ));
    }
    if (DEBUG)
    {
        Serial.println("scanHz:" + String(SAMPLE_RATE_HZ));
    }
    int duration = (millis() - setupStart) / 1000.0;
    if (DEBUG && (duration >= 1))
    {
        Serial.println("Setup (s): " + String(duration));
    }
    bool sync = false;
    while (!sync)
    {
        // wait until 0 degrees so we begin on a new scan
        sync = digitalRead(SYNC_PIN);
    }
}

uint16_t segmentStart[2];
uint16_t segmentEnd[2];

void loop()
{
    static bool scanFinished = true;
    static bool newSegment = true;
    bool hadReading = false;
    bool insideScanAngles = false;
    bool goodSignal = false;
    // Get reading if available
    ScanPacket reading = sweep.getReading(hadReading);
    uint16_t angle = reading.getAngleRaw();
    uint16_t signalStrength = reading.getSignalStrength();
    uint16_t distance = reading.getDistanceCentimeters();

    if (angle >= R_ANGLE_FIXEDP && angle <= L_ANGLE_FIXEDP)
    {
        insideScanAngles = true;
    }
    if (signalStrength > NOISE_FLOOR && distance > 1 && distance < 4000)
    {
        goodSignal = true;
    }

    if (hadReading && goodSignal && insideScanAngles)
    {
        // we are actively scanning
        scanFinished = false;
        if (newSegment)
        {
            // set segment end and start to current scan
            segmentStart[0] = angle;
            segmentEnd[0] = angle;
            segmentStart[1] = distance;
            segmentEnd[1] = distance;
            prettyPrintRect(angle, distance);
            newSegment = false;
        }
        else
        {
            // update the segment end data
            segmentEnd[0] = angle;
            segmentEnd[1] = distance;
        }
        // now test delta
        int delta = segmentEnd[1] - segmentStart[1];
        if (abs(delta) > MAX_DELTA)
        {
            // we add data pairs as a segment and reset "newSegment"
            prettyPrintRect(angle, distance);
            newSegment = true;
        }
    }
    if (hadReading && !insideScanAngles && !scanFinished)
    {
        // we have a data reading
        // we are out of the scan angles
        // we did not send scan segments
        // this means we are at the end of the scan and can transmit data
        if (!newSegment)
        {
            // we are not at the end of a segment, need to end it
            prettyPrintRect(angle, distance);
            newSegment = true;
        }
        // send scan data here
        scanFinished = true;
    }
}