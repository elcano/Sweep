/*
 *  CAN commands for remote control
 *  Tyler Folsom    Nov. 10 2019
 *  CAN commands are only used by the Receiver, which transmits data to Drive-By-Wire.
 */
 #ifndef CAN_REMOTE_H


  #define CAN_REMOTE_H

//   Origin: Receiver; Function: E-stop, auto, reverse
#define RCStatus_CANID 0x50
#define EStop_BIT          0x80
#define Auto_BIT           0x40
#define ReverseActive_BIT  0x04
#define ReversePending_BIT 0x02
#define NoReverse_BIT      0x01
//   Origin: Hi-Level; Function: E-stop, auto, reverse
#define HiStatus_CANID 0x100
//   Origin: Hi-Level; Function: goal reached
#define HiGoal_CANID 0x101
//   Origin: Drive-By-Wire; Function: E-stop, auto, reverse
#define LowStatus_CANID 0x200
//   Origin: Receiver; Function: Throttle, Brake, Steer
#define RCDrive_CANID 0x300
//   Origin: Hi-Level; Function: Throttle, Brake, Steer
#define HiDrive_CANID 0x350
//   Origin: Drive-By-Wire; Function: Actual speed
#define Actual_CANID 0x400

  void CAN_Init(void);
  void sendToCanBus(DataFromTransmitter txData); // Send the whole data struct to Can Bus
  void receiveFromCanBus(void);
#endif
