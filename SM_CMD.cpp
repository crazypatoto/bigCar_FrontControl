#include <Arduino.h>
#include <FlexCAN_T4.h>
//#include "kinetis_flexcan.h"
#include "SM_CMD.h"

static CAN_message_t msg;
extern FlexCAN_T4<CAN1, RX_SIZE_32, TX_SIZE_16> Can1;
extern FlexCAN_T4<CAN2, RX_SIZE_32, TX_SIZE_16> Can2;


void CANBUS_BEGIN(void) {
  Can1.begin();
  Can1.setBaudRate(1000000);
  Can1.setMaxMB(16);
  Can1.setMB(MB8, TX, EXT);
  Can1.setBaudRate(1000000);
  Can2.begin();
  Can2.setBaudRate(1000000);
  Can2.setMaxMB(16);
  Can2.setMB(MB8, TX, EXT);
  Can2.setBaudRate(1000000);
}

void CANBUS_TX(void) {
  Can2.write(MB8, msg);

  delayMicroseconds(5);
}
void CANBUS1_TX(void){
    Can1.write(MB8, msg);

  delayMicroseconds(5);
}
void MessengeStructSetting(void) {
  msg.flags.extended = 1;

}

void Commant_MP(uint8_t MOTOR_NUMBER) {
  int64_t x = 0x1400000000;
  MessengeStructSetting();
  msg.id = 0x09f80402 | (MOTOR_NUMBER << 2);
  msg.len = 6;
  for (int i = 0; i < msg.len; i++) {
    msg.buf[i] = (x & 0xff);
    x = x >> 8;
  }
  CANBUS_TX();
}

void Commant_MV(uint8_t MOTOR_NUMBER) {
  int64_t x = 0x1500000000;
  MessengeStructSetting();
  msg.id = 0x09f80402 | (MOTOR_NUMBER << 2);
  msg.len = 6;
  for (int i = 0; i < msg.len; i++) {
    msg.buf[i] = (x & 0xff);
    x = x >> 8;
  }
  CANBUS_TX();
}

void Commant_MT(uint8_t MOTOR_NUMBER) {
  int64_t x = 0x1600000000;
  MessengeStructSetting();
  msg.id = 0x09f80402 | (MOTOR_NUMBER << 2);
  msg.len = 6;
  for (int i = 0; i < msg.len; i++) {
    msg.buf[i] = (x & 0xff);
    x = x >> 8;
  }
  CANBUS_TX();
}

void Commant_T(uint8_t MOTOR_NUMBER, int32_t T) {
  MessengeStructSetting();
  msg.id = 0x08b80402 | (MOTOR_NUMBER << 2);
  msg.len = 6;
  for (int i = 0; i < msg.len; i++) {
    msg.buf[i] = (T & 0xff);
    T = T >> 8;
  }
  CANBUS_TX();
}

double turnAngleTOpulse( double PTa) {
  return PTa / 360.0 * 4000.0 * 100;
  //A/360*400000;
}

void Commant_PTa(uint8_t MOTOR_NUMBER, double PTa) {
  double PT;
  if (MOTOR_NUMBER == 4)PT = PTa;
  else PT = turnAngleTOpulse(PTa);
  int PT_int = (int)PT;
  MessengeStructSetting();
  msg.id = 0x08100402 | (MOTOR_NUMBER << 2);
  msg.len = 6;
  for (int i = 0; i < msg.len; i++) {
    msg.buf[i] = (PT_int & 0xff);
    PT_int = PT_int >> 8;;
  }
  CANBUS_TX();
}

void Commant_PT(uint8_t MOTOR_NUMBER, int32_t PT) {
  MessengeStructSetting();
  msg.id = 0x08100402 | (MOTOR_NUMBER << 2);
  msg.len = 6;
  for (int i = 0; i < msg.len; i++) {
    msg.buf[i] = (PT & 0xff);
    PT = PT >> 8;
  }
  CANBUS_TX();
}

void Commant_VT(uint8_t MOTOR_NUMBER, int32_t VT) {
  MessengeStructSetting();
  msg.id = 0x08280402 | (MOTOR_NUMBER << 2);
  msg.len = 6;
  for (int i = 0; i < msg.len; i++) {
    msg.buf[i] = (VT & 0xff);
    VT = VT >> 8;
  }
  CANBUS_TX();
}

void Commant_ADT(uint8_t MOTOR_NUMBER, int32_t ADT) {
  MessengeStructSetting();
  msg.id = 0x08400402 | (MOTOR_NUMBER << 2);
  msg.len = 6;
  for (int i = 0; i < msg.len; i++) {
    msg.buf[i] = (ADT & 0xff);
    ADT = ADT >> 8;
  }
  CANBUS_TX();
}

void Commant_PRT(uint8_t MOTOR_NUMBER, int32_t PRT) {
  MessengeStructSetting();
  msg.id = 0x08680402 | (MOTOR_NUMBER << 2);
  msg.len = 6;
  for (int i = 0; i < msg.len; i++) {
    msg.buf[i] = (PRT & 0xff);
    PRT = PRT >> 8;
  }
  CANBUS_TX();
}

double turnAngle2pulse( double PRTa) {
  return PRTa / 360.0 * 400000.0;
}

void Commant_PRTa(uint8_t MOTOR_NUMBER, double PRTa) {
  double PRT = turnAngle2pulse(PRTa);
  int PRT_int = (int)PRT;
  MessengeStructSetting();
  msg.id = 0x08680402 | (MOTOR_NUMBER << 2);
  msg.len = 6;
  for (int i = 0; i < msg.len; i++) {
    msg.buf[i] = (PRT_int & 0xff);
    PRT_int = PRT_int >> 8;
  }
  CANBUS_TX();
}

void Commant_G(uint8_t MOTOR_NUMBER) {
  int64_t x = 0x200000000;
  MessengeStructSetting();
  msg.id = 0x09f80402 | (MOTOR_NUMBER << 2);
  msg.len = 6;
  for (int i = 0; i < msg.len; i++) {
    msg.buf[i] = (x & 0xff);
    x = x >> 8;
  }
  CANBUS_TX();
}

void Commant_X(uint8_t MOTOR_NUMBER) {
  int64_t x = 0x300000000;
  MessengeStructSetting();
  msg.id = 0x09f80402 | (MOTOR_NUMBER << 2);
  msg.len = 6;
  for (int i = 0; i < msg.len; i++) {
    msg.buf[i] = (x & 0xff);
    x = x >> 8;
  }
  CANBUS_TX();
}

void Commant_S(uint8_t MOTOR_NUMBER) {
  int64_t x = 0x400000000;
  MessengeStructSetting();
  msg.id = 0x09f80402 | (MOTOR_NUMBER << 2);
  msg.len = 6;
  for (int i = 0; i < msg.len; i++) {
    msg.buf[i] = (x & 0xff);
    x = x >> 8;
  }
  CANBUS_TX();
}

void Commant_O(uint8_t MOTOR_NUMBER) {
  int64_t x = 0x00;
  MessengeStructSetting();
  msg.id = 0x08c00402 | (MOTOR_NUMBER << 2);
  msg.len = 6;
  for (int i = 0; i < msg.len; i++) {
    msg.buf[i] = (x & 0xff);
    x = x >> 8;
  }
  CANBUS_TX();
}

void PositionMode(uint8_t MOTOR_NUMBER, int32_t VT, uint16_t ADT, double PTa) {
  Commant_VT(MOTOR_NUMBER, VT);
  Commant_ADT(MOTOR_NUMBER, ADT);
  Commant_PTa(MOTOR_NUMBER, PTa);
  Commant_MP(MOTOR_NUMBER);
  Commant_G(MOTOR_NUMBER);
}

void PositionModeSet(uint8_t MOTOR_NUMBER, double PTa) {
  //Commant_PRTa(MOTOR_NUMBER,PRTa);
  Commant_PTa(MOTOR_NUMBER, PTa);
  Commant_MP(MOTOR_NUMBER);
  Commant_G(MOTOR_NUMBER);
}

void VelocityMode(uint8_t MOTOR_NUMBER, int32_t VT, uint16_t ADT) {
  Commant_VT(MOTOR_NUMBER, VT);
  Commant_ADT(MOTOR_NUMBER, ADT);
  Commant_MV(MOTOR_NUMBER);
  Commant_G(MOTOR_NUMBER);
}

void TorqueMode(uint8_t MOTOR_NUMBER, int32_t T) {
  Commant_MT(MOTOR_NUMBER);
  Commant_T(MOTOR_NUMBER, T);
  Commant_G(MOTOR_NUMBER);
}

int32_t RPA(uint8_t MOTOR_NUMBER) {
  int16_t x = 0x0000;
  int32_t data = 0x00;
  MessengeStructSetting();
  msg.id = 0x08080400 | (MOTOR_NUMBER << 2);
  msg.len = 2;
  for (int i = 0; i < msg.len; i++) {
    msg.buf[i] = (x & 0xff);
    x = x >> 8;
  }
  CANBUS_TX();

  //while(CANBUS.read(msg));
  //while (!Can0.read());
  delayMicroseconds(250);

  while (Can2.read(msg)) {
    data = msg.buf[0];
    for (int i = 1; i < msg.len; i++) {
      data = (msg.buf[i] << (8 * i)) | data;
    }
  }
  return data;
}

int32_t RVA(uint8_t MOTOR_NUMBER) {
  int16_t x = 0x0000;
  int32_t data = 0x00;
  MessengeStructSetting();
  msg.id = 0x08200400 | (MOTOR_NUMBER << 2);
  msg.len = 2;
  for (int i = 0; i < msg.len; i++) {
    msg.buf[i] = (x & 0xff);
    x = x >> 8;
  }

  CANBUS_TX();

  //while(CANBUS.read(msg));
  //  while (!Can0.available());
  delayMicroseconds(250);

  while (Can2.read(msg)) {
    data = msg.buf[0];
    for (int i = 1; i < msg.len; i++) {
      data = (msg.buf[i] << (8 * i)) | data;
    }
  }
  return data;
}

uint32_t  GetID(void) {
  return msg.id;
}

uint8_t  GetEXT(void) {
  MessengeStructSetting();
}
uint8_t  GetLEN(void) {
  return msg.len;
}
uint8_t  *GetBUF(void) {
  return msg.buf;
}

void Can_Send(int Mode, int M_Mode, int data1, int data2_h, int data2_l, bool ENCODER_LOSS) {
  msg.id = 0x123;
  msg.len = 8;
  msg.flags.extended = 0;
  if (ENCODER_LOSS == true) {
    Mode = 0 ;
    M_Mode = 1;
    data1 = 0 ;
    data2_h = 0;
    data2_l = 0;
  }
  msg.buf[0] = 255;
  msg.buf[1] = Mode ;
  msg.buf[2] = M_Mode;
  msg.buf[3] = data1 / 256 ;
  msg.buf[4] = data1 % 256;
  msg.buf[5] = data2_h;
  msg.buf[6] = data2_l;
  msg.buf[7] = 254;

  CANBUS1_TX();
}

void Can_Send_Light(bool D1, bool D2, bool D3, bool D4 , bool D5) {
  msg.id = 0x11;
  msg.len = 7;
  msg.buf[0] = 77;
  msg.buf[1] = D1;
  msg.buf[2] = D2;
  msg.buf[3] = D3;
  msg.buf[4] = D4;
  msg.buf[5] = D5;
  msg.buf[6] = 88;

  CANBUS1_TX();
}
