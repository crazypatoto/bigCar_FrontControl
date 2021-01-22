#ifndef __SM_CMD_H__
#define __SM_CMD_H__

void MessengeStructSetting(void);
void CANBUS_TX(void);
void CANBUS1_TX(void);
void Commant_MP(uint8_t MOTOR_NUMBER);
void Commant_MV(uint8_t MOTOR_NUMBER);
void Commant_MT(uint8_t MOTOR_NUMBER);
void Commant_T(uint8_t MOTOR_NUMBER, int32_t T);
void Commant_PT(uint8_t MOTOR_NUMBER, int32_t PT);
void Commant_VT(uint8_t MOTOR_NUMBER, int32_t VT);
void Commant_ADT(uint8_t MOTOR_NUMBER, int32_t ADT);
void Commant_PRT(uint8_t MOTOR_NUMBER, int32_t PRT);
void Commant_G(uint8_t MOTOR_NUMBER);
void Commant_X(uint8_t MOTOR_NUMBER);
void Commant_S(uint8_t MOTOR_NUMBER); 
void Commant_O(uint8_t MOTOR_NUMBER);
void PositionMode(uint8_t MOTOR_NUMBER, int32_t VT, uint16_t ADT, double PTa);
void PositionModeSet(uint8_t MOTOR_NUMBER, int32_t PTa);
void VelocityMode(uint8_t MOTOR_NUMBER, int32_t VT, uint16_t ADT);  
void TorqueMode(uint8_t MOTOR_NUMBER, int32_t T);
int32_t RPA(uint8_t MOTOR_NUMBER); 
int32_t RVA(uint8_t MOTOR_NUMBER);
uint32_t  GetID(void);
uint8_t  GetEXT(void);
uint8_t  GetLEN(void);
uint8_t  *GetBUF(void);
void CANBUS_BEGIN(void);
void Commant_PRTa(uint8_t MOTOR_NUMBER, double PRTa);
void Commant_PTa(uint8_t MOTOR_NUMBER, double PTa);
void Can_Send(int Mode, int M_Mode, int data1, int data2_h, int data2_l, bool ENCODER_LOSS);
#endif
