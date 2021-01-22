#include <FlexCAN_T4.h>
#include "SM_CMD.h"

////////////Pin define///////////////////
//Direction Machine
#define Pin_CLK_Direction_Machine 11
#define Pin_DATA_Direction_Machine 12

//Steering Wheel
#define Pin_CLK_steering_wheel 9
#define Pin_DATA_steering_wheel 10

//Direction Machine & Limit switch
#define Pin_Speed 13
#define Pin_SpeedMode 14
#define Pin_Brake 15
#define Pin_Dir 16
#define Pin_En 17
#define Pin_Limit_Switch 18

//Car Light
#define Pin_Left_Light 2
#define Pin_Right_Light 3
#define Pin_Back_Brake_Light 4
#define Pin_Buzzer 5
#define Pin_Wheel_Brake 6
/////////////Pin define end//////////////////////

//////////////Parameter Setting///////////
//Direction Machine
#define turn_limit_r 85.0 //一圈半 一圈九十度 前輪轉向的
#define turn_limit_l 85.0
#define Wheel_origin 162.0 //266.5 前輪轉向中心點  左負右正
//2020: 右極限點181   左極限23   右負左正  誤差2
float target = Wheel_origin;
float last_target = 0.0;
//Steering Wheel
#define Steering_wheel_origin 183 //方向盤置中encoder值
#define Steering_wheel_limit 90.0 //一圈60
//#define Update_Steering_Cycle 10000  //0.01s
int init_Steering_wheel = 0;
bool over_left_limit = false;
int over_left_count = 0;
bool over_right_limit = false;
int over_right_count = 0;
int steering_wheel_now = 0;
//encoder
#define frameSize 16
float Theta_wheel = 0.0;
float Theta_steering_wheel = 0.0;
float Last_Theta_steering_wheel = 0.0;
int Encoder_loss_cnt = 0;
bool Encoder_loss = false;
///////////parameter define

//car driveing state
int Car_Drive_Mode = 0;
int Car_Manual_State = 1; //0:R,1:N,2:D

//packet
int Data_point = 0;
byte Data[150];

//PID
double error_pre = 0.0;
double error_add = 0.0;

double Kp = 12.0;
double Kd = 1.2;
double Ki = 0.0;
//Timer
#define CarDriveControl_Cycle 2000
#define CanReceive_Cycle 80000
//Car Light
int mission_state = 1;
bool flash_light = false;
#define Flash_Light_Cycle 400000
//IMU
byte axbyte[4] = {0};
byte aybyte[4] = {0};
byte azbyte[4] = {0};

byte gxbyte[4] = {0};
byte gybyte[4] = {0};
byte gzbyte[4] = {0};

byte IMU_yaw[4];

unsigned char Pitchbyte[4] = {0};
unsigned char Rollbyte[4] = {0};
unsigned char Yawbyte[4] = {0};

float roll, pitch, yaw, gx, gy, gz, ax, ay, az;

float adj_yaw, catchdata, final_yaw;
int send_yaw;

byte incomingByte[1024] = {0};
int i = 0;
bool stringComplete = false;
int CorrTime = 0;

//--------Car Info Feedback--------
float scale = 0.0;
int rotate_now = 0;
int RPM_L = 0, RPM_R = 0;

bool limit_switch_error = false;
int limit_switch_error_cnt = 0;
int limit_switch_on_cnt = 0;
int rpm_lh = 0;
int rpm_ll = 0;
int rpm_rh = 0;
int rpm_rl = 0;
double V_cal_L = 0.0;
double a_cal_L = 0.0;
double V_cal_R = 0.0;
double a_cal_R = 0.0;
double V_cal_avg = 0.0;
double a_cal_avg = 0.0;
int delta_now = 0;
int delta_check = 0;
int delta_now_l = 0;
int delta_now_h = 0;
double DIS_L = 0.0;
double DIS_R = 0.0;
double Last_DIS_L = 0.0;
double Last_DIS_R = 0.0;
double DIS_err_L = 0.0;
double DIS_err_R = 0.0;
int int_dis_l = 0;
float flt_dis_l = 0;
int int_dis_r = 0;
float flt_dis_r = 0;
int int_vel_l = 0;
float flt_vel_l = 0;
int int_vel_r = 0;
float flt_vel_r = 0;
int int_acc_l = 0;
float flt_acc_l = 0;
int int_acc_r = 0;
float flt_acc_r = 0;
double Last_Dis_L = 0.0;
double Last_Dis_R = 0.0;
float Sens = 0.0;
double Odm_x = 0.0;
double Odm_y = 0.0;
double Pos_x = 0.0;
double Pos_y = 0.0;
byte feedback_buf[33];
bool Left_Wheel_Recv = false;
bool Right_Wheel_Recv = false;
int Reset_Odm_Sign = 0;
bool Odm_Reset = false;
////////////////// Left Rear Wheel Encode Parameter Define/////////////////
float last_velocity_l = 0.0;
////////////////// Right Rear Wheel Encode Parameter Define/////////////////
float last_velocity_r = 0.0;
//
bool trans = false;
double add_error_AutoSteer = 0.0;
double pre_error_AutoSteer = 0.0;

/////////////Parameter Setting Finish/////////////
IntervalTimer CarDriveControl_Timer, CanReceive_Timer, Light_timer;
CAN_message_t read_msg, light_msg;
FlexCAN_T4<CAN1, RX_SIZE_32, TX_SIZE_16> Can1;
FlexCAN_T4<CAN2, RX_SIZE_32, TX_SIZE_16> Can2;
void setup()
{
  /////////////////Setting////////////////////////
  Serial.print("----------------------");
  Serial.begin(115200);  //Computer
  Serial2.begin(115200); //IMU
  CANBUS_BEGIN();        //SM

  //Front Motor Control Pin
  pinMode(Pin_Speed, OUTPUT);
  pinMode(Pin_SpeedMode, OUTPUT);
  pinMode(Pin_Brake, OUTPUT);
  pinMode(Pin_Dir, OUTPUT);
  pinMode(Pin_En, OUTPUT);
  pinMode(Pin_Wheel_Brake, OUTPUT);
  pinMode(Pin_Limit_Switch, INPUT_PULLUP);

  pinMode(Pin_Left_Light, OUTPUT);
  pinMode(Pin_Right_Light, OUTPUT);
  pinMode(Pin_Back_Brake_Light, OUTPUT);
  pinMode(Pin_Buzzer, OUTPUT);

  pinMode(Pin_CLK_Direction_Machine, OUTPUT);
  pinMode(Pin_DATA_Direction_Machine, INPUT);
  pinMode(Pin_CLK_steering_wheel, OUTPUT);
  pinMode(Pin_DATA_steering_wheel, INPUT);
  //////////////////Setting Finish///////////////////////
  //motor initial
  digitalWrite(Pin_SpeedMode, LOW); //外部速度
  digitalWrite(Pin_Brake, LOW);
  digitalWrite(Pin_Dir, LOW);
  digitalWrite(Pin_En, LOW); //LOW啟動 HIGH停止

  digitalWrite(Pin_Left_Light, LOW);  //亮燈
  digitalWrite(Pin_Right_Light, LOW); //亮燈
  digitalWrite(Pin_Wheel_Brake, LOW);

  digitalWrite(Pin_Back_Brake_Light, HIGH);
  digitalWrite(Pin_Buzzer, LOW);

  //cal scale =>方向盤的兩圈半對應到轉向機轉多少
  scale = ((turn_limit_l + turn_limit_r) / 2) / Steering_wheel_limit;
  Serial.println("Steering Wheel Initial Start!!!");
  Steering_Wheel_Initial();
  Serial.println("Steering Wheel Initial Finish!!!");
  CarDriveControl_Timer.begin(DriveControl, CarDriveControl_Cycle);
  CanReceive_Timer.begin(Can_Receive, CanReceive_Cycle);
}

void loop()
{
  if (!digitalRead(Pin_Limit_Switch))
  {
    limit_switch_error_cnt++;
    if (limit_switch_error_cnt > 5)
      limit_switch_error = true;
  }
  else
    limit_switch_error_cnt = 0;
  if (limit_switch_on_cnt > 5)
  {
    CarDriveControl_Timer.begin(DriveControl, CarDriveControl_Cycle);
    limit_switch_error = false;
    digitalWrite(Pin_Brake, LOW);
    digitalWrite(Pin_En, LOW);
    limit_switch_on_cnt = 0;
  }
  if (limit_switch_error)
  {
    if (digitalRead(Pin_Limit_Switch))
    {
      limit_switch_on_cnt++;
    }
    else
      limit_switch_on_cnt = 0;
    CarDriveControl_Timer.end();
    analogWrite(Pin_Speed, 0);
    digitalWrite(Pin_Brake, HIGH);
    digitalWrite(Pin_En, HIGH);
  }
}

/*
  -9:Packet head
  -8:Drive Mode
  -7:D/R/N(manual)
  -6:Car mission state
  -5:Rotate High Byte
  -4:Rotate Low Byte
  -3:Speed Targe High Byte
  -2:Speed Target Low Byte
  -1:Odometer Sign flag
  -0:Packet Tail
*/
void serialEvent()
{

  //  char rx = Serial.read();
  //  if (rx == 'D')
  //  {
  //    Serial.println("D Mode");
  //    Can_Send(0, 2, 0, 0, 0, Encoder_loss);
  //  }
  //  else if (rx == 'R')
  //  {
  //    Serial.println("R Mode");
  //    Can_Send(0, 0, 0, 0, 0, Encoder_loss);
  //  }
  //  return;

  if (Serial.available())
  {
    Data[Data_point] = Serial.read();
    if (Data_point >= 149) Data_point = 0;
    if (Data_point >= 9) {
      if ((Data[Data_point] == 254) && (Data[Data_point - 9] == 255)) {
        Car_Drive_Mode = Data[Data_point - 8];
        Car_Manual_State = Data[Data_point - 7];
        Reset_Odm_Sign = Data[Data_point - 1];
        if (Reset_Odm_Sign == 1)
        {
          Odm_Reset = true;
        }
        int temp_mission_state = Data[Data_point - 6];
        if (temp_mission_state != mission_state) { //模式改變 直線 轉彎 用來改變大燈行為的
          if (temp_mission_state == 5) {
            digitalWrite(Pin_Wheel_Brake, HIGH);
            digitalWrite(Pin_Wheel_Brake, HIGH);
            digitalWrite(Pin_Wheel_Brake, HIGH);
            digitalWrite(Pin_Back_Brake_Light, LOW);
            digitalWrite(Pin_Back_Brake_Light, LOW);
          }
          else {
            digitalWrite(Pin_Wheel_Brake, LOW);
            digitalWrite(Pin_Wheel_Brake, LOW);
            digitalWrite(Pin_Wheel_Brake, LOW);
            digitalWrite(Pin_Back_Brake_Light, HIGH);
            digitalWrite(Pin_Back_Brake_Light, HIGH);
          }
          if (temp_mission_state == 2 || temp_mission_state == 3 || temp_mission_state == 4 || temp_mission_state == 5 || temp_mission_state == 6 || temp_mission_state == 7) { //改為左轉或右轉
            //Light_timer.begin(Flash_Light, Flash_Light_Cycle);
            digitalWrite(Pin_Buzzer, HIGH);
          }
          else {
            //大燈兩邊恆亮
            digitalWrite(Pin_Left_Light, LOW);  //亮燈
            digitalWrite(Pin_Right_Light, LOW); //亮燈
            //Light_timer.end();
            digitalWrite(Pin_Left_Light, LOW);  //亮燈
            digitalWrite(Pin_Right_Light, LOW); //亮燈
            flash_light = false;
            digitalWrite(Pin_Left_Light, LOW);  //亮燈
            digitalWrite(Pin_Right_Light, LOW); //亮燈
            digitalWrite(Pin_Buzzer, LOW);
          }
          mission_state = temp_mission_state;
        }

        if (Car_Drive_Mode == 0) {
          pre_error_AutoSteer = 0.0;
          add_error_AutoSteer = 0.0;
          TorqueMode(4, 1);
          Can_Send(Car_Drive_Mode, Car_Manual_State, 0, 0, 0, Encoder_loss);
        }
        else if (Car_Drive_Mode == 1) {
          int rotate_data_h = Data[Data_point - 5];
          int rotate_data_l = Data[Data_point - 4];
          rotate_now = ((rotate_data_h * 256 + rotate_data_l) - 9000) / 100.0;;
          trans = true;
          if (!limit_switch_error)Can_Send(Car_Drive_Mode, Car_Manual_State, 0, Data[Data_point - 3], Data[Data_point - 2], Encoder_loss);
          trans = false;
        }
        else {
          pre_error_AutoSteer = 0.0;
          add_error_AutoSteer = 0.0;
          TorqueMode(4, 1);
          Can_Send(0, 1, 0, 0, 0, Encoder_loss);
        }

        if (limit_switch_error)Can_Send(0, 1, 0, 0, 0, Encoder_loss);
        Data_point = 0;
      }
    } Data_point++;
  }
}

void serialEvent2()
{
  incomingByte[i] = Serial2.read();
  //if (i >= 495) i = 0;
  if (i >= 81)
  {
    if (incomingByte[i - 81] == 0x5A && incomingByte[i - 80] == 0xA5 && incomingByte[i - 79] == 0x4C && incomingByte[i - 78] == 0x00 && incomingByte[i - 75] == 0x91)
    {
      axbyte[0] = incomingByte[i - 63];
      axbyte[1] = incomingByte[i - 62];
      axbyte[2] = incomingByte[i - 61];
      axbyte[3] = incomingByte[i - 60];
      aybyte[0] = incomingByte[i - 59];
      aybyte[1] = incomingByte[i - 58];
      aybyte[2] = incomingByte[i - 57];
      aybyte[3] = incomingByte[i - 56];
      azbyte[0] = incomingByte[i - 55];
      azbyte[1] = incomingByte[i - 54];
      azbyte[2] = incomingByte[i - 53];
      azbyte[3] = incomingByte[i - 52];

      gxbyte[0] = incomingByte[i - 51];
      gxbyte[1] = incomingByte[i - 50];
      gxbyte[2] = incomingByte[i - 49];
      gxbyte[3] = incomingByte[i - 48];
      gybyte[0] = incomingByte[i - 47];
      gybyte[1] = incomingByte[i - 46];
      gybyte[2] = incomingByte[i - 45];
      gybyte[3] = incomingByte[i - 44];
      gzbyte[0] = incomingByte[i - 43];
      gzbyte[1] = incomingByte[i - 42];
      gzbyte[2] = incomingByte[i - 41];
      gzbyte[3] = incomingByte[i - 40];

      Rollbyte[0] = incomingByte[i - 27];
      Rollbyte[1] = incomingByte[i - 26];
      Rollbyte[2] = incomingByte[i - 25];
      Rollbyte[3] = incomingByte[i - 24];

      Pitchbyte[0] = incomingByte[i - 23];
      Pitchbyte[1] = incomingByte[i - 22];
      Pitchbyte[2] = incomingByte[i - 21];
      Pitchbyte[3] = incomingByte[i - 20];

      Yawbyte[0] = incomingByte[i - 19];
      Yawbyte[1] = incomingByte[i - 18];
      Yawbyte[2] = incomingByte[i - 17];
      Yawbyte[3] = incomingByte[i - 16];

      typedef union
      {
        float val;
        uint8_t bytes[4];
      } floatval;

      floatval ax_floatval;
      floatval ay_floatval;
      floatval az_floatval;
      memcpy(ax_floatval.bytes, axbyte, 4);
      memcpy(ay_floatval.bytes, aybyte, 4);
      memcpy(az_floatval.bytes, azbyte, 4);

      floatval gx_floatval;
      floatval gy_floatval;
      floatval gz_floatval;
      memcpy(gx_floatval.bytes, gxbyte, 4);
      memcpy(gy_floatval.bytes, gybyte, 4);
      memcpy(gz_floatval.bytes, gzbyte, 4);

      floatval v;
      v.bytes[0] = Pitchbyte[0];
      v.bytes[1] = Pitchbyte[1];
      v.bytes[2] = Pitchbyte[2];
      v.bytes[3] = Pitchbyte[3];

      floatval v1;
      v1.bytes[0] = Rollbyte[0];
      v1.bytes[1] = Rollbyte[1];
      v1.bytes[2] = Rollbyte[2];
      v1.bytes[3] = Rollbyte[3];

      floatval v2;
      v2.bytes[0] = Yawbyte[0];
      v2.bytes[1] = Yawbyte[1];
      v2.bytes[2] = Yawbyte[2];
      v2.bytes[3] = Yawbyte[3];

      pitch = v.val;
      roll = v1.val;
      yaw = v2.val;

      //           Serial.println(ax_floatval.val);
      //           Serial.println(ay_floatval.val);
      //           Serial.println(az_floatval.val);

      if (CorrTime <= 500)
      {
        CorrTime = CorrTime + 1;
        catchdata = yaw;
      }

      adj_yaw = yaw - catchdata;

      if (adj_yaw < 0)
        adj_yaw = adj_yaw + 360; //校正為0~360
      final_yaw = adj_yaw;
      if (final_yaw >= 180)
        final_yaw = final_yaw - 360; //校正為+-180
      i = 0;
      //      Serial.println(final_yaw);

      stringComplete = true;
    }
  }
  i++;
}

void Can_Receive()
{
  if (Can1.read(read_msg))
  {
    if (read_msg.id == 0x10 && read_msg.len == 7 && read_msg.buf[0] == 255 && read_msg.buf[6] == 254)
    { // Left RPM & Distance

      RPM_L = read_msg.buf[1] * 256 + read_msg.buf[2];
      unsigned long delta_time = read_msg.buf[3] * 65536 + read_msg.buf[4] * 256 + read_msg.buf[5];

      V_cal_L = (RPM_L * 3.14159 * 0.55) / (60.0 * 10.0);
      a_cal_L = ((V_cal_L - last_velocity_l) / (delta_time / 1000000.0));               // Left Acceleration
      DIS_L = (((RPM_L * 3.14159 * 0.55) / (60.0 * 10.0)) * (delta_time / 1000000.0)) * 10; //Left Distance

      last_velocity_l = V_cal_L;
      DIS_err_L = DIS_L - Last_DIS_L;

      Left_Wheel_Recv = true;
    }
    else if (read_msg.id == 0x20 && read_msg.len == 7 && read_msg.buf[0] == 255 && read_msg.buf[6] == 254) // Right RPM & Distance
    {
      RPM_R = read_msg.buf[1] * 256 + read_msg.buf[2];
      unsigned long delta_time = read_msg.buf[3] * 65536 + read_msg.buf[4] * 256 + read_msg.buf[5];

      V_cal_R = (RPM_R * 3.14159 * 0.55) / (60.0 * 10.0);
      a_cal_R = ((V_cal_R - last_velocity_r) / (delta_time / 1000000.0));               // Right Acceleration
      DIS_R = (((RPM_R * 3.14159 * 0.55) / (60.0 * 10.0)) * (delta_time / 1000000.0)) * 10; //Right Distance

      last_velocity_r = V_cal_R;
      DIS_err_R = DIS_R - Last_DIS_R;

      Right_Wheel_Recv = true;
    }
    Last_DIS_L = DIS_L;
    Last_DIS_R = DIS_R;
    V_cal_avg = (V_cal_L + V_cal_R) / 2;
    a_cal_avg = (a_cal_L + a_cal_R) / 2;

    Calculate_Odometry();
    Send_Feedback();

    //    Serial.print("RPM_L: ");
    //    Serial.println(RPM_L);
    //    Serial.print("RPM_R: ");
    //    Serial.println(RPM_R);
    //    Serial.print("V_cal_L: ");
    //    Serial.println(V_cal_L);
    //    Serial.print("V_cal_R: ");
    //    Serial.println(V_cal_R);
    //    Serial.print("a_cal_L: ");
    //    Serial.println(a_cal_L);
    //    Serial.print("a_cal_R: ");
    //    Serial.println(a_cal_R);
    //    Serial.print("V_cal_avg: ");
    //    Serial.println(V_cal_avg);
    //    Serial.print("a_cal_avg: ");
    //    Serial.println(a_cal_avg);
    //    Serial.print("DIS_L: ");
    //    Serial.println(DIS_L);
    //    Serial.print("DIS_R: ");
    //    Serial.println(DIS_R);
    //    Serial.print("DIS: ");
    //    Serial.println((DIS_L + DIS_R) / 2);
    //    Serial.print("DIS_err_L: ");
    //    Serial.println(DIS_err_L);
    //    Serial.print("DIS_err_R: ");
    //    Serial.println(DIS_err_R);
    //    Serial.print("Odm_x: ");
    //    Serial.println(Odm_x);
    //    Serial.print("Odm_y: ");
    //    Serial.println(Odm_y);
    //    Serial.print("Pos_x: ");
    //    Serial.println(Pos_x);
    //    Serial.print("Pos_y: ");
    //    Serial.println(Pos_y);
    //    Serial.print("aaaaa: ");
    //    Serial.println(aaaaa);
    //    Serial.print("bbbbb: ");
    //    Serial.println(bbbbb);
    //    Serial.print("final_yaw: ");
    //    Serial.println(final_yaw);
    //    Serial.print(",");
    //    Serial.println(radians(final_yaw));
    //    Serial.print("Theta_wheel: ");
    //    Serial.println(Theta_wheel);
    //    Serial.print("Theta_steering_wheel: ");
    //    Serial.println(Theta_steering_wheel);
  }
}

void DriveControl()
{
  getEncoder_Pos(0);
  //steering_wheel_now = ((Theta_steering_wheel * (1)) - Steering_wheel_origin);
  Update_Steering_Wheel_Data();

  if (Car_Drive_Mode == 1)
  {
    target = Wheel_origin + rotate_now * scale;
    Auto_Steering_Wheel_Control();
  }
  else
  {
    target = Wheel_origin + steering_wheel_now * scale;
  }
  if (target > (Wheel_origin + turn_limit_l))
  {
    target = Wheel_origin + turn_limit_l;
  }
  else if (target < (Wheel_origin - turn_limit_r))
  {
    target = Wheel_origin - turn_limit_r;
  }
  getEncoder_Pos(1);
  Direction_Machine_PID_Control();
}

void Update_Steering_Wheel_Data()
{
  //  Serial.print("gan");
  if (!over_left_limit && !over_right_limit)
  {
    steering_wheel_now = Theta_steering_wheel - Steering_wheel_origin;
    if (Theta_steering_wheel > (Steering_wheel_origin + Steering_wheel_limit))
    {
      over_left_limit = true;
      steering_wheel_now = Steering_wheel_limit;
    }
    if (Theta_steering_wheel < (Steering_wheel_origin - Steering_wheel_limit))
    {
      over_right_limit = true;
      steering_wheel_now = Steering_wheel_limit * (-1);
    }
  }
  if (over_left_limit)
  { //方向盤轉超過2圈半 確認是否轉回來
    if ((Theta_steering_wheel < (Steering_wheel_origin + Steering_wheel_limit)) && (Theta_steering_wheel > (Steering_wheel_origin + Steering_wheel_limit - 10)))
    {
      over_left_limit = false;
      steering_wheel_now = Theta_steering_wheel - Steering_wheel_origin;
    }
  }
  if (over_right_limit)
  { //方向盤轉超過2圈半 確認是否轉回來
    if ((Theta_steering_wheel > (Steering_wheel_origin - Steering_wheel_limit)) && (Theta_steering_wheel < (Steering_wheel_origin - (Steering_wheel_limit - 10))))
    {
      over_right_limit = false;
      steering_wheel_now = Theta_steering_wheel - Steering_wheel_origin;
    }
  } //Serial.println(steering_wheel_now);
}

void Auto_Steering_Wheel_Control()
{
  double error = steering_wheel_now - rotate_now;
  double Kp = 2800.0;
  double Ki = 16;
  double Kd = 300;
  if (fabs(error) > 1.5)
  {
    double Out = error * Kp + Kd * pre_error_AutoSteer + add_error_AutoSteer * Ki;
    if (Out > 250000)
      Out = 250000;
    else if (Out < -250000)
      Out = -250000;
    else if (Out > 0 && Out < 20000)
      Out = 20000;
    else if (Out < 0 && Out > -20000)
      Out = -20000;
    if (!trans)
      VelocityMode(4, Out, 1 * 100);
    pre_error_AutoSteer = error;
    add_error_AutoSteer += error;
  }
  else
  {
    add_error_AutoSteer = 0;
    if (!trans)
    {
      VelocityMode(4, 0, 1 * 100);
      TorqueMode(4, 1);
    }
  }
}

void Direction_Machine_PID_Control()
{
  double err = Theta_wheel - target;
  // Serial.print("Target = ");
  // Serial.println(target);
  // Serial.print("Theta_wheel = ");
  // Serial.println(Theta_wheel);
  if (fabs(err) < 0.5)
  {
    error_add = 0;
    analogWrite(Pin_Speed, 0);
    digitalWrite(Pin_Brake, HIGH);
  }
  else
  {
    digitalWrite(Pin_Brake, LOW);
    double Out = Kp * err + Ki * error_add + Kd * (err - error_pre);
    if (Out > 255)
      Out = 255;
    else if (Out < -255)
      Out = -255;
    if (Out > 0)
    {
      digitalWrite(Pin_Dir, LOW);
      analogWrite(Pin_Speed, Out);
    }
    else
    {
      digitalWrite(Pin_Dir, HIGH);
      Out = Out * (-1);
      analogWrite(Pin_Speed, Out);
    }
    error_pre = err;
    error_add += err;
  }
  Delta_Feedback();
}

void Steering_Wheel_Initial()
{
  getEncoder_Pos(0);
  int value = ((Theta_steering_wheel * (1)) - Steering_wheel_origin);
  while (value != 0)
  {
    getEncoder_Pos(0);
    value = ((Theta_steering_wheel * (1)) - Steering_wheel_origin);
    if (value > 30)
      VelocityMode(4, 6 * 32768, 1 * 100);
    else if (value > 0 && value <= 30)
      VelocityMode(4, 1 * 32768, 1 * 100);
    else if (value < -30)
      VelocityMode(4, -6 * 32768, 1 * 100);
    else if (value < 0 && value >= -30)
      VelocityMode(4, -1 * 32768, 1 * 100);
    delay(10);
  }
  TorqueMode(4, 1);
}

void Flash_Light()
{
  if (flash_light)
  {
    digitalWrite(Pin_Left_Light, LOW);  //左方向燈滅
    digitalWrite(Pin_Right_Light, LOW); //右方向燈滅
  }
  else
  {
    if (mission_state == 2)
    {
      digitalWrite(Pin_Left_Light, HIGH); //左轉方向燈亮
    }
    else if (mission_state == 3)
    {
      digitalWrite(Pin_Right_Light, HIGH); //右轉方向燈亮
    }
    else if (mission_state == 4 || mission_state == 5 || mission_state == 6 || mission_state == 7)
    { //停站閃雙黃燈
      digitalWrite(Pin_Left_Light, HIGH);  //亮掉
      digitalWrite(Pin_Right_Light, HIGH); //亮燈
    }
  }
  flash_light = !flash_light;
}

void Send_Feedback()
{
  //---------------------V_cal_avg---------------------------------
  int integer_V_cal_avg = (int)(V_cal_avg);
  int float_V_cal_avg = (int)((V_cal_avg - (double)(integer_V_cal_avg)) * 1000);

  int int_V_cal_avg_h = integer_V_cal_avg / 256;
  int int_V_cal_avg_l = integer_V_cal_avg % 256;
  int flt_V_cal_avg_h = float_V_cal_avg / 256;
  int flt_V_cal_avg_l = float_V_cal_avg % 256;
  //---------------------a_cal_avg---------------------------------
  float a_cal = a_cal_avg + 1000;
  int integer_a_cal_avg = (int)(a_cal);
  int float_a_cal_avg = (int)((a_cal - (double)(integer_a_cal_avg)) * 1000);

  int int_a_cal_avg_h = integer_a_cal_avg / 256;
  int int_a_cal_avg_l = integer_a_cal_avg % 256;
  int flt_a_cal_avg_h = float_a_cal_avg / 256;
  int flt_a_cal_avg_l = float_a_cal_avg % 256;
  //---------------------final_yaw---------------------------------
  float final_yaw_c = final_yaw + 1000;
  int integer_final_yaw = (int)(final_yaw_c);
  int float_final_yaw = (int)((final_yaw_c - (double)(integer_final_yaw)) * 1000);

  int int_final_yaw_h = integer_final_yaw / 256;
  int int_final_yaw_l = integer_final_yaw % 256;
  int flt_final_yaw_h = float_final_yaw / 256;
  int flt_final_yaw_l = float_final_yaw % 256;
  //---------------------Odm_x---------------------------------
  double Odm_x_c = Odm_x + 10000;
  int integer_Odm_x = (int)(Odm_x_c);
  int float_Odm_x = (int)((Odm_x_c - (double)(integer_Odm_x)) * 1000);

  int int_odm_x_h = integer_Odm_x / 256;
  int int_odm_x_l = integer_Odm_x % 256;
  int flt_odm_x_h = float_Odm_x / 256;
  int flt_odm_x_l = float_Odm_x % 256;
  //---------------------Odm_y---------------------------------
  double Odm_y_c = Odm_y + 10000;
  int integer_Odm_y = (int)(Odm_y_c);
  int float_Odm_y = (int)((Odm_y_c - (double)(integer_Odm_y)) * 1000);

  int int_odm_y_h = integer_Odm_y / 256;
  int int_odm_y_l = integer_Odm_y % 256;
  int flt_odm_y_h = float_Odm_y / 256;
  int flt_odm_y_l = float_Odm_y % 256;
  //---------------------Theta_Wheel----------------------------
  double Theta_wheel_c = Theta_wheel + 1000;
  int integer_Theta_wheel = (int)(Theta_wheel_c);
  int float_Theta_wheel = (int)((Theta_wheel_c - (double)(integer_Theta_wheel)) * 1000);

  int int_theta_wheel_h = integer_Theta_wheel / 256;
  int int_theta_wheel_l = integer_Theta_wheel % 256;
  int flt_theta_wheel_h = float_Theta_wheel / 256;
  int flt_theta_wheel_l = float_Theta_wheel % 256;
  //---------------------Theta_Steering_Wheel----------------------------
  double Theta_steering_wheel_c = Theta_steering_wheel + 1000;
  int integer_Theta_steering_wheel = (int)(Theta_steering_wheel_c);
  int float_Theta_steering_wheel = (int)((Theta_steering_wheel_c - (double)(integer_Theta_steering_wheel)) * 1000);

  int int_theta_steering_wheel_h = integer_Theta_steering_wheel / 256;
  int int_theta_steering_wheel_l = integer_Theta_steering_wheel % 256;
  int flt_theta_steering_wheel_h = float_Theta_steering_wheel / 256;
  int flt_theta_steering_wheel_l = float_Theta_steering_wheel % 256;

  feedback_buf[0] = 100;

  feedback_buf[1] = int_V_cal_avg_h;
  feedback_buf[2] = int_V_cal_avg_l;
  feedback_buf[3] = flt_V_cal_avg_h;
  feedback_buf[4] = flt_V_cal_avg_l;

  feedback_buf[5] = int_a_cal_avg_h;
  feedback_buf[6] = int_a_cal_avg_l;
  feedback_buf[7] = flt_a_cal_avg_h;
  feedback_buf[8] = flt_a_cal_avg_l;

  feedback_buf[9] = int_final_yaw_h;
  feedback_buf[10] = int_final_yaw_l;
  feedback_buf[11] = flt_final_yaw_h;
  feedback_buf[12] = flt_final_yaw_l;

  feedback_buf[13] = delta_now / 256;
  feedback_buf[14] = delta_now % 256;

  feedback_buf[15] = int_odm_x_h;
  feedback_buf[16] = int_odm_x_l;
  feedback_buf[17] = flt_odm_x_h;
  feedback_buf[18] = flt_odm_x_l;

  feedback_buf[19] = int_odm_y_h;
  feedback_buf[20] = int_odm_y_l;
  feedback_buf[21] = flt_odm_y_h;
  feedback_buf[22] = flt_odm_y_l;

  feedback_buf[23] = int_theta_wheel_h;
  feedback_buf[24] = int_theta_wheel_l;
  feedback_buf[25] = flt_theta_wheel_h;
  feedback_buf[26] = flt_theta_wheel_l;

  feedback_buf[27] = int_theta_steering_wheel_h;
  feedback_buf[28] = int_theta_steering_wheel_l;
  feedback_buf[29] = flt_theta_steering_wheel_h;
  feedback_buf[30] = flt_theta_steering_wheel_l;

  feedback_buf[31] = limit_switch_error;

  feedback_buf[32] = 101;

  //  Serial.println(flt_odm_x_h);
  //  Serial.println(flt_odm_x_l);
  //  Serial.println(flt_odm_y_h);
  //  Serial.println(flt_odm_y_l);
  //  Serial.println("===================================");
  Serial.write(feedback_buf, 33);
}
float byteTofloat(byte x1, byte x2)
{
  float out;
  if (x2 >= 0x80)
    out = -(x1 ^ 0xFF + (x2 ^ 0xFF) * 256);
  else
    out = (x1 + x2 * 256);

  return out;
}

void Calculate_Odometry()
{
  //Serial.println("=============Odometry============");
  if (Left_Wheel_Recv && Right_Wheel_Recv)
  {
    if (Odm_Reset)
    {
      Reset_Odm();
      Odm_Reset = false;
    }
    //  if (DIS_err_L < Sens && DIS_err_L > -Sens)
    //  {
    //    DIS_err_L = 0;
    //  }
    //  else if (DIS_err_R < Sens && DIS_err_R > -Sens)
    //  {
    //    DIS_err_R = 0;
    //  }
    if (Car_Manual_State == 2)
    {
      //Serial.print("=============Odometry============");
      //Serial.println("Foward");
      double DIS = (DIS_L + DIS_R) / 2;
      Pos_x = DIS * cos(radians(final_yaw));
      Pos_y = DIS * sin(radians(final_yaw));
    }
    else if (Car_Manual_State == 0)
    {
      //Serial.println("Backward");
      double DIS = (DIS_L + DIS_R) / 2;
      Pos_x = -(DIS * cos(radians(final_yaw)));
      Pos_y = -(DIS * sin(radians(final_yaw)));
    }

    Odm_x += Pos_x;
    Odm_y += Pos_y;

    //    Serial.print("Odm_x: ");
    //    Serial.println(Odm_x);
    //    Serial.print("Odm_y: ");
    //    Serial.println(Odm_y);
    //    Serial.print("Pos_x: ");
    //    Serial.println(Pos_x);
    //    Serial.print("Pos_y: ");
    //    Serial.println(Pos_y);
    //    Serial.println("===================================");
    Left_Wheel_Recv = false;
    Right_Wheel_Recv = false;
  }
}

void Reset_Odm()
{
  //Odm Reset
  Pos_x = 0.0;
  Pos_y = 0.0;
  Odm_x = 0.0;
  Odm_y = 0.0;

  //IMU Reset
  catchdata = yaw;
  adj_yaw = 0;
  final_yaw = 0;
}

void Delta_Feedback()
{
  delta_check = ((Theta_wheel - 162.0) / scale) / 5.0;
  delta_now = delta_check + 9000;
  //  delta_now_h = delta_now / 255;
  //  delta_now_l = delta_now % 255;
}

//encoder function
//0:Steering wheel
//1:Direction machine
void getEncoder_Pos(int index)
{ //index=0 steering_wheel   index=1 wheel
  bool EncoderData[frameSize] = {false};
  bool NewEncoderData[frameSize - 2] = {false};
  delayMicroseconds(1);
  for (int j = 0; j < frameSize; j++)
  {
    if (index == 0)
    { //steering_wheel
      digitalWrite(Pin_CLK_steering_wheel, LOW);
      delayMicroseconds(1); //delayMicroseconds(3) fits a 10us tictac
      EncoderData[j] = digitalRead(Pin_DATA_steering_wheel);
      delayMicroseconds(1);
      digitalWrite(Pin_CLK_steering_wheel, HIGH);
      delayMicroseconds(1);
    }
    else if (index == 1)
    { //wheel
      digitalWrite(Pin_CLK_Direction_Machine, LOW);
      delayMicroseconds(1); //delayMicroseconds(3) fits a 10us tictac
      EncoderData[j] = digitalRead(Pin_DATA_Direction_Machine);
      delayMicroseconds(1);
      digitalWrite(Pin_CLK_Direction_Machine, HIGH);
      delayMicroseconds(1);
    }
  }
  for (int j = 1; j < frameSize - 1; j++)
  {
    NewEncoderData[j - 1] = EncoderData[j];
  }
  if (index == 0)
  {
    Theta_steering_wheel = getAngle(NewEncoderData);
    if (Theta_steering_wheel == 0)
      Encoder_loss_cnt++;
    if (Encoder_loss_cnt == 10)
    {
      Encoder_loss = true;
      Theta_steering_wheel = Last_Theta_steering_wheel;
    }
    Last_Theta_steering_wheel = Theta_steering_wheel;
  }
  else if (index == 1)
  {
    Theta_wheel = getAngle(NewEncoderData);
    // Serial.print("Wheel Theta = ");
    // Serial.println(Theta_wheel);
  }
  delayMicroseconds(1);
}
//encoder function
float getAngle(bool EncoderData[frameSize - 2])
{
  float Data = 0.0;
  Data = EncoderData[12] + EncoderData[11] * 2.0 + EncoderData[10] * 4.0 + EncoderData[9] * 8.0 + EncoderData[8] * 16.0 + EncoderData[7] * 32.0 + EncoderData[6] * 64.0 + EncoderData[5] * 128.0 + EncoderData[4] * 256.0 + EncoderData[3] * 512.0 + EncoderData[2] * 1024.0 + EncoderData[1] * 2048.0;
  Data = float(Data * 180.0 / 4096.0);
  Data = Data + 180.0 * EncoderData[0];
  return Data;
}
