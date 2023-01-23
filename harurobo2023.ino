#include <MsTimer2.h>
#include <FlexCAN.h>
#include "PID.h"
#include <math.h>
#include <kinetis_flexcan.h>

typedef struct
{
  int16_t rotation;
  int16_t denryu;
} wheelEscDataSt;

FlexCAN CANTransmitter(1000000);
static CAN_message_t rxmsg;//can受信用buf
CAN_message_t msg, m_msg, gm_msg;//can送信用buf
wheelEscDataSt wEscData[4];//can受信用

Pid pid0;
Pid pid1;
Pid pid2;
Pid pid3;

int flag = 0;
float vx, vy, vt;
int u[4] = {};
int u2[4] = {};

float rpm,mps;
#define tire_dia 152//こういう感じに定数にしといても良いかも
float rpm2mps(float rpm){//rpmをm/sに変換
  mps=tire_dia*rpm/(60*19);  
  return mps;
}
float mps2rpm(float mps){//m/sをrpmに変換
  rpm=60*19*mps/tire_dia;
  return rpm;
}

void setup(void)
{
  CANTransmitter.begin();
  pinMode(13, OUTPUT);
  Serial.begin(115200);
  Serial1.begin(10000, SERIAL_8E1);
  pinMode(13,OUTPUT);
  digitalWrite(13,HIGH);
  delay(2000);
  digitalWrite(13,LOW);  

  
  pid0.init(3.0, 0.001, 0.03); //p,i,dの順に指定できる
  pid1.init(3.0, 0.001, 0.03);
  pid2.init(3.0, 0.001, 0.03);
  pid3.init(3.0, 0.001, 0.03);

  MsTimer2::set(2, timerInt);
  MsTimer2::start();

}
int cnt=0;
static unsigned long testch[6];///実際にデータを入れる配列
void loop(void)
{
  static int data[18];                      //入力の生データ入れる配列
  static int dataNumber = 0;                //入力データの数(Serial1.available()の返値),受信バッファの数を見る変数
  static unsigned long lastConnectTime = 0; //直前の通信の時間?
  if (Serial1.available() > 0) {//受信バッファが0以上=何か受信している
    for (int dataNum = Serial1.available(); dataNum > 0; dataNum--) {//受信したバイト数を見る
      if (dataNumber < 0) {
        Serial1.read();
        dataNumber++;
        continue;
      }
      data[dataNumber % 18] = Serial1.read();
      dataNumber++;
      if (dataNumber > 18) {
        dataNumber = 0;
      }
      else if (dataNumber == 18) {//データが揃ったとき
        testch[0] = (((data[1] & 0x07) << 8) | data[0]);          //ch0(364～1024～1684)
        testch[1] = (((data[2] & 0x3F) << 5) | (data[1] >> 3));   //ch1(364～1024～1684)
        testch[2] = (((data[4] & 0x01) << 10) | (data[3] << 2) | (data[2] >> 6)); //ch2(364～1024～1684)
        testch[3] = (((data[5] & 0x0F) << 7) | (data[4] >> 1));   //ch3(364～1024～1684)
        if (!(364 <= testch[0] && testch[0] <= 1684 && 364 <= testch[1] && testch[1] <= 1684 && 364 <= testch[2] && testch[2] <= 1684 && 364 <= testch[3] && testch[3] <= 1684)) {
          for (int i = 1; i < 18; i++) {
            testch[0] = (((data[(1 + i) % 18] & 0x07) << 8) | data[(0 + i) % 18]);  //ch0(364～1024～1684)
            testch[1] = (((data[(2 + i) % 18] & 0x3F) << 5) | (data[(1 + i) % 18] >> 3)); //ch1(364～1024～1684)
            testch[2] = (((data[(4 + i) % 18] & 0x01) << 10) | (data[(3 + i) % 18] << 2) | (data[2] >> 6)); //ch2(364～1024～1684)
            testch[3] = (((data[(5 + i) % 18] & 0x0F) << 7) | (data[(4 + i) % 18] >> 1)); //ch3(364～1024～1684)
            if (364 <= testch[0] && testch[0] <= 1684 && 364 <= testch[1] && testch[1] <= 1684 && 364 <= testch[2] && testch[2] <= 1684 && 364 <= testch[3] && testch[3] <= 1684) {
              dataNumber = -i;
              break;
            }
          }
          if (dataNumber > 18) {
            dataNumber = -1;
          }
        }
        else {
          dataNumber = 0;
        }
      }
    }
    digitalWrite(13, !digitalRead(13)); //プロポ受信したらLEDチカチカ
    flag = 1;
  }
  else {//何も受信していない=通信がロスとしている->非常停止した方が良さそう
    flag = 0;
  }
    
  

  if (((data[5] & 0xC0) >> 6) == 1) {
    m_msg.id = 0x200;
    m_msg.len = 8;
    for (int i = 0; i < 4; i++) {
      u[i] = 0;
    }
    if (testch[3] >= 364 && testch[3] <= 1684) {
      vy = map(testch[3], 364, 1684, -5000, 5000);//目標最大rpmは5000
    }
    if (testch[2] >= 364 && testch[2] <= 1684) {
      vx = map(testch[2], 364, 1684, -5000, 5000);
    }
    if (testch[0] >= 364 && testch[0] <= 1684) {
      vt = map(testch[0], 364, 1684, -5000, 5000);
    }
  int u[4] = {0};
  //u[0] = 500;
  //u[1] = 500;
  //u[2] = 500;
  //u[3] = 500; //ここの数字はrpm指定、-5000~5000くらい
    
  float vx=1.0, vy=0.0, vt=0.0;//ここが目標速度、この場合は前進方向に1m/s
  float L=825.7;
  u[0]=mps2rpm(-(1/sqrt(2.0))*vx+(1/sqrt(2.0))*vy+L*vt); //右前
  u[1]=mps2rpm(-(1/sqrt(2.0))*vx-(1/sqrt(2.0))*vy+L*vt); //右後
  u[2]=mps2rpm((1/sqrt(2.0))*vx-(1/sqrt(2.0))*vy+L*vt);//左後
  u[3]=mps2rpm((1/sqrt(2.0))*vx+(1/sqrt(2.0))*vy+L*vt);//左前

  //Serial.print(u[0]);//目標速度
  //Serial.print(",");
  //Serial.print(u[1]);
  //Serial.print(",");
  //Serial.print(u[2]);
  //Serial.print(",");
  //Serial.print(u[3]);
  //Serial.print(",");

  u[0] = pid0.pid_out(u[0]);
  u[1] = pid1.pid_out(u[1]);  
  u[2] = pid2.pid_out(u[2]);
  u[3] = pid3.pid_out(u[3]);
  
   
  Serial.println(u[0]);
  Serial.println(u[1]);
  Serial.println(u[2]);
  Serial.println(u[3]);

   for (int i = 0; i < 4; i++) {
    msg.buf[i * 2] = u[i] >> 8;
    msg.buf[i * 2 + 1] = u[i] & 0xFF;    
  }
  for (int i = 0; i < gm_msg.len; i++) {
      gm_msg.buf[i * 2] = u[i] >> 8;
      gm_msg.buf[i * 2 + 1] = u[i] & 0xFF;
    }
    analogWrite(23, 0);
    analogWrite(21, 0);
  }
  if(digitalRead(2)==1){
    analogWrite(23, 0);
    analogWrite(21, 0);
  }
  delay(50);

  //Serial.print(pid0.debug());//現在速度
  //Serial.print(",");
  //Serial.print(pid1.debug());
  //Serial.print(",");
  //Serial.print(pid2.debug());  
  //Serial.print(",");
  //Serial.print(pid3.debug());
  //Serial.println("");
}

void timerInt() {
  while ( CANTransmitter.read(rxmsg) ) {
    if (rxmsg.id == 0x201) {
      pid0.now_value(rxmsg.buf[2] * 256 + rxmsg.buf[3]);
    }    
    if (rxmsg.id == 0x202) {
      pid1.now_value(rxmsg.buf[2] * 256 + rxmsg.buf[3]);
    } 
    if (rxmsg.id == 0x203) {
      pid2.now_value(rxmsg.buf[2] * 256 + rxmsg.buf[3]);
    }
    if (rxmsg.id == 0x204) {
      pid3.now_value(rxmsg.buf[2] * 256 + rxmsg.buf[3]);
    }
     }
  CANTransmitter.write(msg);
}
