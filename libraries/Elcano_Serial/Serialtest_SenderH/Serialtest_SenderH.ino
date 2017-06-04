#include "Elcano_Serial.h"
using namespace elcano;
ParseState ps;
SerialData sd;
char thebuffer[64];
char received_info ;
void setup() {
  Serial1.begin(baudrate);
  //Serial1.begin(9600);
  Serial.begin(9600);
  ps.output = &Serial1;
  ps.capture = MsgType::drive;
  ps.dt= &sd;
}

void loop(){
   MsgTypetest("drive"); //we have "goal","drive","sensor","seg"
   int i = 0;
   while(Serial1.available() > 0){
      received_info = (char)Serial1.read();//
      thebuffer[i] = received_info;
      i++;
   }
   for(int j = 0; j<i;j++){
      Serial.print(thebuffer[j]);
   }
   Serial.println("");
   //sd.clear();    10
    
   sd.write(&Serial1);//TX
   delay(10);
  // Serial1.flush();
}

void MsgTypetest(String code){
    if( code == "goal"){
        sd.kind =MsgType::goal;
        sd.number = 01;
        sd.posE_cm = 10;
        sd.posN_cm = 12;
        sd.bearing_deg = 10;
      }
      else if( code == "drive"){
        sd.kind = MsgType::drive;
        sd.speed_cmPs = 32769;
        sd.angle_mDeg = 45;
      }
      else if( code == "sensor"){
        sd.kind = MsgType::sensor;
        sd.speed_cmPs = 30;
        sd.angle_mDeg = 45;
        sd.bearing_deg =  45;
        sd.posE_cm = 10;
        sd.posN_cm = 11;
      }
      //X{n number}{p posE_cm,posN_cm}{b bearing_deg}{a angle_mDeg
      else if( code == "seg"){
        sd.kind = MsgType::seg;
        sd.number = 2;
        sd.posE_cm = 40;
        sd.posN_cm = 23;
        sd.bearing_deg = 10;
        sd.angle_mDeg = 10;
      }
}

