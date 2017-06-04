
#include "Elcano_Serial.h"
//#include "Elcano_Serial.cpp"
using namespace elcano;
ParseState ps;
SerialData sd;
void setup() {
 Serial1.begin(baudrate);
 Serial.begin(9600);
 ps.input = &Serial1;
 ps.dt=&sd;
}

void loop() {
  
   ParseStateError pse = ps.update();
//   Serial.println("Values-------------------------");
//   Serial.print("kind:");
//   Serial.println(ps.dt->kind);
//   Serial.print("number:");
//   Serial.println(ps.dt->number);
//   Serial.print("posE_cm:");
//   Serial.println(ps.dt->posE_cm);
//   Serial.print("posN_cm:");
//   Serial.println(ps.dt->posN_cm);
//   Serial.print("bearing_deg:");
//   Serial.println(ps.dt->bearing_deg);
//   Serial.print("angle_mDeg:");
//   Serial.println(ps.dt->angle_mDeg);
//   Serial.print("speed_cmPs:");
//   Serial.println(ps.dt->speed_cmPs);
//   Serial.print("crc:");
//   Serial.println(ps.dt->crc);
//   Serial.println("Values------------------------");
  
   delay(10);
   if(pse == ParseStateError::success){
      Serial.println("Successful!");
   }else if( pse == ParseStateError::unavailable){
       Serial.println("Unaval!");
   }else if( pse == ParseStateError::noise_in_data){
        Serial.println("Noise!");
   }else if( pse == ParseStateError::bad_type){
       Serial.println("Bad type!");
   }else if( pse == ParseStateError::unavailable){
       Serial.println("Unaval!");
   }else if( pse == ParseStateError::bad_lcurly){
        Serial.println("Bad Curly!");
   }else if( pse == ParseStateError::bad_attrib){
       Serial.println("Bad Attrib!");
   }else if( pse == ParseStateError::bad_number){
       Serial.println("Bad number");
   }else if( pse == ParseStateError::no_space){
        Serial.println("No Space!");
   }else if( pse == ParseStateError::no_comma){
        Serial.println("No Comma!");
   }else if( pse == ParseStateError::inval_comb){
       Serial.println("Invalid Comb!");
   }
}


















