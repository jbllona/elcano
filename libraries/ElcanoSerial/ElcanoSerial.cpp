#include "ElcanoSerial.h"
#include <FastCRC.h>
FastCRC8 CRC8;

namespace elcano
{	
	bool SerialData::write(HardwareSerial * out)
	{
		if(!verify()) return false;
    //DRIVE STRING---------------------------
		if(kind == MsgType::drive)
		{
			//create the drive string as D{s speed_cmPS}{a angle_mDeg}CRC\n
			int position = 0;
			char buffer[64];

      //D{s speed_cmPS}
			buffer[position++] = 'D';
			buffer[position++] = '{';
			buffer[position++] = 's';
			buffer[position++] = ' ';
			
			String num = String(speed_cmPs);
			for(int i = 0; i < num.length(); i++)
			{
				buffer[position++] = num.charAt(i);
			}
			buffer[position++] = '}';

      //{a angle_mDeg}
			buffer[position++] = '{';
			buffer[position++] = 'a';
			buffer[position++] = ' ';
			num = String(angle_mDeg);
			for(int i = 0; i < num.length(); i++)
			{
				buffer[position++] = num.charAt(i);
			}
			buffer[position++] = '}';
			out->print(buffer);
			out->print(CRC8.smbus(buffer, position));
			out->print('\n');
			
		}
    //SENSOR STRING--------------------------
    else if (kind == MsgType::sensor)
    {
      //create the sensor string as S{s speed_cmPS}{p posE_cm,posN_cm}{b bearing_deg}{a angle_mDeg}CRC\n
      int position = 0;
      char buffer[64];

      //S{s speed_cmPS}
      buffer[position++] = 'S';
      buffer[position++] = '{';
      buffer[position++] = 's';
      buffer[position++] = ' ';
      
      String num = String(speed_cmPs);
      for(int i = 0; i < num.length(); i++)
      {
        buffer[position++] = num.charAt(i);
      }
      buffer[position++] = '}';

      //{p posE_cm,posN_cm}
      buffer[position++] = '{';
      buffer[position++] = 'p';
      buffer[position++] = ' ';

      num = String(posE_cm);
      for(int i = 0; i < num.length(); i++)
      {
        buffer[position++] = num.charAt(i);
      }
      buffer[position++] = ',';

       num = String(posN_cm);
      for(int i = 0; i < num.length(); i++)
      {
        buffer[position++] = num.charAt(i);
      }

      buffer[position++] = '}';

      //{b bearing_deg}
      buffer[position++] = '{';
      buffer[position++] = 'b';

      num = String(bearing_deg);
      for(int i = 0; i < num.length(); i++)
      {
        buffer[position++] = num.charAt(i);
      }
      buffer[position++] = '}';

      //{a angle_mDeg}
      buffer[position++] = '{';
      buffer[position++] = 'a';
      buffer[position++] = ' ';
      num = String(angle_mDeg);
      for(int i = 0; i < num.length(); i++)
      {
        buffer[position++] = num.charAt(i);
      }
      buffer[position++] = '}';

      out->print(buffer);
      out->print(CRC8.smbus(buffer, position));
      out->print('\n');
    }
    //GOAL STRING --------------------------------
    else if(kind == MsgType::goal)
    {
      //create the goal string as G{n number}{p posE_cm,posN_cm}{b bearing_deg}CRC\n
      int position = 0;
      char buffer[64];

      //G{n number}
      int position = 0;
      buffer[position++] = 'G';
      buffer[position++] = '{';
      buffer[position++] = 'n';
      buffer[position++] = ' ';
      num = String(number);
      for(int i = 0; i < num.length(); i++)
      {
        buffer[position++] = num.charAt(i);
      }
      buffer[position++] = '}';

      //{p posE_cm,posN_cm}
      buffer[position++] = '{';
      buffer[position++] = 'p';
      buffer[position++] = ' ';
      num = String(posE_cm);
      for(int i = 0; i < num.length(); i++)
      {
        buffer[position++] = num.charAt(i);
      }
      buffer[position++] = ',';
      num = String(posN_cm);
      for(int i = 0; i < num.length(); i++)
      {
        buffer[position++] = num.charAt(i);
      }
      buffer[position++] = '}';

      //{b bearing_deg}
      buffer[position++] = '{';
      buffer[position++] = 'b';
      num = String(bearing_deg);
      for(int i = 0; i < num.length(); i++)
      {
        buffer[position++] = num.charAt(i);
      }
      buffer[position++] = '}';
     

      out->print(buffer);
      out->print(CRC8.smbus(buffer, position));
      out->print('\n');
   
    }
    //SEG STRING -------------------------------
    else if (kind == MsgType::seg)   
    {
      //create the seg string as X{n number}{p posE_cm,posN_cm}{b bearing_deg}{a angle_mDeg}CRC\n
      int position = 0;
      char buffer[64];

      //X{n number}
      buffer[position++] = 'X';     
      buffer[position++] = '{';
      buffer[position++] = 'n';
      buffer[position++] = ' ';
      
      String num = String(number);
      for(int i = 0; i < num.length(); i++)
      {
        buffer[position++] = num.charAt(i);
      }
      buffer[position++] = '}';

      //{p posE_cm,posN_cm}
      buffer[position++] = '{';    
      buffer[position++] = 'p';
      buffer[position++] = ' ';

      num = String(posE_cm);
      for(int i = 0; i < num.length(); i++)
      {
        buffer[position++] = num.charAt(i);
      }
      buffer[position++] = ',';

       num = String(posN_cm);
      for(int i = 0; i < num.length(); i++)
      {
        buffer[position++] = num.charAt(i);
      }

      buffer[position++] = '}';

      //{b bearing_deg}
      buffer[position++] = '{';
      buffer[position++] = 'b';
      buffer[position++] = ' ';

      num = String(bearing_deg);
      for(int i = 0; i < num.length(); i++)
      {
        buffer[position++] = num.charAt(i);
      }
      buffer[position++] = '}';

      //{a angle_mDeg}
      buffer[position++] = '{';
      buffer[position++] = 'a';
      buffer[position++] = ' ';
      num = String(angle_mDeg);
      for(int i = 0; i < num.length(); i++)
      {
        buffer[position++] = num.charAt(i);
      }
      buffer[position++] = '}';

      out->print(buffer);
      out->print(CRC8.smbus(buffer, position));
      out->print('\n');
    }
  }

	void SerialData::clear()
	{
		kind = MsgType::none;
		number = NaN;
		speed_cmPs = NaN;
		angle_mDeg = NaN;
		bearing_deg = NaN;
		posE_cm = NaN;
		posN_cm = NaN;
		probability = NaN;
		outSize = 0;
		for(int i = 0; i < 60; i++) outBuffer[i] = 0;
	}

	bool SerialData::verify()
	{
		switch (kind) {
	    case MsgType::drive: 
	      if (speed_cmPs  == NaN) return false;
	      if (angle_mDeg   == NaN) return false;
	      break;
	    case MsgType::sensor: 
	      if (speed_cmPs  == NaN) return false;
	      if (posE_cm     == NaN) return false;
	      if (posN_cm     == NaN) return false;
	      if (bearing_deg == NaN) return false;
	      if (angle_mDeg   == NaN) return false;
	      break;
	    case MsgType::goal:
	      if (number      == NaN) return false;
	      if (posE_cm     == NaN) return false;
	      if (posN_cm     == NaN) return false;
	      if (bearing_deg == NaN) return false;
	      break;
	    case MsgType::seg:
	      if (number      == NaN) return false;
	      if (posE_cm     == NaN) return false;
	      if (posN_cm     == NaN) return false;
	      if (bearing_deg == NaN) return false;
	      if (speed_cmPs  == NaN) return false;
	      break;
	    default:
	      return false;
	    }
	    return true;
	}
  
  ParseStateError ParseState::update(void)
  {
       //Update statement 
    numStarted = false;
    int c,b;
    while(1){
      for (int i = 0; i < 64; i++){  
        c = input->read();
        if (c == -1) return ParseStateError:unavailable;
        if (c != ' ' || c != '\t' || c != '\0' || c != '\r') outBuffer[i] = c;
      }

      dt->clear()
      for (int i = 0; i<64; i++){
        b = outBuffer[i];
        
        //State 0 problems -- 
        if      (state == 0 && b == 'D' && (capture & MsgType:: drive )) { dt->kind = MsgType::drive;  state = 1;}
        else if (state == 0 && b == 'S' && (capture & MsgType:: sensor)) { dt->kind = MsgType::sensor; state = 1;} 
        else if (state == 0 && b == 'G' && (capture & MsgType:: goal  )) { dt->kind = MsgType::goal;   state = 1;} 
        else if (state == 0 && b == 'X' && (capture & MsgType:: seg   )) { dt->kind = MsgType::seg;    state = 1;}
        else if (state == 0)                                                                           state = 50;
        //State 50 problems -- check if pass thru ( no title but end)
        if      (state == 50 && c == '\n') { state = 0; return ParseStateError::passthru;}
        //State 1 problems  --  
        if      (state == 1 && b == '\n')  { state = 0; return dt->verify() ? ParseStateError::success : ParseStateError::inval_comb;}
        else if (state == 1 && b == '{')     state = 2;
        else                               { state = 0; return ParseStateError::bad_lcurly;}
        
        //State 2 problems 
        if      (state == 2 && ( b != 'n' || b != 's' || b != 'a' ||  b != 'r' || b != 'p' )) {state = 0; return ParseStateError::bad_attrib; }
        else if (state == 2 &&  b = 'p' ) state = 3;
        else if (state == 2)              state = 4;
        
        //State 3-5-6-7-8 problems -check position statement
        if(state == 3 && b != ' ')       {state = 0; return ParseStateError::bad_attrib; }
        else if (state == 3 && b == ' ' ) state 5;


        if(state == 5){
          if(dt->verify()){
            state = 6;
          } else ParseStateError::inval_comb;
        }

        if(state == 6 && b == ',') state = 7;
        else { state = 0; return ParseStateError::bad_attrib;}

        if(state == 7){
          if(dt->verify()){
            state = 8;
          }
        }else {state = 0;ParseStateError::inval_comb;}

        if(state == 8 && b =='}') state = 1;
        else {state = 0; ParseStateError::bad_attrib;}

      
        //State 4
        if(state == 4 && b != ' ')       {state = 0; return ParseStateError::bad_attrib; }
        else if (state == 4 && b == ' ' ) state 9;

        if(state == 9){
          if(dt->verify()){
            state = 10;
          }else {state =0;  ParseStateError::inval_comb;}
        }

        if(state == 10 && b =='}')state = 1;
        else {state = 0; ParseStateError::bad_attrib;}


       


















        if(state == 3) 
        
        if (state == 3 && (capture & MsgType::drive) && b !=',')) {return ParseStateError::bad_attrib;
        else if (state == 3 && (capture & MsgType::drive) && b ==',') state = 5;
        else if     (state == 3 ) {
          if (dt->verify()){
            state = 4;
            ParseStateError::success
          }else ParseStateError::inval_comb;
        }

        //state 4 



        //State 4 problems - check closing "}"
        if      (state == 4 && b!='}'){
                state = 0;
        }

        //State 5 problems. check posN_cm
        if      (state == 5) {
          if (dt->verify()){
            state = 4;
            ParseStateError::success
          }
          else ParseStateError::inval_comb; 
        }
      }
    }  
  }
}



































