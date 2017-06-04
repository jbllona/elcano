#include "Elcano_Serial.h"
#include "Arduino.h"
#include <FastCRC.h>
#include <math.h>

FastCRC8 CRC8;
char buffer[64];      //used to collect strings from Sender 
int intBuffer[64];    //used to convert string number back to int 
int intBuffInx = 0;   //index for intBuff
int attriValue[5];
int i = 0;
int newCRC = 0;

// Elcano_Serial.h
// By HUY NGUYEN 
// 
// Manages our protocal for robust communication of SerialData over serial
// connections. Complete documentation and usage is on the wiki.
namespace elcano {
  ParseStateError ParseState::update(void) {
      Serial.println("HELLO**************************************");
      /*CLEAN BUFFERS*/
      newCRC =0;
      for(int j = 0; j<64;j++){
      attriValue[j]=NaN;
      }
      for(int j = 0; j<5;j++){
      buffer[j]=NaN;
      }
      for(int j = 0; j<64;j++){
      intBuffer[j]=NaN;
      }
      /*COLLECT GOOD DATA*/
      for(i = 0; i<64;i++){
        buffer[i]= input->read();
        if (buffer[i] == -1) return ParseStateError::unavailable;
        if (buffer[i] == '\t' || buffer[i] == '\0' || buffer[i] == '\r' ){ 
          return ParseStateError::noise_in_data ;
        }
        if(buffer[i] =='D' ||buffer[i] =='G'|| buffer[i] =='S' ||buffer[i] =='X'){
          i = 0;
        }
        if(buffer[i]=='\n'){  //the previous entry of buffer is not an attribute
          i++;
          break;
        }
        Serial.print(buffer[i]);
      }
      Serial.println("");
      Serial.println("ACTUAL ACCEPTED BUFFER:");
      for(int j = 0; j<i ; j++){
        Serial.print(buffer[j]);
      }
      Serial.println("RESULT:");
      if(buffer[0] == 'D'){
        return checkDrive();
      }else if(buffer[0] == 'G'){
        return checkGoal();
      }else if(buffer[0] =='S') {
        return checkSensor();
      }else if(buffer[0] == 'X'){
        return checkSeg();
      }else{
        Serial.println((char)buffer[0]);
        return ParseStateError::bad_type;
      }
  }
  bool SerialData::write(HardwareSerial * out){ 
      String num="";
      if(!verify()) return false;
      int position = 0;
      char buffer[64] ="";
      //DRIVE STRING---------------------------
      if(kind == MsgType::drive)
      {

        //create the drive string as D{s speed_cmPS}{a angle_mDeg}CRC\n
        

        //D{s speed_cmPS}
        buffer[position++] = 'D';
        out->print('D');
        buffer[position++] = '{';
        out->print('{');
        buffer[position++] = 's';
        out->print('s');
        buffer[position++] = ' ';
        out->print(' ');
    
        num = String(speed_cmPs);
        for(int i = 0; i < num.length(); i++)
        {
          buffer[position++] = num.charAt(i);
          out->print(buffer[position-1]);
        }
        buffer[position++] = '}';
        out->print('}');
       // buffer[position++] ='\n';
        //{a angle_mDeg}
        buffer[position++] = '{';
        out->print('{');
        buffer[position++] = 'a';
        out->print('a');
        buffer[position++] = ' ';
        out->print(' ');
        num = String(angle_mDeg);
        for(int i = 0; i < num.length(); i++)
        {
          buffer[position++] = num.charAt(i);
          out->print(buffer[position-1]);
        }
        buffer[position++] = '}';
        out->print('}');
       

       // buffer[position]=0;      
      }
      //SENSOR STRING--------------------------
      else if (kind == MsgType::sensor)
      {
        //create the sensor string as S{s speed_cmPS}{p posE_cm,posN_cm}{b bearing_deg}{a angle_mDeg}CRC\n
        //S{s speed_cmPS}
        buffer[position++] = 'S';
        out->print('S');
        buffer[position++] = '{';
        out->print('{');
        buffer[position++] = 's';
        out->print('s');
        buffer[position++] = ' ';
        out->print(' ');
        
        num = String(speed_cmPs);
        for(int i = 0; i < num.length(); i++)
        {
          buffer[position++] = num.charAt(i);
          out->print(buffer[position-1]);
        }
        buffer[position++] = '}';
        out->print('}');

        //{p posE_cm,posN_cm}
        buffer[position++] = '{';
        out->print('{');
        buffer[position++] = 'p';
        out->print('p');
        buffer[position++] = ' ';
        out->print(' ');

        num = String(posE_cm);
        for(int i = 0; i < num.length(); i++)
        {
          buffer[position++] = num.charAt(i);
          out->print(buffer[position-1]);
        }
        buffer[position++] = ',';
        out->print(',');

         num = String(posN_cm);
        for(int i = 0; i < num.length(); i++)
        {
          buffer[position++] = num.charAt(i);
          out->print(buffer[position-1]);
        }

        buffer[position++] = '}';
        out->print('}');

        //{b bearing_deg}
        buffer[position++] = '{';
        out->print('{');
        buffer[position++] = 'b';
        out->print('b');
        buffer[position++] = ' ';
        out->print(' ');

        num = String(bearing_deg);
        for(int i = 0; i < num.length(); i++)
        {
          buffer[position++] = num.charAt(i);
          out->print(buffer[position-1]);
        }
        buffer[position++] = '}';
        out->print('}');

        //{a angle_mDeg}
        buffer[position++] = '{';
        out->print('{');
        buffer[position++] = 'a';
        out->print('a');
        buffer[position++] = ' ';
        out->print(' ');

        num = String(angle_mDeg);
        for(int i = 0; i < num.length(); i++)
        {
          buffer[position++] = num.charAt(i);
          out->print(buffer[position-1]);
        }
        buffer[position++] = '}';
        out->print('}');
      }
      //GOAL STRING --------------------------------
      else if(kind == MsgType::goal)
      {
        //create the goal string as G{n number}{p posE_cm,posN_cm}{b bearing_deg}CRC\n
        //G{n number}
       // int position = 0;

        buffer[position++] = 'G';
        out->print('G');
        buffer[position++] = '{';
        out->print('{');
        buffer[position++] = 'n';
        out->print('n');
        buffer[position++] = ' ';
        out->print(' ');
        num = String(number);
        for(int i = 0; i < num.length(); i++)
        {
          buffer[position++] = num.charAt(i);
          out->print(buffer[position-1]);
        }
        buffer[position++] = '}';
        out->print('}');

        //{p posE_cm,posN_cm}
        buffer[position++] = '{';
        out->print('{');
        buffer[position++] = 'p';
        out->print('p');
        buffer[position++] = ' ';
        out->print(' ');
        num = String(posE_cm);
        for(int i = 0; i < num.length(); i++)
        {
          buffer[position++] = num.charAt(i);
          out->print(buffer[position-1]);
        }
        buffer[position++] = ',';
        out->print(',');
        num = String(posN_cm);
        for(int i = 0; i < num.length(); i++)
        {
          buffer[position++] = num.charAt(i);
          out->print(buffer[position-1]);
        }
        buffer[position++] = '}';
        out->print('}');
        //{b bearing_deg}
        buffer[position++] = '{';
        out->print('{');
        buffer[position++] = 'b';
        out->print('b');
        buffer[position++] = ' ';
        out->print(' ');
        num = String(bearing_deg);
        for(int i = 0; i < num.length(); i++)
        {
          buffer[position++] = num.charAt(i);
          out->print(buffer[position-1]);
        }
        buffer[position++] = '}';
        out->print('}');
      }
      //SEG STRING -------------------------------
      else if (kind == MsgType::seg)   
      {
        //create the seg string as X{n number}{p posE_cm,posN_cm}{b bearing_deg}{a angle_mDeg}CRC\n
        //X{n number}
        buffer[position++] = 'X'; 
        out->print('X');    
        buffer[position++] = '{';
        out->print('{');
        buffer[position++] = 'n';
        out->print('n');
        buffer[position++] = ' ';
        out->print(' ');
        
        String num = String(number);
        for(int i = 0; i < num.length(); i++)
        {
          buffer[position++] = num.charAt(i);
          out->print(buffer[position-1]);
        }
        buffer[position++] = '}';
        out->print('}');
        //{p posE_cm,posN_cm}
        buffer[position++] = '{'; 
        out->print('{');   
        buffer[position++] = 'p';
        out->print('p');
        buffer[position++] = ' ';
        out->print(' ');

        num = String(posE_cm);
        for(int i = 0; i < num.length(); i++)
        {
          buffer[position++] = num.charAt(i);
          out->print(buffer[position-1]);
        }
        buffer[position++] = ',';
        out->print(',');

        num = String(posN_cm);
        for(int i = 0; i < num.length(); i++)
        {
          buffer[position++] = num.charAt(i);
          out->print(buffer[position-1]);
        }

        buffer[position++] = '}';
        out->print('}');

        //{b bearing_deg}
        buffer[position++] = '{';
        out->print('{');
        buffer[position++] = 'b';
        out->print('b');
        buffer[position++] = ' ';
        out->print(' ');

        num = String(bearing_deg);
        for(int i = 0; i < num.length(); i++)
        {
          buffer[position++] = num.charAt(i);
          out->print(buffer[position-1]);
        }
        buffer[position++] = '}';
        out->print('}');

        //{a angle_mDeg}
        buffer[position++] = '{';
        out->print('{');
        buffer[position++] = 'a';
        out->print('a');
        buffer[position++] = ' ';
        out->print(' ');

        num = String(angle_mDeg);
        for(int i = 0; i < num.length(); i++)
        {
          buffer[position++] = num.charAt(i);
          out->print(buffer[position-1]);
        }
        buffer[position++] = '}';
        out->print('}');
        
      }
      buffer[position] = 0;
      out->print(CRC8.smbus(buffer, position));
      out->print('\n');
      return true;
  }
  void SerialData::clear(void) {
    kind        = MsgType::none;
    number      = NaN;
    speed_cmPs  = NaN;
    angle_mDeg   = NaN;
    bearing_deg = NaN;
    posE_cm     = NaN;
    posN_cm     = NaN;
    probability = NaN;
  }
  bool SerialData::verify() {
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
  // SUPPORTING FUNCTIONS FOR UPDATE() 
  bool ParseState::is_expectedChar(char c, char sample){
    if(c == sample){
      return true;
    }
    return false;
  }
  bool ParseState::is_Number(char c){
    if(c>='0' && c <='9'){
      return true;
    }
    return false;
  }
  int ParseState::char2Int(char c){
    int value = c -'0';
    return value;
  }
  int ParseState::intBuff2Int(int n){
    int sum = 0;
    int m = 0;
    int k =0;
    int testBuffRev[64];
    for (int k = n; k>=0; k--){
       testBuffRev[m] = intBuffer[k];
       m++;
    }
    for(int j = 0; j< m; j++){
      sum = sum + testBuffRev[j]*power(10,j);
    }
    return sum;
  }
  int ParseState::power(int base, int iexp){
    int power = 1;
    for(int j = 1; j<=iexp; j++){
      power = power*base;
    }
    return power;
  }
  ParseStateError ParseState::checkDrive(void){
    #pragma region
      //check the  drive string
      dt->kind = MsgType::drive;
      Serial.println("Check Drive");
      i =1;
      if(!is_expectedChar(buffer[i],'{')) {
        return ParseStateError::bad_lcurly;
      }
      i++;
      if(!is_expectedChar(buffer[i],'s')){
        return ParseStateError::bad_attrib;
      }
      i++;
      if(!is_expectedChar(buffer[i],' ')){
        return ParseStateError::no_space;
      }
      i++;
      while(buffer[i] != '}'){
        intBuffer[intBuffInx] = char2Int(buffer[i]);
        if(!is_Number(buffer[i])){
          return ParseStateError::bad_number;
        }
        i++;
        intBuffInx++;
      }
    #pragma endregion
    attriValue[0]= intBuff2Int(intBuffInx-1);//speed_cmPs
    intBuffInx =0;

    #pragma region
      if(!is_expectedChar(buffer[i],'}')){
        return ParseStateError::bad_lcurly;
      }
      i++;
        
      if(!is_expectedChar(buffer[i],'{')){
        return ParseStateError::bad_lcurly;
      }
      i++;
      if(!is_expectedChar(buffer[i],'a')){
        return ParseStateError::bad_attrib;
      }
      i++;
      if(!is_expectedChar(buffer[i],' ')){
        return ParseStateError::no_space;
      }
      i++;
      while(buffer[i] != '}'){
        intBuffer[intBuffInx] = char2Int(buffer[i]);
        if(!is_Number(buffer[i])){
          return ParseStateError::bad_number;
        }
        i++;
        intBuffInx++;
      }
    #pragma endregion
      attriValue[1] = intBuff2Int(intBuffInx-1);//angle_mDeg
      intBuffInx =0;

    #pragma region
      if(!is_expectedChar(buffer[i],'}')){
        Serial.println('4');
        return ParseStateError::bad_lcurly;
      }
      i++;

      //check CRC is a num 
      while(buffer[i] != '\n'){
        intBuffer[intBuffInx] = char2Int(buffer[i]);
        if(!is_Number(buffer[i])){
          return ParseStateError::bad_number;
        }
        i++;
        intBuffInx++;
      }
    #pragma endregion
    attriValue[2] = intBuff2Int(intBuffInx-1);//CRC from received string
    newCRC = CRC8.smbus(buffer,i-intBuffInx); //CRC generated by received string
    Serial.println(newCRC);
    intBuffInx = 0;

    if(attriValue[2] == newCRC){
      dt->speed_cmPs = attriValue[0];
      dt->angle_mDeg = attriValue[1];
      dt->crc = newCRC;
    }else{
      return ParseStateError::inval_comb;
    }

    //check the last item in String and confirm data is succesfully input 
    if(is_expectedChar(buffer[i],'\n') && dt->verify()){
      return ParseStateError::success;
    }else{
      return ParseStateError::inval_comb; 
    }
  }
  ParseStateError ParseState::checkGoal(void){
    //check the goal string 
    #pragma region
      Serial.println("Check Goal");
      dt->kind = MsgType::goal;
      i =1;
   
      //check_CurlyPack(1,'s');++++++++++++++++++++++++++
      if(!is_expectedChar(buffer[i],'{')) {
        return ParseStateError::bad_lcurly;
      }
      i++;

      if(!is_expectedChar(buffer[i],'n')){
        return ParseStateError::bad_attrib;
      }
      i++;
     
      if(!is_expectedChar(buffer[i],' ')){
        return ParseStateError::no_space;
      }
      i++;
    
      while(buffer[i] != '}'){
        intBuffer[intBuffInx] = char2Int(buffer[i]);
     
        if(!is_Number(buffer[i])){
          return ParseStateError::bad_number;
        }
        i++;
        intBuffInx++;
      }
    #pragma endregion
      attriValue[0] = intBuff2Int(intBuffInx-1);
      intBuffInx =0;
    
    #pragma region
      if(!is_expectedChar(buffer[i],'}')){
        return ParseStateError::bad_lcurly;
      }
      i++;
      if(!is_expectedChar(buffer[i],'{')) {
        return ParseStateError::bad_lcurly;
      }
      i++;
     
      if(!is_expectedChar(buffer[i],'p')){
        return ParseStateError::bad_attrib;
      }
      i++;
  
      if(!is_expectedChar(buffer[i],' ')){
        return ParseStateError::no_space;
      }
      i++; 
      while(buffer[i] != ','){
        intBuffer[intBuffInx] = char2Int(buffer[i]);
        if(!is_Number(buffer[i])){
          return ParseStateError::bad_number;
        }
        i++;
        intBuffInx++;
      }
    #pragma endregion
      attriValue[1] = intBuff2Int(intBuffInx-1);
      intBuffInx =0;

    #pragma region
      if(!is_expectedChar(buffer[i],',')){
        return ParseStateError::no_comma;
      } 
      i++;
   
      while(buffer[i] != '}'){
        intBuffer[intBuffInx] = char2Int(buffer[i]);
        if(!is_Number(buffer[i])){
          return ParseStateError::bad_number;
        }
        i++;
        intBuffInx++;
      }
    #pragma endregion
      attriValue[2] = intBuff2Int(intBuffInx-1);
      intBuffInx =0;

    #pragma region
      if(!is_expectedChar(buffer[i],'}')){
        return ParseStateError::bad_lcurly;
      }
      i++;
    
      if(!is_expectedChar(buffer[i],'{')) {
        return ParseStateError::bad_lcurly;
      }
      i++;
     
      if(!is_expectedChar(buffer[i],'b')){
        return ParseStateError::bad_attrib;
      }
      i++;
     
      if(!is_expectedChar(buffer[i],' ')){
        return ParseStateError::no_space;
      }
      i++;
      
      while(buffer[i] != '}'){
        intBuffer[intBuffInx] = char2Int(buffer[i]);
        if(!is_Number(buffer[i])){
          return ParseStateError::bad_number;
        }
        i++;
        intBuffInx++;
      }
    #pragma endregion
      attriValue[3] = intBuff2Int(intBuffInx-1);
      intBuffInx =0;

    #pragma region
      if(!is_expectedChar(buffer[i],'}')){
        return ParseStateError::bad_lcurly;
      }
      i++;
      while(buffer[i] != '\n'){
        intBuffer[intBuffInx] = char2Int(buffer[i]);
        if(!is_Number(buffer[i])){
          return ParseStateError::bad_number;
        }
        i++;
        intBuffInx++;
      }
    #pragma endregion
      attriValue[4] = intBuff2Int(intBuffInx-1);
      newCRC = CRC8.smbus(buffer,i-intBuffInx); //CRC generated by received string
      Serial.print("number:");
      Serial.println(attriValue[0]);
      Serial.print("posE_cm:");
      Serial.println(attriValue[1]);
      Serial.print("posN_cm:");
      Serial.println(attriValue[2]);
      Serial.print("bearing_deg:");
      Serial.println(attriValue[3]);
      Serial.print("old CRC:");
      Serial.println(attriValue[4]);
      Serial.println(newCRC);
      Serial.print("newCRC:");
      Serial.println(newCRC);
      intBuffInx = 0;

      if(attriValue[4] == newCRC){
        dt->number = attriValue[0];
        dt->posE_cm = attriValue[1];
        dt->posN_cm = attriValue[2];
        dt->bearing_deg = attriValue[3];
        dt->crc = newCRC;
      }else{
        return ParseStateError::inval_comb;
      }

      if(is_expectedChar(buffer[i],'\n')){
        return ParseStateError::success;
      }else{
        return ParseStateError::inval_comb; 
      }  
  }
  ParseStateError ParseState::checkSensor(void){
    //check the sensor string as S{s speed_cmPS}{p posE_cm,posN_cm}{b bearing_deg}{a angle_mDeg}CRC\n
    #pragma region 
      Serial.println("Check Sensor");
      dt->kind = MsgType::sensor;
      i =1;
      Serial.println(i);
      //check_CurlyPack(1,'s');++++++++++++++++++++++++++
      if(!is_expectedChar(buffer[i],'{')) {
        return ParseStateError::bad_lcurly;
      }
      i++;
      Serial.println(i);
      if(!is_expectedChar(buffer[i],'s')){
        return ParseStateError::bad_attrib;
      }
      i++;
      Serial.println(i);
      if(!is_expectedChar(buffer[i],' ')){
        return ParseStateError::no_space;
      }
      i++;
      Serial.println(i);
      while(buffer[i] != '}'){
        intBuffer[intBuffInx] = char2Int(buffer[i]);
        if(!is_Number(buffer[i])){
          return ParseStateError::bad_number;
        }
        i++;
        intBuffInx++;
      }  //checking "S{s speed_cmPS" then store speed_cmPs
    #pragma endregion
      attriValue[0] = intBuff2Int(intBuffInx-1);
      intBuffInx =0;

    #pragma region
      if(!is_expectedChar(buffer[i],'}')){
        return ParseStateError::bad_lcurly;
      }
      i++;
    
       // check_CurlyPack(temp,'p'); 
      //+++++++++++++++++++++++++
      if(!is_expectedChar(buffer[i],'{')) {
        return ParseStateError::bad_lcurly;
      }
      i++;
      if(!is_expectedChar(buffer[i],'p')){

        return ParseStateError::bad_attrib;
      }
      i++;
      if(!is_expectedChar(buffer[i],' ')){
        return ParseStateError::no_space;
      }
      i++;
      Serial.println(i);
      while(buffer[i] != ','){
        intBuffer[intBuffInx] = char2Int(buffer[i]);
        if(!is_Number(buffer[i])){
          return ParseStateError::bad_number;
        }
        i++;
        intBuffInx++;     
      }  //checking "}{p posE_cm" then store posE_cm
    #pragma endregion
      attriValue[1] = intBuff2Int(intBuffInx-1);
      intBuffInx =0;

    #pragma region 
      if(!is_expectedChar(buffer[i],',')){
        return ParseStateError::no_comma;//
      }
      i++;
      while(buffer[i] != '}'){
        intBuffer[intBuffInx] = char2Int(buffer[i]);
        if(!is_Number(buffer[i])){
          return ParseStateError::bad_number;
        }
        i++;
        intBuffInx++;
      }  //checking ",posN_cm" then store posN_cm
    #pragma endregion
      attriValue[2]= intBuff2Int(intBuffInx-1);
      intBuffInx =0;

    #pragma region
      if(!is_expectedChar(buffer[i],'}')){
        return ParseStateError::bad_lcurly;
      }
      i++;
    
      if(!is_expectedChar(buffer[i],'{')) {
        return ParseStateError::bad_lcurly;
      }
      i++;
     
      if(!is_expectedChar(buffer[i],'b')){
        return ParseStateError::bad_attrib;
      }
      i++;

      if(!is_expectedChar(buffer[i],' ')){
        return ParseStateError::no_space;
      }
      i++;
   
      while(buffer[i] != '}'){
        intBuffer[intBuffInx] = char2Int(buffer[i]);
        if(!is_Number(buffer[i])){
          return ParseStateError::bad_number;
        }
        i++;
        intBuffInx++;
      } //checking "}{b bearing_deg" then store bearing_deg
    #pragma endregion
      attriValue[3] = intBuff2Int(intBuffInx-1);
      intBuffInx =0;

    #pragma region
      if(!is_expectedChar(buffer[i],'}')){
        return ParseStateError::bad_lcurly;
      }
      i++;

      if(!is_expectedChar(buffer[i],'{')) {
        return ParseStateError::bad_lcurly;
      }
      i++;

      if(!is_expectedChar(buffer[i],'a')){
        return ParseStateError::bad_attrib;
      }
      i++;

      if(!is_expectedChar(buffer[i],' ')){
        return ParseStateError::no_space;
      }
      i++;

      while(buffer[i] != '}'){
        intBuffer[intBuffInx] = char2Int(buffer[i]);
        if(!is_Number(buffer[i])){
          return ParseStateError::bad_number;
        }
        i++;
        intBuffInx++;
        
      } //checking "}{a angle_mDeg" then store angle_mDeg
    #pragma endregion
      attriValue[4] = intBuff2Int(intBuffInx-1);
      intBuffInx =0;

    #pragma region
      if(!is_expectedChar(buffer[i],'}')){
        return ParseStateError::bad_lcurly;
      }
      i++;
      ///check CRC num & end 
      while(buffer[i] != '\n'){
        intBuffer[intBuffInx] = char2Int(buffer[i]);
        if(!is_Number(buffer[i])){
          Serial.println(buffer[i]);
          return ParseStateError::bad_number;
        }
        i++;
        intBuffInx++;
      } //checking "}CRC" then store CRC 
    #pragma endregion
      attriValue[5] = intBuff2Int(intBuffInx-1);

      /*GENERATE NEW CRC FROM RECEIVED STRING*/
      newCRC = CRC8.smbus(buffer,i-intBuffInx); 
      Serial.println(newCRC);
      intBuffInx = 0;

      /*CHECK CRC & ASSIGN VALUES OF ATTRIBUTE*/
      if(attriValue[5] == newCRC){
        dt->speed_cmPs = attriValue[0];
        dt->posE_cm = attriValue[1];
        dt->posN_cm = attriValue[2];
        dt->bearing_deg = attriValue[3];
        dt->angle_mDeg = attriValue[4];
        dt->crc = newCRC;
      }else{
        return ParseStateError::inval_comb;
      }

      /*FINAL CHECK*/
      if(is_expectedChar(buffer[i],'\n')){
        return ParseStateError::success;
      }else{
        return ParseStateError::inval_comb; 
      }  
  }
  ParseStateError ParseState::checkSeg(void){
    //check the seg string as X{n number}{p posE_cm,posN_cm}{b bearing_deg}{a angle_mDeg}CRC\n
    #pragma region         //checking till number 
      Serial.println("Check Seg");
      dt->kind = MsgType::seg;
      i =1;
      if(!is_expectedChar(buffer[i],'{')) {
        return ParseStateError::bad_lcurly;
      }
      i++;
      if(!is_expectedChar(buffer[i],'n')){
        return ParseStateError::bad_attrib;
      }
      i++;
      if(!is_expectedChar(buffer[i],' ')){
        return ParseStateError::no_space;
      }
      i++;
      while(buffer[i] != '}'){
        intBuffer[intBuffInx] = char2Int(buffer[i]);
        if(!is_Number(buffer[i])){
          return ParseStateError::bad_number;
        }
        i++;
        intBuffInx++;;
      }
    #pragma endregion
      attriValue[0] = intBuff2Int(intBuffInx-1);
      intBuffInx =0;
    #pragma region
      if(!is_expectedChar(buffer[i],'}')){
        return ParseStateError::bad_lcurly;
      }
      i++;
      if(!is_expectedChar(buffer[i],'{')) {
        return ParseStateError::bad_lcurly;
      }
      i++;
      if(!is_expectedChar(buffer[i],'p')){
        return ParseStateError::bad_attrib;
      }
      i++;
      if(!is_expectedChar(buffer[i],' ')){
        return ParseStateError::no_space;
      }
      i++;
      while(buffer[i] != ','){
        intBuffer[intBuffInx] = char2Int(buffer[i]);
        if(!is_Number(buffer[i])){
          return ParseStateError::bad_number;
        }
        i++;
        intBuffInx++;
      }   //checking till posE_cm
    #pragma endregion
      attriValue[1] = intBuff2Int(intBuffInx-1);
      intBuffInx =0;
    
    #pragma region     //checking till posN_cm
      if(!is_expectedChar(buffer[i],',')){
        return ParseStateError::no_comma;//
      }
     
      i++;
      Serial.println(i);
      while(buffer[i] != '}'){
        intBuffer[intBuffInx] = char2Int(buffer[i]);
        if(!is_Number(buffer[i])){
          return ParseStateError::bad_number;
        }
        i++;
        intBuffInx++;
      }
    #pragma endregion
      attriValue[2] = intBuff2Int(intBuffInx-1);
      intBuffInx =0;

    #pragma region
      if(!is_expectedChar(buffer[i],'}')){
        return ParseStateError::bad_lcurly;
      }
      i++;
  
      if(!is_expectedChar(buffer[i],'{')) {
        return ParseStateError::bad_lcurly;
      }
      i++;

      if(!is_expectedChar(buffer[i],'b')){
        return ParseStateError::bad_attrib;
      }
      i++;

      if(!is_expectedChar(buffer[i],' ')){
        return ParseStateError::no_space;
      }
      i++;
   
      while(buffer[i] != '}'){
        intBuffer[intBuffInx] = char2Int(buffer[i]);
        if(!is_Number(buffer[i])){
          return ParseStateError::bad_number;
        }
        i++;
        intBuffInx++; 
      }  //check till bearing_deg
    #pragma endregion
      attriValue[3]= intBuff2Int(intBuffInx-1);
      intBuffInx =0;

    #pragma region
      if(!is_expectedChar(buffer[i],'}')){
        return ParseStateError::bad_lcurly;
      }
      i++;

      if(!is_expectedChar(buffer[i],'{')) {
        return ParseStateError::bad_lcurly;
      }
      i++;

      if(!is_expectedChar(buffer[i],'a')){
        return ParseStateError::bad_attrib;
      }
      i++;
 
      if(!is_expectedChar(buffer[i],' ')){
        return ParseStateError::no_space;
      }
      i++;
      while(buffer[i] != '}'){
        intBuffer[intBuffInx] = char2Int(buffer[i]);
        if(!is_Number(buffer[i])){
          return ParseStateError::bad_number;
        }
        i++;
        intBuffInx++; 
      }
    #pragma endregion
      attriValue[4] = intBuff2Int(intBuffInx-1);
      intBuffInx =0;

    #pragma region
      if(!is_expectedChar(buffer[i],'}')){
        return ParseStateError::bad_lcurly;
      }
      i++;
      while(buffer[i] != '\n'){
        intBuffer[intBuffInx] = char2Int(buffer[i]);
        if(!is_Number(buffer[i])){
          return ParseStateError::bad_number;
        }
        i++;
        intBuffInx++; 
      }
    #pragma endregion
      attriValue[5]= intBuff2Int(intBuffInx-1);

      /*GENERATE NEW CRC FROM RECEIVED STRING*/
      newCRC = CRC8.smbus(buffer,i-intBuffInx); 
      Serial.println(newCRC);
      intBuffInx = 0;

      /*CHECK CRC & ASSIGN VALUES OF ATTRIBUTE*/
      if(attriValue[5] == newCRC){
        dt->number = attriValue[0];
        dt->posE_cm = attriValue[1];
        dt->posN_cm = attriValue[2];
        dt->bearing_deg = attriValue[3];
        dt->angle_mDeg = attriValue[4];
        dt->crc = newCRC;
      }else{
        return ParseStateError::inval_comb;
      }

      if(is_expectedChar(buffer[i],'\n')){
        return ParseStateError::success;
      }else{
        return ParseStateError::inval_comb; 
      }  
  }
} // namespace elcano