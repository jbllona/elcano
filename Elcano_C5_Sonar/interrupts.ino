//// -------------------------------------------------------------------------------------------------
//void ISRLeft()
//{  
//    //ISR for end of pulse on sonar 9, 10 or 11    
//    noInterrupts();
//
//    isr1Count++;
//
//    //if (digitalRead(PW_OR3) == HIGH && timeStartSet == false)
//    if (ibh(PINE, PE6) && !timeStartSet)
//    {
//        timeStartSet = true;      
//        timeStart = micros();
//    }
//    else
//    {
//        timeLeft = micros();        
//        SignalsReceived++;
//    }
//    interrupts();
//}
//
//
//// -------------------------------------------------------------------------------------------------
//void ISRCenter()
//{  
//    //ISR for end of pulse on sonar 6 or 12
//    noInterrupts();
//
//    isr2Count++;
//
//    //if (digitalRead(PW_OR1) == HIGH && timeStartSet == false)
//    if (ibh(PIND, PD0) && !timeStartSet)
//    {
//        timeStart = micros();
//        timeStartSet = true;
//    }
//    else
//    {
//        timeCenter = micros();        
//        SignalsReceived++;
//    }
//
//    interrupts();
// }
// 
//
//// -------------------------------------------------------------------------------------------------
//void ISRRight()
//{
//    //ISR for end of pulse on sonar 1, 2 or 3
//    noInterrupts(); 
//
//    isr3Count++; 
//
//    //if (digitalRead(PW_OR2) == HIGH && timeStartSet == false)
//    if (ibh(PIND, PD1) && !timeStartSet)
//    {
//        timeStart = micros();
//        timeStartSet = true;
//    }
//    else
//    {
//        timeRight = micros();        
//        SignalsReceived++;
//    }
//    interrupts();
//}
