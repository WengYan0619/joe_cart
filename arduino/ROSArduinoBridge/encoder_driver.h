/* *************************************************************
   Encoder driver function definitions - by James Nugen
   ************************************************************ */
   
   
#ifdef ARDUINO_ENC_COUNTER
  //below can be changed, but should be PORTD pins; 
  //otherwise additional changes in the code are required
  #define LEFT_ENC_PIN_A PD2  //pin 2
  #define LEFT_ENC_PIN_B PD3  //pin 3
  
  //below can be changed, but should be PORTC pins
  #define RIGHT_ENC_PIN_A PC4  //pin A4
  #define RIGHT_ENC_PIN_B PC5   //pin A5
#endif

#ifdef ARDUINIO_MEGA_ENC_COUNTER
  const byte LEFT_ENC_A = 18;
  const byte LEFT_ENC_B = 19;
  const byte RIGHT_ENC_A = 20;
  const byte RIGHT_ENC_B = 21;
#endif
   
long readEncoder(int i);
void resetEncoder(int i);
void resetEncoders();