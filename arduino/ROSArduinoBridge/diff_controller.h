/* Functions and type-defs for PID control.

   Taken mostly from Mike Ferguson's ArbotiX code which lives at:
   
   http://vanadium-ros-pkg.googlecode.com/svn/trunk/arbotix/
*/

/* PID setpoint info For a Motor */
typedef struct {
  double TargetTicksPerFrame;    // target speed in ticks per frame
  long Encoder;                  // encoder count
  long PrevEnc;                  // last encoder count

  /*
  * Using previous input (PrevInput) instead of PrevError to avoid derivative Ki_rightck,
  * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-derivative-Ki_rightck/
  */
  int PrevInput;                // last input
  //int PrevErr;                   // last error

  /*
  * Using integrated term (ITerm) instead of integrated error (Ierror),
  * to allow tuning changes,
  * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/
  */
  //int Ierror;
  int ITerm;                    //integrated term

  long output;                    // last motor setting
}
SetPointInfo;

SetPointInfo leftPID, rightPID;

/* PID Parameters */
int Kp_right = 30;
int Kd_right = 1;
int Ki_right = 3;
int Ko_right = 100;

int Kp_left = 40;
int Kd_left = 35;
int Ki_left = 2;
int Ko_left = 100;

unsigned char moving = 0; // is the base in motion?

/*
* Initialize PID variables to zero to prevent startup spikes
* when turning PID on to start moving
* In particular, assign both Encoder and PrevEnc the current encoder value
* See http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-initialization/
* Note that the assumption here is that PID is only turned on
* when going from stop to moving, that's why we can init everything on zero.
*/
void resetPID(){
   leftPID.TargetTicksPerFrame = 0.0;
   leftPID.Encoder = readEncoder(LEFT);
   leftPID.PrevEnc = leftPID.Encoder;
   leftPID.output = 0;
   leftPID.PrevInput = 0;
   leftPID.ITerm = 0;

   rightPID.TargetTicksPerFrame = 0.0;
   rightPID.Encoder = readEncoder(RIGHT);
   rightPID.PrevEnc = rightPID.Encoder;
   rightPID.output = 0;
   rightPID.PrevInput = 0;
   rightPID.ITerm = 0;
}

/* PID routine to compute the next motor commands */
void doPIDRight(SetPointInfo * p) {
  long Perror;
  long output;
  int input;

  //Perror = p->TargetTicksPerFrame - (p->Encoder - p->PrevEnc);
  input = p->Encoder - p->PrevEnc;
  Perror = p->TargetTicksPerFrame - input;

  Serial.print("Input:");
  Serial.print(input);
  Serial.print(" TargetTicks: ");
  Serial.print(p->TargetTicksPerFrame);
  Serial.print(" Perror: ");
  Serial.print(Perror);
  Serial.print(" Kp_right: ");
  Serial.print(Kp_right);
  Serial.print(" Kd_right: ");
  Serial.print(Kd_right);
  Serial.print(" Ki_right: ");
  Serial.print(Ki_right);
  Serial.print(" PrevInput: ");
  Serial.print(p->PrevInput);
  Serial.print(" ITerm: ");
  Serial.print(p->ITerm);


  /*
  * Avoid derivative Ki_rightck and allow tuning changes,
  * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-derivative-Ki_rightck/
  * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/
  */
  //output = (Kp_right * Perror + Kd_right * (Perror - p->PrevErr) + Ki_right * p->Ierror) / Ko_right;
  // p->PrevErr = Perror;
  long output_temp = (Kp_right * Perror - Kd_right * (input - p->PrevInput) + p->ITerm);

  Serial.print(output_temp);
  Serial.print(" Output NO Ko_right: ");

  output = output_temp / Ko_right;
  p->PrevEnc = p->Encoder;
  
  Serial.print(" Not added with prev output:");
  Serial.print(output);

  output += p->output;
  // Accumulate Integral error *or* Limit output.
  // Stop accumulating when output saturates

  Serial.print(" Added with prev output:");
  Serial.println(output);

  if (output >= MAX_PWM)
    output = MAX_PWM;
  else if (output <= -MAX_PWM)
    output = -MAX_PWM;
  else
  /*
  * allow turning changes, see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/
  */
    p->ITerm += Ki_right * Perror;

  p->output = output;
  p->PrevInput = input;

  
}

void doPIDLeft(SetPointInfo * p) {
  long Perror;
  long output;
  int input;

  //Perror = p->TargetTicksPerFrame - (p->Encoder - p->PrevEnc);
  input = p->Encoder - p->PrevEnc;
  Perror = p->TargetTicksPerFrame - input;

  Serial.print("Input:");
  Serial.print(input);
  Serial.print(" TargetTicks: ");
  Serial.print(p->TargetTicksPerFrame);
  Serial.print(" Perror: ");
  Serial.print(Perror);
  Serial.print(" Kp_left: ");
  Serial.print(Kp_left);
  Serial.print(" Kd_left: ");
  Serial.print(Kd_left);
  Serial.print(" Ki_left: ");
  Serial.print(Ki_left);
  Serial.print(" PrevInput: ");
  Serial.print(p->PrevInput);
  Serial.print(" ITerm: ");
  Serial.print(p->ITerm);


  /*
  * Avoid derivative Ki_leftck and allow tuning changes,
  * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-derivative-Ki_leftck/
  * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/
  */
  //output = (Kp_left * Perror + Kd_left * (Perror - p->PrevErr) + Ki_left * p->Ierror) / Ko_left;
  // p->PrevErr = Perror;
  long output_temp = (Kp_left * Perror - Kd_left * (input - p->PrevInput) + p->ITerm);

  Serial.print(output_temp);
  Serial.print(" Output NO Ko_left: ");

  output = output_temp / Ko_left;
  p->PrevEnc = p->Encoder;
  
  Serial.print(" Not added with prev output:");
  Serial.print(output);

  output += p->output;
  // Accumulate Integral error *or* Limit output.
  // Stop accumulating when output saturates

  Serial.print(" Added with prev output:");
  Serial.println(output);

  if (output >= MAX_PWM)
    output = MAX_PWM;
  else if (output <= -MAX_PWM)
    output = -MAX_PWM;
  else
  /*
  * allow turning changes, see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/
  */
    p->ITerm += Ki_left * Perror;

  p->output = output;
  p->PrevInput = input;

  
}

/* Read the encoder values and call the PID routine */
void updatePID() {
  /* Read the encoders */
  leftPID.Encoder = readEncoder(LEFT);
  rightPID.Encoder = readEncoder(RIGHT);
  
  /* If we're not moving there is nothing more to do */
  if (!moving){
    /*
    * Reset PIDs once, to prevent startup spikes,
    * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-initialization/
    * PrevInput is considered a good proxy to detect
    * whether reset has already happened
    */
    if (leftPID.PrevInput != 0 || rightPID.PrevInput != 0) resetPID();
    return;
  }

  /* Compute PID update for each motor */
  doPIDRight(&rightPID);
  doPIDLeft(&leftPID);

  // Serial.println(rightPID.TargetTicksPerFrame);


  Serial.print(leftPID.output);
  Serial.print(",");
  Serial.println(rightPID.output);

  /* Set the motor speeds accordingly */
  setMotorSpeeds(leftPID.output, rightPID.output);
}
