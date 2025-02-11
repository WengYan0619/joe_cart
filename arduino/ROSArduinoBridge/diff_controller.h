/* Functions and type-defs for PID control.

  Taken mostly from Mike Ferguson's ArbotiX code which lives at:

  http://vanadium-ros-pkg.googlecode.com/svn/trunk/arbotix/
*/

#define PRINT_ENABLE_SERIAL

/* PID setpoint info For a Motor */
typedef struct
{
  double TargetTicksPerFrame; // target speed in ticks per frame
  long Encoder;               // encoder count
  long PrevEnc;               // last encoder count

  /*
   * Using previous input (PrevInput) instead of PrevError to avoid derivative kick,
   * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-derivative-kick/
   */
  int PrevInput; // last input
  // int PrevErr;                   // last error

  /*
   * Using integrated term (ITerm) instead of integrated error (Ierror),
   * to allow tuning changes,
   * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/
   */
  // int Ierror;
  double ITerm; // integrated term

  long output; // last motor setting
} SetPointInfo;

SetPointInfo leftPID, rightPID;

/* PID Parameters */
int Kp_right = 20;
int Kp_left = 25;
int Kd = 100;
double Ki = 0.01;
int Ko = 100;

unsigned char moving = 0; // is the base in motion?

/*
 * Initialize PID variables to zero to prevent startup spikes
 * when turning PID on to start moving
 * In particular, assign both Encoder and PrevEnc the current encoder value
 * See http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-initialization/
 * Note that the assumption here is that PID is only turned on
 * when going from stop to moving, that's why we can init everything on zero.
 */
void resetPID()
{
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
void doPID(SetPointInfo *p, char side)
{
  long Perror;
  long output;
  int input;

  // Perror = p->TargetTicksPerFrame - (p->Encoder - p->PrevEnc);
  input = p->Encoder - p->PrevEnc;
  Perror = p->TargetTicksPerFrame - input;

#ifdef PRINT_ENABLE_SERIAL
  if (side == 'R')
    Serial.print(">Input_R:");
  if (side == 'L')
    Serial.print(">Input_L:");
  Serial.println(input);
  if (side == 'R')
    Serial.print(">TargetTicks_R: ");
  if (side == 'L')
    Serial.print(">TargetTicks_L: ");
  Serial.println(p->TargetTicksPerFrame);
  // Serial.print(" Perror: ");
  // Serial.print(Perror);
  if (side == 'R')
  {
    Serial.print(">Kp:");
    Serial.println(Kp_right);
    Serial.print(">Kd: ");
    Serial.println(Kd);
    Serial.print(">Ki: ");
    Serial.println(Ki);
  }
  // Serial.print(" PrevInput: ");
  // Serial.print(p->PrevInput);
  // Serial.print(" ITerm: ");
  // Serial.print(p->ITerm);
#endif

#ifdef PRINT_ENABLE_SERIAL3
  if (side == 'R')
    Serial3.print(">Input_R:");
  if (side == 'L')
    Serial3.print(">Input_L:");
  Serial3.println(input);
  Serial3.print(">TargetTicks: ");
  Serial3.println(p->TargetTicksPerFrame);
  // Serial3.print(" Perror: ");
  // Serial3.print(Perror);
  if (side == 'R')
  {
    Serial3.print(">Kp:");
    Serial3.println(Kp);
    Serial3.print(">Kd: ");
    Serial3.println(Kd);
    Serial3.print(">Ki: ");
    Serial3.println(Ki);
  }
  // Serial3.print(" PrevInput: ");
  // Serial3.print(p->PrevInput);
  // Serial3.print(" ITerm: ");
  // Serial3.print(p->ITerm);
#endif

  /*
   * Avoid derivative kick and allow tuning changes,
   * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-derivative-kick/
   * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/
   */
  // output = (Kp * Perror + Kd * (Perror - p->PrevErr) + Ki * p->Ierror) / Ko;
  //  p->PrevErr = Perror;

  double leftMotorKpMultiplier = 1.8;

  long output_temp;

  if (side == 'L')
  {
    // double Kp_left = Kp * leftMotorKpMultiplier;
#ifdef PRINT_ENABLE_SERIAL
    Serial.print(">leftMotorKpMultiplier:");
    Serial.println(leftMotorKpMultiplier);

    Serial.print(">Kp_left:");
    Serial.println(Kp_left);

#endif

#ifdef PRINT_ENABLE_SERIAL3
    Serial3.print(">leftMotorKpMultiplier:");
    Serial3.println(leftMotorKpMultiplier);

    Serial3.print(">Kp_left:");
    Serial3.println(Kp_left);
#endif

    output_temp = (Kp_left * Perror - Kd * (input - p->PrevInput) + static_cast<int>(p->ITerm));

    #ifdef PRINT_ENABLE_SERIAL
    Serial.print(">Kp_term_L:");
    Serial.println(Kp_left * Perror / Ko);

    Serial.print(">Kd_term_L:");
    Serial.println(- Kd * (input - p->PrevInput) / Ko);

    Serial.print(">ITerm_L:");
    Serial.println(static_cast<int>(p->ITerm) / Ko);
    #endif
  }
  else
  {
    output_temp = (Kp_right * Perror - Kd * (input - p->PrevInput) + static_cast<int>(p->ITerm));
    #ifdef PRINT_ENABLE_SERIAL
    Serial.print(">Kp_term_R:");
    Serial.println(Kp_right * Perror / Ko);

    Serial.print(">Kd_term_R:");
    Serial.println(- Kd * (input - p->PrevInput) / Ko);

    Serial.print(">ITerm_R:");
    Serial.println(static_cast<int>(p->ITerm)/ Ko);
    #endif
  }

  // Serial.print(output_temp);
  // Serial.print(" Output NO KO: ");

  output = output_temp / Ko;
  p->PrevEnc = p->Encoder;

#ifdef PRINT_ENABLE_SERIAL
  if (side == 'R')
    Serial.print(">PID_output_R:");
  if (side == 'L')
    Serial.print(">PID_output_L:");
  Serial.println(output);
#endif

#ifdef PRINT_ENABLE_SERIAL3
  if (side == 'R')
    Serial3.print(">PID_output_R:");
  if (side == 'L')
    Serial3.print(">PID_output_L:");
  Serial3.println(output);
#endif

  output += p->output;
  // Accumulate Integral error *or* Limit output.
  // Stop accumulating when output saturates

  // Serial.print(" Added with prev output:");
  // Serial.println(output);

#ifdef PRINT_ENABLE_SERIAL
  if (side == 'R')
    Serial.print(">PWM_output_R:");
  if (side == 'L')
    Serial.print(">PWM_output_L:");
  Serial.println(output);
#endif

#ifdef PRINT_ENABLE_SERIAL3
  if (side == 'R')
    Serial3.print(">PWM_output_R:");
  if (side == 'L')
    Serial3.print(">PWM_output_L:");
  Serial3.println(output);
#endif

  double leftMotorPWMMultiplier = 1;

  if (side == 'L' /*&& output < 0*/)
  {
    output = output * leftMotorPWMMultiplier;
#ifdef PRINT_ENABLE_SERIAL
    Serial.print(">leftMotorPWMMultipler:");
    Serial.println(leftMotorPWMMultiplier);
#endif

#ifdef PRINT_ENABLE_SERIAL3
    Serial3.print(">leftMotorPWMMultipler:");
    Serial3.println(leftMotorPWMMultiplier);
#endif
  }

  if (output >= MAX_PWM)
    output = MAX_PWM;
  else if (output <= -MAX_PWM)
    output = -MAX_PWM;
  else
    /*
     * allow turning changes, see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/
     */
    p->ITerm += Ki * Perror;

  p->output = output;
  p->PrevInput = input;
}

/* Read the encoder values and call the PID routine */
void updatePID()
{
  /* Read the encoders */
  leftPID.Encoder = readEncoder(LEFT);
  rightPID.Encoder = readEncoder(RIGHT);

  /* If we're not moving there is nothing more to do */
  if (!moving)
  {
    /*
     * Reset PIDs once, to prevent startup spikes,
     * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-initialization/
     * PrevInput is considered a good proxy to detect
     * whether reset has already happened
     */
    if (leftPID.PrevInput != 0 || rightPID.PrevInput != 0)
      resetPID();
    return;
  }

  /* Compute PID update for each motor */
  doPID(&rightPID, 'R');
  doPID(&leftPID, 'L');

  // Serial.println(rightPID.TargetTicksPerFrame);

  // Serial.print(leftPID.output);
  // Serial.print(",");
  // Serial.println(rightPID.output);

  /* Set the motor speeds accordingly */
  setMotorSpeeds(leftPID.output, rightPID.output);
}
