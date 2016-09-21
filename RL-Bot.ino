/************************************************************************/
/* Obstacle Avoiding Robot Using RL 
/* Team Members: Tanushree Das/ Trinoy Hazarika
/* Course: - CS5100[Foundation of Artificial Intelligence]  
/* Instructor - Robert Platt 
 *  
 *  Code is Tested using Arduino Yun with a 2WD drive chasis containing 
 *  two DC motors and a sonar sensor to detect obstacles. Seperate 
 *  Power source is required to drive the motors as well as the arduino
 *  board. L293D is use to drive the motors
 *  A modified version of RL is used to make the robot learn to avoid 
 *  obstacles.
 *  A simulator is also provided to test the code in a Arduino Yun 
 *  Terminal. 
 */
/************************************************************************/
#include <AFMotor.h> // required to operate the L293D 

// define the sonar sensor and sonar LED pins
#define trigPin 13
#define echoPin 12
#define rled 11
#define yled 10

// define the motor of the bot
const int speed = 10; // percent of maximum speed
AF_DCMotor Motor_Left(3, MOTOR34_8KHZ);   // Motor 1
AF_DCMotor Motor_Right(4, MOTOR34_8KHZ);  // Motor 2
int pwm;
boolean start = true;

//timing vars
long timer = 0;
long t50ms = 0;
long t200ms = 0;
long t500ms = 0;
long t5000ms = 0;
long t8000ms = 0;
long t1s = 0;

/************************************************************************/
// Q learing variables
int priorSAs[3] = {0, 0, 0}; //obstructed
int priorAct = 6;
float SA[14][7];
float alpha = 0.9;
float gamma = 0.1;

int count;
int obstructed = 1;   //0-unobstructed;1-obstructed
int drivemode = 3;   // 1-F 2-B  3-S  initially STOP
int drivepos  = 2;  // 0-R 1-L 2-S  initially Straight


/**********************Sonar Sensor Code ***************************/
void initializeSonar() {
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(rled, OUTPUT);
  pinMode(yled, OUTPUT);
}

long calculateDistanceFromSonar() {
  long duration, distance;
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = (duration / 2) / 29.1;

 
  Serial.print("Distance: "); Serial.print(distance);
  Serial.println();
  return distance;
}

void calculateObstructed()
{
  long dist = calculateDistanceFromSonar();
  //Serial.print(dist);
  if ((dist >= 0.0) & (dist <= 10.0)) {
    obstructed = 1;
    Serial.println();
    Serial.print("Bot is obstructed");
    Serial.println();
    glowREDLed();

  }
  else if ((dist > 10.0) & (obstructed == 1)) {
    obstructed = 0;
    Serial.println();
    Serial.print("Bot is not obstructed");
    Serial.println();
    glowYellowLed();
  }

}

void glowREDLed() {
  digitalWrite(rled, HIGH);
  digitalWrite(yled, LOW);
}

void glowYellowLed() {
  digitalWrite(yled, HIGH);
  digitalWrite(rled, LOW);
}

/************************************************************************/


/************Core Methods of Arduino[Setup/Loop] ************************/
void setup() {
  // put your setup code here, to run once:
  Serial.begin (9600);
  //while(!Serial);
  initializeSonar();
  //pwm= map(60, 0,100, 0,255);
  //Motor_Left.setSpeed(105);
  //Motor_Right.setSpeed(105);
  glowYellowLed();
  Serial.print("******Robot Simulation Testing************");
  Serial.println();

}

void loop() {

  //timing loops
  if (millis() - t5000ms > 5000) {
    //UO status:
   Serial.println();
   Serial.print("******Obstacle Detection Start************");
   Serial.println();
    calculateObstructed();
    t5000ms = millis();
   Serial.print("*********Obstacle Detection Ends***********");
   Serial.println();
  }

  if (millis() - t8000ms > 8000) {
    Serial.println();
    Serial.print("*********Q Caluclation Start*********");
    Serial.println();
    calculateQValue();
    t8000ms = millis();
    Serial.print("***********Q Caluclation Ends*******");
    Serial.println();
  }
}

void calculateQValue() {
 
  count = 0;

  /* Compute S from Sb[current meaured state], Ap[previous action] */
  int Sb = 7 * (1 - obstructed);

  // forward 1  backward 2 stop 3
  int Ap = 3 * (drivemode - 1);
  if (Ap < 6) {
    Ap = Ap + drivepos;
  }
  int S = Sb + Ap;

  //compute past reward (use Sp->S)
  int Sp = priorSAs[0];
  int Sps = (Sp - Sp % 7); // can be either 0 or 7
  /* reward table: (reward based on state transition)
      Sb[new]  0  7
    Sp
    0         -1  1
    7          0  0
    from Sp -> Sb
  */
  
  int RWD[2][2] = {{ -1, 1}, { 0, 0}};
  int reward = RWD[Sps / 7][Sb / 7];
  Serial.print("Reward: ");  Serial.print(reward);
  Serial.println();

  /*for all 3 in priosSAs:
     update Q table- use max Q of state to which Sp,Ap lead:
     i.e. the current state.*/
    
  //for Sn-1
  float Qmax = SA[S][0];
  for (int ind = 1; ind < 7; ind++) {
    Qmax = max(Qmax, SA[S][ind]);
  }
  float Q = SA[Sp][Ap] + alpha * (reward + gamma * Qmax);
  Serial.print("Calculated Q value is: ");  Serial.print(float2s(Q,3));
  Serial.println();
  SA[Sp][Ap] = Q;

  //for Sn-2
  int SN2 = priorSAs[1];
  int AP2 = priorSAs[0] % 7;
  float Q2 = SA[SN2][AP2] + (0.5) * alpha * (reward + gamma * Qmax);
  SA[SN2][AP2] = Q2;

  //for sn-3
  SN2 = priorSAs[2];
  AP2 = priorSAs[1] % 7;
  Q2 = SA[SN2][AP2] + (0.25) * alpha * (reward + gamma * Qmax);
  SA[SN2][AP2] = Q2;

  //policy for action
  //uses weighted random choice
  float Qsum = 0;
  for (int ind = 0; ind < 7; ind++) {
    Qsum = Qsum + SA[S][ind];
  }
  int rand = random(0, 1000);
  int Av = 0;
  int gval = 0;
  while ((Av < 7) & (gval < rand)) {
    gval = gval + abs(1000 * ((1 + SA[S][Av]) / (Qsum + 7)));
    Av = Av + 1;
  }
   Serial.print("Random Action Generated is: ");  Serial.print(Av);
   Serial.println();

  //Action
  drivemode = (Av - Av % 3) / 3;
  if (drivemode != 3) {
    drivepos = (Av % 3);
  }

  rotate();
  drive(drivemode);

  //update priorSAs queue
  priorSAs[2] = priorSAs[1];
  priorSAs[1] = priorSAs[0];
  priorSAs[0] = S;


  Serial.print("State Action Q values are: ");
  Serial.println();
  Serial.print("        FR     FL     FS      BR      BL      BS      Stop");
  Serial.println();
  for (int i = 0; i < 14; i++) {
    if (i == 0){ Serial.print("O-FR: ");}
    else if (i == 1){ Serial.print("O-FL: "); }
    else if (i == 2){ Serial.print("O-FS: ");}
    else if (i == 3){ Serial.print("O-BR: ");}
    else if (i == 4){ Serial.print("O-BL: ");}
    else if (i == 5){ Serial.print("O-BS: ");}
    else if (i == 6){ Serial.print("O-ST: ");}
    else if (i == 7){ Serial.print("U-FR: ");}
    else if (i == 8){ Serial.print("U-FL: ");}
    else if (i == 9){ Serial.print("U-FS: ");}
    else if (i == 10){ Serial.print("U-BR: ");}
    else if (i == 11){ Serial.print("U-BL: ");}
    else if (i == 12){ Serial.print("U-BS: ");}
    else if (i == 13){ Serial.print("U-ST: ");}
    for (int j = 0; j < 7; j++) {
      Serial.print(float2s(SA[i][j], 3)); Serial.print("    ");
    }
    Serial.println();
  }

}

/*********************Motor Simulation in Terminal*********************************/
/* Current setup is made to run the code in terminal so that values can be seen 
 *  without the hardware. But by changing to the motor methods below we can run the 
 *  code in hardware.
 */
void rotate() {
  
  Serial.print("New DrivePos is: ");  Serial.print(drivepos);
  if (drivepos == 0)
  {
    Serial.print("   New Drive Position is Right");
  }
  else if (drivepos == 1)
  {
    Serial.print("   New Drive Position is Left");
  }
  else if (drivepos == 2)
  {
    Serial.print("   New Drive Position is Straight");
    
  }
  Serial.println();
}

void drive(int cmd) {
  Serial.print("New Drivemode is: ");  Serial.print(cmd);
  drivemode = cmd;
  if (cmd == 1) {
    Serial.print("   Next Action is Forward");
  }
  if (cmd == 2) {
    Serial.print("   Next Action is Back");
  }
  if (cmd == 3) {
    Serial.print("   Next Action is Stop");
  }
  Serial.println();
}
/*********************Motor Simulation in Terminal*******************************/


/*************************Motor logic For Hardware Implementation****************/
void motorForward(int duration)
{
  Serial.print(" move forward for 2 seconds");
  Motor_Left.run(FORWARD);
  Motor_Right.run(FORWARD);
  delay(2000);
  Serial.print(" move stop ");
  Motor_Left.setSpeed(0);
  Motor_Right.setSpeed(0);
  delay(2000);
  Motor_Left.run(RELEASE);  // set motor pins to zero
  Motor_Right.run(RELEASE);
  delay(2000);
}

void motorBackward(int duration)
{
  Serial.print(" move forward for 2 seconds");
  Motor_Left.run(BACKWARD);
  Motor_Right.run(BACKWARD);
  delay(2000);
  Serial.print(" move stop ");
  Motor_Left.setSpeed(0);
  Motor_Right.setSpeed(0);
  delay(2000);
  Motor_Left.run(RELEASE);  // set motor pins to zero
  Motor_Right.run(RELEASE);
  delay(2000);
}

void rotateRight( )
{
  Serial.print(" move left forward and right backward for 2 seconds");
  Motor_Left.run(FORWARD);
  Motor_Right.run(BACKWARD);
  delay(2000);
  Serial.print(" move stop ");
  Motor_Left.setSpeed(0);
  Motor_Right.setSpeed(0);
  delay(2000);
  Motor_Left.run(RELEASE);  // set motor pins to zero
  Motor_Right.run(RELEASE);
  delay(2000);
}

void rotateLeft()
{
  Serial.print(" move left backward and right forward for 2 seconds");
  Motor_Left.run(BACKWARD);
  Motor_Right.run(FORWARD);
  delay(2000);
  Serial.print(" move stop ");
  Motor_Left.setSpeed(0);
  Motor_Right.setSpeed(0);
  delay(2000);
  Motor_Left.run(RELEASE);  // set motor pins to zero
  Motor_Right.run(RELEASE);
  delay(2000);
}


char * float2s(float f, unsigned int digits)
{
 int index = 0;
 static char s[16];                    // buffer to build string representation
 // handle sign
 if (f < 0.0)
 {
   s[index++] = '-';
   f = -f;
 } 
 // handle infinite values
 if (isinf(f))
 {
   strcpy(&s[index], "INF");
   return s;
 }
 // handle Not a Number
 if (isnan(f)) 
 {
   strcpy(&s[index], "NaN");
   return s;
 }

 // max digits
 if (digits > 6) digits = 6;
 long multiplier = pow(10, digits);     // fix int => long

 int exponent = int(log10(f));
 float g = f / pow(10, exponent);
 if ((g < 1.0) && (g != 0.0))      
 {
   g *= 10;
   exponent--;
 }

 long whole = long(g);                     // single digit
 long part = long((g-whole)*multiplier);   // # digits
 char format[16];
 sprintf(format, "%%ld.%%0%dld E%%+d", digits);
 sprintf(&s[index], format, whole, part, exponent);
 
 return s;
}

