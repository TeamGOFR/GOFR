#include <SparkFun_TB6612.h>

/*Lift Wire Spec
  Actu. Neg.   --> Red
  Actu. Pos.   --> Black
  Pot          --> Blue
  5V           --> Yellow
  GND          --> White*/

#include <PS2X_lib.h> 
#include <SoftwareSerial.h>
#include <Cytron_SmartDriveDuo.h>
#define IN1F 4 // Arduino pin 4 is connected to MDDS60 pin DIG1.
#define AN1F 5 // Arduino pin 5 is connected to MDDS30 pin AN1.
#define AN2F 6 // Arduino pin 6 is connected to MDDS30 pin AN2.
#define IN2F 7 // Arduino pin 7 is connected to MDDS30 pin IN2.
#define IN1R 8 // Arduino pin 4 is connected to MDDS60 pin DIG1.
#define AN1R 9 // Arduino pin 5 is connected to MDDS30 pin AN1.
#define AN2R 10 // Arduino pin 6 is connected to MDDS30 pin AN2.
#define IN2R 11 // Arduino pin 7 is connected to MDDS30 pin IN2.
//Mecanum Wheel Motor Drivers
Cytron_SmartDriveDuo frontDrive(PWM_INDEPENDENT, IN1F, IN2F, AN1F, AN2F);
Cytron_SmartDriveDuo rearDrive(PWM_INDEPENDENT, IN1R, IN2R, AN1R, AN2R);
signed int speedFrontLeft, speedFrontRight,speedBackLeft, speedBackRight;
//Lift Motor Driver
#define AIN1 48
#define AIN2 47
#define PWMA 12
#define STBY 22

Motor lift = Motor(AIN1, AIN2, PWMA, 1,STBY);
  
//PS2
#define PS2_DAT        52  //14    
#define PS2_CMD        51  //15
#define PS2_SEL        53  //16
#define PS2_CLK        50  //17

char speed;
//#define pressures   true
#define pressures   false
#define rumble      true
//#define rumble      false
PS2X ps2x; // create PS2 Controller Class

int error = 0;
byte type = 0;
byte vibrate = 0;
boolean L2keyPressed= false;
boolean R2keyPressed= false;
void (* resetFunc) (void) = 0;


//#define SERIAL  Serial
//
//#define LOG_DEBUG
//
//#ifdef LOG_DEBUG
//#define M_LOG SERIAL.print
//#else
//#define M_LOG 
//#endif

#define MAX_PWM   200
#define MIN_PWM   130
int Motor_PWM = 70;
int DeadZone = 3;
//控制电机运动    宏定义


//    ↑A-----B↑   
//     |  ↑  |
//     |  |  |
//    ↑C-----D↑
void ADVANCE()
{
  frontDrive.control(Motor_PWM,Motor_PWM);    
  rearDrive.control(Motor_PWM,Motor_PWM);    
}

//    ↓A-----B↓   
//     |  |  |
//     |  ↓  |
//    ↓C-----D↓
void BACK()
{
  frontDrive.control(-Motor_PWM,-Motor_PWM);    
  rearDrive.control(-Motor_PWM,-Motor_PWM);    
}
//    =A-----B↑   
//     |   ↖ |
//     | ↖   |
//    ↑C-----D=
void LEFT_1()
{
  frontDrive.control(0,Motor_PWM);    
  rearDrive.control(Motor_PWM,0);
}

//    ↓A-----B↑   
//     |  ←  |
//     |  ←  |
//    ↑C-----D↓
void LEFT_2()
{
  frontDrive.control(-Motor_PWM,Motor_PWM);    
  rearDrive.control(Motor_PWM,-Motor_PWM);
}
//    ↓A-----B=   
//     | ↙   |
//     |   ↙ |
//    =C-----D↓
void LEFT_3()
{
  frontDrive.control(-Motor_PWM,0);    
  rearDrive.control(0,-Motor_PWM);    
  
}
//    ↑A-----B=   
//     | ↗   |
//     |   ↗ |
//    =C-----D↑
void RIGHT_1()
{
  frontDrive.control(Motor_PWM,0);    
  rearDrive.control(0,Motor_PWM);    
}
//    ↑A-----B↓   
//     |  →  |
//     |  →  |
//    ↓C-----D↑
void RIGHT_2()
{
  
  frontDrive.control(-Motor_PWM,Motor_PWM);    
  rearDrive.control(-Motor_PWM,Motor_PWM);
}
//    =A-----B↓   
//     |   ↘ |
//     | ↘   |
//    ↓C-----D=
void RIGHT_3()
{
  frontDrive.control(0,-Motor_PWM);    
  rearDrive.control(-Motor_PWM,0);
}
void ROTATECW(int pwmR)
{
 frontDrive.control(pwmR,-pwmR);    
 rearDrive.control(pwmR,-pwmR);    
  }
void ROTATECCW(int pwmR)
{
 frontDrive.control(-pwmR,pwmR);    
 rearDrive.control(-pwmR,pwmR);    
 }
 void LIFTUP(){
 lift.drive(255);
 }
 void LIFTDOWN(){
 lift.drive(-255);
 } 

void DRIVE (int pwmA,int pwmB,int pwmC, int pwmD){
 frontDrive.control(pwmA,pwmB);    
 rearDrive.control(pwmC,pwmD);  
 Serial.print("Driving");Serial.print("\t");
    Serial.print(pwmA);Serial.print("\t");
    Serial.print(pwmB);Serial.print("\t");
    Serial.print(pwmC);Serial.print("\t");
    Serial.print(pwmD);Serial.println("\t");
}

//    =A-----B=  
//     |  =  |
//     |  =  |
//    =C-----D=
void STOP()
{
  
  frontDrive.control(0,0);    
 rearDrive.control(0,0);    
}


void setup()
{
  Serial.begin(115200);
  delay(300) ;//added delay to give wireless ps2 module some time to startup, before configuring it
  //CHANGES for v1.6 HERE!!! **************PAY ATTENTION*************

  //setup pins and settings: GamePad(clock, command, attention, data, Pressures?, Rumble?) check for error
  error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, pressures, rumble);

  if (error == 0) {
    Serial.print("Found Controller, configured successful ");
    Serial.print("pressures = ");
    if (pressures)
      Serial.println("true ");
    else
      Serial.println("false");
    Serial.print("rumble = ");
    if (rumble)
      Serial.println("true)");
    else
      Serial.println("false");
    Serial.println("Try out all the buttons, X will vibrate the controller, faster as you press harder;");
    Serial.println("holding L1 or R1 will print out the analog stick values.");
    Serial.println("Note: Go to www.billporter.info for updates and to report bugs.");
  }
  else if (error == 1)
  {
    Serial.println("No controller found, check wiring, see readme.txt to enable debug. visit www.billporter.info for troubleshooting tips");
    resetFunc();
    
  }

  else if (error == 2)
    Serial.println("Controller found but not accepting commands. see readme.txt to enable debug. Visit www.billporter.info for troubleshooting tips");

  else if (error == 3)
    Serial.println("Controller refusing to enter Pressures mode, may not support it. ");

//  Serial.print(ps2x.Analog(1), HEX);

  type = ps2x.readType();
  switch (type) {
  case 0:
    Serial.print("Unknown Controller type found ");
    break;
  case 1:
    Serial.print("DualShock Controller found ");
    break;
  case 2:
    Serial.print("GuitarHero Controller found ");
    break;
  case 3:
    Serial.print("Wireless Sony DualShock Controller found ");
    break;
  }
  STOP();
  
//  Serial.print("Start");
}


void loop() {
  /* You must Read Gamepad to get new values and set vibration values
    ps2x.read_gamepad(small motor on/off, larger motor strenght from 0-255)
    if you don't enable the rumble, use ps2x.read_gamepad(); with no values
    You should call this at least once a second
  */

 // UART_Control();// Serial port receiving processing
  if (error == 1) //skip loop if no controller found
    return;

  if (type == 2) { //Guitar Hero Controller
    return;
  }
  else  { //DualShock Controller
    ps2x.read_gamepad(false, vibrate); //read controller and set large motor to spin at 'vibrate' speed


//start ，Initial Motor PWM=120；
    if (ps2x.Button(PSB_START))  {
      Serial.println("Start is being held");
     //Motor_PWM = 90;
      ADVANCE();


    }
// Motor Rotates Forward
    if (ps2x.Button(PSB_PAD_UP)) {
      Serial.println("Up held this hard: ");
      //Motor_PWM = 120;
     ADVANCE();
    }

// Reverse；
    if (ps2x.Button(PSB_PAD_DOWN)) {
      Serial.println("Down held this hard: ");
       //Motor_PWM = 120;
      BACK();
    }

//Left
    if (ps2x.Button(PSB_PAD_LEFT)) {
      Serial.println("turn left ");
        //Motor_PWM = 120;//200
      LEFT_1();
    }

//Right
    if (ps2x.Button(PSB_PAD_RIGHT)) {
      Serial.println("turn right");
       // Motor_PWM = 120;//200
      RIGHT_1();
    }
// Stop
    if (ps2x.Button(PSB_SELECT)) {
      Serial.println("stop");
      STOP();
    }
// Diagonal Left Forward
    if (ps2x.Button(PSB_PINK)) {
      Serial.println("motor_pmove_left");
      LEFT_2();
    }
// Diagonal Right Forward
    if (ps2x.Button(PSB_RED)) {
      Serial.println("motor_pmove_right");
      RIGHT_2();
    }
    
    if (ps2x.Button(PSB_L3)) {
      Serial.println("L3 Pressed");  
    }
    if (ps2x.Button(PSB_R3)) {
      Serial.println("R3 Pressed");  
    }
    
    if(ps2x.Button(PSB_L2)){
      if(not L2keyPressed){
        L2keyPressed=true;
        Motor_PWM-=10;
        if(Motor_PWM<20){
          Motor_PWM=20;          
        }
        Serial.print("Speed Decreased to ");Serial.println(Motor_PWM);
      }  
    }
    else 
    {
      L2keyPressed = false;
    }
    
    if (ps2x.Button(PSB_R2)){
      if (not R2keyPressed){
        R2keyPressed=true;
        Motor_PWM+=10;
        if (Motor_PWM >100){
          Motor_PWM=100;
        }
        Serial.print("Speed Increased to ");Serial.println(Motor_PWM);
      }
    }
    else {R2keyPressed = false;}    
       delay(20);
  }
  if (ps2x.Button(PSB_L1) ) { //print stick values if either is TRUE
//    Serial.print("Stick Values:");
//    Serial.print(ps2x.Analog(PSS_LY), DEC); //Left stick, Y axis. Other options: LX, RY, RX
//    Serial.print(",");
//    Serial.print(ps2x.Analog(PSS_LX), DEC);
//    Serial.print(",");
//    Serial.print(ps2x.Analog(PSS_RY), DEC);
//    Serial.print(",");
//    Serial.println(ps2x.Analog(PSS_RX), DEC);
//
//    int LY = map(ps2x.Analog(PSS_LY),0,255,100,-100);
//    int LX = map(ps2x.Analog(PSS_LX),0,255,-100,100);
//    int RY = map(ps2x.Analog(PSS_RY),0,255,100,-100);
//    int RX = map(ps2x.Analog(PSS_RX),0,255,-100,100);
//
//    if (LY > 0) //forward
//    {
//
//     Motor_PWM = LY;
//      ADVANCE();
//      //delay(20);
//    }
//    //Back
//    if (LY < 0)
//    {
//      Motor_PWM = LY;
//      ADVANCE();
//      //delay(20);
//    }
//    //Left
//    if (LX < 0)
//    {
//      Motor_PWM = abs(LX);
//       LEFT_2();
//      //delay(20);
//    }
//    //右转
//    if (LX > 0)
//    {
//      Motor_PWM = LX;
//      RIGHT_2();
//      //delay(20);
//    }
//    //如果摇杆居中
//    if (LY >= -5 && LY <= 5 && LX >= -5 && LX <= 5)
//    {
//      STOP();
//      //delay(20);
//    }
//delay(20);
  }
  if(ps2x.Button(PSB_R1) )
  {
    

   double angle;
   double rpm;
    int RY = map(ps2x.Analog(PSS_RY),0,255,-MotorPWM,MotorPWM);
   int RX = map(ps2x.Analog(PSS_RX),0,255,MotorPWM,-MotorPWM);
    int LX = map(ps2x.Analog(PSS_LX),0,255,MotorPWM,-MotorPWM);
    /*Serial.print("Stick Values:");
    Serial.print(RY, DEC);
    Serial.print(",");
    Serial.println(RX, DEC);*/
    angle = atan2(RY,-RX)+(PI/2);
    rpm = sqrt(RX^2+RY^2);
    int V1=0,V2=0,V3=0,V4=0;
    if (not ((RX <DeadZone && RX>-DeadZone) && (RY <DeadZone && RY>-DeadZone))){
      
    Serial.println(Motor_PWM);
    V1=Motor_PWM*(sin(angle+PI/4));
    V2=Motor_PWM*(cos(angle+PI/4));
    V3=Motor_PWM*(cos(angle+PI/4));
    V4=Motor_PWM*(sin(angle+PI/4));
    Serial.print((180/PI)*angle);Serial.print("\t");
    Serial.print(V1);Serial.print("\t");
    Serial.print(V2);Serial.print("\t");
    Serial.print(V3);Serial.print("\t");
    Serial.print(V4);Serial.println("\t");
    DRIVE(V1,V2,V3,V4);
    }
    else {STOP();
    Serial.print((180/PI)*angle);Serial.print("\t");
    Serial.print(0);Serial.print("\t");
    Serial.print(0);Serial.print("\t");
    Serial.print(0);Serial.print("\t");
    Serial.print(0);Serial.println("\t");  
}
    if (not (LX <DeadZone && LX>-DeadZone)){
      if (LX < -DeadZone){
        ROTATECW(-LX);
      }
        else{ROTATECCW(LX);}
    }
    //else{STOP();}
delay(20);
    
  }
  else{STOP();}
}
