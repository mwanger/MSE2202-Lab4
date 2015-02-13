/*

 MSE 2202 MSEBot base code for Labs 3 and 4
 Language: Arduino
 Authors: Michael Naish and Eugen Porter
 Date: 15/01/18
 
 Rev 1 - Initial version
 
 */
//grip goes from 0 closed to 460-470 open (encoder counts)
//servo arm goes 60ish-180(degrees)

#include <Servo.h>
#include <EEPROM.h>
#include <uSTimer2.h>
#include <CharliePlexM.h>
#include <Wire.h>
#include <I2CEncoder.h>

Servo servo_RightMotor;
Servo servo_LeftMotor;
Servo servo_ArmMotor;    
Servo servo_GripMotor;
I2CEncoder encoder_RightMotor;
I2CEncoder encoder_LeftMotor;
I2CEncoder encoder_GripMotor;

// Uncomment keywords to enable debugging output

//#define DEBUG_MODE_DISPLAY
//#define DEBUG_MOTORS
//#define DEBUG_LINE_TRACKERS
//#define DEBUG_ENCODERS
//#define DEBUG_ULTRASONIC
//#define DEBUG_LINE_TRACKER_CALIBRATION
//#define DEBUG_MOTOR_CALIBRATION
#define DEBUG_STOP_COUNTERS
//#define DEBUG_LIGHT_SENSOR


boolean bt_Motors_Enabled = true;

//port pin constants
const int ci_Ultrasonic_Ping = 2;   //input plug
const int ci_Ultrasonic_Data = 3;   //output plug
const int ci_Charlieplex_LED1 = 4;
const int ci_Charlieplex_LED2 = 5;
const int ci_Charlieplex_LED3 = 6;
const int ci_Charlieplex_LED4 = 7;
const int ci_Mode_Button = 7;
const int ci_Right_Motor = 8;
const int ci_Left_Motor = 9;
const int ci_Arm_Motor = 10;
const int ci_Grip_Motor = 11;
const int ci_Motor_Enable_Switch = 12;
const int ci_Right_Line_Tracker = A0;
const int ci_Middle_Line_Tracker = A1;
const int ci_Left_Line_Tracker = A2;
const int ci_Light_Sensor = A3;
const int ci_I2C_SDA = A4;         // I2C data = white
const int ci_I2C_SCL = A5;         // I2C clock = yellow

// Charlieplexing LED assignments
const int ci_Heartbeat_LED = 1;
const int ci_Indicator_LED = 10;
const int ci_Right_Line_Tracker_LED = 6;
const int ci_Middle_Line_Tracker_LED = 9;
const int ci_Left_Line_Tracker_LED = 12;

//constants

// EEPROM addresses
const int ci_Left_Line_Tracker_Dark_Address_L = 0;
const int ci_Left_Line_Tracker_Dark_Address_H = 1;
const int ci_Left_Line_Tracker_Light_Address_L = 2;
const int ci_Left_Line_Tracker_Light_Address_H = 3;
const int ci_Middle_Line_Tracker_Dark_Address_L = 4;
const int ci_Middle_Line_Tracker_Dark_Address_H = 5;
const int ci_Middle_Line_Tracker_Light_Address_L = 6;
const int ci_Middle_Line_Tracker_Light_Address_H = 7;
const int ci_Right_Line_Tracker_Dark_Address_L = 8;
const int ci_Right_Line_Tracker_Dark_Address_H = 9;
const int ci_Right_Line_Tracker_Light_Address_L = 10;
const int ci_Right_Line_Tracker_Light_Address_H = 11;
const int ci_Left_Motor_Offset_Address_L = 12;
const int ci_Left_Motor_Offset_Address_H = 13;
const int ci_Right_Motor_Offset_Address_L = 14;
const int ci_Right_Motor_Offset_Address_H = 15;

const int ci_Left_Motor_Stop = 1500;        // 200 for brake mode; 1500 for stop
const int ci_Right_Motor_Stop = 1500;
const int ci_Grip_Motor_Stop = 1500;
const int ci_Grip_Motor_Open = 460;         // Experiment to determine appropriate value
const int ci_Grip_Motor_Zero = 90;          //  "
const int ci_Grip_Motor_Closed = 5;       //  "
const int ci_Arm_Servo_Retracted = 61;      //  "
const int ci_Arm_Servo_Extended = 180;      //  "
const int ci_Display_Time = 500;
const int ci_Line_Tracker_Calibration_Interval = 100;
const int ci_Line_Tracker_Cal_Measures = 20;
const int ci_Line_Tracker_Tolerance = 50;   // May need to adjust this
const int ci_Motor_Calibration_Time = 5000;



//variables
byte b_LowByte;
byte b_HighByte;
unsigned long ul_Echo_Time;
unsigned int ui_Left_Line_Tracker_Data;
unsigned int ui_Middle_Line_Tracker_Data;
unsigned int ui_Right_Line_Tracker_Data;
unsigned int ui_Motors_Speed = 1650;        // Default run speed
unsigned int ui_Left_Motor_Speed;
unsigned int ui_Right_Motor_Speed;
unsigned int ui_Grip_Motor_Speed;
unsigned long ul_Left_Motor_Position;
unsigned long ul_Right_Motor_Position;
unsigned long ul_Grip_Motor_Position;

unsigned long ul_3_Second_timer = 0;
unsigned long ul_Display_Time;
unsigned long ul_Calibration_Time;
unsigned long ui_Left_Motor_Offset;
unsigned long ui_Right_Motor_Offset;

unsigned int ui_Cal_Count;
unsigned int ui_Left_Line_Tracker_Dark;
unsigned int ui_Left_Line_Tracker_Light;
unsigned int ui_Middle_Line_Tracker_Dark;
unsigned int ui_Middle_Line_Tracker_Light;
unsigned int ui_Right_Line_Tracker_Dark;
unsigned int ui_Right_Line_Tracker_Light;
unsigned int ui_Line_Tracker_Tolerance;

unsigned int  ui_Robot_State_Index = 0;
//0123456789ABCDEF
unsigned int  ui_Mode_Indicator[6] = {
  0x00,    //B0000000000000000,  //Stop
  0x00FF,  //B0000000011111111,  //Run
  0x0F0F,  //B0000111100001111,  //Calibrate line tracker light level
  0x3333,  //B0011001100110011,  //Calibrate line tracker dark level
  0xAAAA,  //B1010101010101010,  //Calibrate motors
  0xFFFF   //B1111111111111111   //Unused
};

unsigned int  ui_Mode_Indicator_Index = 0;

//display Bits 0,1,2,3, 4, 5, 6,  7,  8,  9,  10,  11,  12,  13,   14,   15
int  iArray[16] = {
  1,2,4,8,16,32,64,128,256,512,1024,2048,4096,8192,16384,65536};
int  iArrayIndex = 0;

boolean bt_Heartbeat = true;
boolean bt_3_S_Time_Up = false;
boolean bt_Do_Once = false;
boolean bt_Cal_Initialized = false;

//added variables*****************************************************************************************************************************************************/
unsigned int motorState=0;
unsigned int prevMotorState=0;
const int ci_NumberStops=4;
unsigned int ui_StopCounter=0;
unsigned int stopStart=0;
int previousMillis=0;
int currentMillis=0;
unsigned int ls_Reading;
unsigned int pls_Reading;
unsigned int sensingLED=1; //might not use this anymore
unsigned int sensingUltrasonic=1; //might not use this anymore
unsigned int stage=1;
boolean partialExtend=false;
int distance=0;
int lightInt=0;
boolean turnt=false;
boolean closer=false;
boolean lost=true;

float leftMotorPos=0;
float rightMotorPos=0;
float positionHolder=0;
boolean openGrip=false;
boolean part1=true;
//***********************************************************************************************************************************************************************/
void setup() {
  Wire.begin();	      // Wire library required for I2CEncoder library
  Serial.begin(9600);

  CharliePlexM::setBtn(ci_Charlieplex_LED1,ci_Charlieplex_LED2,
  ci_Charlieplex_LED3,ci_Charlieplex_LED4,ci_Mode_Button);

  // set up ultrasonic
  pinMode(ci_Ultrasonic_Ping, OUTPUT);
  pinMode(ci_Ultrasonic_Data, INPUT);

  // set up drive motors
  pinMode(ci_Right_Motor, OUTPUT);
  servo_RightMotor.attach(ci_Right_Motor);
  pinMode(ci_Left_Motor, OUTPUT);
  servo_LeftMotor.attach(ci_Left_Motor);

  // set up arm motors
  pinMode(ci_Arm_Motor, OUTPUT);
  servo_ArmMotor.attach(ci_Arm_Motor);
  pinMode(ci_Grip_Motor, OUTPUT);
  servo_GripMotor.attach(ci_Grip_Motor);
  servo_GripMotor.write(ci_Grip_Motor_Zero);

  // set up motor enable switch
  pinMode(ci_Motor_Enable_Switch, INPUT);

  // set up encoders. Must be initialized in order that they are chained together, 
  // starting with the encoder directly connected to the Arduino. See I2CEncoder docs
  // for more information
  encoder_LeftMotor.init(1.0/3.0*MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
  encoder_LeftMotor.setReversed(false);  // adjust for positive count when moving forward
  encoder_RightMotor.init(1.0/3.0*MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);  
  encoder_RightMotor.setReversed(true);  // adjust for positive count when moving forward
  encoder_GripMotor.init(MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);

  // set up line tracking sensors
  pinMode(ci_Right_Line_Tracker, INPUT);
  pinMode(ci_Middle_Line_Tracker, INPUT);
  pinMode(ci_Left_Line_Tracker, INPUT);
  ui_Line_Tracker_Tolerance = ci_Line_Tracker_Tolerance;

  // read saved values from EEPROM
  b_LowByte = EEPROM.read(ci_Left_Line_Tracker_Dark_Address_L);
  b_HighByte = EEPROM.read(ci_Left_Line_Tracker_Dark_Address_H);
  ui_Left_Line_Tracker_Dark = word(b_HighByte, b_LowByte);
  b_LowByte = EEPROM.read(ci_Left_Line_Tracker_Light_Address_L);
  b_HighByte = EEPROM.read(ci_Left_Line_Tracker_Dark_Address_H);
  ui_Left_Line_Tracker_Light = word(b_HighByte, b_LowByte);
  b_LowByte = EEPROM.read(ci_Middle_Line_Tracker_Dark_Address_L);
  b_HighByte = EEPROM.read(ci_Left_Line_Tracker_Dark_Address_H);
  ui_Middle_Line_Tracker_Dark = word(b_HighByte, b_LowByte); 
  b_LowByte = EEPROM.read(ci_Middle_Line_Tracker_Light_Address_L);
  b_HighByte = EEPROM.read(ci_Left_Line_Tracker_Dark_Address_H);
  ui_Middle_Line_Tracker_Light = word(b_HighByte, b_LowByte);
  b_LowByte = EEPROM.read(ci_Right_Line_Tracker_Dark_Address_L);
  b_HighByte = EEPROM.read(ci_Left_Line_Tracker_Dark_Address_H);
  ui_Right_Line_Tracker_Dark = word(b_HighByte, b_LowByte);
  b_LowByte = EEPROM.read(ci_Right_Line_Tracker_Light_Address_L);
  b_HighByte = EEPROM.read(ci_Left_Line_Tracker_Dark_Address_H);
  ui_Right_Line_Tracker_Light = word(b_HighByte, b_LowByte);
  b_LowByte = EEPROM.read(ci_Left_Motor_Offset_Address_L);
  b_HighByte = EEPROM.read(ci_Left_Motor_Offset_Address_H);
  ui_Left_Motor_Offset = word(b_HighByte, b_LowByte);
  b_LowByte = EEPROM.read(ci_Right_Motor_Offset_Address_L);
  b_HighByte = EEPROM.read(ci_Right_Motor_Offset_Address_H);
  ui_Right_Motor_Offset = word(b_HighByte, b_LowByte);
}

void loop()
{
  if((millis() - ul_3_Second_timer) > 3000)
  {
    bt_3_S_Time_Up = true;
  }

  // button-based mode selection
  if(CharliePlexM::ui_Btn)
  {
    if(bt_Do_Once == false)
    {
      bt_Do_Once = true;
      ui_Robot_State_Index++;
      ui_Robot_State_Index = ui_Robot_State_Index & 7;
      ul_3_Second_timer = millis();
      bt_3_S_Time_Up = false;
      bt_Cal_Initialized = false;
    }
  }
  else
  {
    bt_Do_Once = LOW;
  }

  // check if drive motors should be powered
  bt_Motors_Enabled = digitalRead(ci_Motor_Enable_Switch);

  // modes 
  // 0 = default after power up/reset
  // 1 = Press mode button once to enter. Run robot.
  // 2 = Press mode button twice to enter. Calibrate line tracker light level.
  // 3 = Press mode button three times to enter. Calibrate line tracker dark level.
  // 4 = Press mode button four times to enter. Calibrate motor speeds to drive straight.
  switch(ui_Robot_State_Index)
  {
  case 0:    //Robot stopped
    {

#ifdef DEBUG_LIGHT_SENSOR
      servo_ArmMotor.write(90);
      lightInt=analogRead(ci_Light_Sensor);
      Serial.println(lightInt);
#endif

      readLineTrackers();
      Ping();
      servo_LeftMotor.writeMicroseconds(ci_Left_Motor_Stop); 
      servo_RightMotor.writeMicroseconds(ci_Right_Motor_Stop); 
      servo_ArmMotor.write(ci_Arm_Servo_Retracted);
      servo_GripMotor.writeMicroseconds(ci_Grip_Motor_Stop);
      encoder_LeftMotor.zero();
      encoder_RightMotor.zero();
      encoder_GripMotor.zero();
      ui_Mode_Indicator_Index = 0;
      break;
    } 

  case 1:    //Robot Run after 3 seconds
    {
      if(bt_3_S_Time_Up)
      {
        readLineTrackers();

#ifdef DEBUG_ENCODERS           
        ul_Left_Motor_Position = encoder_LeftMotor.getPosition();
        ul_Right_Motor_Position = encoder_RightMotor.getPosition();
        ul_Grip_Motor_Position = encoder_GripMotor.getRawPosition();

        Serial.print("Encoders L: ");
        Serial.print(encoder_LeftMotor.getPosition());
        Serial.print(", R: ");
        Serial.print(encoder_RightMotor.getPosition());
        Serial.print(", G: ");
        Serial.println(encoder_GripMotor.getRawPosition());
#endif

        // set motor speeds
        ui_Left_Motor_Speed = constrain(ui_Motors_Speed - ui_Left_Motor_Offset, 1600, 2100);
        ui_Right_Motor_Speed = constrain(ui_Motors_Speed - ui_Right_Motor_Offset, 1600, 2100);

        /**************************************************************************************
         * Add line tracking code here. 
         * Adjust motor speed according to information from line tracking sensors and 
         * possibly encoder counts.
         * 
        /*************************************************************************************/

#ifdef DEBUG_STOP_COUNTERS //if you comment this out it skips to stage 2 neer the start, idk why
        //        Serial.print("stage counter: ");
        //        Serial.print(stage);
        Serial.print(", Stop counter: ");
        Serial.println(ui_StopCounter);
        //        Serial.print(", stopStart value: ");
        //        Serial.println(stopStart);
        //        Serial.println(motorState);
#endif

        if(stage==1){ //line trackers+stop counters  ------------------------------------------------------------------------------------------------------

          CharliePlexM::Write(11,HIGH);

          motorState=0;

          //          ui_Left_Line_Tracker_Data = analogRead(ci_Left_Line_Tracker);
          //          ui_Middle_Line_Tracker_Data = analogRead(ci_Middle_Line_Tracker);
          //          ui_Right_Line_Tracker_Data = analogRead(ci_Right_Line_Tracker);

          if(ui_Left_Line_Tracker_Data < (ui_Left_Line_Tracker_Dark - ui_Line_Tracker_Tolerance))
          {
            //CharliePlexM::Write(ci_Left_Line_Tracker_LED, HIGH);
            motorState+=100;
          }
          else
          { 
            //CharliePlexM::Write(ci_Left_Line_Tracker_LED, LOW);
          }
          if(ui_Middle_Line_Tracker_Data < (ui_Middle_Line_Tracker_Dark - ui_Line_Tracker_Tolerance))
          {
            //CharliePlexM::Write(ci_Middle_Line_Tracker_LED, HIGH);
            motorState+=10;
          }
          else
          { 
            //CharliePlexM::Write(ci_Middle_Line_Tracker_LED, LOW);
          }
          if(ui_Right_Line_Tracker_Data < (ui_Right_Line_Tracker_Dark - ui_Line_Tracker_Tolerance))
          {
            //CharliePlexM::Write(ci_Right_Line_Tracker_LED, HIGH);
            motorState+=1;
          }
          else
          { 
            //CharliePlexM::Write(ci_Right_Line_Tracker_LED, LOW);
          }



          if(motorState==11)
          {
            ui_Left_Motor_Speed+=75;
            ui_Right_Motor_Speed-=25;
          }
          else if(motorState==1)
          {
            ui_Left_Motor_Speed+=150;
            ui_Right_Motor_Speed-=25;
          }
          else if (motorState==110)
          {
            ui_Right_Motor_Speed+=75;
            ui_Left_Motor_Speed-=25;
          }
          else if(motorState==100)
          {
            ui_Right_Motor_Speed+=150;
            ui_Left_Motor_Speed-=25;
          }
          else if (motorState==10)
          {
            ui_Right_Motor_Speed=1650;
            ui_Left_Motor_Speed=1650;
          }
          else if(motorState==111)
          {
            if(stopStart==0)
            {
              previousMillis=millis();
              // Serial.println(motorState);
              //Serial.println(previousMillis);
              stopStart=1;
              //Serial.println(stopStart);
            }
            //            else if(stopStart==1)
            //            {
            //              currentMillis=millis();
            //            }

            if(millis()-previousMillis>=213)  
            {
              // Serial.println(currentMillis-previousMillis);
              stopStart=2;
              // Serial.println(stopStart);
              currentMillis=0;
              previousMillis=0;
            }

          }
          else if(motorState==0)
          {
            if(prevMotorState==1)
            {
              ui_Left_Motor_Speed+=250;
              ui_Right_Motor_Speed-=150;
            }
            else if(prevMotorState==100)
            {
              ui_Right_Motor_Speed+=250;
              ui_Left_Motor_Speed-=150;
            }
          }

          //          if(motorState!=111 && stopStart==1)
          //          {stopStart=0; previousMillis=0;}


          if(motorState!=111 && stopStart==2)
          {
            ui_StopCounter++;
            stopStart=0;

            if(ui_StopCounter==ci_NumberStops)
            {
              ui_Left_Motor_Speed=1500;
              ui_Right_Motor_Speed=1500;
              ul_Left_Motor_Position=0;
              stage=2;

            }
          }

          //Serial.print(ui_StopCounter);
          if(bt_Motors_Enabled)
          {
            servo_LeftMotor.writeMicroseconds(ui_Left_Motor_Speed);
            servo_RightMotor.writeMicroseconds(ui_Right_Motor_Speed);
          }
          else
          {  
            servo_LeftMotor.writeMicroseconds(ci_Left_Motor_Stop); 
            servo_RightMotor.writeMicroseconds(ci_Right_Motor_Stop); 
          }

          if(motorState!=0)
            prevMotorState=motorState;

          //when it gets to the last stop this code starts
        }//end of stage one

          else if(stage==2) //turns and uses ultrasonic sensor to get into a range to find the light------------------------------------------------------------------------------------------------
        {
          CharliePlexM::Write(8,HIGH);
          CharliePlexM::Write(11,HIGH);
          if(!partialExtend)
          {
            servo_ArmMotor.write(100);
            partialExtend=true;
            ul_Left_Motor_Position=0;            
            delay(1000);
          }


          //            if(!turnt){//this is the spot that was causing us problems last time i belive.  by putting the other if statements within the turnt if statement, the problem should be voided. hopefully <3
          //            ul_Left_Motor_Position= encoder_LeftMotor.getPosition();
          //            if(ul_Left_Motor_Position<=1.1)
          //            {ui_Left_Motor_Speed=1700;
          //             ui_Right_Motor_Speed=1500;}
          //            else
          //            {ui_Left_Motor_Speed=1500;
          //             ui_Right_Motor_Speed=1500;
          //             turnt=true;}
          //          } 






          if(!turnt){//this is the spot that was causing us problems last time i belive.  by putting the other if statements within the turnt if statement, the problem should be voided. hopefully <3

            leftMotorPos=encoder_LeftMotor.getPosition();
            //Serial.print("Left motor Position: ");
            //Serial.println(leftMotorPos);
            positionHolder=leftMotorPos+1.6;


            while(encoder_LeftMotor.getPosition()<positionHolder)
            {
              leftMotorPos=encoder_LeftMotor.getPosition();
//              Serial.print("Left motor Position: ");
//              Serial.println(leftMotorPos); 
              ui_Left_Motor_Speed=1700;
              ui_Right_Motor_Speed=1500;
              servo_LeftMotor.writeMicroseconds(ui_Left_Motor_Speed);
              servo_RightMotor.writeMicroseconds(ui_Right_Motor_Speed);

              if(encoder_LeftMotor.getPosition()>=positionHolder)
              {
                break;
              }
            }
            //            if(leftMotorPos<=0.2)
            //            {ui_Left_Motor_Speed=1700;
            //             ui_Right_Motor_Speed=1500;}

            ui_Left_Motor_Speed=1500;
            ui_Right_Motor_Speed=1500;
            servo_LeftMotor.writeMicroseconds(ui_Left_Motor_Speed);
            servo_RightMotor.writeMicroseconds(ui_Right_Motor_Speed);
            delay(1000);
            turnt=true;
          }

          else{
            //Serial.println( " its turnt");

            Ping();
            distance=ul_Echo_Time/60;
//            Serial.print("Distance:");
//            Serial.println(distance);


            if(distance>6){
              ui_Left_Motor_Speed=1700;
              ui_Right_Motor_Speed=1700;
              //servo_LeftMotor.writeMicroseconds(ui_Left_Motor_Speed);
              //servo_RightMotor.writeMicroseconds(ui_Right_Motor_Speed);
            }

            else if(distance==0)
            {
              ui_Left_Motor_Speed=1700;
              ui_Right_Motor_Speed=1500;
            }
            else{
              ui_Left_Motor_Speed=1500;
              ui_Right_Motor_Speed=1500;
              servo_LeftMotor.writeMicroseconds(ui_Left_Motor_Speed);
              servo_RightMotor.writeMicroseconds(ui_Right_Motor_Speed);
              delay(1000);
              stage=3; // when it gets close enough to the box it stops and goes to next stage(using led sensor)
            }
          }



          servo_LeftMotor.writeMicroseconds(ui_Left_Motor_Speed);
          servo_RightMotor.writeMicroseconds(ui_Right_Motor_Speed);
        }

        else if(stage==3) //LED sensor finds the light, there by aiming the robot ---------------------------------------------------------------------------------------------------------------------------------
        {
          CharliePlexM::Write(11,HIGH);
          CharliePlexM::Write(5,HIGH);
          CharliePlexM::Write(8,HIGH);

          servo_ArmMotor.write(90);
          if(!openGrip)
          {
            ul_Grip_Motor_Position=encoder_GripMotor.getRawPosition();
            // Serial.println(ul_Grip_Motor_Position);
            if(ul_Grip_Motor_Position<=350){
              servo_GripMotor.write(1700);
            }
            else{
              servo_GripMotor.write(1500);
              openGrip=true;
            }
          }

          lightInt=analogRead(ci_Light_Sensor);

          if(lightInt<=175)
          {
            ui_Left_Motor_Speed=1675;
            ui_Right_Motor_Speed=1500;


            if(lightInt<=90)
            {
              ui_Left_Motor_Speed=1500;
              ui_Right_Motor_Speed=1500;
              servo_LeftMotor.writeMicroseconds(ui_Left_Motor_Speed);
              servo_RightMotor.writeMicroseconds(ui_Right_Motor_Speed);
              delay(750);
              stage=4;
            }        
          }

          else
          {
            ui_Left_Motor_Speed=1725;
            ui_Right_Motor_Speed=1500;
          }

          servo_LeftMotor.writeMicroseconds(ui_Left_Motor_Speed);
          servo_RightMotor.writeMicroseconds(ui_Right_Motor_Speed);          
        }

        else if(stage==4)  // getting close to the light and opening the grip----------------------------------------------------------------------------------------------------------------------------------------
        {
          CharliePlexM::Write(2,HIGH);
          CharliePlexM::Write(5,HIGH);
          CharliePlexM::Write(8,HIGH);
          CharliePlexM::Write(11,HIGH);          

          if(!closer){
            servo_ArmMotor.write(120);
            closer=true;
            delay(500);
            stage=5;





            //            Ping();
            //            distance=ul_Echo_Time/58; //gives distance in cm
            //            if(distance>5){
            //              ui_Left_Motor_Speed=1700;
            //              ui_Right_Motor_Speed=1700;
            //            }
            //            else{
            //              ui_Left_Motor_Speed=1500;
            //              ui_Right_Motor_Speed=1500;
            //              stage=5;
            //              delay(100);
            //              closer=true;// when it gets close enough to the box it stops and initializes gripper               
            //            }
          }//statement ensures that the robot is close enough to grab the LED

          //          servo_LeftMotor.writeMicroseconds(ui_Left_Motor_Speed);
          //          servo_RightMotor.writeMicroseconds(ui_Right_Motor_Speed);
        }

        else if(stage==5) // closing the gripper around the light --------------------------------------------------------------------------------------------------------------------------------------------
        {
          CharliePlexM::Write(2,LOW);
          CharliePlexM::Write(5,HIGH);
          CharliePlexM::Write(8,HIGH);
          CharliePlexM::Write(11,HIGH);

          // servo_ArmMotor.write(120);//full extension
          ul_Grip_Motor_Position=encoder_GripMotor.getRawPosition();
          //Serial.println(ul_Grip_Motor_Position);
          if(ul_Grip_Motor_Position>=60)
          {
            servo_GripMotor.write(1300);
          }//closes grip motor
          else{
            servo_GripMotor.write(1500);//stops the grip motor
            delay(100);
            servo_ArmMotor.write(65); //grabs the light and pulls it back to about a 90 degree position
            delay(500);
            ui_Left_Motor_Speed=1300;
            ui_Right_Motor_Speed=1300;
            servo_LeftMotor.writeMicroseconds(ui_Left_Motor_Speed);
            servo_RightMotor.writeMicroseconds(ui_Right_Motor_Speed);
            stage=6;
          } 
        }

        else if(stage==6) // finding the track again---------------------------------------------------------------------------------------------------------------------------------------------
        {


          CharliePlexM::Write(5,LOW);
          CharliePlexM::Write(8,HIGH);
          CharliePlexM::Write(11,HIGH);
          //CharliePlexM::Write(NEXT LED???,HIGH);

          if(lost){ //if the middle line tracker is not over a line, back dat ass up

            ui_Left_Motor_Speed=1300;
            ui_Right_Motor_Speed=1280;
            servo_LeftMotor.writeMicroseconds(ui_Left_Motor_Speed);
            servo_RightMotor.writeMicroseconds(ui_Right_Motor_Speed);
            delay(300);

            if(ui_Middle_Line_Tracker_Data < (ui_Middle_Line_Tracker_Dark - ui_Line_Tracker_Tolerance)||ui_Right_Line_Tracker_Data < (ui_Right_Line_Tracker_Dark - ui_Line_Tracker_Tolerance)||ui_Left_Line_Tracker_Data < (ui_Left_Line_Tracker_Dark - ui_Line_Tracker_Tolerance))//once it finds a line with any sensor, stop backing up
            {
              //delay(100);//may not need this, im thinking it reads middle line tracker then still will back up alittle past it?
              //if we need it we might have to change the value slightly
              lost=false;
              ui_Left_Motor_Speed=1500;
              ui_Right_Motor_Speed=1500;
              servo_LeftMotor.writeMicroseconds(ui_Left_Motor_Speed); //next 8 lines Matt added after cam left
              servo_RightMotor.writeMicroseconds(ui_Right_Motor_Speed); 
              //delay(300); //might have to change value
              ui_Left_Motor_Speed=1750; //wide right turn to find the line
              ui_Right_Motor_Speed=1550;
              servo_LeftMotor.writeMicroseconds(ui_Left_Motor_Speed);
              servo_RightMotor.writeMicroseconds(ui_Right_Motor_Speed); //to here
              ui_StopCounter=0;//this is for the next stage. resetting the stop counter will allow it to re-follow the course
              rightMotorPos=encoder_RightMotor.getPosition();
              delay(500);
              ui_Left_Motor_Speed=1700;
              ui_Right_Motor_Speed=1700;//start driving straight
              servo_LeftMotor.writeMicroseconds(ui_Left_Motor_Speed);
              servo_RightMotor.writeMicroseconds(ui_Right_Motor_Speed);
              
            }

            if((lost==false) && ((ui_Left_Line_Tracker_Data < (ui_Left_Line_Tracker_Dark - ui_Line_Tracker_Tolerance)) || (ui_Middle_Line_Tracker_Data < (ui_Middle_Line_Tracker_Dark - ui_Line_Tracker_Tolerance)) || (ui_Right_Line_Tracker_Data < (ui_Right_Line_Tracker_Dark - ui_Line_Tracker_Tolerance))))
            {
              stage=7;
            } // will go to next stage when it finds the line again

          }  
          servo_LeftMotor.writeMicroseconds(ui_Left_Motor_Speed);
          servo_RightMotor.writeMicroseconds(ui_Right_Motor_Speed);
        }

        else if(stage==7)//driving back through the course, avoiding the false tracks, with the flag --------------------------------------------------------------------------------------------------------------------------------------------
          //false track code is not yet included in this version
        {
          readLineTrackers();
\
          CharliePlexM::Write(8,LOW);
          CharliePlexM::Write(11,HIGH);
          //copied and pasted code from stage 1
          motorState=0;
          
          if(part1)//this if statement ensures the stopCounter will be accurate
          {
            if(encoder_RightMotor.getPosition()>rightMotorPos+2)
            {
              ui_StopCounter=1;
            }
          
            if(ui_StopCounter==1)
            {
              part1=false;
            }  
          }
          
          
          if(ui_Left_Line_Tracker_Data < (ui_Left_Line_Tracker_Dark - ui_Line_Tracker_Tolerance))
          {
            //CharliePlexM::Write(ci_Left_Line_Tracker_LED, HIGH);
            motorState+=100;
          }
          else
          { 
            //CharliePlexM::Write(ci_Left_Line_Tracker_LED, LOW);
          }
          if(ui_Middle_Line_Tracker_Data < (ui_Middle_Line_Tracker_Dark - ui_Line_Tracker_Tolerance))
          {
            //CharliePlexM::Write(ci_Middle_Line_Tracker_LED, HIGH);
            motorState+=10;
          }
          else
          { 
            //CharliePlexM::Write(ci_Middle_Line_Tracker_LED, LOW);
          }
          if(ui_Right_Line_Tracker_Data < (ui_Right_Line_Tracker_Dark - ui_Line_Tracker_Tolerance))
          {
            //CharliePlexM::Write(ci_Right_Line_Tracker_LED, HIGH);
            motorState+=1;
          }
          else
          { 
            //CharliePlexM::Write(ci_Right_Line_Tracker_LED, LOW);
          }



          if(motorState==11 && ui_StopCounter!=2 && ui_StopCounter!=3)
          {
            ui_Left_Motor_Speed+=75;
            ui_Right_Motor_Speed-=25;
          }
          else if(motorState==1 && ui_StopCounter!=2 && ui_StopCounter!=3)
          {
            ui_Left_Motor_Speed+=125;
            ui_Right_Motor_Speed-=25;
          }
          else if (motorState==110 && (ui_StopCounter!=1||ui_StopCounter!=0))
          {
            ui_Right_Motor_Speed+=75;
            ui_Left_Motor_Speed-=25;
          }
          else if(motorState==100 && (ui_StopCounter!=1||ui_StopCounter!=0))
          {
            ui_Right_Motor_Speed+=125;
            ui_Left_Motor_Speed-=25;
          }
          else if (motorState==10)
          {
            ui_Right_Motor_Speed=1650;
            ui_Left_Motor_Speed=1650;
          }
          else if(motorState==111)//when the bot hits a stop
          {
            if(stopStart==0)
            {
              previousMillis=millis();
              // Serial.println(motorState);
              //Serial.println(previousMillis);
              stopStart=1;
              //Serial.println(stopStart);
            }

            if(millis()-previousMillis>=213)  
            {
              // Serial.println(currentMillis-previousMillis);
              stopStart=2;
              // Serial.println(stopStart);
              currentMillis=0;
              previousMillis=0;
            }

          }
          else if(motorState==0)//gone off track
          {
            if(prevMotorState==1)
            {
              ui_Left_Motor_Speed+=250;
              ui_Right_Motor_Speed-=100;
            }
            else if(prevMotorState==100)
            {
              ui_Right_Motor_Speed+=250;
              ui_Left_Motor_Speed-=150;
            }
          }


          if(motorState!=111 && stopStart==2)//count the stop
          {
            ui_StopCounter++;
            stopStart=0;

            if(ui_StopCounter==ci_NumberStops)
            {
              ui_Left_Motor_Speed=1500;
              ui_Right_Motor_Speed=1500;
              turnt=false;//sets turnt to false again
              stage=8;              
            }

          }


          if(bt_Motors_Enabled)
          {
            servo_LeftMotor.writeMicroseconds(ui_Left_Motor_Speed);
            servo_RightMotor.writeMicroseconds(ui_Right_Motor_Speed);
          }
          else
          {  
            servo_LeftMotor.writeMicroseconds(ci_Left_Motor_Stop); 
            servo_RightMotor.writeMicroseconds(ci_Right_Motor_Stop); 
          }


          if(motorState!=0)//so the bot can remember where it was relative to the line
            prevMotorState=motorState;



          servo_LeftMotor.writeMicroseconds(ui_Left_Motor_Speed);//are these two lines needed or are they redundant?
          servo_RightMotor.writeMicroseconds(ui_Right_Motor_Speed);
        }

        else if(stage==8)//getting close enough to the box to drop off the LED, drops off LED --------------------------------------------------------------------------------------------------------------------------------------------
        {
          Ping(); //wont need this if we just use delays(which would be easier/less buggy)
          distance=ul_Echo_Time/58; //gives distance in cm

          //may need to turn here??? we sure do


          if(!turnt){//this is the spot that was causing us problems last time i belive.  by putting the other if statements within the turnt if statement, the problem should be voided. hopefully <3

            rightMotorPos=encoder_RightMotor.getPosition();
//            Serial.print("Right motor Position: ");
//            Serial.println(rightMotorPos);
            positionHolder=rightMotorPos+1.6;


            while(encoder_RightMotor.getPosition()<positionHolder)
            {
              rightMotorPos=encoder_RightMotor.getPosition();
  //              Serial.print("Right motor Position: ");
  //              Serial.println(rightMotorPos); 
              ui_Left_Motor_Speed=1500;
              ui_Right_Motor_Speed=1700;
              servo_LeftMotor.writeMicroseconds(ui_Left_Motor_Speed);
              servo_RightMotor.writeMicroseconds(ui_Right_Motor_Speed);

              if(encoder_RightMotor.getPosition()>=positionHolder)
              {
                break;
              }
            }
            //            if(leftMotorPos<=0.2)
            //            {ui_Left_Motor_Speed=1700;
            //             ui_Right_Motor_Speed=1500;}

            ui_Left_Motor_Speed=1500;
            ui_Right_Motor_Speed=1500;
            servo_LeftMotor.writeMicroseconds(ui_Left_Motor_Speed);
            servo_RightMotor.writeMicroseconds(ui_Right_Motor_Speed);
            delay(1000);
            turnt=true;
          }

          else
          {
            ui_Left_Motor_Speed=1700; // moves towards box
            ui_Right_Motor_Speed=1700;
            servo_ArmMotor.write(120);
            servo_LeftMotor.writeMicroseconds(ui_Left_Motor_Speed);
            servo_RightMotor.writeMicroseconds(ui_Right_Motor_Speed);
            delay(100);//may need to change this depending on how long it takes to rach brown box
            ui_Left_Motor_Speed=1500; //stops at box
            ui_Right_Motor_Speed=1500;
            servo_LeftMotor.writeMicroseconds(ui_Left_Motor_Speed);
            servo_RightMotor.writeMicroseconds(ui_Right_Motor_Speed);
            stage=9;

          }



          //doo we ne this? i think its better to just turn and delay drive fwd?
          //          if(distance>3)
          //            ui_Left_Motor_Speed=1700;
          //            ui_Right_Motor_Speed=1700;
          //          }
          //          else if(distance<=3){
          //            ui_Left_Motor_Speed=1500;
          //            ui_Right_Motor_Speed=1500;
          //            servo_ArmMotor.write(120);//extends arm to drop off the LED
          //            stage=9;//gets close enough, changes stages
          //          } 
        }

        else if(stage==9)
        {
          ul_Grip_Motor_Position=encoder_GripMotor.getRawPosition();
          //Serial.println(ul_Grip_Motor_Position);
          if(ul_Grip_Motor_Position<=350){
            servo_GripMotor.write(1700);//gripper is opened, dropping the LED
          }
          else{
            servo_GripMotor.write(1500);
            shakeClaw();
            ui_Left_Motor_Speed=1300;
            ui_Right_Motor_Speed=1300;
            servo_LeftMotor.writeMicroseconds(ui_Left_Motor_Speed);
            servo_RightMotor.writeMicroseconds(ui_Right_Motor_Speed);
            delay(200);
            ui_Left_Motor_Speed=1500;
            ui_Right_Motor_Speed=1500;
            servo_ArmMotor.write(65);
            servo_LeftMotor.writeMicroseconds(ui_Left_Motor_Speed);
            servo_RightMotor.writeMicroseconds(ui_Right_Motor_Speed);
            
            ul_Grip_Motor_Position=encoder_GripMotor.getRawPosition();
            if(ul_Grip_Motor_Position>=60)
            {
              servo_GripMotor.write(1300);
            }//closes grip motor
            
            ui_Robot_State_Index=0;//put it back to mode 0?
            stage=10;//kills the program. go slick :D
          }
        }

        delay(10);
        //        Ping();
        //        distance=ul_Echo_Time/58;
        //        Serial.print("Distance:");
        //        Serial.println(distance);
//        Serial.print("Stage: ");
//        Serial.println(stage);


        //        while(encoder_GripMotor.getRawPosition()<ci_Grip_Motor_Open)
        //        {servo_GripMotor.writeMicroseconds(1800);}
        //        
        //        while(encoder_GripMotor.getRawPosition()>ci_Grip_Motor_Closed)
        //        {servo_GripMotor.writeMicroseconds(1300);}

#ifdef DEBUG_MOTORS  
        Serial.print("Motors: Default: ");
        Serial.print(ui_Motors_Speed);
        Serial.print(" , Left = ");
        Serial.print(ui_Left_Motor_Speed);
        Serial.print(" . Right = ");
        Serial.println(ui_Right_Motor_Speed);
#endif    
        ui_Mode_Indicator_Index = 1;
      }
      break;
    } 

  case 2:    //Calibrate line tracker light levels after 3 seconds
    {
      if(bt_3_S_Time_Up)
      {
        if(!bt_Cal_Initialized)
        {
          bt_Cal_Initialized = true;
          ui_Left_Line_Tracker_Light = 0;
          ui_Middle_Line_Tracker_Light = 0;
          ui_Right_Line_Tracker_Light = 0;
          ul_Calibration_Time = millis();
          ui_Cal_Count = 0;
        }
        else if((millis() - ul_Calibration_Time) > ci_Line_Tracker_Calibration_Interval)
        {
          ul_Calibration_Time = millis();
          readLineTrackers();
          ui_Left_Line_Tracker_Light += ui_Left_Line_Tracker_Data;
          ui_Middle_Line_Tracker_Light += ui_Middle_Line_Tracker_Data;
          ui_Right_Line_Tracker_Light += ui_Right_Line_Tracker_Data;
          ui_Cal_Count++;
        }
        if(ui_Cal_Count == ci_Line_Tracker_Cal_Measures)
        {
          ui_Left_Line_Tracker_Light /= ci_Line_Tracker_Cal_Measures;
          ui_Middle_Line_Tracker_Light /= ci_Line_Tracker_Cal_Measures;
          ui_Right_Line_Tracker_Light /= ci_Line_Tracker_Cal_Measures;
#ifdef DEBUG_LINE_TRACKER_CALIBRATION
          Serial.print("Light Levels: Left = ");
          Serial.print(ui_Left_Line_Tracker_Light,DEC);
          Serial.print(", Middle = ");
          Serial.print(ui_Middle_Line_Tracker_Light,DEC);
          Serial.print(", Right = ");
          Serial.println(ui_Right_Line_Tracker_Light,DEC);
#endif           
          EEPROM.write(ci_Left_Line_Tracker_Light_Address_L, lowByte(ui_Left_Line_Tracker_Light));
          EEPROM.write(ci_Left_Line_Tracker_Light_Address_H, highByte(ui_Left_Line_Tracker_Light));
          EEPROM.write(ci_Middle_Line_Tracker_Light_Address_L, lowByte(ui_Middle_Line_Tracker_Light));
          EEPROM.write(ci_Middle_Line_Tracker_Light_Address_H, highByte(ui_Middle_Line_Tracker_Light));
          EEPROM.write(ci_Right_Line_Tracker_Light_Address_L, lowByte(ui_Right_Line_Tracker_Light));
          EEPROM.write(ci_Right_Line_Tracker_Light_Address_H, highByte(ui_Right_Line_Tracker_Light));
          ui_Robot_State_Index = 0;    // go back to Mode 0
        }
        ui_Mode_Indicator_Index = 2; 
      }
      break;
    }

  case 3:    // Calibrate line tracker dark levels after 3 seconds
    {
      if(bt_3_S_Time_Up)
      {
        if(!bt_Cal_Initialized)
        {
          bt_Cal_Initialized = true;
          ui_Left_Line_Tracker_Dark = 0;
          ui_Middle_Line_Tracker_Dark = 0;
          ui_Right_Line_Tracker_Dark = 0;
          ul_Calibration_Time = millis();
          ui_Cal_Count = 0;
        }
        else if((millis() - ul_Calibration_Time) > ci_Line_Tracker_Calibration_Interval)
        {
          ul_Calibration_Time = millis();
          readLineTrackers();
          ui_Left_Line_Tracker_Dark += ui_Left_Line_Tracker_Data;
          ui_Middle_Line_Tracker_Dark += ui_Middle_Line_Tracker_Data;
          ui_Right_Line_Tracker_Dark += ui_Right_Line_Tracker_Data;
          ui_Cal_Count++;
        }
        if(ui_Cal_Count == ci_Line_Tracker_Cal_Measures)
        {
          ui_Left_Line_Tracker_Dark /= ci_Line_Tracker_Cal_Measures;
          ui_Middle_Line_Tracker_Dark /= ci_Line_Tracker_Cal_Measures;
          ui_Right_Line_Tracker_Dark /= ci_Line_Tracker_Cal_Measures;
#ifdef DEBUG_LINE_TRACKER_CALIBRATION
          Serial.print("Dark Levels: Left = ");
          Serial.print(ui_Left_Line_Tracker_Dark,DEC);
          Serial.print(", Middle = ");
          Serial.print(ui_Middle_Line_Tracker_Dark,DEC);
          Serial.print(", Right = ");
          Serial.println(ui_Right_Line_Tracker_Dark,DEC);
#endif           
          EEPROM.write(ci_Left_Line_Tracker_Dark_Address_L, lowByte(ui_Left_Line_Tracker_Dark));
          EEPROM.write(ci_Left_Line_Tracker_Dark_Address_H, highByte(ui_Left_Line_Tracker_Dark));
          EEPROM.write(ci_Middle_Line_Tracker_Dark_Address_L, lowByte(ui_Middle_Line_Tracker_Dark));
          EEPROM.write(ci_Middle_Line_Tracker_Dark_Address_H, highByte(ui_Middle_Line_Tracker_Dark));
          EEPROM.write(ci_Right_Line_Tracker_Dark_Address_L, lowByte(ui_Right_Line_Tracker_Dark));
          EEPROM.write(ci_Right_Line_Tracker_Dark_Address_H, highByte(ui_Right_Line_Tracker_Dark));
          ui_Robot_State_Index = 0;    // go back to Mode 0
        }
        ui_Mode_Indicator_Index = 3;
      }
      break;
    }

  case 4:    //Calibrate motor straightness after 3 seconds.
    {
      if(bt_3_S_Time_Up)
      {
        if(!bt_Cal_Initialized)
        {
          bt_Cal_Initialized = true;
          encoder_LeftMotor.zero();
          encoder_RightMotor.zero();
          ul_Calibration_Time = millis();
          servo_LeftMotor.writeMicroseconds(ui_Motors_Speed);
          servo_RightMotor.writeMicroseconds(ui_Motors_Speed);
        }
        else if((millis() - ul_Calibration_Time) > ci_Motor_Calibration_Time)
        {
          servo_LeftMotor.writeMicroseconds(ci_Left_Motor_Stop); 
          servo_RightMotor.writeMicroseconds(ci_Right_Motor_Stop); 
          ul_Left_Motor_Position = encoder_LeftMotor.getRawPosition();
          ul_Right_Motor_Position = encoder_RightMotor.getRawPosition();
          if(ul_Left_Motor_Position < ul_Right_Motor_Position)
          {
            // May have to update this if different calibration time is used
            ui_Right_Motor_Offset = (ul_Left_Motor_Position - ul_Right_Motor_Position)/3.43;
            ui_Left_Motor_Offset = 0;  
          }
          else
          {
            // May have to update this if different calibration time is used
            ui_Right_Motor_Offset = 0;
            ui_Left_Motor_Offset = (ul_Right_Motor_Position - ul_Left_Motor_Position)/3.43;
          }

#ifdef DEBUG_MOTOR_CALIBRATION
          Serial.print("Motor Offsets: Right = ");
          Serial.print(ui_Right_Motor_Offset);
          Serial.print(", Left = ");
          Serial.println(ui_Left_Motor_Offset);
#endif              
          EEPROM.write(ci_Right_Motor_Offset_Address_L, lowByte(ui_Right_Motor_Offset));
          EEPROM.write(ci_Right_Motor_Offset_Address_H, highByte(ui_Right_Motor_Offset));
          EEPROM.write(ci_Left_Motor_Offset_Address_L, lowByte(ui_Left_Motor_Offset));
          EEPROM.write(ci_Left_Motor_Offset_Address_H, highByte(ui_Left_Motor_Offset));

          ui_Robot_State_Index = 0;    // go back to Mode 0 
        }
#ifdef DEBUG_ENCODERS           
        Serial.print("Encoders L: ");
        Serial.print(encoder_LeftMotor.getRawPosition());
        Serial.print(", R: ");
        Serial.println(encoder_RightMotor.getRawPosition());
#endif        
        ui_Mode_Indicator_Index = 4;
      } 
      break;
    }    
  }

  if((millis() - ul_Display_Time) > ci_Display_Time)
  {
    ul_Display_Time = millis();

#ifdef DEBUG_MODE_DISPLAY  
    Serial.print("Mode: ");
    Serial.println(ui_Mode_Indicator[ui_Mode_Indicator_Index], DEC);
#endif
    bt_Heartbeat = !bt_Heartbeat;
    CharliePlexM::Write(ci_Heartbeat_LED, bt_Heartbeat);
    digitalWrite(13, bt_Heartbeat);
    Indicator();
  }
} 

// set mode indicator LED state
void Indicator()
{
  //display routine, if true turn on led
  CharliePlexM::Write(ci_Indicator_LED,!(ui_Mode_Indicator[ui_Mode_Indicator_Index] & 
    (iArray[iArrayIndex])));
  iArrayIndex++;
  iArrayIndex = iArrayIndex & 15;
}

// read values from line trackers and update status of line tracker LEDs
void readLineTrackers()
{
  ui_Left_Line_Tracker_Data = analogRead(ci_Left_Line_Tracker);
  ui_Middle_Line_Tracker_Data = analogRead(ci_Middle_Line_Tracker);
  ui_Right_Line_Tracker_Data = analogRead(ci_Right_Line_Tracker);

  if(ui_Left_Line_Tracker_Data < (ui_Left_Line_Tracker_Dark - ui_Line_Tracker_Tolerance))
  {
    CharliePlexM::Write(ci_Left_Line_Tracker_LED, HIGH);
  }
  else
  { 
    CharliePlexM::Write(ci_Left_Line_Tracker_LED, LOW);
  }
  if(ui_Middle_Line_Tracker_Data < (ui_Middle_Line_Tracker_Dark - ui_Line_Tracker_Tolerance))
  {
    CharliePlexM::Write(ci_Middle_Line_Tracker_LED, HIGH);
  }
  else
  { 
    CharliePlexM::Write(ci_Middle_Line_Tracker_LED, LOW);
  }
  if(ui_Right_Line_Tracker_Data < (ui_Right_Line_Tracker_Dark - ui_Line_Tracker_Tolerance))
  {
    CharliePlexM::Write(ci_Right_Line_Tracker_LED, HIGH);
  }
  else
  { 
    CharliePlexM::Write(ci_Right_Line_Tracker_LED, LOW);
  }

#ifdef DEBUG_LINE_TRACKERS
  Serial.print("Trackers: Left = ");
  Serial.print(ui_Left_Line_Tracker_Data,DEC);
  Serial.print(", Middle = ");
  Serial.print(ui_Middle_Line_Tracker_Data,DEC);
  Serial.print(", Right = ");
  Serial.println(ui_Right_Line_Tracker_Data,DEC);
#endif

}

// measure distance to target using ultrasonic sensor  
void Ping()
{
  //Ping Ultrasonic
  //Send the Ultrasonic Range Finder a 10 microsecond pulse per tech spec
  digitalWrite(ci_Ultrasonic_Ping, HIGH);
  delayMicroseconds(10);  //The 10 microsecond pause where the pulse in "high"
  digitalWrite(ci_Ultrasonic_Ping, LOW);
  //use command pulseIn to listen to Ultrasonic_Data pin to record the
  //time that it takes from when the Pin goes HIGH until it goes LOW 
  ul_Echo_Time = pulseIn(ci_Ultrasonic_Data, HIGH, 10000);

  // Print Sensor Readings
#ifdef DEBUG_ULTRASONIC
  Serial.print("Time (microseconds): ");
  Serial.print(ul_Echo_Time, DEC);
  Serial.print(", Inches: ");
  Serial.print(ul_Echo_Time/148); //divide time by 148 to get distance in inches
  Serial.print(", cm: ");
  Serial.println(ul_Echo_Time/58); //divide time by 58 to get distance in cm 
#endif
} 

void shakeClaw()
{
  ui_Left_Motor_Speed=1700;
  ui_Right_Motor_Speed=1400;
  servo_LeftMotor.writeMicroseconds(ui_Left_Motor_Speed);
  servo_RightMotor.writeMicroseconds(ui_Right_Motor_Speed);
  delay(500); //may need to change delay values
  ui_Left_Motor_Speed=1400;
  ui_Right_Motor_Speed=1700;
  servo_LeftMotor.writeMicroseconds(ui_Left_Motor_Speed);
  servo_RightMotor.writeMicroseconds(ui_Right_Motor_Speed);
  delay(500);
  ui_Left_Motor_Speed=1700;
  ui_Right_Motor_Speed=1400;
  servo_LeftMotor.writeMicroseconds(ui_Left_Motor_Speed);
  servo_RightMotor.writeMicroseconds(ui_Right_Motor_Speed);
  delay(500);
  ui_Left_Motor_Speed=1400;
  ui_Right_Motor_Speed=1700;
  servo_LeftMotor.writeMicroseconds(ui_Left_Motor_Speed);
  servo_RightMotor.writeMicroseconds(ui_Right_Motor_Speed);
  delay(500);
}

















