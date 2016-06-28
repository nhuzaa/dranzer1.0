#include <QTRSensors.h>
#include <SoftwareSerial.h>

#define Kp 0.04// experiment to determine this, start by something small that just makes your bot follow the line at a slow speed
#define Kd 0.02// experiment to determine this, slowly increase the speeds and adjust this value. ( Note: Kp < Kd) 
#define Ki 0.05
#define rightMaxSpeed 190 // max speed of the robot
#define leftMaxSpeed 190// max speed of the robot
#define rightBaseSpeed 255 // this is the speed at which the motors should spin when the robot is perfectly on the line
#define leftBaseSpeed 255// this is the speed at which the motors should spin when the robot is perfectly on the line
#define NUM_SENSORS  6     // number of sensors used
#define TIMEOUT       2500  // waits for 2500 us for sensor outputs to go low
//#define EMITTER_PIN   1    // emitter is controlled by digital pin 2

#define rightMotor2 12
#define rightMotor1 11
#define rightMotorPWM 10
#define leftMotor2 6
#define leftMotor1 5
#define leftMotorPWM 9
#define HORN 13
#define roller1 7
#define roller2 8
QTRSensorsRC qtrrc((unsigned char[]) {  14, 15, 16, 17, 18, 19} ,NUM_SENSORS, TIMEOUT);// sensor connected through analog pins A0 - A5 i.e. digital pins 14-19



const int RX_PIN = 2;
const int TX_PIN = 4;
SoftwareSerial bluetooth(RX_PIN, TX_PIN);

unsigned int sensorValues[NUM_SENSORS];
bool hornFlag = 0;
bool servoFlag = 0;
int pos;

void setup()
{
  pinMode(rightMotor1, OUTPUT);
  pinMode(rightMotor2, OUTPUT);
  pinMode(rightMotorPWM, OUTPUT);
  pinMode(leftMotor1, OUTPUT);
  pinMode(leftMotor2, OUTPUT);
  pinMode(leftMotorPWM, OUTPUT);
  pinMode(roller1, OUTPUT);
  pinMode(roller2, OUTPUT);
  pinMode(HORN, OUTPUT);
   
  
  digitalWrite( HORN, HIGH);
  
  
  for (int i = 0; i < 100; i++)
  {
  // calibrate for sometime by sliding the sensors across the line, or you may use auto-calibration instead

  /* comment this part out for automatic calibration 
  if ( i  < 25 || i >= 75 ) // turn to the left and right to expose the sensors to the brightest and darkest readings that may be encountered
     turn_right();  
   else
     turn_left(); */ 
     
    qtrrc.calibrate();   
    delay(20);

  }
//delay(2000); // wait for 2s to position the bot before entering the main loop 
    digitalWrite(HORN, LOW);
    // Calibration Over 
    //Print the Min and Max Values
    
    Serial.begin(9600);
    bluetooth.begin(9600);
    Serial.println();
    Serial.print('Minimun');
    for (int i = 0; i < NUM_SENSORS; i++)
    {
      Serial.print(qtrrc.calibratedMinimumOn[i]);
      Serial.print(' ');
    }
    Serial.println();
    
    Serial.print('Maximum');
    for (int i = 0; i < NUM_SENSORS; i++)
    {
      Serial.print(qtrrc.calibratedMaximumOn[i]);
      Serial.print(' ');
    }
    Serial.println();
    Serial.println();
    
  } 

int lastError = 0;

void enablePWM()
{
     analogWrite(rightMotorPWM ,  255 );
     analogWrite(leftMotorPWM , 255);
}
void motorRightHalt()
{
    digitalWrite (leftMotor1, LOW);
    digitalWrite (leftMotor2, LOW);
  
    
}

void motorRightForward()
{
    digitalWrite (leftMotor1, LOW);
    digitalWrite (leftMotor2, HIGH);
    enablePWM();    
}

void motorRightBackward()
{
    
    digitalWrite (leftMotor1, HIGH);
    digitalWrite (leftMotor2, LOW);
    enablePWM();

  
   
}

void motorLeftHalt()
{
    digitalWrite (rightMotor1, LOW);
    digitalWrite (rightMotor2, LOW);
     
    
}

void motorLeftForward()
{
    digitalWrite (rightMotor1, LOW);
    digitalWrite (rightMotor2, HIGH);
    enablePWM();
    
    
}

void motorLeftBackward()
{
    digitalWrite (rightMotor1, HIGH );
    digitalWrite (rightMotor2, LOW);
    enablePWM();
 
}

void motorRight( )
{
 
       motorRightBackward();
       motorLeftForward();
       enablePWM();
        
}
void motorLeft()
{  
      motorRightForward();
      motorLeftBackward();    
      enablePWM();
}


void motorRightAuto( )
{
 
       motorRightBackward();
       motorLeftForward();
      
        
}
void motorLeftAuto()
{  
      motorRightForward();
      motorLeftBackward();    
     
}
void loop()
{
 
 while(bluetooth.available())
  {
      char input = bluetooth.read();
      Serial.println(input);
      
      
      if( input == '5')
{
         
          while(bluetooth.read() != '6')
          {
            startMotorsAutomatic();
          }
          motorLeftHalt();
          motorRightHalt();
          
         
}else if(input == '7')
{ 
       digitalWrite(roller1, HIGH);
       digitalWrite(roller2,LOW);
     
         
}else if(input == '8')
{
        digitalWrite(roller1, LOW);
        digitalWrite(roller2,HIGH);

}else if( input == '1')
{
    motorRightForward();
    motorLeftForward();
        
}else if( input == '0')
{
    motorRightBackward();
    motorLeftBackward(); 
          
}else if(input =='2')
{
    motorLeft();
}else if(input == '3')
{
    
    motorRight();
}else if(input == 'U')
{
    Serial.print("Halt after u ");
    motorLeftHalt();
    motorRightHalt();
     digitalWrite(roller1, LOW);
        digitalWrite(roller2,LOW);
    
}else if(input == '4')
{
    if(hornFlag == 0)
  {
        digitalWrite(HORN, HIGH);
        hornFlag = 1;
    }else if(hornFlag == 1)
    {
        digitalWrite(HORN, LOW);
        hornFlag = 0;
    }
}else if( input =='R')
{
    software_Reset();      
         
}
}
}

void startMotorsAutomatic()
{

   unsigned int sensors[6];
  int position = qtrrc.readLine(sensors); // get calibrated readings along with the line position, refer to the QTR Sensors Arduino Library for more details on line position.
    int error = position - 2500 ;
  int integral = integral + error;
  int derivative = error - lastError;
  int motorSpeed = Kp * error + Kd * derivative + integral*Ki;
  lastError = error;
  
  valueSerial(error,derivative,integral, position);
  int rightMotorSpeed = rightBaseSpeed + motorSpeed;
  int leftMotorSpeed = leftBaseSpeed - motorSpeed;
  
    if (rightMotorSpeed > rightMaxSpeed ) rightMotorSpeed = rightMaxSpeed; // prevent the motor from going beyond max speed
  if (leftMotorSpeed > leftMaxSpeed ) leftMotorSpeed = leftMaxSpeed; // prevent the motor from going beyond max speed
  if (rightMotorSpeed < 0) rightMotorSpeed = 0; // keep the motor speed positive
  if (leftMotorSpeed < 0) leftMotorSpeed = 0; // keep the motor speed positive

  float revRate = error / 1000;
   if(rightMotorSpeed > leftMotorSpeed )
  {
        
        analogWrite(rightMotorPWM ,  rightMotorSpeed );
         motorRightAuto();
     
       // analogWrite(leftMotorPWM , leftMotorSpeed);
        analogWrite(leftMotorPWM ,  0.5 * rightMotorSpeed );
    
  }else if( rightMotorSpeed  < leftMotorSpeed )
  {

     
            motorLeftAuto();
        //analogWrite(rightMotorPWM ,  rightMotorSpeed );
        analogWrite(rightMotorPWM , 0.5  *  leftMotorSpeed );
      
         
        analogWrite(leftMotorPWM , leftMotorSpeed );
    
  }else
  {
  
       
        digitalWrite(rightMotor1 , LOW);
        digitalWrite(rightMotor2 , HIGH);
        analogWrite(rightMotorPWM ,  rightBaseSpeed);
     
        digitalWrite(leftMotor1 , LOW);
        digitalWrite(leftMotor2 , HIGH);
        analogWrite(leftMotorPWM , rightBaseSpeed);
    
 }
   
  
}

void valueSerial(int error ,int derivative , int integral, int position)
{
    Serial.print("error");
    Serial.print(error);
    Serial.print("\t"); 
    Serial.print("derivative");
    Serial.print(derivative);
        Serial.print("\t"); 
    Serial.print("integral");
    Serial.print(integral);
    Serial.print("\t"); 
    Serial.print("position");
    Serial.println(position);

    
}

void software_Reset()
// Restarts program from beginning but 
// does not reset the peripherals and registers
{
  asm volatile ("  jmp 0");  
} 

