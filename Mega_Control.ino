#include <PID_v1.h>
#include <ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>

#define encoder1PinA      18
#define encoder1PinB      28
#define M1                4

#define encoder2PinA      19
#define encoder2PinB      30
#define M2                5

#define encoder3PinA      20
#define encoder3PinB      32
#define M3                6

#define encoder4PinA      21
#define encoder4PinB      34
#define M4                7

#define Motors_Right_Dir1  36
#define Motors_Right_Dir2  38
#define Motors_Left_Dir1   40
#define Motors_Left_Dir2   42

ros::NodeHandle  ard;
std_msgs::Int32 str_msg;
ros::Publisher vel("velocity", &str_msg);
String x;
void messageCb( const std_msgs::String& toggle_msg){
  x=toggle_msg.data;
}

ros::Subscriber<std_msgs::String> sub("keys", &messageCb );

volatile long encoder1Pos=0,encoder2Pos=0,encoder3Pos=0,encoder4Pos=0;
long newposition1,oldposition1=0,newposition2,oldposition2=0,newposition3,oldposition3=0,newposition4,oldposition4=0;
unsigned long newtime,oldtime=0,timestep=0;
double Speed1=0,Speed2=0,Speed3=0,Speed4=0;
double M1_pwm=0,M2_pwm=0,M3_pwm=0,M4_pwm=0;
double Setpoint=0;
int Forward,Backward,Right,Left;
long rpm1,rpm2,rpm3,rpm4;
long count1=0,count2=0,count3=0,count4=0;


PID M1_PID(&Speed1, &M1_pwm, &Setpoint, 1.217,6.436, 0, DIRECT);
PID M2_PID(&Speed2, &M2_pwm, &Setpoint, 1.217,6.436, 0, DIRECT);
PID M3_PID(&Speed3, &M3_pwm, &Setpoint, 1.217,6.436, 0, DIRECT);
PID M4_PID(&Speed4, &M4_pwm, &Setpoint, 1.217,6.436, 0, DIRECT);


void setup(){

      ard.initNode();
      ard.advertise(vel);
      ard.subscribe(sub);

  Serial.begin(57600);
      pinMode(Motors_Right_Dir1,OUTPUT);
      pinMode(Motors_Left_Dir1,OUTPUT);
      pinMode(Motors_Right_Dir2,OUTPUT);
      pinMode(Motors_Left_Dir2,OUTPUT);

   // Motor 1 + encoder configuration
      
      pinMode(M1,OUTPUT);
      pinMode(encoder1PinA, INPUT_PULLUP);
      digitalWrite(encoder1PinA, HIGH);      
      pinMode(encoder1PinB, INPUT_PULLUP);
      digitalWrite(encoder1PinB, HIGH);   
      attachInterrupt(5, doEncoder1, RISING);  

   // Motor 2 + encoder configuration
      
      pinMode(M2,OUTPUT); 
      pinMode(encoder2PinA, INPUT_PULLUP);
      digitalWrite(encoder2PinA, HIGH);      
      pinMode(encoder2PinB, INPUT_PULLUP);
      digitalWrite(encoder2PinB, HIGH); 
      attachInterrupt(4, doEncoder2, RISING);  

    // Motor 3 + encoder configuration
      
      pinMode(M3,OUTPUT);
      pinMode(encoder3PinA, INPUT_PULLUP);
      digitalWrite(encoder3PinA, HIGH);      
      pinMode(encoder3PinB, INPUT_PULLUP);
      digitalWrite(encoder3PinB, HIGH);   
      attachInterrupt(3, doEncoder3, RISING);  

    // Motor 4 + encoder configuration
      
      pinMode(M4,OUTPUT);
      pinMode(encoder4PinA, INPUT);
      digitalWrite(encoder4PinA, HIGH);      
      pinMode(encoder4PinB, INPUT);
      digitalWrite(encoder4PinB, HIGH);   
      attachInterrupt(2, doEncoder4, RISING);  
  
   //turn the PID on
   
      M1_PID.SetMode(AUTOMATIC);
      M1_PID.SetSampleTime(5);
      M1_PID.SetOutputLimits(100, 255); 
      M2_PID.SetMode(AUTOMATIC);
      M2_PID.SetSampleTime(5);
      M2_PID.SetOutputLimits(100, 255); 
      M3_PID.SetMode(AUTOMATIC);
      M3_PID.SetSampleTime(5);
      M3_PID.SetOutputLimits(100, 255); 
      M4_PID.SetMode(AUTOMATIC);
      M4_PID.SetSampleTime(5);
      M4_PID.SetOutputLimits(100, 255); 
      Serial.println("Start");
}

void loop(){           
 
                newtime = micros();
                timestep=(newtime-oldtime);
                
                velocity1();
                velocity2();
                velocity3();
                velocity4();
                
                Speed1=abs(rpm1);
                Speed2=abs(rpm2);
                Speed3=abs(rpm3);
                Speed4=abs(rpm4);
                
                M1_PID.Compute();
                M2_PID.Compute();
                M3_PID.Compute();
                M4_PID.Compute();
              
                
                oldtime=newtime;
                
                str_msg.data = rpm1;
                vel.publish(&str_msg);
                ard.spinOnce();
                control();
         
                delay(100);

  }

int control(){
   if(x=="s"){    
        Setpoint = 80;
        digitalWrite(Motors_Right_Dir1,HIGH);
        digitalWrite(Motors_Right_Dir2,LOW);
        analogWrite(M1,M1_pwm);
        analogWrite(M2,M2_pwm);
        digitalWrite(Motors_Left_Dir1,HIGH);
        digitalWrite(Motors_Left_Dir2,LOW);
        analogWrite(M3,M3_pwm); 
        analogWrite(M4,M4_pwm); 
//Serial.println("Forward");
   }
   
   else if(x=="a"){    
         Setpoint = 80;
        digitalWrite(Motors_Right_Dir1,HIGH);
        digitalWrite(Motors_Right_Dir2,LOW);
        analogWrite(M1,M1_pwm);
        analogWrite(M2,M2_pwm);
        digitalWrite(Motors_Left_Dir1,LOW);
        digitalWrite(Motors_Left_Dir2,HIGH);
        analogWrite(M3,M3_pwm); 
        analogWrite(M4,M4_pwm); 
        //Serial.println("Left");
   }
   
       
  else if(x=="w"){    
        Setpoint = 80;
        digitalWrite(Motors_Right_Dir1,LOW);
        digitalWrite(Motors_Right_Dir2,HIGH);
        analogWrite(M1,M1_pwm);
        analogWrite(M2,M2_pwm);
        digitalWrite(Motors_Left_Dir1,LOW);
        digitalWrite(Motors_Left_Dir2,HIGH);
        analogWrite(M3,M3_pwm); 
        analogWrite(M4,M4_pwm); 
   //     Serial.println("Backward");
   }
   

   else if(x=="d"){    
        Setpoint = 80;
        digitalWrite(Motors_Right_Dir1,LOW);
        digitalWrite(Motors_Right_Dir2,HIGH);
        analogWrite(M1,M1_pwm);
        analogWrite(M2,M2_pwm);
        digitalWrite(Motors_Left_Dir1,HIGH);
        digitalWrite(Motors_Left_Dir2,LOW);
        analogWrite(M3,M3_pwm); 
        analogWrite(M4,M4_pwm); 
  //      Serial.println("Right");
   }

   else {      
        Setpoint = 0;
        digitalWrite(Motors_Right_Dir1,LOW);
        digitalWrite(Motors_Right_Dir2,HIGH);
        analogWrite(M1,0);
        analogWrite(M2,0);
        digitalWrite(Motors_Left_Dir1,LOW);
        digitalWrite(Motors_Left_Dir2,HIGH);
        analogWrite(M3,0); 
        analogWrite(M4,0);  
  //      Serial.println("Stopped");
    }
}

void doEncoder1()
{
  if (digitalRead(encoder1PinA) == digitalRead(encoder1PinB)) {
    encoder1Pos++;
  } else {
    encoder1Pos--;
  }
}

void doEncoder2()
{
  if (digitalRead(encoder2PinA) == digitalRead(encoder2PinB)) {
    encoder2Pos++;
  } else {
    encoder2Pos--;
  }
}

void doEncoder3()
{
  if (digitalRead(encoder3PinA) == digitalRead(encoder3PinB)) {
    encoder3Pos++;
  } else {
    encoder3Pos--;
  }
}

void doEncoder4()
{
  if (digitalRead(encoder4PinA) == digitalRead(encoder4PinB)) {
    encoder4Pos++;
  } else {
    encoder4Pos--;
  }
}

float velocity1(){
  
    newposition1 = encoder1Pos;
    count1=(newposition1-oldposition1);
    rpm1=259740.25*count1/timestep; //60*10^6/21/11=259740.25
    oldposition1=newposition1;
  return rpm1;
  }

float velocity2(){
  
    newposition2 = encoder2Pos;
    count2=(newposition2-oldposition2);
    rpm2=259740.25*count2/timestep; //60*10^6/21/11=259740.25
    oldposition2=newposition2;
  return rpm2;
  }

float velocity3(){
  
    newposition3 = encoder3Pos;
    count3=(newposition3-oldposition3);
    rpm3=259740.25*count3/timestep; //60*10^6/21/11=259740.25
    oldposition3=newposition3;
  return rpm3;
  }

float velocity4(){
  
    newposition4 = encoder4Pos;
    count4=(newposition4-oldposition4);
    rpm4=259740.25*count4/timestep; //60*10^6/21/11=259740.25
    oldposition4=newposition4;
  return rpm4;
  }
