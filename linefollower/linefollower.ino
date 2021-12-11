//PID LINE FOLLOWER 
//(imagine that you are driving the bot for right-left convention)

// use MEGA

//motor driver-pins must be pwm to control speed!!!

#define i1 10  //right
#define i2 9
#define i3 5  //left
#define i4 6
 
//IR ARRAY
#define  sensor1 A0     // RIGHT most sensor
#define  sensor2 A1
#define  sensor3 A2
#define  sensor4 A3    
#define  sensor5 A4     
     // LEFT most sensor


float ir[6];  //ir array values
int irv[6];
int s=150; //fixed speed
//int s2 = 130;
int rs,ls;    //right speed , left speed
//int z=70;

// PID Constants
float Kp=25;
float Ki=0;
float Kd=10;
 
float sum;

float pos;    //instantaneous position
float setpoint;
float PID_value = 0;


void setup() {

//weightage
irv[1]=15;
irv[2]=10;
irv[3]=5;
irv[4]=-10;
irv[5]=-15;

  
//motor driver
  pinMode(i1,OUTPUT);
  pinMode(i2,OUTPUT);
  pinMode(i3,OUTPUT);
  pinMode(i4,OUTPUT);
  

// ir array
  pinMode(sensor1,INPUT);
  pinMode(sensor2,INPUT);
  pinMode(sensor3,INPUT);
  pinMode(sensor4,INPUT);
  pinMode(sensor5,INPUT);
  
  
  Serial.begin(9600);

  setpoint=0 ;
}


float error=0, previous_error = 0, P = 0, I = 0, D = 0;

void loop() {
pos=find_position();
calculate_PID();
turnspeed();
forward();
delay(50);

}


int find_position(){ 
 
  ir[1] = digitalRead(sensor1);
  ir[1]=(!ir[1]);
  //Serial.println(ir[1]);
  ir[2] = digitalRead(sensor2);
  ir[2]=(!ir[2]);
  //Serial.println(ir[2]);
  ir[3] = digitalRead(sensor3);
  ir[3]=(!ir[3]);
  //Serial.println(ir[3]);
  ir[4] = digitalRead(sensor4);
  ir[4]=(!ir[4]);
  //Serial.println(ir[4]);
  ir[5] = digitalRead(sensor5);
  ir[5]=(!ir[5]);
  //Serial.println(ir[5]);
  
  
  for (int i = 1; i < 6; i++){
  sum = sum + (ir[i] * irv[i]);   //Calculating the weighted mean
 
  }
  pos = sum/5 ;
  Serial.print("position is  "); 
  Serial.println(pos);
  sum=0;

  /*//////////////////////////////RIGHT TURN
  if(ir[1]==0)
  {
   //delay(500);
   while(1){
  analogWrite(i1,0);
  analogWrite(i2,100);
  analogWrite(i3,255);
  analogWrite(i4,0);

ir[6] = digitalRead(sensor6);
ir[6]=(!ir[6]);
if(ir[6]==0){
  break;
  }
    }
  }
/////////////////////////////////////LEFT TURN
  if(ir[7]==0)
  { 
    //delay(5000);
   while(1){
  analogWrite(i1,255);
  analogWrite(i2,0);
  analogWrite(i3,0);
  analogWrite(i4,100);

ir[2] = digitalRead(sensor2);
 ir[2]=(!ir[2]);
if(ir[2]==0){
  break;
  }
 }
}*/
  return pos;
}


void calculate_PID(){ 
  error=setpoint-pos;
  P= error;
  if(I>240)
  {
    I=240;
  }
  if(I<-240)
  {
    I=-240;
  }
  I = I + error;
  D =  previous_error - error;
  previous_error = error;
  PID_value = int(P*Kp + I*Ki - D*Kd);
 /* Serial.print("error value is   ");
  Serial.println(error);
  Serial.print("pid value is   ");
  Serial.println(PID_value);
  Serial.print("I value is   ");
  Serial.println(I);*/
  
}


void turnspeed(){  
  //Restricting the error value between +255.
rs = s;
ls = s;
 if(PID_value<0)
 {
  ls = s+PID_value;
  }
  if(PID_value>0)
  {
    rs = s-PID_value;
    }
  
  if (rs < 0){   
    rs = 0;
    }
  if (rs> 255){
    rs = 255;
    }

    if (ls < 0){
    ls = 0;
    }
  if (ls> 255){
    ls = 255;
    }
  
}


void forward(){
  
  /*Serial.print("rs value is  ");
  Serial.println(rs);
  Serial.print("ls value is  ");
  Serial.println(ls);
  if(rs>ls)
  {
    Serial.println("Left Turn");
  }
  else if(rs<ls)
  {
    Serial.println("Right Turn");
  }
  else if(rs=ls)
  {
    Serial.println("Forward");
  }
  
  Serial.println("************");*/
  
  analogWrite(i1,rs);
  analogWrite(i2,0);
  analogWrite(i3,ls);
  analogWrite(i4,0);
 }
