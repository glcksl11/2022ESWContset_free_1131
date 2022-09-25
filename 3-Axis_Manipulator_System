#include <SoftwareSerial.h>
#include <Servo.h>

#define BLDC1 5
#define BLDC2 4

Servo motor1, motor2;

unsigned int target_throttle;
//
int count_ist =0;
char flag_insert_y = 0;
char fiy = 0;
const int insert_dx[4] = {1,0,-1,0};
const int insert_dy[4] = {0,1,0,-1};
int insert_index = 0;
int k = 1;
float pre_target_deg =0;
float pre_target_deg2 =0;
float pre_target_deg3 =0;
//
const int sensorln = A0;
int readValue = 5000;
int pre_readValue=5000;
//path planning
char flag_dir=0;
char flag_insert_x=0;
char flag_insert_z=0;
int flag_do=0;
char flag_do2=0;
float x_tornado=0;
float z_tornado=0;
int loop_count=0;
float t_tor1 = 0;
float t_tor2 = 0;
int tornado_scale=0;

// MCU TIMER //
unsigned long stime, stime_prev, stime_now;

unsigned long loop_time, loop_time_prev, dt_now, dt_prev;
float dt, dt_target;

uint16_t MCU_TIMER_start;
#define MCU_CONTROL_RATE 20
void timer_loop(unsigned int mcu_ms)
{
  while((uint16_t)millis()-MCU_TIMER_start < mcu_ms);
  MCU_TIMER_start = (uint16_t)millis();
  
}

char buf[10]={0};
int i = 0;
String temp2_1;
String temp2_2;

String temp3_1;

//motor control pin
const int PWMpin = 6;   //
const int Dir_1 = 8; //
const int Dir_2 = 9;

const int PWMpin_2 = 11;   //
const int Dir_1_2 = 13; //
const int Dir_2_2 = 12;

const int PWMpin_3 = 7;   //15
const int Dir_1_3 = 26; //17
const int Dir_2_3 = 24;//16


// encoder pin
const int en_pinA = 2;  //z-axis
const int en_pinB = 3;
                     
const int en_pinA_2 = 21; //x-axis
const int en_pinB_2 = 20;

const int en_pinA_3 = 18; //y-axis
const int en_pinB_3 = 19;

int64_t en_pos = 0;
int64_t en_pos_2 = 0;
int64_t en_pos_3 = 0;

const float pulse_ratio = 360./90./52.;
const float pulse_ratio2 = 360./10./12.;
const float pulse_ratio3 = 360./302./14.;

// PID control
float Kp = 5.0;   // defalut 70
float Ki = 0.01;   // defalut 0.03
float Kd = 0.5;  // defalut 5.0

float P_control, I_control, D_control;
float control;
float error_previous;
float error;

float target_deg = 0;
float motor_deg;

String temp;
int spin_temp = 0, spin_temp2 = 0, spin_temp3 = 0;
int z_temp = 0, z_target = 0;

////////////////////////////////////////////////////////

#define tact_pin  30 //4
#define tact_pin2  32 //14

// PID control
float Kp2 = 0.4;
float Ki2 = 0.01;
float Kd2 = 0.1;

float P_control2, I_control2, D_control2;
float control2;
float error_previous2;
float error2;

float target_deg2 = 0;
float motor_deg2;

String temp2;

////////////////////////////////////////////////////////
// PID control
float Kp3 = 5.0;
float Ki3 = 0.01;
float Kd3 = 0.3;

float P_control3, I_control3, D_control3;
float control3;
float error_previous3;
float error3;

float target_deg3 = 0;
float motor_deg3;

////////////////////////////////////////////////////////

char flag = 0, origin_flag = 0, tact_flag = 0;
char origin2_flag = 0, tact2_flag = 0; 

void doEncoderA(void)
{
  en_pos += (digitalRead(en_pinA) == digitalRead(en_pinB))?1:-1;
}

void doEncoderB(void)
{
  en_pos += (digitalRead(en_pinA) == digitalRead(en_pinB))?-1:1;
}

///////////////////////////////////////////////////////////////////

void doEncoderA2(void)
{
  en_pos_2 += (digitalRead(en_pinA_2) == digitalRead(en_pinB_2))?1:-1;
}

void doEncoderB2(void)
{
  en_pos_2 += (digitalRead(en_pinA_2) == digitalRead(en_pinB_2))?-1:1;
}

///////////////////////////////////////////////////////////////////

void doEncoderA3(void)
{
  en_pos_3 += (digitalRead(en_pinA_3) == digitalRead(en_pinB_3))?1:-1;
}

void doEncoderB3(void)
{
  en_pos_3 += (digitalRead(en_pinA_3) == digitalRead(en_pinB_3))?-1:1;
}


void doMotor(bool dir, int vel)
{
  digitalWrite(Dir_1, dir);
  digitalWrite(Dir_2, !(dir));
  //analogWrite(PWMpin, dir?(255-vel):vel);
  analogWrite(PWMpin, min(vel,255));
}

void doMotor2(bool dir2, int vel2)
{
  digitalWrite(Dir_1_2, dir2);
  digitalWrite(Dir_2_2, !(dir2));
  //analogWrite(PWMpin, dir?(255-vel):vel);
  analogWrite(PWMpin_2, min(vel2,255));
}

void doMotor3(bool dir3, int vel3)
{
  digitalWrite(Dir_1_3, dir3);
  digitalWrite(Dir_2_3, !(dir3));
  //analogWrite(PWMpin, dir?(255-vel):vel);
  analogWrite(PWMpin_3, min(vel3,255));
}


void setup() {
  // put your setup code here, to run once:
  pinMode(en_pinA, INPUT_PULLUP);
  attachInterrupt(0, doEncoderA, CHANGE);

  pinMode(en_pinB, INPUT_PULLUP);
  attachInterrupt(1, doEncoderB, CHANGE);
  /////////////////////////////////////////////
  pinMode(en_pinA_2, INPUT_PULLUP);
  attachInterrupt(3, doEncoderA2, CHANGE);

  pinMode(en_pinB_2, INPUT_PULLUP);
  attachInterrupt(2, doEncoderB2, CHANGE);
  /////////////////////////////////////////////
  pinMode(en_pinA_3, INPUT_PULLUP);
  attachInterrupt(5, doEncoderA3, CHANGE);

  pinMode(en_pinB_3, INPUT_PULLUP);
  attachInterrupt(4, doEncoderB3, CHANGE);


  pinMode(Dir_1, OUTPUT);
  pinMode(Dir_2, OUTPUT);

  pinMode(Dir_1_2, OUTPUT);
  pinMode(Dir_2_2, OUTPUT);

  pinMode(Dir_1_3, OUTPUT);
  pinMode(Dir_2_3, OUTPUT);   


  pinMode(tact_pin, INPUT_PULLUP);
  pinMode(tact_pin2, INPUT_PULLUP);
  ///////////////////////////////////////

  Serial3.begin(115200);
  Serial2.begin(115200);
  //Serial.begin(115200);

  ///////////////////////////////////////

  motor_init();

  ///////////////////////////////////////
  
  while(!origin_flag)
  {
    tact_flag = digitalRead(tact_pin);

    if(!tact_flag)
    {
      digitalWrite(Dir_1, 1);
      digitalWrite(Dir_2, 0);
      analogWrite(PWMpin, 120); 
    }

    else
    {
      digitalWrite(Dir_1, 1);
      digitalWrite(Dir_2, 0);
      analogWrite(PWMpin, 0);       
      en_pos = 0;
      origin_flag = 1;      
    }

  }

  //////////////////encoder_origin///

  while(!origin2_flag)
  {
    tact2_flag = digitalRead(tact_pin2);

    if(!tact2_flag)
    {
      digitalWrite(Dir_1_2, 1);
      digitalWrite(Dir_2_2, 0);
      analogWrite(PWMpin_2, 180);      
    }

    else
    {
      digitalWrite(Dir_1_2, 1);
      digitalWrite(Dir_2_2, 0);
      analogWrite(PWMpin_2, 0);       
      en_pos_2 = 0;
      origin2_flag = 1;
    }
  }
  delay(100);
  en_pos = 0;
  en_pos_2 = 0;
  en_pos_3 = 0;
}

void loop() {
  // put your main code here, to run repeatedly:
  
  /*if(Serial.available())
  {
    temp = Serial.readStringUntil('\r');
    spin_temp = temp.toInt();
    target_deg = float(360 * spin_temp); 
  }*/
  
  timer_loop(MCU_CONTROL_RATE);

  loop_time = millis();
  dt = float(loop_time - loop_time_prev)/1000;

  
  motor_deg = float(en_pos)*pulse_ratio;
  error = target_deg - motor_deg;

  P_control = (Kp * error);
  I_control += (Ki * error * dt);
  D_control = Kd*(error - error_previous)/(dt);

  control = P_control + I_control + D_control;

  /////////////////////////////////////////////////

  error_previous = error;
  
  doMotor((control >= 0)?0:1, min(abs(control), 255));
/////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////

  motor_deg2 = float(en_pos_2)*pulse_ratio2;
  error2 = target_deg2 - motor_deg2;

  P_control2 = (Kp2 * error2);
  I_control2 += (Ki2 * error2 * dt);
  D_control2 = Kd2*(error2 - error_previous2)/(dt);

  control2 = P_control2 + I_control2 + D_control2;

  /////////////////////////////////////////////////

  error_previous2 = error2;
  
  doMotor2((control2 >= 0)?0:1, min(abs(control2), 255));

/////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////

  motor_deg3 = float(en_pos_3)*pulse_ratio3;
  error3 = target_deg3 - motor_deg3;

  P_control3 = (Kp3 * error3);
  I_control3 += (Ki3 * error3 * dt);
  D_control3 = Kd3*(error3 - error_previous3)/(dt);
  if(pre_target_deg3 != target_deg3) I_control3 = 0;
  control3 = P_control3 + I_control3 + D_control3;

  /////////////////////////////////////////////////

  error_previous3 = error3;
  
  doMotor3((control3 >= 0)?0:1, min(abs(control3), 255));
  

  /////////////////////////////////////////////////////////// 
//  if(flag_insert_y == 1 && abs(error)<=8&&abs(error2)<=15){
//    target_deg3 = 300;
//    flag_insert_y = 0;
//  }
//  readValue = 10*analogRead(sensorln);
//  readValue = 0.1*readValue + 0.9*pre_readValue;
//  pre_readValue = readValue;
//  //target 좌표 도달 후에 y축 삽입 진행
//  if(flag_insert_y == 1 && abs(error)<=8&&abs(error2)<=15){
//    target_deg3 = 300;
//    }
//  if((readValue < 4980 && target_deg3 == 300) ){
//    ++count_ist;
//    if(count_ist > 50){  
//      target_deg3 = -100;
//      flag_insert_y = 0;
//      fiy = 1;
//      count_ist = 0;
//    }
//  }
//  
//
//  if(fiy==1 && abs(error3)<=100 && target_deg3 == -100){
//      inserting_hs();
//      //target_deg += 450;
//      //target_deg2 += 450;
//       flag_insert_y = 1;
//       fiy = 0;  
//    }
//
//   if( pre_target_deg3==target_deg3 && flag_insert_y == 1 && readValue > 5000 && target_deg3 == 300 && abs(error3)<10 ){
//       flag_insert_y = 0;
//       fiy = 0;
//       target_deg = 0;
//    }
//    
  //
  
//  if(flag_do){
//      if(abs(error)<=8&&abs(error2)<=15){ 
//          inserting(10);
//      }
//
//  }
  ///////////////////////////////////////////////////////////
  //Serial.print("target_x : ");
  //Serial.print(spin_temp3);
  //Serial.print("mm  ||  ");
  //Serial.print("Position_x : ");
  //Serial.print("   ");
  //Serial.println(((en_pos_3)*pulse_ratio3)/45);
  //Serial.println(((en_pos_3)*pulse_ratio3));
  //Serial.println("mm");
  //Serial.println((int)en_pos);
  //Serial.print("sensor : ");
  //Serial.println(readValue);
//  Serial.print("  target : ");
//  Serial.print(spin_temp);
//  Serial.print(", ");
//  Serial.println(spin_temp2);
 // Serial3.print("  encoder3 : ");
  //Serial3.println((int)en_pos_3);
  //Serial.print("  error3 : ");
  //Serial.println(error3);
  //Serial3.print((int)digitalRead(tact_pin));
  //Serial3.println((int)digitalRead(tact_pin2));

  /////////////////////////////////////////////////
  
//  Serial.print(target_deg);
//  Serial.print("  ");
//  Serial.print(((en_pos)*pulse_ratio));
//
//  /////////////////////////////////////////////////
//
//  
//  Serial.print(" || ");
  //Serial.print(target_deg2);
  //Serial.print(" ");
  //Serial.println(((float)(en_pos_2)*pulse_ratio2));  
//  
//  Serial.print(" || ");
//  Serial.print(target_deg3);
//  Serial.print(" ");
//  Serial.println((float)(en_pos_3));
  
  if(target_throttle > 2000)    target_throttle = 2000;
  else if(target_throttle <= 1000)  target_throttle = 1000;
    
  motor1.writeMicroseconds(target_throttle);
  motor2.writeMicroseconds(target_throttle);

  pre_target_deg = target_deg;
  pre_target_deg2 = target_deg2;
  pre_target_deg3 = target_deg3;
  loop_time_prev = loop_time;
}

void motor_init(){
  //motor1.attach(BLDC1, 1000, 2000);
  //motor2.attach(BLDC2, 1000, 2000);
  //motor3.attach(BLDC3, 1000, 2000);
  //motor4.attach(BLDC4, 1000, 2000);
  motor1.attach(BLDC1);
  motor2.attach(BLDC2);
      
  delay(100);
  
  motor1.writeMicroseconds(1000);
  motor2.writeMicroseconds(1000);
  delay(500);
 }


void motor_off(){
  motor1.writeMicroseconds(1000);
  motor2.writeMicroseconds(1000);
}


void serialEvent3()
{
  buf[i] = Serial3.read();
  //Serial.println(buf);
  i++;
  if(buf[i-1] == ')')
  {
    temp = String(buf);
    unsigned int index_2[3];
    index_2[0]=temp.indexOf("(");
    index_2[1]=temp.indexOf(",");
    index_2[2]=temp.indexOf(")");

    temp2_1=temp.substring(index_2[0]+1,index_2[1]);
    temp2_2=temp.substring(index_2[1]+1,index_2[2]);


    spin_temp = temp2_1.toInt();
    target_deg2 = float(45 * spin_temp);

    spin_temp2 = temp2_2.toInt();
    target_deg = float(45 * spin_temp2);    

    
    for(int j=0;j<=i;j++){
      buf[j]=0;
    }
    i=0;
    flag_insert_y = 1;
  }

  else if(buf[i-1] == ']')
  {
    temp = String(buf);
    signed int index_3[2];
    index_3[0]=temp.indexOf("[");
    index_3[1]=temp.indexOf("]");

    temp3_1=temp.substring(index_3[0]+1,index_3[1]);
    //temp2_2=temp.substring(index_2[1]+1,index_2[2]);


    spin_temp3 = temp3_1.toInt();
    target_deg3 = spin_temp3;//float(45 * spin_temp3); 

    
    for(int q=0;q<=i;q++){
      buf[q]=0;
    }
    i=0;
  }
//  else if(buf[i-1] == 'a'){
//    flag_do = 1;
//  }
  else if(buf[i-1] == '>')
  {
    if((buf[2]=='D')&&(buf[3]=='O')){
        flag_do=1;
     }
    for(int s=0;s<=i;s++){
      buf[s]=0;
    }
    i=0;
  }
}

void inserting(int scale){
//  if(x_tornado<scale*2){
//    if(!flag_dir){
//      target_deg+=x_tornado*cos(t_tor1);
//      target_deg2+=x_tornado*sin(t_tor1);
//      t_tor1+=0.2;
//      x_tornado+=0.02;
//      }
//    if(flag_dir) flag_dir=0;
//    else flag_dir=1;
//    }
//    else {flag_do=0; x_tornado=0; z_tornado=0; t_tor1=0; t_tor2=0;
//    }
    if(x_tornado<scale*2||z_tornado<scale*2){
    if(!flag_dir){
      if(!flag_insert_x) target_deg+=x_tornado;
      else             target_deg-=x_tornado;  
      x_tornado+=20;
      if(flag_insert_x) flag_insert_x=0;
      else flag_insert_x=1;
      }
    else {
      if(!flag_insert_z) target_deg2+=z_tornado;
      else             target_deg2-=z_tornado;
      z_tornado+=20;
      if(flag_insert_z) flag_insert_z=0;
      else flag_insert_z=1; 
      }
    if(flag_dir) flag_dir=0;
    else flag_dir=1;
    }
    else {flag_do=0; x_tornado=0; z_tornado=0;
    }
}
void inserting_hs(){
  target_deg2 += float(45*k*insert_dx[insert_index]);
  target_deg += float(45*k*insert_dy[insert_index]);
  if(target_deg2 < 0 || target_deg2 > 245.*45. || target_deg < 0 || target_deg > 4500){
    target_deg = pre_target_deg;
    target_deg2 = pre_target_deg2;
  }
  ++insert_index;
  insert_index = insert_index %4;
  if(insert_index%2==0)++k;
}

void serialEvent2()
{
  unsigned int temp = Serial2.parseInt();
  if(temp <= 2000 && temp >= 1000){
      target_throttle = temp;
    }
  else{
    
  } 
}
