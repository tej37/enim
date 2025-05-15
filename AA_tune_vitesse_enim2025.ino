#include <Arduino.h>
#include <Ticker.h>
#include <PID_v1.h>
/*
float spacing_encoder = 133.0;  //132
#define T 10
#define rampmax 450.0
#define spacing_wheel 133
*/
//#define MaxPami
#define T 10
#define TimeOut 100;
//float kp1r = 5.0;
//float kd1r = 0.0005;


//#define PIN_INPUT 0
//#define PIN_OUTPUT 3
long int j=0;
//Define Variables we'll be connecting to
double Setpoint, Input, Output;

//Specify the links and initial tuning parameters
double Kp=5, Ki=30.0, Kd=0;
//double Kp=4.5, Ki=32.0, Kd=0;

const int demi_periode=1000;
int p=0;

/*
float kp1r = 5.0;
float kd1r = 0.0005;
float kp1m = 3.0;
float kd1m = 0.0005;
float ki1 = 0.0135;
float kp2 = 1.3;
float right_radius = 45.0;
float left_radius = 45.0;
float kbm = 10;
float kbr = 5;
*/
float right_radius = 45.0;
float left_radius = 45.0;
//#define MSG_BUFFER_SIZE (30)
/*
unsigned long timer = 0;
float timeStep = 0.01;
float mean, variance;
*/
int prevcountR = 0;
int prevcountL = 0;
int dalpha_counter = 0;
int d_right;
int d_left;
float totalR;
float totalL;
float dL, dC, dR;
float X = 0, Y = 0, alpha = 0, alpha_deg = 0;
float speedL, speedR;
double ZoneX = 0, ZoneY = 0;
int tickmmR, tickmmL;
int prevCountL, prevCountR;
float right_resolution = 205;
float right_precision = 1;

float left_resolution =205;
float left_precision = 1;

volatile float d_right_counter;
volatile float d_left_counter;
volatile float total_centre;
volatile float right_encoder_speed;
volatile float left_encoder_speed;
volatile float alpha_speed;
volatile float d_alpha_counter;
volatile float right_speed;
volatile float left_speed;
volatile float GyroSpeed = 0;
volatile float GyroAngle = 0;


float rawalpha;
float prev_pos_error;
float prev_vel_error;
float integ_error;
float d_error;
float t;
float first;
float now;
float lastTime;
unsigned long Gtime;
unsigned long p_lastTime;
double ramp;
float pos_error;
float consigne_vel;
float vel_error;
float consigne_right;
float VR;
float VL;
float consigne_left;
float vitesse_right;
float left_distance;
float bal_error;
int Timeout=50;
int i = 0;
//ESP32
/*
const int encoderLPin1 = 35;
const int encoderLPin2 = 34;
const int encoderRPin1 = 23;
const int encoderRPin2 = 18;
*/
const int encoderLPin1 = 23;
const int encoderLPin2 = 18;
const int encoderRPin1 = 34;
const int encoderRPin2 = 35;
/*
const int motorleft1 =16; //in2A  
const int motorleft2 = 17; //in1A 
const int motorright1 = 32;  //in2B
const int motorright2 =  33 ;  //in1B
const int pwm_R=26 ;           // pwmB 5
const int pwm_L=25;          // pwmA
*/
const int motorleft1 = 17;  //in2B
const int motorleft2 =  16;  //in1B
const int motorright1 =     33; //in2A
const int motorright2 =  32; //in1A    
const int pwm_R= 26;          // pwmA
const int pwm_L=25 ;           // pwmB 5
//ESP86
/*

const int motorleft1 = 32;
const int motorleft2 = 33;
const int motorright1 = 25;
const int motorright2 = 26;


//*/
unsigned long gtime;
volatile int counterL = 0;
volatile int counterR = 0;


long duration;
float distanceCm;
float distanceInch;
unsigned long Tt = 0, Ttt = 0;
uint8_t TimeInterval = 10000;

/*
void callback(char* topic, byte* payload, unsigned int length) {
  if (!strcmp(topic, "systemCommand")) {
    double distance = 0;
    double rotation = 0;
    strlcpy(SendMsg, (char*)payload, length + 1);
    sscanf(SendMsg, "%lf,%lf", &distance, &rotation);
    move_distance(distance);
    if (rotation != 0) {
      //calibrate(20);
      rotate(rotation);
      //orientate(rotation);
    }
    //Robot_Locate(distance, rotation);
  } 
  else if (!strcmp(topic, "setCoeffs")) {
    double newKpp=0,newKdp=0,newKpv=0,newKiv=0,newKb=0;
    strlcpy(SendMsg, (char*)payload, length + 1);
    sscanf(SendMsg, "%lf,%lf,%lf,%lf,%lf", &newKpp,&newKdp,&newKpv,&newKiv,&newKb); 
    if(newKpp!=-1)
      kp1m=newKpp;
    if(newKdp!=-1)
      kd1m=newKdp;
    if(newKpv!=-1)
      kp2=newKpv;
    if(newKiv!=-1)
      ki1=newKiv;
    if(newKb!=-1)
      kbm=newKb;
    delay(100);
    sprintf(SendMsg, "%lf,%lf,%lf,%lf,%lf", kp1m,kd1m,kp2,ki1,kbm);
    client.publish("tuneVerif",SendMsg);
  }
  else if (!strcmp(topic, "robotLocate")) {
    double x = 0;
    double y = 0;
    strlcpy(SendMsg, (char*)payload, length + 1);
    sscanf(SendMsg, "%lf,%lf", &x, &y);
    Robot_Locate(x, y);
  } 

}
*/
           

/*void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    // Create a random client ID
    String clientId = "ESP8266Client-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str())) {
      client.subscribe("Pami
Orders2");
      client.subscribe("TogglePosition2");
      client.subscribe("Pami
Position2");
      client.subscribe("Pami
Timeout2");
      
    } else {
      // Wait 5 seconds before retrying
      delay(1000);
    }
  }
  if (GyroTnek) {
    client.publish("Pami
Feedback", "Gyro tnek");
  }
  client.publish("Pami
Feedback", "Hani Connectit");
}*/
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd,true, DIRECT);
void IRAM_ATTR handleEncoderA() {
  int a = digitalRead(encoderLPin1);
  int b = digitalRead(encoderLPin2);

  if (a == b) {
    counterL++;
  } else {
    counterL--;
  }
}
void IRAM_ATTR handleEncoderB() {
  int a = digitalRead(encoderRPin1);
  int b = digitalRead(encoderRPin2);

  if (a == b) {
    counterR++;
  } else {
    counterR--;
  }
}

float ticks_to_distance(int x, float r, int resolution, int precision) {
  return (x * 2 * PI * r / (resolution * precision)); //return distance that map the number of ticks 
}

void update_position(void) {
  d_right = counterR - prevcountR;
  prevcountR = counterR;
  dR = ticks_to_distance(d_right, right_radius, right_resolution, right_precision);  // d_right count - prevvount en 10 ms
  totalR += dR;
  d_right_counter += dR;
  d_left = counterL - prevcountL;
  prevcountL = counterL;     
  dL = ticks_to_distance(d_left, left_radius, left_resolution, left_precision);
  totalL += dL;
  d_left_counter += dL;
  dC = (dR + dL) / 2;
  total_centre += dC;
}
void speed_calcul(void) {
  right_speed = d_right_counter / T * 1000;
  d_right_counter = 0;
  left_speed = d_left_counter / T * 1000;
  d_left_counter = 0;
  alpha_speed = d_alpha_counter / T * 1000;
  d_alpha_counter = 0;
  /* right_speed = (right_encoder_speed + left_encoder_speed) / 2 + alpha_speed * spacing_wheel / 2;
  left_speed = (right_encoder_speed + left_encoder_speed) / 2 - alpha_speed * spacing_wheel / 2;*/
}

void odometry() {
  update_position();
  if (i == 2) {   //because T=2*0.005 of timer odometry
    speed_calcul();
    i = 0;
  }
  i++;
}
void set_motors(int cmdR, int cmdL) {
  if (cmdR > 0) {
    digitalWrite(motorright1, HIGH);
    digitalWrite(motorright2, LOW);
    analogWrite(pwm_R,abs(cmdR));
  } else {
    digitalWrite(motorright2,HIGH);
    digitalWrite(motorright1, LOW);
    analogWrite(pwm_R,abs(cmdR));

  }
  if (cmdL > 0) {
    digitalWrite(motorleft1, HIGH);
    digitalWrite(motorleft2, LOW);
    analogWrite(pwm_L,abs(cmdL));

  } else {
    digitalWrite(motorleft2,HIGH);
    digitalWrite(motorleft1, LOW);
    analogWrite(pwm_L,abs(cmdL));
  }
}

void stop_motors() {
  digitalWrite(motorleft1, LOW);
  digitalWrite(motorleft2, LOW);
  digitalWrite(motorright1, LOW);
  digitalWrite(motorright2, LOW);

}

Ticker Timer1;
Ticker Timer2;
void setup() {
  Serial.begin(500000);
  Setpoint = 500;

  //turn the PID on
  myPID.SetMode(AUTOMATIC);
  
  //myPID.SetMode(MANUAL);
  //zmyPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-700, 700); // Limites de la commande PWM
  myPID.SetSampleTime(T);
  pinMode(encoderLPin1, INPUT_PULLUP);
  pinMode(encoderRPin1, INPUT_PULLUP);
  pinMode(encoderLPin2, INPUT_PULLUP);
  pinMode(encoderRPin2, INPUT_PULLUP);

  pinMode(motorleft1, OUTPUT);
  pinMode(motorleft2, OUTPUT);
  pinMode(motorright1, OUTPUT);
  pinMode(motorright2, OUTPUT);
  pinMode(pwm_R, OUTPUT);
  pinMode(pwm_L, OUTPUT);

 

  attachInterrupt(digitalPinToInterrupt(encoderLPin1), handleEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderRPin1), handleEncoderB, CHANGE);
  Timer1.attach(0.005, odometry);

  Tt=millis();
  p_lastTime=millis();

  
  
/*
  if (!gyro.init()) {
    Serial.println("Failed to autodetect gyro type!");
    GyroTnek = true;
  }
  */
  /*if (!GyroTnek) {
    delay(1000);
    gyro.enableDefault();
    calibrate(200);
  }*/   
  //***********************
  //gtime = millis();
  //Timer2.attach(0.01, UpdateAngle);
  




}




void loop()
{ 
  /*
Serial.print("Tt:");
Serial.println(Tt);
Serial.print("Ttt:");
Serial.println(Ttt);

  Ttt = millis();
  if( (Ttt -Tt)< TimeInterval )
  {
Input =right_speed;
myPID.Compute();
set_motors(Output, Output);
  //analogWrite(PIN_OUTPUT, Output);
  //set_motors((int)constrain(Output, -255, 255), (int)constrain(Output, -255, 255));
j++;
if (j==5){

Serial.print("PI_output:");
Serial.print(Output);
Serial.print(",");
Serial.print("target_velocity==PD_output:");
Serial.print(Setpoint);
Serial.print(",");
Serial.print("measured_velocity_R:");
Serial.print(right_speed);
Serial.println(",");

  j=0;
}

    //Tt= millis();
  }
  else stop_motors();
*/

//for(int p=0;p<500;p++){

//}
//delay(3000);
/*
Input =right_speed;
myPID.Compute();
set_motors(Output, Output);
//j++;
//if (j==5){

Serial.print("PI_output:");
Serial.print(Output);
Serial.print(",");
*/
//Serial.print("target_velocity==PD_output:");
//Serial.print(Setpoint);
//Serial.print(",");
/*
set_motors(0,255);
Serial.print("measured_velocity_R:");
Serial.print(left_speed);
Serial.println(",");
*/

//  j=0;
//}

unsigned long p_now = millis();
   int p_timeChange = (p_now - p_lastTime);
   //if(timeChange>=SampleTime)
   if(p_timeChange>=demi_periode)
   {   
    Setpoint = - Setpoint;
    p_lastTime = p_now;   
 }
Input =right_speed ;
myPID.Compute();
set_motors(((Output*200)/2300),0);
j++;
if(j==2500){
  Serial.print("Consigne:");
  Serial.print(Setpoint);
  Serial.print(",");
  Serial.print("pi_out:");
  Serial.print(Output);
  Serial.print(",");
  Serial.print("measured_velocity_R:");
  Serial.println(right_speed);
  j=0;
  }






}
