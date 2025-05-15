#include <Arduino.h>
#include <Ticker.h>
#include <PID_v1.h>
float spacing_encoder = 95.0;  //132
#define T 10
#define rampmax 500
#define rampmax_a 350.0

#define spacing_wheel 95.0
//#define MaxPami
#define TimeOut 100;
//float kp1r = 5.0;
//float kd1r = 0.0005;



long int j=0;
//Define Variables we'll be connecting to
double Setpoint, Input, Output;
double Setpointd, Inputd, Outputd;
double Setpointl, Inputl, Outputl;
double SetpointA , InputA ,OutputA;
double Kp=2.5, Ki=20.0, Kd=0;
double Kpl=2.5, Kil=20.0, Kdl=0;
double Kpd=2.5, Kid=0, Kdd=0;
double KpA=20, KiA=10.0, KdA=0;

//Specify the links and initial tuning parameters
/*
double Kp=5, Ki=10, Kd=0;
double Kpl=5, Kil=10, Kdl=0;
double Kpd=3, Kid=0, Kdd=0;
*/


double Setpoint_a, Input_a, Output_a;    //vitesse ymin PI
double Setpointd_a, Inputd_a, Outputd_a;  //angle   P
double Setpointl_a, Inputl_a, Outputl_a;   //vitesse ysar PI
double Kp_a=2.5, Ki_a=20.0, Kd_a=0;            //vitesse ymin PI
double Kpl_a=2.5, Kil_a=20.0, Kdl_a=0;         //vitesse ysar PI
double Kpd_a=2.5, Kid_a=0, Kdd_a=0;          //angle   P


/*
float kp1r = 5.0;
float kd1r = 0.0005;
float kp1m = 3.0;
float kd1m = 0.0005;
float ki1 = 0.0135;
float kp2 = 1.3;
*/
float right_radius = 45.0;
float left_radius = 45.0;
//float kbm =70.0;
//float kbr = 0.0;
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
float right_resolution = 700;
float right_precision = 2;

float left_resolution =700;
float left_precision = 2;

volatile float d_right_counter;
volatile float d_left_counter;
volatile float total_centre;
volatile float right_encoder_speed;
volatile float left_encoder_speed;
volatile float alpha_speed;
volatile float d_alpha_counter;
volatile float right_speed;
volatile float left_speed;



float rawalpha;
float prev_pos_error;
float prev_vel_error;
float integ_error;
float d_error;
float t;
float first;
float now;
float lastTime;
double ramp;
float pos_error;
float consigne_vel;
float consigne_vel_right;
float consigne_vel_left;
float consigne_vel_right_a;
float consigne_vel_left_a;

float vitesse_right;
float left_distance;
float bal_error;
int i = 0;
//ESP32
const int encoderLPin1 = 23;
const int encoderLPin2 = 18;
const int encoderRPin1 = 35;
const int encoderRPin2 = 34;



const int motorleft1 = 33 ;  //in2B
const int motorleft2 =  32;  //in1B
const int motorright1 =     16; //in2A
const int motorright2 =  17; //in1A    
const int pwm_R= 25;          // pwmA
const int pwm_L=26 ;           // pwmB 5
  
//ESP86
/*

const int motorleft1 = 32;
const int motorleft2 = 33;
const int motorright1 = 25;
const int motorright2 = 26;


//*/
volatile int counterL = 0;
volatile int counterR = 0;




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
PID myPID_L(&Inputl, &Outputl, &Setpointl, Kpl, Kil, Kdl,true, DIRECT);
PID myPID_distance(&Inputd, &Outputd, &Setpointd, Kpd, Kid, Kdd,true, DIRECT);
PID myPIDA(&InputA, &OutputA, &SetpointA, KpA, KiA, KdA,true, DIRECT);

PID myPID_a(&Input_a, &Output_a, &Setpoint_a, Kp_a, Ki_a, Kd_a,true, DIRECT);
PID myPID_L_a(&Inputl_a, &Outputl_a, &Setpointl_a, Kpl_a, Kil_a, Kdl_a,true, DIRECT);
PID myPID_distance_a(&Inputd_a, &Outputd_a, &Setpointd_a, Kpd_a, Kid_a, Kdd_a,true, DIRECT);


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

Ticker Timer1;
Ticker Timer2;
void setup() {
  //Serial.begin(500000);
  myPID_a.SetMode(AUTOMATIC);
  myPID_a.SetOutputLimits(-350, 350); // Limites de la commande PWM
  myPID_a.SetSampleTime(T);
  myPID_L_a.SetMode(AUTOMATIC);
  myPID_L_a.SetOutputLimits(-350, 350); // Limites de la commande PWM
  myPID_L_a.SetSampleTime(T);
  myPID_distance_a.SetMode(AUTOMATIC);
  myPID_distance_a.SetOutputLimits(-350,350); // Limites de la commande PWM
  myPID_distance_a.SetSampleTime(T);

  //turn the PID on
  //myPID.SetMode(MANUAL);
  myPID.SetMode(AUTOMATIC);

  myPID.SetOutputLimits(-500,500); // Limites de la commande PWM
  myPID.SetSampleTime(T);

  //myPID_L.SetMode(MANUAL);
  myPID_L.SetMode(AUTOMATIC);

  myPID_L.SetOutputLimits(-500,500); // Limites de la commande PWM
  myPID_L.SetSampleTime(T);


  //myPID_distance.SetMode(MANUAL);
  myPID_distance.SetMode(AUTOMATIC);

  myPID_distance.SetOutputLimits(-500,500); // Limites de la commande PWM
  myPID_distance.SetSampleTime(T);

  myPIDA.SetMode(AUTOMATIC);
  myPIDA.SetOutputLimits(-500,500); // Limites de la commande PWM
  myPIDA.SetSampleTime(T);


  pinMode(encoderLPin1, INPUT_PULLUP);
  pinMode(encoderRPin1, INPUT_PULLUP);
  pinMode(encoderLPin2, INPUT_PULLUP);
  pinMode(encoderRPin2, INPUT_PULLUP);

  pinMode(motorleft1, OUTPUT);
  pinMode(motorleft2, OUTPUT);
  pinMode(motorright1, OUTPUT);
  pinMode(motorright2, OUTPUT);

 

  attachInterrupt(digitalPinToInterrupt(encoderLPin1), handleEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderRPin1), handleEncoderB, CHANGE);
  Timer1.attach(0.005, odometry);

  
  
  
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
move_distance(800);
delay(2000);
rotate(30);
delay(2000);
move_distance(400);
delay(2000);
rotate(-30);
delay(2000);
move_distance(900);
delay(10000);
*/
//Robot_Locate(x,y);
Robot_Locate(700,0);
Robot_Locate(1100,-400);
Robot_Locate(1800,-400);
Robot_Locate(1900,-150);


while(true){
  stop_motors();
  }


/*
myPID.SetMode(MANUAL);
myPID_L.SetMode(MANUAL);
myPID_distance.SetMode(MANUAL);
*/
  //analogWrite(PIN_OUTPUT, Output);
  //set_motors((int)constrain(Output, -255, 255), (int)constrain(Output, -255, 255));



}
/*
unsigned long T1 = 0, T2 = 0;
uint8_t TimeInterval = 1000;
 

 
void loop()
{
  T2 = millis();
  if( (T2-T1) >= TimeInterval )
  {
    
    T1 = millis();
  }
}*/
unsigned long wala = 0;
/*
void loop(){
delay(1000);
  move_distance(3000);

  


  
}
 */
 /* Serial.println(GyroAngle);
  delay(25);*/



/*
void UpdateAngle() {
  if (!GyroTnek) {
    if (!IsCalibrating && enablegyro) {
      GyroSpeed = getspeed();
      GyroAngle += GyroSpeed * float(millis() - gtime) / 1000;
      gtime = millis();
      //GyroAngle += GyroSpeed * 0.015;
    }
  } else {
    //GyroAngle = alpha;
  }
}
*/
float ticks_to_distance(int x, float r, int resolution, int precision) {
  return (x * 2 * PI * r / (resolution * precision)); //return distance that map the number of ticks 
}
float rad_to_deg(double x) {
  return (x * 360 / (2 * PI));
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


  alpha += ((dR - dL) / spacing_encoder);
  X += dC * cos(alpha);
  Y += dC * sin(alpha);
  dalpha_counter += ((dR - dL) / spacing_encoder);
  while (alpha > PI) {
    alpha -= 2 * PI;                                  //????????????????????????????????????????????????????????
  }
  while (alpha < -PI) {
    alpha += 2 * PI;
  }
  alpha_deg = rad_to_deg(alpha);
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


double timee1;
double timee2;
int counter_ult;
float target_angle, goal_distance;
int sens;
/*
void orientate(float orientation) {
  target_angle = orientation - alpha_deg;
  if (target_angle > 180) target_angle -= 360;
  if (target_angle < -180) target_angle += 360;
  rotate(target_angle);
}*/
/*
void Robot_Locate(float goal_x, float goal_y) {
  sens = (asinf((goal_y - Y) / sqrtf((X - goal_x) * (X - goal_x) + (Y - goal_y) * (Y - goal_y))) > 0) ? 1 : -1;
  target_angle = sens * rad_to_deg(acosf((goal_x - X) / sqrtf((X - goal_x) * (X - goal_x) + (Y - goal_y) * (Y - goal_y))));
  orientate(target_angle);
  goal_distance = sqrtf((X - goal_x) * (X - goal_x) + (Y - goal_y) * (Y - goal_y));
  move_distance(goal_distance);
}
*/
float sangle;
int timecounter = 0;
void initi(void) {
  totalR = 0;
  totalL = 0;
  prev_pos_error = 0;
  prev_vel_error = 0;
  integ_error = 0;
  d_error = 0;
  t = 0;
  vitesse_right = 0;
  left_distance = 0;
  d_right_counter = 0;
  d_left_counter = 0;
  d_alpha_counter = 0;
}
double constraint(double val, double min, double max) {
  if (val > max) {
    return max;
  } else if (val<100.0 & val> 0) {
    return 100.0;
  } else if (val > -100.0 & val < 0) {
    return -100.0;
  } else if (val < min) {
    return min;
  } else {
    return val;
  }
}
double constraint_rot(double val, double min, double max) {
  if (val > max) {
    return max;
  } else if (val<60.0 & val> 0) {
    return 60.0;
  } else if (val > -60.0 & val < 0) {
    return -60.0;
  } else if (val < min) {
    return min;
  } else {
    return val;
  }
} 

double calculateRamp(double t,float rampe) {
  double ramp = rampe * t;
  ramp = constraint(ramp, -rampmax, rampmax);
  return ramp;
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
void orientate(float orientation) {
  target_angle = orientation - alpha_deg;
  if (target_angle > 180) target_angle -= 360;
  if (target_angle < -180) target_angle += 360;
  /*
  Serial.print("target_angle= ");
  Serial.println(target_angle);
  */
  rotate(target_angle);
}
void Robot_Locate(float goal_x, float goal_y) {
  /*
  Serial.print("alpha= ");
  Serial.println(alpha);
  Serial.print("X= ");
  Serial.println(X);
  Serial.print("Y= ");
  Serial.println(Y);
  */
  sens = (asinf((goal_y - Y) / sqrtf((X - goal_x) * (X - goal_x) + (Y - goal_y) * (Y - goal_y))) > 0) ? 1 : -1;
  target_angle = sens * rad_to_deg(acosf((goal_x - X) / sqrtf((X - goal_x) * (X - goal_x) + (Y - goal_y) * (Y - goal_y))));


  orientate(target_angle);
  goal_distance = sqrtf((X - goal_x) * (X - goal_x) + (Y - goal_y) * (Y - goal_y));
  /*
  Serial.print("goal_distance= ");
  Serial.println(goal_distance);
  */
  move_distance(goal_distance);
}

void rotate(double goal) {
  if (goal == 0) { return; }
  initi();
  //timecounter = 0;
  long int k=0;
  //enablegyro = true;
  int signe = (int)(goal / fabs(goal));
  float now = millis();
  //sangle = GyroAngle;
  double inter;
  double target;
  //double salpha = GyroAngle;
  double odemetry_start_angle = alpha_deg;
  target = goal * PI * spacing_encoder / 360;   //regle 3  2*pi*r--->360
  first = now;
  while (((fabs(target - totalR) > 2) || (fabs(target + totalL) > 2)) ) {  //(fabs(target-totalR)>2) || (fabs(target+totalL)>2)
                                                                                                                     // while (fabs((GyroAngle-sangle)-goal)>2){
    lastTime = millis();                                                                                             /*
        GyroSpeed = getspeed();
        GyroAngle += GyroSpeed*float(millis()-Gtime)/1000;
		    Gtime=millis();*/
    ramp = signe * calculateRamp(now - first,0.3);
   // pos_error = target - (GyroAngle - salpha) / 180 * PI * spacing_encoder / 2;  //(alpha-salpha)*spacing_encoder/2;
    /*
     pos_error=target -(totalR-totalL)/2;
    d_error = (pos_error - prev_pos_error);
    prev_pos_error = pos_error;
    consigne_vel = kp1r * pos_error + kd1r * d_error;
    */
    Setpointd_a = target;
    Inputd_a =(totalR-totalL)/2 ;
    myPID_distance_a.Compute();
    consigne_vel=Outputd_a;

    consigne_vel = constraint_rot(consigne_vel, -rampmax, rampmax);
    //Serial.println(consigne_vel);
    if (signe == 1)
      consigne_vel = fmin(ramp, consigne_vel);
    else
      consigne_vel = fmax(ramp, consigne_vel);
/*
    vel_error = consigne_vel - right_speed;
    integ_error += vel_error * T;
    consigne_right = kp2 * vel_error + ki1 * integ_error;
    //Report();
    vel_error = (-consigne_vel) - left_speed;
    integ_error += vel_error * T;
    consigne_left = kp2 * vel_error + ki1 * integ_error;

    bal_error = totalR + totalL;
    set_motors((int)constraint_rot(consigne_right - kbr * bal_error, -255, 255), (int)constraint_rot(consigne_left - kbr * bal_error, -255, 255));
*/
      bal_error = totalR + totalL;
      consigne_vel_right_a=consigne_vel;//- kbm * bal_error;
      consigne_vel_left_a=-consigne_vel ;//- kbm * bal_error;


      Setpoint_a = consigne_vel_right_a;
      Setpointl_a = consigne_vel_left_a;
      Input_a =right_speed;
      Inputl_a =left_speed;
      myPID_a.Compute();
      myPID_L_a.Compute();

      //set_motors(Output_a, Outputl_a);
      set_motors(((Output_a*200)/1000),((Outputl_a*200)/1000));

    /*
    timecounter++;
    if(timecounter>MaxPamiTimeOut){
      break;
    }*/
    now = millis();
    delay(T - now + lastTime);
   // k++;
    /*
      if (k==10){
      Serial.print("target_angel:");
      Serial.print(Setpointd_a);
      Serial.print(",");
      Serial.print("target_velocity==PD_output:");
      Serial.print(consigne_vel);
      Serial.print(",");
      Serial.print("Pi_output_right:");
      Serial.print(Output_a);
      Serial.print(",");
      Serial.print("Pi_output_left:");
      Serial.print(Outputl_a);
      Serial.print(",");
      Serial.print("measured_velocity_R:");
      Serial.print(right_speed);
      Serial.print(",");
      Serial.print("measured_velocity_L:");
      Serial.print(left_speed);
      Serial.print(",");
      Serial.print("measured_position:");
      Serial.println((totalR-totalL)/2);
      k=0;
      }
      */
  }
  stop_motors();
  //alpha_deg = 0;//odemetry_start_angle + GyroAngle - salpha;
  //alpha = 0;//alpha_deg * PI / 180;
  //enablegyro = false;
}

void move_distance(double target) {
  /*
  myPID.SetMode(AUTOMATIC);
  myPID_L.SetMode(AUTOMATIC);
  myPID_distance.SetMode(AUTOMATIC);
  */
  double AccumilatedTarget = 0;
  bool edged = false;
  long int j=0;
    initi();
    edged = false;
    target -= AccumilatedTarget;
    int signe = (int)(target / fabs(target));
    float now = millis();
    //bool breaked = false;
    double dist;
    first = now;

    //while (fabs(target - ((totalR+totalL)/2 ))>2  ) {
    while ((fabs(target - totalR) > 2) || (fabs(target - totalL) > 2) ) {


      //counter_ult++;
      lastTime = millis();
      ramp = signe * calculateRamp(now - first,0.66);
      Setpointd = target;
      Inputd =(totalL+totalR)/2 ;
      myPID_distance.Compute();
      consigne_vel=Outputd;
      consigne_vel = constraint(consigne_vel, -rampmax, rampmax);
      if (signe == 1)
        consigne_vel = fmin(ramp, consigne_vel);
      else
        consigne_vel = fmax(ramp, consigne_vel);

      //Report();
      
      SetpointA=totalR;
      InputA=totalL;
      myPIDA.Compute();
      consigne_vel_right=consigne_vel-OutputA;
      consigne_vel_left=consigne_vel + OutputA;
      //consigne_vel_right=consigne_vel;
      //consigne_vel_left=consigne_vel ;
      Setpoint = consigne_vel_right;
      Setpointl = consigne_vel_left;
      Input =right_speed;
      Inputl =left_speed;
      myPID.Compute();
      myPID_L.Compute();
//((Outputl*200)/1000)
      //set_motors((((Output- (kbm * bal_error))*200)/1000),(((Outputl+ (kbm * bal_error))*200)/1000));
      //Output=constraint((Output- (kbm * bal_error)),-500,500);
      //Outputl=constraint((Outputl+ (kbm * bal_error)),-500,500);

      set_motors(((Output*200)/1000),((Outputl*200)/1000));

      /*
      vel_error = consigne_vel - right_speed;
      integ_error += vel_error * T;
      consigne_right = kp2 * vel_error + ki1 * integ_error;
      vel_error = consigne_vel - left_speed;
      integ_error += vel_error * T;
      consigne_left = kp2 * vel_error + ki1 * integ_error;
      bal_error = totalR - totalL;
      VR=consigne_right - kbm * bal_error;
      VL=consigne_left + kbm * bal_error;

      set_motors((int)constrain(VR, -255, 255), (int)constrain(VL, -255, 255));
      */
      now = millis();
      delay(T - now + lastTime);
      j++;
      if (j==5){
        /*
      Serial.print("target_position:");
      Serial.print(target);
      Serial.print(",");
      */
      //Serial.print("target_velocity==PD_output:");
      //Serial.print(consigne_vel);
      //Serial.print(",");
/*
      Serial.print("Pi_output_right:");
      Serial.print(Output);
      Serial.print(",");
      Serial.print("measured_velocity_R:");
      Serial.print(Input);
      Serial.print(",");
      Serial.print("mR:");
      Serial.print(totalR);
      Serial.print(",");
      Serial.print("consigne_vel_left:");
      Serial.print(Setpointl);
      Serial.print(",");
      Serial.print("Pi_output_left:");
      Serial.print(Outputl);
      Serial.print(",");
      Serial.print("measured_velocity_L:");
      Serial.print(Inputl);
      Serial.print(",");
      Serial.print("mL:");
      Serial.print(totalL);
      Serial.print(",");
      Serial.print("consigne_vel_right:");
      Serial.println(Setpoint);
      */
  

      //Serial.print(",");  
      //Serial.print("measured_position:");
      //Serial.println((totalR+totalL)/2);
      

      /*
      Serial.print("Pi_output_left:");
      Serial.print(consigne_left);
      Serial.print(",");
      */

      /*
      Serial.print("target_position:");
      Serial.print(1000/10);
      Serial.print(",");
      Serial.print("target_velocity==PD_output:");
      Serial.print(consigne_vel/10);
      Serial.print(",");
      Serial.print("Pi_output_right:");
      Serial.print(consigne_right/10);
      Serial.print(",");
       Serial.print("Pi_output_left:");
      Serial.print(consigne_left/10);
      Serial.print(",");
      */
      /*
      Serial.print("measured_velocity_R:");
      Serial.print(right_speed/10);
      Serial.print(",");
      */
      /*
      Serial.print("measured_velocity_L:");
      Serial.print(left_speed/10);
      Serial.print(",");
      Serial.print("measured_position:");
      Serial.println(((totalR + totalL) / 2)/10);
      Serial.print(",");
      Serial.print("mp_R:");
      Serial.println(totalR/10);
      Serial.print(",");
      Serial.print("mp_L:");
      Serial.println(totalL/10);
      */
      /*
       Serial.print("bal_error:");
       Serial.print(bal_error);
       Serial.print(","); 
       Serial.print("target_velocity==PD_output:");
       Serial.print(consigne_vel);
       Serial.print(",");
       Serial.print("VR:");
       Serial.print(consigne_right - kbm * bal_error);
       Serial.print(",");
       Serial.print("VL:");
       Serial.print(consigne_left + kbm * bal_error);
       Serial.print(",");
       Serial.print("measured_velocity_R:");
       Serial.print(right_speed);
       Serial.print(",");
       Serial.print("measured_velocity_L:");
       Serial.print(left_speed);
       Serial.println(",");
       */
      
      j=0;
      }



    }
    stop_motors();

    /*
    if (breaked) {
      for(int i = 0; i < Timeout;i++){
        delay(10);
        dist = Ultrasonic_dist();
        if (!(dist < 10 && dist != 0)) {
          edged = true;
          break;
        }
      }
      dist = Ultrasonic_dist();
      if (dist < 10 && dist != 0) {
        UltraActive = false || edged;
      }
      AccumilatedTarget = (totalR + totalL) / 2;
    }
    */
    /*
    else{
      return;
    }
    */
  
}

