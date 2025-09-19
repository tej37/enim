 #include <AccelStepper.h>
//#include <MultiStepper.h>
#include <math.h>

// Define the stepper motor and the pins that is connected to
AccelStepper stepper1(1, 12, 14); // (Typeof driver: with 2 pins, STEP, DIR)
AccelStepper stepper2(1, 25, 32);
long int currentstepR=0;
long int currentstepL=0;
//float R=38.5; //reyan roue
//float r=37.5;// antraxe/2

float R=79/2; //reyan roue
float r=82/2;// antraxe/2

int j=0;
int strategy=2;

//MultiStepper steppersControl;  // Create instance of MultiStepper
#define blue 26
#define jack 27
#define actionneur 23
//long gotoposition[2]; // An array to store the target positions for each stepper motor
void moveforward(float steps){
  long int numstep=0;
  numstep=ceil(distance_to_steps(steps));
   //long int stepsR=0;
   //long int stepsL=0;
    currentstepR=numstep+currentstepR;
    currentstepL=numstep+currentstepL;
    //gotoposition[0] =-stepsR; 
    //gotoposition[1] = stepsL; 
    //steppersControl.moveTo(gotoposition); 
    //steppersControl.runSpeedToPosition(); 
    stepper1.moveTo(-currentstepR);
    stepper2.moveTo( currentstepL);
    while (stepper1.currentPosition() != -currentstepR|| stepper2.currentPosition() != currentstepL) {
    stepper1.run();  // Move or step the motor implementing accelerations and decelerations to achieve the target position. Non-blocking function
    stepper2.run();
    //
    //
  }



   //currentstepR=stepsR;
   //currentstepL=stepsL;
    
    }
void turntesinleft(long int angel){
  float numstep=0;
  //long int stepsR=0;
  //long int stepsL=0;
  //numstep=(angel*r)/(0.0314*R);
  numstep=((angel*M_PI)/180)*r;
  numstep=ceil(distance_to_steps(numstep));

  currentstepR=currentstepR+numstep;
  currentstepL=currentstepL-numstep;
  /*
  gotoposition[0] = -stepsR;
  gotoposition[1] = stepsL;

  steppersControl.moveTo(gotoposition);
  steppersControl.runSpeedToPosition();
  */
    stepper1.moveTo(-currentstepR);
    stepper2.moveTo( currentstepL);
    while (stepper1.currentPosition() != -currentstepR|| stepper2.currentPosition() != currentstepL) {
    stepper1.run();  // Move or step the motor implementing accelerations and decelerations to achieve the target position. Non-blocking function
    stepper2.run();
    //
    //
  }

   //currentstepR=stepsR;
   //currentstepL=stepsL;
  }

  void turntesinright(long int angel){
  long int numstep=0;
  //long int stepsR=0;
  //long int stepsL=0;
  numstep=((angel*M_PI)/180)*r;
  numstep=ceil(distance_to_steps(numstep));

  currentstepR=currentstepR-numstep;
  currentstepL=currentstepL+numstep;
  /*
  gotoposition[0] = -stepsR;
  gotoposition[1] = stepsL;

  steppersControl.moveTo(gotoposition);
  steppersControl.runSpeedToPosition();
  */
    stepper1.moveTo(-currentstepR);
    stepper2.moveTo( currentstepL);
    while (stepper1.currentPosition() != -currentstepR|| stepper2.currentPosition() != currentstepL) {
    stepper1.run();  // Move or step the motor implementing accelerations and decelerations to achieve the target position. Non-blocking function
    stepper2.run();
    //
    //
  }

   //currentstepR=stepsR;
   //currentstepL=stepsL;
  }

  float distance_to_steps(float distance){
    return((distance*800)/(M_PI*2*R));

  }

  void run_acctionneur(){
  analogWrite(23, 150);
  }

void setup() {
  //Serial.begin(9600);
  stepper1.setMaxSpeed(1200); // Set maximum speed value for the stepper
  stepper1.setAcceleration(1500); // Set acceleration value for the stepper
  stepper1.setCurrentPosition(0); // Set the current position to 0 steps

  stepper2.setMaxSpeed(1200);
  stepper2.setAcceleration(1500);
  stepper2.setCurrentPosition(0);
  pinMode(blue,INPUT_PULLUP);
  pinMode(jack,INPUT_PULLUP);
  pinMode(2,OUTPUT);



}
void strategy_blue(){
  //Serial.println("blue");
  delay(87500);// (l3abt beha lbera7 belgiqiya  86500 zone loula 560 )pami sgir 
  //init_wakt=millis();
  run_acctionneur();
  moveforward(250);
  turntesinleft(45);
  moveforward(560);
  turntesinright(50);//(kenet 45)
  moveforward(320);
  //run_acctionneur();
}   
void strategy_yellow(){
 // Serial.println("yello");
  delay(87500);//(l3abt beha lbera7 86500 loula 560 )
  //init_wakt=millis();
  run_acctionneur();
  moveforward(250);
  turntesinright(45);
  moveforward(560);
  turntesinleft(50);
  moveforward(320);
  //run_acctionneur();
}


void loop() {
  /*
  // Store the target positions in the "gotopostion" array
  gotoposition[0] =-1400; 
  gotoposition[1] = 1400; // 800 steps - full rotation with quater-step resolution


  steppersControl.moveTo(gotoposition); // Calculates the required speed for all motors
  steppersControl.runSpeedToPosition(); // Blocks until all steppers are in position

  delay(1000);

  gotoposition[0] = 0;
  gotoposition[1] = 0;


  steppersControl.moveTo(gotoposition);
  steppersControl.runSpeedToPosition();

  delay(1000);
  */
//moveforward(868,currentstepR,currentstepL);

while((strategy!=LOW) && (strategy!=HIGH) ){
//Serial.print("initila=");
//Serial.println(strategy);
strategy=digitalRead(blue); 
if(strategy==HIGH) {
  //Serial.print("strategy_high=");
 // Serial.println(strategy);
  digitalWrite(2, HIGH);
 // delay(2000);
  }
else if(strategy==LOW) {
  //Serial.print("strategy_low=");
  //Serial.println(strategy);
  digitalWrite(2,HIGH);
  delay(500);
  digitalWrite(2,LOW);
  delay(500);
  digitalWrite(2,HIGH);
  delay(500);
  digitalWrite(2,LOW);
  delay(500);
  }
  
}
//Serial.print("strategy=");
//Serial.println(strategy);
j=digitalRead(jack);
if(j==HIGH){
if(strategy==1){
  strategy_blue();
  while(1){  
    //Serial.println("fin_blue");
}

}
else if(strategy==0){
  strategy_yellow();
  while(1){//Serial.println("fin_yellow");
  }
}
  



}


//delay(2000);



}
