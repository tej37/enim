
const int encoderLPin1 = 23;
const int encoderLPin2 = 18;
const int encoderRPin1 = 34;
const int encoderRPin2 = 35;


volatile int counterL = 0;
volatile int counterR = 0;

float ticks_to_distance(int x, float r, int resolution, int precision) {
  return (x * 2 * PI * r / (resolution * precision));
}

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

void setup() {
  Serial.begin(9600);
  pinMode(encoderLPin1, INPUT_PULLUP);
 pinMode(encoderRPin1, INPUT_PULLUP);
  pinMode(encoderLPin2, INPUT_PULLUP);
pinMode(encoderRPin2, INPUT_PULLUP);





  attachInterrupt(digitalPinToInterrupt(encoderLPin1), handleEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderRPin1), handleEncoderB, CHANGE);

}
void loop(){
//dR = ticks_to_distance(counterR, right_radius, right_resolution, 1);  // d_right count - prevvount en 10 ms
//dL = ticks_to_distance(counterL, left_radius, left_resolution, 1);
Serial.print("counterL= ");
Serial.println(counterL);
Serial.print("counterR= ");
Serial.println(counterR);
delay(1000);
//analogWrite(motorleft1,100);
//analogWrite(motorright1,100);


}