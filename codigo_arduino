#include <PID_v1.h>

//#define PIN_INPUT 0
//#define PIN_OUTPUT 11

double Setpoint, Input, Output;
double Kp=75, Ki=378, Kd=268;
PID M1PID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT); 
//*******************

int IN1=12; //Pin giro M1
int IN2=13; //Pin giro M1
int ENA=11; //Pin velocidad M1
int PWM=100; //Velocidad en rpm

int D0=2; //Pin encoder
int pulsos=20; //Muescas del encoder

volatile unsigned tiactual=0;
volatile unsigned tianterior=0;
volatile unsigned dti=0;

volatile unsigned manterior=0;
volatile unsigned mactual=0;
volatile double dM=0;

int contador=0;
double frecuencia=0;
double rpm=0;


void setup() {
  // put your setup code here, to run once:

  //Input=analogRead(PIN_INPUT);
  Input=rpm;
  Setpoint=100;
  M1PID.SetMode(AUTOMATIC);
  //****************
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);
  pinMode(ENA,OUTPUT);

  attachInterrupt(digitalPinToInterrupt(D0),Encoder,CHANGE);
  Serial.begin(9600);
  pinMode(D0,INPUT);
  
  
}

void Encoder()
{
  contador++;
  frecuencia=(1/1000)/(double)dti;
  tianterior=tiactual;
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(IN1,HIGH);
  digitalWrite(IN2,LOW);
  analogWrite(ENA,PWM);
  
  tiactual=millis();
  mactual=millis();
  dM=(double)mactual-manterior;

  if(dM >= 1)
  {
    dti=tiactual-tianterior;
    rpm=frecuencia*(60/1)/pulsos; //Hz->s / dividir entre el número de ranuras
  
    Serial.print(rpm);
    manterior=mactual;
  };
  //*******************
  //Input=analogRead(PIN_INPUT);
  Input=rpm;
  M1PID.Compute();
  //analogWrite(PIN_OUTPUT, Output);
  analogWrite(ENA,Output);
}
