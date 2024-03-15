#include <Servo.h>

Servo myservo1;
Servo myservo2;

// Definir los pines a los que están conectados los fotoresistores
const int fotoresistorPin1 = A0;
const int fotoresistorPin2 = A1;
const int fotoresistorPin3 = A2;
const int fotoresistorPin4 = A3;

// Definir los pines a los que están conectados los servomotores
const int servoPin1 = 9;
const int servoPin2 = 10;

// Definir el umbral de diferencia de lectura de los fotoresistores
const int umbralDiferencia = 15;

float angulo1 = 0.0;
float angulo2 = 0.0;


void setup() {
  // Inicializar los servomotores
  Serial.begin(9600);

  myservo1.attach(servoPin1);
  myservo2.attach(servoPin2);

  myservo1.write(90);
  myservo2.write(45);
}

void loop() {
  // Leer valores de los fotoresistores
  float valorA = analogRead(fotoresistorPin1);
  float valorB = analogRead(fotoresistorPin2);
  float valorC = analogRead(fotoresistorPin3);
  float valorD = analogRead(fotoresistorPin4);

  float diferencia1 = valorA - valorB;
  float diferencia2 =valorD -valorC;
  // float angulo = map(diferencia,0,1023,0,180);

  if (abs(diferencia1) > umbralDiferencia) {
    angulo1 = map(diferencia1, -800, 800, 0, 180); 
    myservo1.write(angulo1);
  }

  if (abs(diferencia2) > umbralDiferencia) {
    angulo2 = map(diferencia2, -800, 800, 0, 180); 
    myservo2.write(angulo2);
  }
 
  Serial.print(valorA);
  Serial.print("\t");
  Serial.print(valorB);
  Serial.print("\t");
  Serial.print(angulo1);
  Serial.print("\t");
  Serial.print(angulo2);
  Serial.print("\t");
  Serial.print(diferencia1);
  Serial.print("\t");
  Serial.print(diferencia2);
  Serial.print("\n");
  delay(500);


 
}
