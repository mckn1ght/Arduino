#include <Servo.h>        // Include biblioteca Servo.h 
#include <NewPing.h>      // Include biblioteca Newping.h 

// Modul Driver Motoare dual L298N
const int MotorDreaptaInainte = 4;
const int MotorDreaptaInapoi = 5;
const int MotorStangaInapoi = 6;
const int MotorStangaInainte = 7;


#define TRIGGER_PIN  A1  // Pinul A1 de pe placa Arduino legat la pinul "trig" al senzorului ultrasonic.
#define ECHO_PIN     A2  // Pinul A2 de pe placa Arduino legat la pinul de Echo al senzorului ultrasonic.
#define DISTANTA_MAX 250 // Distanta maxima de bataie a senzorului in mm. Distanta maxima a senzorului HC-SR04 fiind de 45cm.

Servo servo_motor;  // Numele Servo-Motorului SG90
NewPing sonar(TRIGGER_PIN, ECHO_PIN, DISTANTA_MAX); // NewPing setup of pins and maximum distanta.
boolean avanseaza = false;
int distanta = 100;

void setup()
{
  // Pregatim pinii Modulului L298N pentru output.
  pinMode(MotorDreaptaInainte, OUTPUT);
  pinMode(MotorStangaInainte, OUTPUT);
  pinMode(MotorStangaInapoi, OUTPUT);
  pinMode(MotorDreaptaInapoi, OUTPUT);

  servo_motor.attach(10);   // Atasam Servo-Motorul SG90 in pinul nr 10 de pe placa Arduino
  servo_motor.write(115);   // 115 grade
  delay(2000);              // Asteapta 2 secunde
  distanta = citestePing();    // Ping distanta.
  delay(100);               // Asteapta 100ms.
  distanta = citestePing();
  delay(100);
  distanta = citestePing();
  delay(100);
  distanta = citestePing();
  delay(100);
}

void loop()
{
  int distantaDreapta = 0;
  int distantaStanga = 0;
  delay(50);

  if (distanta <= 20)
  {
    opreste();
      delay(300);
    mergeMarsarier();
      delay(400);
    opreste();
      delay(300);
    distantaDreapta = veziDreapta();
        delay(300);
    distantaStanga = veziStanga();
        delay(300);

    if (distantaDreapta >= distantaStanga)
    {
      vireazaDreapta();
            delay(300);
      opreste();
    }
    else
    {
      vireazaStanga();
            delay(300);
      opreste();
    }

  }
  else
  {
    mergeInainte();
  }

  distanta = citestePing();
}

int veziDreapta()     //Functie pentru Servo Motor
{
  servo_motor.write(50);
  delay(500);
  int distanta = citestePing();
  delay(100);
  servo_motor.write(115);
  return distanta;
}

int veziStanga()      // Functie pentru Servo Motor
{
  servo_motor.write(180);
  delay(500);
  int distanta = citestePing();
  delay(100);
  servo_motor.write(115);
  return distanta;
}

int citestePing()      // Functie pentru Senzorul Ultrasonic.
{
  delay(100);                 // Asteapta 100 ms intre ping-uri (aproximativ 20 pings/sec).
  int cm = sonar.ping_cm();  //Trimite ping, si obtine ping cu distanta in cm.
  if (cm == 0)
  {
    cm = 250;
  }
  return cm;
}




//INCEP FUNCTIILE PENTRU DRIVERUL MOTOARELOR
void opreste()       
{
  digitalWrite(MotorDreaptaInainte, LOW);
  digitalWrite(MotorDreaptaInapoi, LOW);
  digitalWrite(MotorStangaInainte, LOW);
  digitalWrite(MotorStangaInapoi, LOW);
}

void mergeInainte()
{
  if(!avanseaza){
    avanseaza = true;
  digitalWrite(MotorDreaptaInainte, HIGH);
  digitalWrite(MotorDreaptaInapoi, LOW);
  digitalWrite(MotorStangaInainte, HIGH);
  digitalWrite(MotorStangaInapoi, LOW);
  }
}

void mergeMarsarier()   
{
  avanseaza = false;
  digitalWrite(MotorDreaptaInainte, LOW);
  digitalWrite(MotorDreaptaInapoi, HIGH);
  digitalWrite(MotorStangaInainte, LOW);
  digitalWrite(MotorStangaInapoi, HIGH);
}

void vireazaDreapta()     
{
  digitalWrite(MotorDreaptaInainte, LOW);
  digitalWrite(MotorDreaptaInapoi, HIGH);
  digitalWrite(MotorStangaInainte, HIGH);
  digitalWrite(MotorStangaInapoi, LOW);
}

void vireazaStanga()      
{
  digitalWrite(MotorDreaptaInainte, HIGH);
  digitalWrite(MotorDreaptaInapoi, LOW);
  digitalWrite(MotorStangaInainte, LOW);
  digitalWrite(MotorStangaInapoi, HIGH);
}
