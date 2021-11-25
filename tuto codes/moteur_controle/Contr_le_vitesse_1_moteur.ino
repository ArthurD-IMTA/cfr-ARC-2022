#include <PID_v1.h>  // Bibliothèque pour le PID
#include <TimerOne.h> // Bibliothèque pour le timer

//Ports Moteur
int ENA = 10;
int IN1 = 9;
int IN2 = 8;
int w_max=255; //Définir la vitesse maximale de rotation possible pour le circuit de puissance envisagé

//Ports PID
int ENCODEURA = 2;
int ENCODEURB = 4;


void setup() {
  // Initialisation des variables pour le moteur
  pinMode (ENA, OUTPUT);
  pinMode (IN1, OUTPUT);
  pinMode (IN2, OUTPUT);

  //Codeur
  pinMode(ENCODEURA, INPUT_PULLUP);
  pinMode(ENCODEURB, INPUT_PULLUP);

  
  Serial.begin(9600);

}

void Vitesse_max (){
 //permet de faire tourner le moteur à sa vitesse maximale
 digitalWrite (IN1, HIGH);
 digitalWrite (IN2, LOW);
 analogWrite (ENA, 255); //Vitesse moteur A entre 0 et 255
 
}

void Avancer (){
 //permet de faire tourner le moteur à sa vitesse maximale
 digitalWrite (IN1, HIGH);
 digitalWrite (IN2, LOW);
 analogWrite (ENA, 100); //Vitesse moteur A entre 0 et 255
 
}

void Arreter (){
 //permet de faire tourner le moteur à sa vitesse maximale
 digitalWrite (IN1, HIGH);
 digitalWrite (IN2, LOW);
 analogWrite (ENA, 0); //Vitesse moteur A entre 0 et 255
 
}

void Tourner (int speed){
 //sens moteur A
 //map(w,0,wMax,0,255) ou w/w_max ou map(w,0,wMax,0,T) avec T la période
 digitalWrite (IN1, HIGH);
 digitalWrite (IN2, LOW);
 analogWrite (ENA, 100); //Vitesse moteur A entre 0 et 255
 
}

void loop() {
  // Une fonction pour donner une commande en vitesse de rotation
  Avancer ();
  delay (5000);
  
  Vitesse_max ();
  delay (5000);
  
  Arreter ();
  delay (4000);


}
