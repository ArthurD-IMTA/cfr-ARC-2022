// Moteur 1
int ENA = 10;
int IN1 = 9;
int IN2 = 8;
int pot = A0; /*port du potentiomètre */
int potValue = 0;

void setup (){
 // on déclare les pins comme sorties
 pinMode (ENA, OUTPUT);
 pinMode (IN1, OUTPUT);
 pinMode (IN2, OUTPUT);
 pinMode(pot, INPUT);
 Serial.begin(9600);

}
void Avancer (){
 //sens moteur A
 digitalWrite (IN1, HIGH);
 digitalWrite (IN2, LOW);
 analogWrite (ENA, 100); //Vitesse moteur A entre 0 et 255
 
}
void Reculer (){
 //sens moteur A
 digitalWrite (IN1, LOW);
 digitalWrite (IN2, HIGH);
 analogWrite (ENA, 128); //Vitesse moteur A
}
void Droite (){
 //sens moteur A
 digitalWrite (IN1, HIGH);
 digitalWrite (IN2, LOW);
 analogWrite (ENA, 200); //Vitesse moteur A
}
void Gauche (){
 //Sens moteur A
 digitalWrite (IN1, LOW);
 digitalWrite (IN2, HIGH);
 analogWrite (ENA, 50); //Vitesse moteur A
}
void Arreter (){
 //Sens moteur A
 digitalWrite (IN1, LOW);
 digitalWrite (IN2, LOW);
 analogWrite (ENA, 0); //Vitesse moteur A
}

void loop (){
 /* ----------- Utilisation des fonctions */
 /**
 Avancer ();
 delay (5000);
 Reculer ();
 delay (2000);
 Droite ();
 delay (2000);
 Gauche ();
 delay (2000);
 Arreter ();
 delay (4000);**/

 /* ----------- Utilisation d'un potentiomètre pour contorler la vitesse du moteur*/
 potValue = analogRead(pot);
 Serial.print("sensor before map =");
 Serial.print(potarValue);
 potValue = map(potValue,0,1024,0,255);
 Serial.print("sensor after map =");
 Serial.print(potValue);

 if (potValue>=5){
  digitalWrite (IN1, HIGH);
  digitalWrite (IN2, LOW);
  analogWrite (ENA, potValue);
 }
 else{
  digitalWrite (IN1, LOW);
  digitalWrite (IN2, LOW);
  analogWrite (ENA, 0); //Vitesse moteur A
 }
 delay(100);


 /* ----------- Accélération puis décélération*/
 /**
 for (int t=0; t<255; t++){
  digitalWrite (IN1, HIGH);
  digitalWrite (IN2, LOW);
  analogWrite (ENA, t); //Vitesse moteur A entre 0 et 255
  delay(10);
 }

 delay(3000);
 
 for (int t=255; t>=0; t--){
   digitalWrite (IN1, HIGH);
   digitalWrite (IN2, LOW);
   analogWrite (ENA, t); //Vitesse moteur A entre 0 et 255
   delay(10);
 }

 delay(1000);**/
 
 
}
