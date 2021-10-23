// Ce programme décrit la commande d'un moteur CC avec le driver HW-95 (possibilité de l'adapter à deux moteurs) ainsi que la lecture de position par codeur incrémental

// Ressources :
// 1. La fonction attachInterrupt avec les pins correspondant : https://www.arduino.cc/en/Reference/AttachInterrupt&usg=ALkJrhhWYHEj0DN1X7n1tI6tN5s4jxNe9A
// 2. Le codeur incrémental : https://www.farnell.com/datasheets/1728690.pdf
// 3. Motor Driver utilisé : https://components101.com/modules/l293n-motor-driver-module



int ENA = 10; //sortie PWM pour controler la vitesse du moteur
int IN1 = 9; 
int IN2 = 8;
int pot = A0; //La vitesse du moteur est ici controlée par un potentiometre
int potValue = 0;
float tours = 0;

volatile byte state = LOW;
long rotationA = 0L;
long rotationB = 0L;

const byte pisteA = 2;
const byte pisteB = 3;
const byte pisteI = 18;



void setup() {
  pinMode (ENA, OUTPUT);
  pinMode (IN1, OUTPUT);
  pinMode (IN2, OUTPUT);
  pinMode(pot, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(pisteA, INPUT_PULLUP);
  pinMode(pisteB, INPUT_PULLUP);
  pinMode(pisteI, INPUT_PULLUP);

  Serial.begin(9600);
  attachInterrupt(digitalPinToInterrupt(pisteA), compteurA, RISING); //interruption aux fronts montant
  attachInterrupt(digitalPinToInterrupt(pisteB), compteurB, RISING);
  attachInterrupt(digitalPinToInterrupt(pisteI), blink, RISING);
}

void loop() {
  digitalWrite(LED_BUILTIN, state);
  moteur_controle_pot();
}

void blink() {
  tours +=0.02;
  //A chaque tour on affiche le nbr de tours, le nbr d'impulsions détectées sur chaque piste
  //On change aussi l'etat de la led interne à la carte pour avoir un repere visuel
  if ((tours-(int)tours)<0.01 || (tours-(int)tours)>0.99){
    state = !state;
    Serial.println("Tours :");
    Serial.println(tours);
    Serial.println((int)tours);
    Serial.println("Rotation A:");
    Serial.println(rotationA);
    Serial.println("Rotation B:");
    Serial.println(rotationB);
    }
  }

void compteurA(){
  rotationA++;  
}
void compteurB(){
  rotationB++;  
}

void moteur_controle_pot(){
  //Utilisation d'un potentiomètre pour contorler la vitesse du moteur
  potValue = analogRead(pot);
  potValue = map(potValue,0,1024,0,255);
  if (potValue>=5){
    digitalWrite (IN1, HIGH);
    digitalWrite (IN2, LOW);
    analogWrite (ENA, potValue);
  }
  else{
    digitalWrite (IN1, LOW);
    digitalWrite (IN2, LOW);
    analogWrite (ENA, 0);
  }
}
