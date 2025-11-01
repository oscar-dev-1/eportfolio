#include "Wire.h"  // bibliothèque Wire 
#include "I2Cdev.h"  //bibliothèque I2Cdev 
#include "MPU6050.h" //bibliothèque MPU6050 
MPU6050 accelgyro1(0x69); //définition de l'adresse I2C associée au capteur 1
MPU6050 accelgyro2(0x68); //définition de l'adresse I2C associée au capteur 2
 
//Variables acquisition de theta1 (boîte)
int16_t ax1, ay1, az1;  //accélérations linéaires brutes
int16_t gx1, gy1, gz1;  //vitesses de rotation brutes à diviser par la sensibilité 
float theta_1_gyro=0;
float tehta_1_accel=0;
float theta_1_filtre=0;
float offset1 = -0.00526; //offset pour empecher la dérive du gyroscope 1

// Variables acquisition de theta2 (plateau)
int16_t az2, ax2, ay2 ;  //accélérations linéaires brutes
int16_t gz2, gx2, gy2 ;  //vitesses de rotation brutes à diviser par la sensibilité 

float angle_gyro2;
float theta_2_accel=0;
float theta_2_filtre;
float offset2 = 0.057; //offset pour empecher la dérive du gyroscope 2

float K=0.03; //constante pour le filtre comlémentaire


//Variables consigne angulaire
float ecart;
float consigne = 0; //Position angulaire autour de laquelle on cherche à se stabiliser (0° pour être à l'horizontale)
float seuil = 360*4/1024; //angele qui correspond à 4 pas = 1.4°

//Variables moteur 
const int pinBobine1A = 8 ;
const int pinBobine1B = 9 ;
const int pinBobine2A = 10 ;
const int pinBobine2B = 11 ;
float Tps =4;//délai entre chaque pas (4ms en pas entiers, 2ms en demi pas)
float dt = 4*Tps;//durée de chaque tour de boucle (16ms en pas entiers, 8*Tps = 16ms en demi pas)

unsigned long temps;//variable d'acquisition du temps pour python


void setup() {

accelgyro.initialize();  // initialisation du gyroscope 1
accelgyro2.initialize();  // initialisation du gyroscope 2

pinMode(pinBobine1A, OUTPUT); //définition des sorties pour le moteur
pinMode(piBobine1B, OUTPUT); // 
pinMode(piBobine2A, OUTPUT); // 
pinMode(piBobine2B, OUTPUT); //
}

void loop() {

    accelgyro1.getMotion6(&ax1, &ay1, &az1, &gx1, &gy1, &gz1); //accélérations linéaires et vitesses angulaires brutes du capteur 1
    accelgyro2.getMotion6(&az2,&ax2, &ay2, &gz2, &gx2, &gy2); //Permutations des axes d'origine pour respecter le paramétrage

    //acquisition de l'angle du manche (pour affichage seulement)    
    theta_1_accel = atan2((double)ax1,(double)ay1)*180/PI;  //Arctan(ax1/ay1) et 180/PI permet d'avoir l'angle en degré (double : variable locale)
    theta_1_filtre= K*theta_1_accel + (1-K)*(theta_1_filtre + offset1 +float(gz1)*dt/65.5); // 65.5 cf doc MPU6050, dt en s 

    //acquisition de l'angle du plateau (caméra)
    theta_2_accel = atan2((double)ax2,(double)ay2)*180/PI; 
    theta_2_filtre= K*theta_2_acel + (1-K)*(theta_2_filtre + offset2 +float(gz2)*dt/65.5); 

    ecart = consigne - theta_2_filtre; //écart angulaire de la caméra avec l'horizontale

    if (ecart>seuil){
        pas_entier_sens_horaire(); //ou demi_pas_sens_horaire();
    }
    else if (ecart<-1*seuil){ //ou demi_pas_sens_trigo();
        pas_entier_sens_trigo();
    }
    else{
        delay(4*Tps);
    }
    }
void pas_entier_sens_horaire(){
    // Pas n°1 |
    digitalWrite(pinBobine1A, LOW);
    digitalWrite(piBobine1B, LOW);   
    digitalWrite(piBobine2A, HIGH);
    digitalWrite(piBobine2B, LOW);
    delay(Tps); 


    // Pas n°2 |
    digitalWrite(pinBobine1A, HIGH);
    digitalWrite(piBobine1B, LOW);  
    digitalWrite(piBobine2A, LOW);
    digitalWrite(piBobine2B, LOW);
    delay(Tps); 

    // Pas n°3 |
    digitalWrite(pinBobine1A, LOW);
    digitalWrite(piBobine1B, LOW);   
    digitalWrite(piBobine2A, LOW);
    digitalWrite(piBobine2B, HIGH);
    delay(Tps); 

    // Pas n°4 |
    digitalWrite(pinBobine1A, LOW);
    digitalWrite(piBobine1B, HIGH);  
    digitalWrite(piBobine2A, LOW);
    digitalWrite(piBobine2B, LOW);
    delay(Tps); 
}    


void pas_entier_sens_trigo(){
    // Pas n°1 | 
    digitalWrite(pinBobine1A, LOW);
    digitalWrite(pinBobine1B, HIGH);  
    digitalWrite(pinBobine2A, LOW);
    digitalWrite(pinBobine2B, LOW);
    delay(Tps);

    // Pas n°2 | 
    digitalWrite(pinBobine1A, LOW);
    digitalWrite(pinBobine1B, LOW);   
    digitalWrite(pinBobine2A, LOW);
    digitalWrite(pinBobine2B, HIGH);
    delay(Tps); 

    // Pas n°3 | 
    digitalWrite(pinBobine1A, HIGH);
    digitalWrite(pinBobine1B, LOW);  
    digitalWrite(pinBobine2A, LOW);
    digitalWrite(pinBobine2B, LOW);
    delay(Tps); 

    // Pas n°4 | 
    digitalWrite(pinBobine1A, LOW);
    digitalWrite(pinBobine1B, LOW);   
    digitalWrite(pinBobine2A, HIGH);
    digitalWrite(pinBobine2B, LOW);
    delay(Tps); 

}

void demi_pas_sens_horaire(){
    // Pas n°1 | 
    digitalWrite(pinBobine1A, HIGH);
    digitalWrite(pinBobine1B, LOW);  
    digitalWrite(pinBobine2A, LOW);
    digitalWrite(pinBobine2A, LOW);
    delay(Tps);

    //Pas n°1.5 |
    digitalWrite(pinBobine1A, HIGH);
    digitalWrite(pinBobine1B, LOW);  
    digitalWrite(pinBobine2A, LOW);
    digitalWrite(pinBobine2A, HIGH);
    delay(Tps);
    // Pas n°2 | 
    digitalWrite(pinBobine1A, LOW);
    digitalWrite(pinBobine1B, LOW);  
    digitalWrite(pinBobine2A, LOW);
    digitalWrite(pinBobine2A, HIGH);
    delay(Tps); 
    // Pas n°2.5 | 
    digitalWrite(pinBobine1A, LOW);
    digitalWrite(pinBobine1B, HIGH);   
    digitalWrite(pinBobine2A, LOW);
    digitalWrite(pinBobine2A, HIGH);
    delay(Tps); 

    // Pas n°3 | 
    digitalWrite(pinBobine1A, LOW);
    digitalWrite(pinBobine1B, HIGH);   
    digitalWrite(pinBobine2A, LOW);
    digitalWrite(pinBobine2A, LOW);
    delay(Tps); 
    // Pas n°3.5 | 
    digitalWrite(pinBobine1A, LOW);
    digitalWrite(pinBobine1B, HIGH);  
    digitalWrite(pinBobine2A, HIGH);
    digitalWrite(pinBobine2A, LOW);
    delay(Tps); 

    // Pas n°4 | 
    digitalWrite(pinBobine1A, LOW);
    digitalWrite(pinBobine1B, LOW);   
    digitalWrite(pinBobine2A, HIGH);
    digitalWrite(pinBobine2A, LOW);
    delay(Tps); 

    // Pas n°4.5 | 
    digitalWrite(pinBobine1A, HIGH);
    digitalWrite(pinBobine1B, LOW);   
    digitalWrite(pinBobine2A, HIGH);
    digitalWrite(pinBobine2A, LOW);
    delay(Tps); 
    }


void demi_pas_sens_trigo(){
    // Pas n°1 | 
    digitalWrite(pinBobine1A, HIGH);
    digitalWrite(pinBobine1B, LOW);  
    digitalWrite(pinBobine2A, LOW);
    digitalWrite(pinBobine2A, LOW);
    delay(Tps);

    //Pas n°1.5 | 
    digitalWrite(pinBobine1A, HIGH);
    digitalWrite(pinBobine1B, LOW);  
    digitalWrite(pinBobine2A, HIGH);
    digitalWrite(pinBobine2A, LOW);
    delay(Tps);

    // Pas n°2 | 
    digitalWrite(pinBobine1A, LOW);
    digitalWrite(pinBobine1B, LOW);  
    digitalWrite(pinBobine2A, HIGH);
    digitalWrite(pinBobine2A, LOW);
    delay(Tps);

    // Pas n°2.5 | 
    digitalWrite(pinBobine1A, LOW);
    digitalWrite(pinBobine1B, HIGH);   
    digitalWrite(pinBobine2A, HIGH);
    digitalWrite(pinBobine2A, LOW);
    delay(Tps); 

    // Pas n°3 | 
    digitalWrite(pinBobine1A, LOW);
    digitalWrite(pinBobine1B, HIGH);   
    digitalWrite(pinBobine2A, LOW);
    digitalWrite(pinBobine2A, LOW);
    delay(Tps); 
    // Pas n°3.5 | 
    digitalWrite(pinBobine1A, LOW);
    digitalWrite(pinBobine1B, HIGH);  
    digitalWrite(pinBobine2A, LOW);
    digitalWrite(pinBobine2A, HIGH);
    delay(Tps); 

    // Pas n°4 | 
    digitalWrite(pinBobine1A, LOW);
    digitalWrite(pinBobine1B, LOW);   
    digitalWrite(pinBobine2A, LOW);
    digitalWrite(pinBobine2A, HIGH);
    delay(Tps); 

    // Pas n°4.5 | 
    digitalWrite(pinBobine1A, HIGH);
    digitalWrite(pinBobine1B, LOW);   
    digitalWrite(pinBobine2A, LOW);
    digitalWrite(pinBobine2A, HIGH);
    delay(Tps); 
}