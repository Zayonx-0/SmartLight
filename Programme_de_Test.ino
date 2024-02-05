//-----------------------------------------------------------------------
//
//    BUT GEII TOULON Semestre 1 - SAE 1.01
//
//    Programme de test de la carte SmartLight à compléter
//
//-----------------------------------------------------------------------

//-------------Inclusion des librairies nécessaires au programme

#include <IRremote.h>            // Bibliothèque pour utiliser le récepteur infrarouge.
#include <Wire.h>                // Bibliothèque permettant effectuer une liaison I2C.
#include "Adafruit_VEML7700.h"   // Bibliothèque permettant d'utiliser notre capteur de luminosité ambiante VEML7700. 

//------------- Définition des différentes broches utilisées de l'arduino Nano, ainsi que leur composant associé. 

#define TX 1                      // Pin TX
#define RX 0                      // Pin RX
#define BP2 2                     // Bouton poussoir 2
#define PWM2 3                    // 
#define BP1 4                     // Bouton poussoir 1
#define LED_R 5                   // Led Rouge de la led RGB
#define LED_G 6                   // Led verte de la led RGB
#define DOUT 7                    // Permet de contrôler un bandeau de led qui serais connecté à la carte
#define IR 8                      // Pin du récepteur infrarouge de la carte
#define PWM1 9                    // Pin contrôlant la grille du mosfet associé à la lampe.
#define SPI_SS 9                  // Pin permettant une liaison SPI
#define LED_B 10                  // LED Bleue de la led RGB
#define TEMP A0                   // Entrée de température via le bus I2C
#define A1 A1                     // Pin d'entrée associé à notre capteur de lumière ambiante VEML7700
#define LED2 A2                   // Pin permettant de contrôler la Led2
#define LED1 A3                   // Pin permettant de contrôler la Led1
#define SDA A4                    //
#define SCL A5                    //
#define POT A6                    // Pin d'entrée du potentiomètre de la acrte
#define LIGHT1 A7                 // Pin d'entrée associé à notre photorésistance




//-------------Déclaration des sous-programmes (Prototypes)
  void testLeds();
  void testBoutons();
  void testAnalogique();
  void testPwm();
  void testVEML();
  void testIr();
  

//-------------Déclaration des variables globales (Utilisées dans tout le programme)
  unsigned long currentMillis1,currentMillis2,previousMillis1 = 0,previousMillis2 = 0;            // Variables permettant l'éxecution des fonctions après un minimum de temps donné
  int k=0;                                                                                        // Variable incrémentée dans la fonction testPwm()
  Adafruit_VEML7700 veml = Adafruit_VEML7700();                                                   // Variable associée au capteur de lumière ambiante

//-------------Programme SETUP lancé au démarrage

void setup() {
  Serial.begin(9600);                                     // Démarrage du moniteur de série à 
  Serial.println("Test carte SmartLight ");               // Affichage de "Test carte Smartlight" informant sur le démarrage du programme 
  pinMode(LED1, OUTPUT);                                  // Définition de la LED1 en tant que sortie
  pinMode(LED2, OUTPUT);                                  //
  pinMode(BP1, INPUT_PULLUP);                             // Définition du bouton poussoir 1 en tant qu'entrée associé à une résistance de rappel
  pinMode(BP2, INPUT_PULLUP);                             //
  pinMode(LIGHT1, INPUT);                                 // Définition de la photorésistance en tant qu'entrée
  pinMode(POT, INPUT);                                    //
  pinMode(A1, INPUT);                                     //
  pinMode(LED_R, OUTPUT);                                 //
  pinMode(LED_G, OUTPUT);                                 //
  pinMode(LED_B, OUTPUT);                                 //
  pinMode(PWM1, OUTPUT);                                  //


  testLeds();
   

    // configurations/initialisations pour le capteur Veml7700
  veml.begin();                                         // Démarrage de la liaison avec le capteur de luminosité ambiante
  veml.setGain(VEML7700_GAIN_1);                        // Définition du gain du VEML par défaut
  veml.setIntegrationTime(VEML7700_IT_800MS);           // Définition du délai entre chaque mesure de luminosité à 800ms (délai haut permettant d'une grande précision)


    // configurations/initialisations pour le recepteur Inra Rouge 
  IrReceiver.begin(IR);                                   // Permet de démarrer la réception infrarouge
    
}

//-------------Programme LOOP lancé après le SETUP et en boucle (Programme principale)

void loop() {

    
  currentMillis1 = millis();                              // Durée actuelle d'éxecution du programme
  if (currentMillis1 - previousMillis1 >= 10) {           // On attend que au moins 10ms se soient écoulées avant d'éxécuter les fonctions présentent dans le if        
    previousMillis1 = currentMillis1;                     // Reset du temps écoulé depuis la dernière éxecution des fonctions présentent dans le if
      
    testBoutons();                    // Fonction de test des boutons poussoirs
    testPwm();                        // Fonction de test des sorties PWM (transistor de puissance et led RGB)
  } 

  
    
  currentMillis2 = millis();                              // Durée actuelle d'éxecution du programme
  if (currentMillis2 - previousMillis2 >= 1000) {         // On attend que au moins 1000ms se soient écoulées avant d'éxecuter les fonctions présentent dans le if
    previousMillis2 = currentMillis2;                     // Reset du temps écoulé depuis la dernière éxecution des fonctions présentent dans le if
    
    testAnalogique();                                     // Fonction de test du capteur de lumière ambiante, de la photorésistance et du potentiomètre
    testVEML();                                           // Fonction de test du capteur de lumière ambiante et affichage des valeurs en lux.
    testIr();                                             // Fonction de test du récepteur infrarouge. 
  } 
}

//-------------Définition des sous programmes à compléter
  void testLeds(){                                        // Fonction de test des leds
    Serial.println("test des Leds");                      // Annonce d'entrée dans la fontion de test des leds
    for (int i = 0; i < 5; i++) {                         // Boucle for s'éxecutant tant que i est inférieur à 5 (Pour faire clignoter la led1 5 fois)
      digitalWrite(LED1, HIGH);                           // Mise à l'état haut de la broche associé à la led1
      delay(500);                                         // On laisse la led allumée 500ms 
      digitalWrite(LED1, LOW);                            // Mise à l'état bas de la led1
      delay(500);                                         // On attend 500ms avant d'allumer à nouveau la led
      
      }
    for (int j = 0; j < 5; j++) {                         // Même fonctionnement que la led1 pour la led 2.
      digitalWrite(LED2, HIGH);
      delay(500);
      digitalWrite(LED2, LOW);
      delay(500);
      }
    
  }
  
  void testBoutons(){                                     // Fonction de test des boutons poussoirs
    if (!digitalRead(BP1)) {                              // Lecture de l'état de BP1. On inverse cette entrée car il y a une résistance de tirage.
      digitalWrite(LED1, HIGH);                           // Mise à l'état haut de la led1 tant que le bouton poussoir est appuyé
    } else {
      digitalWrite(LED1, LOW);                            // Mise à l'état bas de la led 2 lors du relâché du bouton poussoir. 
      }
    if (!digitalRead(BP2)) {                              // Même fonctionnement que pour le bouton 2 et la led2 
      digitalWrite(LED2, HIGH);
    } else {
      digitalWrite(LED2, LOW);
      }
    
  }
  
  void testAnalogique(){                                                    // Fonction de test des entrées analogiques (photodiode, photorésistance et potentiomètre)
    Serial.println((String)"Photorésistance = " + analogRead(A1));          // Affichage de la valeur analogique de la photorésistance. (String) ici permet de convertir
                                                                            // la valeur numérique récupérée par le AnalogRead en type String.
                                                                            // La fonction analogRead permet de récupérer la valeur d'une entrée analogique et de la convertir
                                                                            // en entier compris entre 0 et 1023.
    Serial.println((String)"Potentiomètre = " + analogRead(POT));           // Affichage de la valeur analogique du potentiomètre
    Serial.println((String)"Photodiode = " + analogRead(LIGHT1));           // Affichage de la valeur analogique brute de la photodiode
    Serial.println("\n");                                                   // Affichage d'un retour à la ligne (Pour espacer les affichages pour plus de lisibilité)
    
  }
  
  void testPwm(){                                         // Fonction de test des sorties PWM (Power Width Modulation)
    if (k >= 1280) {                                      // Si la valeur incrémentée k est supérieur à 1280, celà signifie que l'on a finit tous les tests des PWM
                                                          // dans ce cas, j'ai décidé de mettre toutes les sorties PWM à 0 pour éviter d'avoir trop de lumière dans les yeux 
                                                          //pendant les phases de tests. K s'incrémente de 0 à 1280, chaque valeur incrémentée de k signifie que 10ms se sont écoulées.
      analogWrite(LED_G, 0);
      analogWrite(LED_B, 0);
      analogWrite(LED_R, 0);
      analogWrite(PWM1, 0);
      k = 9999;
      
      } else k++;                                         // Si on arrive ici, celà signifie que l'on est encore dans la phase de test des PWM, on incrémente donc la valeur K.

      if (k < 256) {                                      // Si K est inférieur à 256, on incrémente la luminosité de la led RGB Rouge grâce à l'augmentation du rapport cyclique.
                                                          // La fonnction analogWrite permet de faire varier le rapport cyclique d'une sortie PWM. 
        
        analogWrite(LED_R, k);                            // Paramétrage du rapport cyclique pour la Led rouge de la led RGB à la valeur de K. 
        analogWrite(PWM1, k);                             // Idem que ci-dessus
      
      }
      else {
        
        if (k < 512) {                                    // Lorsque K est ici, il est entre 256 et 511. 
          analogWrite(LED_G, k - 256);                    // La fonction analogwrite se commande uniquement grâce à une valeur entre 0 et 256. On soustrait donc à k 256 étant donné 
                                                          // que ce dernier sera entre 256 et 511. 
          analogWrite(PWM1, 0);                           // Mise à l'état bas de la lampe
          analogWrite(LED_R, 0);                          // Mise à l'état bas de la led rouge de la led rgb comme demandé dans l'activité 4.
          }

         else
         {
          if (k < 768) {                                  // Idem que ci-dessus pour des valeurs comprises entre 512 et 767 (donc on soustrait de 512 la valeur de k).
            analogWrite(LED_G, 0);                        // Ici, on va désormais après avoir éteint la led verte de la led rgb, allumer les 3 leds et les laisser allumée progressivement.
            analogWrite(LED_B, k - 512);
            }
           else {
            
            if (k < 1024) {                               // Idem que ci-dessus pour des valeurs comprises entre 768 et 1023 (donc on soustrait de 768 la valeur de k).
              analogWrite(LED_R, k - 768);
              }
             else {
              if (k < 1280) {                             // Idem que ci-dessus pour des valeurs comprises entre 1024 et 1279 (donc on soustrait de 1024 la valeur de k).
                analogWrite(LED_G, k - 1024);
                
                }
              
              }
            }
          
          }
        
        }
    
  }
  
  void testVEML(){                                                                                          // Programme de test du capteur de luminosité ambiante
    Serial.print("Luminosité de rérérence : "); Serial.print(veml.readLux()); Serial.println("lux");        // Affichage de la valeur relevée par le capteur de luminosité ambiante en lux.
  }
  
  void testIr(){                                                                      // programme de test du récepteur infrarouge. 
    if (IrReceiver.decode()) {                                                        // Si IrReceiver.decode() est vraie, alors un signal infrarouge a été intercepté.
      uint16_t command = IrReceiver.decodedIRData.command;                            // On décode l'infrarouge reçu et on en récupère la commande qui a été transmise.
      Serial.println((String)"Code touche télécommande :" + command);                 // Affichage de la commande reçue par l'arduino dans le moniteur de série.

      if (command == 147) {                                                           // La commande 147 correspond correspond à la touche 1 de la télécommande infrarouge. 
        digitalWrite(LED1, HIGH);                                                     // On met donc à l'état haut la led1 suite à la commande reçue.
        } else if (command == 146) {                                                  // Idem mais pour la touche 2 correspondant à la touche 2.
          digitalWrite(LED2, HIGH);                                                   // Idem que précédemment pour la led2
          } else if (command == 144) {                                                // Idem mais pour la touche On/Off correspondant à la commande 144
            digitalWrite(LED1, LOW);                                                  // Dans ce cas, on éteint les leds 1 et 2 (mise à l'état bas de ces dernières). 
            digitalWrite(LED2, LOW);
            }
       delay(500);                                                                    // Délai de 500ms pour éviter la répétion de l'affichage sur un appui long.      
      IrReceiver.resume();                                                            // On ré active la réception des commandes infrarouges ici.
        }
     
    
  }
