/*
  Pilotage automatique de l'abri du telescope
  Serge CLAUS
  GPL V3
  Version 4.0
  22/10/2018-12/02/2021

  Bouton à clef pour ouverture de l'abri:
    Position 1: Ouverture des portes seules
    Position 2: Ouverture de l'abri
  2 Boutons à l'intérieur:
    Bouton ouverture des portes
    Bouton fermeture de l'abri
  2 boutons arrêt d'urgence (1 intérieur, 1 extérieur)
  2 capteurs de position de l'abri (ouvert, fermé)
  1 capteur de position du télescope (parqué soft ou hard)
  2 capteurs d'ouverture des portes
  1 barrière IR de sécurité
  /*********************************/

//---------------------------------------Modules--------------------------------
#include "RollOffIndi.h"

//---------------------------------------PERIPHERIQUES--------------------------
// Temps maxi de park en secondes
#define TPSPARK 120
// Timer
//#include <SimpleTimer.h>

// LEDs neopixel
#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
#include <avr/power.h> // Required for 16 MHz Adafruit Trinket
#endif
#define LEDPIN 13
#define NBLEDS 24  // Nombre total de LEDs (3 barrettes de 8 LEDs)
Adafruit_NeoPixel pixels(NBLEDS, LEDPIN, NEO_GRB + NEO_KHZ400);
/* 0:   Status abri
   1-8: Eclairage table
   9-16:Eclairage intérieur
  // Afficheur OLED SSD1306
*/

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET     -1
#define SCREEN_ADDRESS 0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

//---------------------------------------CONSTANTES-----------------------------
// Sorties
#define ALIM12V A3  // (R3) Mise en marche de l'alimentation 12V de l'abri (vérins portes, capteurs)
#define ALIMTEL A4   // (R4) Alimentation télescope    Relais dans le boitier OnStep)
#define ALIMMOT A2  // (R2) Alimentation 220V moteur abri
#define MOTEUR  A1  // (R1) Ouverture/fermeture abri Commande moteur de porte de garage
#define P11     3   // (R5) Relais 1 porte 1
#define P12     5   // (R6) Relais 2 porte 1
#define P21     6   // (R7) Relais 1 porte 2
#define P22     7   // (R8) Relais 2 porte 2

//#define RESET   A13 // Reset de l'arduino

// Entrées
#define PARK  A5     // Entrée Park: Etat du telescope 0: non parqué, 1: parqué

#define AO  49       // Capteur abri ouvert
#define AF  48       // Capteur abri fermé
#define Po1 24       // Capteur porte 1 ouverte
#define Po2 25       // Capteur porte 2 ouverte

// Boutons
#define B1CLEF   A12    // Bouton à clef d'ouverture/fermeture des portes (pos 1 & 2)
#define BARU     22    // Bouton d'arret d'urgence
//#define BINT   99    // Bouton intérieur d'ouverture des portes (au cas où)
#define BLUMT    A10    // Bouton d'éclairage de la table (rouge)  Interrupteur double
#define BLUMI    A11    // Bouton d'éclairage de l'abri   (rouge)

// Constantes globales
#define DELAIPORTES 40000L          // Durée d'ouverture/fermeture des portes (40000L)
#define DELAIPORTESCAPTEUR  30000L  // Durée d'ouverture/fermeture des portes (40000L)
#define DELAIMOTEUR 10000L          // Durée d'initialisation du moteur (40000L)
#define DELAIABRI   11000L          // Durée de déplacement de l'abri (15000L)
#define INTERVALLEPORTES 8000       // Intervalle entre la fermeture de la porte 1 et de la porte 2
#define INTERVALLECAPTEUR 6000      // Temps de fermeture/ouverture des portes pour tests capteurs
#define MOTOFF HIGH                 // Etat pour l'arret du moteur
#define MOTON !MOTOFF
#define IMPMOT 300                 // Durée d'impulsion moteur

//---------------------------------------Macros---------------------------------
#define AlimTelStatus (!digitalRead(ALIMTEL))    // Etat de l'alimentation télescope
#define Alim12VStatus (digitalRead(ALIM12V))  // Etat de l'alimentation 12V ATX
#define PortesOuvert  (!digitalRead(Po1) && !digitalRead(Po2) && Alim12VStatus)
#define AbriFerme     (!digitalRead(AF))
#define AbriOuvert    (!digitalRead(AO))
#define MoteurStatus  (!digitalRead(ALIMMOT))
#define StartTel      digitalWrite(ALIMTEL, LOW)
#define StopTel       digitalWrite(ALIMTEL, HIGH)
#define StartMot      digitalWrite(ALIMMOT, LOW)
#define StopMot       digitalWrite(ALIMMOT, MOTOFF)
#define Stop12V       digitalWrite(ALIM12V, LOW)
#define Start12V      digitalWrite(ALIM12V, HIGH)
#define TelPark       !digitalRead(PARK)
//#define TelPark true
#define OuvreP1       digitalWrite(P12,LOW);digitalWrite(P11,HIGH)
#define OuvreP2       digitalWrite(P22,LOW);digitalWrite(P21,HIGH)
#define FermeP1       digitalWrite(P11,LOW);digitalWrite(P12,HIGH)
#define FermeP2       digitalWrite(P21,LOW);digitalWrite(P22,HIGH)
#define Bclef         !digitalRead(B1CLEF)
#define Baru          !digitalRead(BARU)

#define BAPPUILONG  3000  //Durée pour un appui long sur le bouton
//---------------------------------------Activation------------------------------------------
// Timer
#include <SimpleTimer.h>
SimpleTimer timer;

// ------------------------------------Variables globales------------------------------------
int ETAPE = 0;  // Etape du grafcet (0 étape initiale, 100 étape principale)
int ETARU = 0;  // Etape du grafcet d'ARU
int ETASURV = 0;  // Etape du grafcet de surveillance
int ETARET = 0;   // Etape de retour pour les sous programmes

int tmButton;         // Bouton(s) du TM1638
int countM;           // Nombre d'essais ouverture/fermeture

bool TEMPO = false; // Temporisation
bool TBOUTON = false; // Temporisation bouton
bool INIT = false;    // Abri initialisé
bool DEPL = false;    // Abri en cours de déplacement
bool FERM = false;    // Portes en cours de fermeture
bool SURV = false;    // Activation du graphe de surveillance
bool CMDARU = false;  // Commande interne d'arret d'urgence
bool AUTO = false;    // Abri en mode automatique (commande reçue à distance)
bool Bmem = false;    // Mémorisation du bouton

bool BLUMTO = false;  // Dernier etat du bouton d'éclairage table
bool BLUMIO = false;  // Dernier etat du bouton d'éclairage intérieur


//---------------------------------------SETUP-----------------------------------------------
void setup() {
  // LEDs APA106
  pixels.begin();
  pixels.clear();
  pixels.show();
  //SSD1306
  display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS);
  //display.display();
  display.clearDisplay();
  display.setTextSize(3);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.cp437(true);
  display.write("Init");
  display.display();
  // Initialisation des ports série
  Serial.begin(9600);  // Connexion à AstroPi (port Indi)

  // Initialisation des relais
  digitalWrite(ALIM12V, HIGH); pinMode(ALIM12V, OUTPUT);
  digitalWrite(ALIMTEL, HIGH); pinMode(ALIMTEL, OUTPUT);
  digitalWrite(ALIMMOT, HIGH); pinMode(ALIMMOT, OUTPUT);
  digitalWrite(MOTEUR, HIGH); pinMode(MOTEUR, OUTPUT);
  digitalWrite(P11, HIGH); pinMode(P11, OUTPUT);
  digitalWrite(P12, HIGH); pinMode(P12, OUTPUT);
  digitalWrite(P21, HIGH); pinMode(P21, OUTPUT);
  digitalWrite(P22, HIGH); pinMode(P22, OUTPUT);
  digitalWrite(ALIMMOT, MOTOFF); // Coupure alimentation moteur abri

  // Activation des entrées (capteurs...)
  pinMode(AO, INPUT_PULLUP);
  pinMode(AF, INPUT_PULLUP);
  pinMode(Po1, INPUT_PULLUP);
  pinMode(Po2, INPUT_PULLUP);
  pinMode(B1CLEF, INPUT_PULLUP);
  pinMode(BARU, INPUT_PULLUP);
  pinMode(BLUMT, INPUT_PULLUP);
  pinMode(BLUMI, INPUT_PULLUP);
  pinMode(PARK, INPUT); // TODO voir pour input_pullup (modifier code auxiliaire)
  //timer.setInterval(1000,debug);
}

void loop() {
  readIndi();
  timer.run();
  ssd1306Info();
  eclairages();
  // Gestion de l'abri Grafcet
  grafPrincipal();
  grafARU();
  grafSurv();
}

void eclairages() {
  // Gestion des écairages
  bool Etat = digitalRead(BLUMI);
  if (Etat != BLUMIO) {
    BLUMIO = Etat;
    if (Etat) {
      for (byte i =  8 ; i < (16); i++) {
        pixels.setPixelColor(i, pixels.Color(0, 128, 0));
      }
      pixels.show();
    }
    else {
      for (byte i =  8 ; i < (16); i++) {
        pixels.setPixelColor(i, pixels.Color(0, 0, 0));
      }
      pixels.show();
    }
    delay(100); // Anti-rebonds
  }
  Etat = digitalRead(BLUMT);
  if (Etat != BLUMTO) {
    BLUMTO = Etat;
    if (Etat) {
      for (byte i =  8 ; i < (16); i++) {
        pixels.setPixelColor(i, pixels.Color(0, 128, 0));
      }
      pixels.show();
    }
    else {
      for (byte i =  8 ; i < (16); i++) {
        pixels.setPixelColor(i, pixels.Color(0, 0, 0));
      }
      pixels.show();
    }
    delay(100); // Anti-rebonds
  }
}

void tempo(long duree) {
  // Temporisateur
  TEMPO = false;
  timer.setTimeout(duree, timertemp);
}
void timertemp() {
  TEMPO = true;
}


void Btempo() {
  // Temporisation appui long sur le bouton
  TBOUTON = false;
  timer.setTimeout(BAPPUILONG, timerBouton);
}
void timerBouton() {
  if (Bclef) TBOUTON = true;
}

//int OldEtape = 9999;
void ssd1306Info() {
  // Affiche l'étape en cours et l'état des E/S. Si étape 100, affiches d'autres infos TODO (heure ? T°/H% ?)
  //if (ETAPE != OldEtape) {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextSize(3);
  display.println(ETAPE);
  display.setTextSize(2);
  //display.setCursor(0,18);
  display.println(TelPark ? "Park" : "Non P.");
  //tm.displayText("        ");
  //if (ETAPE != 100) tm.displayIntNum(ETAPE, 0);
  //   OldEtape = ETAPE;
  // }
  // Affiche l'état de l'abri
  //Abri ouvert,  Abri fermé, Portes ouvertes,  Alim12V,  Alim télescope, Alim moteur,  Commande moteur,  Park
  //int LedState = 256 * (AbriFerme + AbriOuvert * 2 + PortesOuvert * 4 + Alim12VStatus * 8 + AlimTelStatus * 16 + MoteurStatus * 32 + !digitalRead(MOTEUR) * 64 + TelPark * 128);
  display.setTextSize(1);
  if (AbriOuvert) display.print("AO ");
  if (AbriFerme) display.print("AF ");
  if (!digitalRead(Po1)) display.print("P1 ");
  if (!digitalRead(Po2)) display.print("P2 ");
  if (Alim12VStatus) display.print("12V ");
  if (MoteurStatus) display.print("M ");
  if (!digitalRead(MOTEUR)) display.print("*");
  //display.print("AO AF P1 P2 12V M * ");



  //if (ETAPE != 100) tm.setLEDs(LedState); else tm.setLEDs(0);
  //Serial.println(tmButton);
  //delay(500);
  display.display();
}

void grafARU() {
  switch (ETARU) {
    case 0:
      if (Baru || CMDARU) {  // Arret d'urgence
        // Mise à zéro de toutes les sorties
        //Stop12V;
        digitalWrite(ALIMTEL, HIGH);
        digitalWrite(ALIMMOT, HIGH);
        digitalWrite(MOTEUR, HIGH);
        digitalWrite(P11, HIGH);
        digitalWrite(P12, HIGH);
        digitalWrite(P21, HIGH);
        digitalWrite(P22, HIGH);
        digitalWrite(ALIMMOT, MOTOFF); // Coupure alimentation moteur abri
        // Blocage du graphe principal
        ETAPE = 999;
        ETARU = 1;
        INIT = false;
        SURV = false;
      }
      break;
    case 1:
      if (!CMDARU) ETARU = 2;
      else if (CMDARU) ETARU = 10;
      break;
    case 2:
      if (!Baru) {
        Start12V;
        delay(3000);
        //OuvreP1;
        //tempo(INTERVALLEPORTES);
        ETARU = 0;
        ETAPE = 0;
      }
      break;
    case 10:
      CMDARU = false;
      if (Baru) ETARU = 2;
      break;
  }

}

void grafSurv() {
  if (SURV) {
    switch (ETASURV) {
      case 0:
        if ((FERM && !TelPark) + (DEPL && (!TelPark || !PortesOuvert))) ETASURV = 1;
        else if (!DEPL && !FERM && (!AbriOuvert && ! AbriFerme) && INIT) ETASURV = 1;
        break;
      case 1:
        tempo(300);
        ETASURV = 2;
        break;
      case 2:
        if ((FERM && TelPark) || (DEPL && PortesOuvert && TelPark)) ETASURV = 0;
        else {
          CMDARU = true;
          ETASURV = 0;
        }
    }
  }
}

void grafPrincipal() {
  switch (ETAPE) {
    case 0: //
      INIT = false;
      SURV = true;
      if (PortesOuvert) StartMot;
      if ((AbriOuvert && !AbriFerme) || (AbriFerme && !AbriOuvert)) ETAPE = 2;
      else if ((!AbriOuvert && !AbriFerme) || (AbriOuvert && AbriFerme)) {
        ETARET = 1;
        ETAPE = 200;
      }
      break;
    case 1:
      if (TelPark && !AbriOuvert && !AbriFerme && PortesOuvert) {
        ETARET = 2;
        ETAPE = 300;
      }
      else if ((!TelPark) || (AbriOuvert && AbriFerme)) CMDARU = true;
      break;
    case 2: // Abri dans une position inconnue
      INIT = true;
      if (AbriFerme) ETAPE = 3;
      else if (AbriOuvert) ETAPE = 30;
      break;
    case 3:
      ETAPE = 100;
      SURV = false;
      break;
    case 30:
      StartTel;
      ETAPE = 100;
      break;
    case 100:
      if (Bclef || tmButton == 128) {
        Bmem = true;
        AUTO = false;
        Btempo();
        ETAPE = 102;
      }
      else if (BoutonOpenState || BoutonCloseState) {
        AUTO = true;
        ETAPE = 102;
        TBOUTON = false;
      }
      else if (BoutonStopState) CMDARU = true;
      /*
        else if (cmdIndi) {
        AUTO = true;
        OuvreAbri
        FermeAbri
        }
      */
      break;
    //-----------------------------------------
    case 102: // Abri fermé, portes fermées
      if (AbriFerme && !PortesOuvert && (Bmem || BoutonOpenState)) {
        BoutonOpenState = false;
        Bmem = false;
        ETAPE = 104;
      }
      // TODO Abri ouvert, portes ouvertes, commande Indi "ouvre" -> 150
      else if (AbriFerme && PortesOuvert && (Bmem || BoutonOpenState)) {
        Bmem = false;
        BoutonOpenState = false;
        ETAPE = 120;
      }
      // TODO Abri ouvert, télescope non parqué, fermeture -> Park télescope  (160)
      else if (AbriOuvert && TelPark && (Bmem || BoutonCloseState)) {
        Bmem = false;
        BoutonCloseState = false;
        ETAPE = 130;
      }
      // TODO Portes fermées ou Abri ouvert et commande ouvre -> Retour à l'étape 100
      // Abri fermé, portes ouvertes
      else if (!AbriOuvert && ! AbriFerme && PortesOuvert && Bmem) ETAPE = 140;
      break;
    //-----------------------------------------
    case 104:
      ETAPE = 200;
      ETARET = 105;
      break;
    case 105:
      if (PortesOuvert && TBOUTON) ETAPE = 100;
      else if (PortesOuvert && TelPark) {
        Bmem = false;
        ETAPE = 110;
      }
      break;
    case 110:
      ETAPE = 300;
      ETARET = 100;
      break;
    case 120:
      tempo(BAPPUILONG + 500);
      ETAPE = 121;
      break;
    case 121:
      if (TEMPO && !TBOUTON) ETAPE = 110;
      else if (TEMPO && TBOUTON) ETAPE = 131;
      break;
    case 130:
      ETAPE = 300;
      ETARET = 131;
      break;
    case 131:
      if (AbriFerme) ETAPE = 132;
      break;
    case 132:
      FERM = true;
      FermeP2;
      tempo(INTERVALLEPORTES);
      ETAPE = 133;
      break;
    case 133:
      if (TEMPO) ETAPE = 134;
      break;
    case 134:
      FermeP1;
      StopMot;
      tempo(DELAIPORTES);
      ETAPE = 135;
      break;
    case 135:
      if (TEMPO) ETAPE = 136;
      break;
    case 136:
      FERM = false;
      SURV = false;
      //Stop12V;
      ETAPE = 100;
      break;
    // Macro Ouvre portes
    case 200:
      if (PortesOuvert) ETAPE = 222; //201;
      else if (!PortesOuvert) ETAPE = 220;
      break;
    case 220:
      OuvreP1;
      tempo(INTERVALLEPORTES);
      ETAPE = 221;
      break;
    case 221:
      if (TEMPO) ETAPE = 222;
      break;
    case 222:
      StartMot;
      OuvreP2;
      if (AUTO) tempo(DELAIPORTES);
      ETAPE = 223;
      break;
    case 223:
      if (PortesOuvert && !AUTO) ETAPE = ETARET;
      else if (PortesOuvert && AUTO && TEMPO) ETAPE = ETARET;
      break;

    // Deplace abri
    case 300:
      if (PortesOuvert && TelPark && (AbriOuvert || AbriFerme)) ETAPE = 301;
      else if (!PortesOuvert || !TelPark) ETAPE = 308;
      else if (!AbriOuvert && !AbriFerme && PortesOuvert && TelPark) ETAPE = 350;
      break;
    case 301:
      countM = 0;
      StopTel;
      DEPL = 1;
      if (MoteurStatus) ETAPE = 304;
      else if (!MoteurStatus) ETAPE = 302;
      break;
    case 302:
      StartMot;
      tempo(DELAIMOTEUR);
      ETAPE = 303;
      break;
    case 303:
      if (TEMPO) ETAPE = 304;
      break;
    case 304:
      digitalWrite(MOTEUR, LOW);
      tempo(IMPMOT);
      ETAPE = 305;
      break;
    case 305:
      if (TEMPO) {
        digitalWrite(MOTEUR, HIGH);
        countM++;
        tempo(5000);
        ETAPE = 306;
      }
      break;
    case 306:
      if (TEMPO && countM > 4 && (AbriOuvert || AbriFerme)) ETAPE = 308;
      else if (TEMPO && !AbriOuvert && !AbriFerme) {
        if (AUTO) tempo(DELAIABRI);
        ETAPE = 307;
      }
      else if (TEMPO && (AbriOuvert || AbriFerme) && countM < 5) ETAPE = 304;
      break;
    case 307:
      if ((AbriOuvert || AbriFerme) && ((AUTO && TEMPO) || !AUTO)) ETAPE = 308;
      break;
    case 308:
      DEPL = 0;
      // AlimMot=0
      if (AbriFerme) ETAPE = ETARET;
      else if (AbriOuvert) ETAPE = 309;
      break;
    case 309:
      StartTel;
      ETAPE = ETARET;
      break;
    case 350:
      tempo(DELAIMOTEUR);
      ETAPE = 351;
      break;
    case 351:
      if (TEMPO) ETAPE = 304;
      break;
  }
}
