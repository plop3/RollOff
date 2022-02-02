/*
  Pilotage automatique de l'abri du telescope
  Serge CLAUS
  GPL V3
  Version 7.1
  22/10/2018-02/02/2022
  /*********************************/

/***********/
/* MODULES */
/***********/
#include "infos.h";		// Informations de connexion
#include "Config.h";		// Fichier de configuration
#include "Pinmap.h";		// Pins Arduino Mega
#include "Constants.h";		// Constantes

#include "RollOffIno.h";	// Fonctions rollOffIno

/*****************/
/* PERIPHERIQUES */
/*****************/
// Timer
#include <SimpleTimer.h>
SimpleTimer timer;

// LEDs neopixel
#include <Adafruit_NeoPixel.h>

#ifdef __AVR__
#include <avr/power.h> // Required for 16 MHz Adafruit Trinket
#endif

// APA106
Adafruit_NeoPixel pixels(NBLEDS, LEDPIN, NEO_GRB + NEO_KHZ800);
/*
   0-7:   Eclairage extérieur
   8-15:  Eclairage intérieur
   16-23: Eclairage table
*/

/**********************/
/* VARIABLES GLOBALES */
/**********************/
int cmd = 0;                        // Commande a réaliser (0: pas de commande)
bool BLUMTO;                        // Dernier etat du bouton d'éclairage table
bool BLUMIO;                        // Dernier etat du bouton d'éclairage intérieur
bool BappuiLong = false;            // Appui long sur le bouton vert ou la clef
bool MotReady = false;              // Moteur abri pret (DELAIMOTEUR)
bool Remote = true;                 // Commande distante (+ de sécurité)
bool LOCK=false;                    // Abri locké

//---------- RollOffIno ----------
const int cLen = 15;
const int tLen = 15;
const int vLen = MAX_RESPONSE;
char command[cLen+1];
char value[vLen+1];
char target[tLen+1];
unsigned long timeMove = 0;
int TypeCon=0;  // 0: USB, 1: Telnet 9999, 2: Telnet 9998

/*********/
/* SETUP */
/*********/
void setup() {
  // Initialisation des ports série
  Serial.begin(BAUDRATE);  		// Port Indi

  delay(200);                // Attente d'initialisation du matériel

  // LEDs shield
  pinMode(LEDV, OUTPUT);
  pinMode(LEDB,OUTPUT);

  // LEDs APA106
  pixels.begin();
  pixels.clear();
  pixels.show();
  barre(0, 0); // Extinction des barres de LEDs
  barre(1, 0);
  barre(2, 0);
  BLUMTO = !dRead(BLUMT);  // Dernier etat du bouton d'éclairage table
  BLUMIO = !dRead(BLUMI);  // Dernier etat du bouton d'éclairage intérieur

  // Initialisation des relais
  pinMode(CMDMOT, OUTPUT);       // Coupure de la commande du moteur de déplacement
  pinMode(ALIMMOT,OUTPUT);       // Coupure du moteur d'abri
  pinMode(ALIM12V, OUTPUT);      // Mise en marche de l'alimentation 12V
  pinMode(ALIMTEL, OUTPUT);      // Alimentation télescope
  pinMode(SPARK, INPUT);         // Sortie demande de Park (collecteur ouvert)

  // Initialisation du LM298
  pinMode(P11, OUTPUT);
  pinMode(P12, OUTPUT);
  pinMode(P21, OUTPUT);
  pinMode(P22, OUTPUT);
    
  // Activation des entrées (capteurs...)
  pinMode(AO, INPUT_PULLUP);    // Capteur abri ouvert
  pinMode(AF, INPUT_PULLUP);    // Capteur abri fermé
  pinMode(Po1, INPUT_PULLUP);   // Capteur porte ouverte 1
  pinMode(Po2, INPUT_PULLUP);   // Capteur porte ouverte 2 
  pinMode(BCLEF, INPUT_PULLUP); // Bouton à clef
  pinMode(BNOIR, INPUT_PULLUP); // Bouton noir
  pinMode(BARU, INPUT_PULLUP);  // Bouton ARU
  pinMode(BVERT, INPUT_PULLUP); // Bouton vert
  pinMode(BROUGE, INPUT_PULLUP); // Bouton rouge
  pinMode(BLUMI, INPUT_PULLUP); // Bouton éclairage intérieur
  pinMode(BLUMT, INPUT_PULLUP); // Bouton éclairage table
  pinMode(PARK, INPUT);         // TODO Mettre à INPUT
  pinMode(PLUIE,INPUT_PULLUP);  // Capteur de pluie
  
  sendMsg("Deb init");

  //delay(1000);  // Attente d'initialisation des capteurs

  // Arret d'urgence appuyé, on attend
  while(Baru) {};
 
  // TODO récupération de l'état Lock de l'abri

  // Portes fermées: moteur abri OFF, sinon ON
  if (PortesOuvert) {
    startMot();
    sendMsg("Start M");
  }
  // Abri ouvert, télescope alimenté
  if (AbriOuvert) {
    StartTel;
    sendMsg("Start tel");
  }

  // Etat initial des boutons d'éclairage
  BLUMIO=!dRead(BLUMI);
  BLUMTO=!dRead(BLUMT);
}

/*********************/
/* BOUCLE PRINCIPALE */
/*********************/
void loop() {

  // Attente des commandes
  cmd=0;                        // Initialisation des commandes
  if (AbriOuvert && AbriFerme) stopAbri();  // Problème de capteurs
  readBoutons();                // Lecture des boutons
  
  pool();
  
  if (cmd) traiteCommande(cmd); // Traitement de la commande recue
  if (MoteurStatus) survDepl(); // Surveillance déplacement intempestif de l'abri
  if (AbriOuvert || PortesOuvert) meteo();  // Sécurité météo
}

/*************/
/* FONCTIONS */
/*************/

void traiteCommande(int commande) {
  // Traitement des commandes
  switch (commande){
  case 1: // Ouvre abri
    ouvreAbri();
    break;
  case 2: // Ferme abri
    fermeAbri();
    break;
  case 3: // Arret de l'abri
    stopAbri();
    break;
  case 4:
    lockAbri();
    break;
  case 5:
    bougePorte2();
    break;
  }
}

bool deplaceAbri() {
  // Déplace l'abri
  // Conditions: télescope parqué, portes ouvertes, pas de déplacement en cours
  sendMsg("Dep abri");
  if (!PortesOuvert) 
  {
    sendMsg("Err depl");
    return false;
  }
  if (!Park) {
    if (!parkTelescope()) {
      sendMsg("Err park");
      return false;
    }
  }
  if (!MoteurStatus) startMot();      // Mise en marche du moteur de l'abri si besoin
  barre(0, 128);
  while(!MotReady) attend(1000,1);
  sendMsg("Start dep");
  CmdMotOn;
  delay(IMPMOT);
  CmdMotOff;
  attend(DELAIABRI,1);
  // Attend le positionnement de l'abri ou l'annulation du déplacement
  barre(0, 0);
  for (int i=0;i<10;i++) {
    if (AbriOuvert || AbriFerme) {
        return true; // Attente des capteurs
    }
    // Délai supplémentaire
    attend(1000,1);
   }
  return false;
}

bool ouvreAbri() {
	// Ouvre l'abri
  // Conditions: 
  if (AbriOuvert) return true;  	// Abri déjà ouvert
  sendMsg("Ouv abri");
  // Gestion appui long (clef et bouton vert)
  if (PortesOuvert && BappuiLong) {
    fermePortes();
    return false;
  }
  if (!MoteurStatus) startMot();      // Mise en marche du moteur de l'abri
  if (ouvrePortes()) {
	  if (!BappuiLong) {
		  if (deplaceAbri() && AbriOuvert) {
        StartTel;                    // Mise en marche du télescope
		    return true;
		  }
	  }
	  else return false;              // Appui long: ouvre seulement les portes
	}
  else return false;                // Portes non ouvertes
  return true;
}

bool fermeAbri() {
  // Ferme l'abri
  sendMsg("F abri");
  if (AbriFerme) return true; // Abri déjà fermé
  if (!PortesOuvert) {
      if (!ouvrePortes()) return false;
  }
  if (deplaceAbri() && AbriFerme) {
    if (fermePortes()) {
      return true;
    }
  }
  return false;
}

bool ouvrePortes() {
	// Ouvre les portes
  sendMsg("O portes");
  if (PortesOuvert && !Remote) {
    // Portes ouvertes
    OuvreP1;
    OuvreP2;
    return true;
  }
  else {
    OuvreP1;
    attend(INTERVALLEPORTES,0);
		OuvreP2;
    if (Remote) {
      attend(DELAIPORTES,0);
      for (int i=0;i<10;i++) {
        if (PortesOuvert) return true;
        // Délai supplémentaire   
        attend(1000,0);
      }
    }
    else {
      for (int i=0;i<DELAIPORTES+10;i++) {
        if (PortesOuvert) return true;
        // Délai supplémentaire   
        attend(1000,0);
      }
    }
  }
    return false;
}

bool fermePortes() {
	// Ferme les portes
  sendMsg("F portes");
  if (!AbriFerme) {
    return false;
  }
  FermeP2;
  attend(INTERVALLEPORTES,1);
  FermeP1;
	attend(DELAIPORTES,1);
  abriOff();
  return true;
}

void bougePorte2() {
  // Ouvre/ferme la porte 1
  if (Porte2Ouvert) {
    FermeP2;
  }
  else {
    OuvreP2;
  }
}

void attend(unsigned long delai, bool secu) 
// Attend un délai déterminé 
// secu: true -> surveillance du park
{
  unsigned long previousMillis = millis();
  unsigned long currentMillis;
  do {
    currentMillis = millis();
    if (secu) survPark();
    pool();
  } while (currentMillis - previousMillis <= delai);
}

void readBoutons() {
  // Lecture des boutons de l'abri
  // Déplacement en cours, on ne fait rien
  // Touches ou clef mode principal
	if (Bclef || Bvert) {
    BappuiLong=false;
    // Temporisation pour appui long
    timer.setTimeout(BAPPUILONG,appuiLong);
	  // Déplacement de l'abri
		if (!AbriOuvert) {
			// Ouverture abri (abri non fermé)
			Remote=false;
            cmd=1;
		}
		// Fermeture abri
		else if (AbriOuvert) {
            Remote=false;
			cmd=2;
		}
  }
  if (Brouge) {
    // Ouverture/fermeture de la porte1
    Remote=false;
    cmd=5;
  }
  while(Bclef || Bvert || Brouge) {timer.run();} // Attente de relachement des boutons
}

void readARU() {
  // Gestion du bouton d'arret d'urgence
  if (Baru) stopAbri();
}

void pool() {
  // Timers
  timer.run();  
  // Fonctions périodiques
  readIndi();     // Lecture des commandes Indi
  // Gestion ARU
  readARU();
  // Gestion des éclairages
  eclairages();
  // LEDs shield
  gereLeds();
}

void stopAbri() {
  // Arret de l'abri (pas en arret d'urgence)
  sendMsg("ARU");
  // Arret du moteur abri
  MotOff;
  // Arret des portes
  digitalWrite(P11, LOW);
  digitalWrite(P12, LOW);
  digitalWrite(P21, LOW);
  digitalWrite(P22, LOW);
  // Plus d'action en cours
  if (!Baru) {
    while(!Baru) {
      barre(0,128);
      barre(1,128);
      delay(500);
      barre(0,0);
      barre(1,0);
      delay(500);
    }	
  }
  delay(500);
  while(Baru) {
    barre(0,128);
    barre(1,128);
    delay(500);
    barre(0,0);
    barre(1,0);
    delay(500);
  }
    // Reset de l'arduino
    pinMode(RESETMEGA, OUTPUT);
}

void survDepl() {
  // Surveillance du déplacement intempestif de l'abri
  if (!AbriOuvert && !AbriFerme && (!PortesOuvert || !Park)) stopAbri();
}

void survPark() {
  // Surveillance du park télescope
  if (!Park) stopAbri();
}

void eclairages() {
  // Gestion des écairages
  bool Etat = dRead(BLUMI);
  if (Etat != BLUMIO) {
    BLUMIO = Etat;
    if (Etat) {
      barre(1, 128);
    }
    else {
      barre(1, 0);
    }
    delay(100); // Anti-rebonds
  }
  Etat = !dRead(BLUMT);
  if (Etat != BLUMTO) {
    BLUMTO = Etat;
    if (Etat) {
      barre(2, 128);
    }
    else {
      barre(2, 0);
    }
    delay(100); // Anti-rebonds
  }
}

void barre(byte barreau, byte valeur) {
  for (byte i = 8 * barreau; i < (8 + 8 * barreau); i++) {
    pixels.setPixelColor(i, pixels.Color(valeur, 0, 0));
  }
  pixels.show();
}

bool parkTelescope() {
  sendMsg("Park T");
  // Park du télescope
  // Park du télescope par Pin
  pinMode(SPARK, OUTPUT);
    delay(300);
  pinMode(SPARK,INPUT);
  // On attend 3mn max que le télescope soit parqué
  unsigned long tpsdebut = millis();
  do {
  } while (((millis() - tpsdebut) < TPSPARK) && !Park);
  attend(5000,0);  // Attente du park complet
    if (!Park) {
      return (false);
    }
  return true;
}

bool meteo() {
  // Sécurité météo: pluie, (vent...)
  if (Pluie) {
   fermeAbri();
  }
}

void startMot() {
  sendMsg("Mot On");
  // Alimente le moteur de l'abri
  MotOn;
  MotReady=false;
  timer.setTimeout(DELAIMOTEUR,tpsInitMoteur);
}

void abriOff() {
  sendMsg("Abri Off");
  // Mise en veille de l'abri
  MotOff;
  StopTel; 
  // Eteint les éclairages
  barre(0,0);
  barre(1,0);
  barre(2,0);
}

void lockAbri() {
  sendMsg("Lock abri");
  // Bloque les commandes d'ouverture de l'abri
  // TODO
}

void sendMsg(char *message) {
  // Messages de debug (USB, Telnet, Ecran Oled)
  if (DEBUG) Serial.println(message);
}

void gereLeds() {
  // Gestion des LEDs du shield
  // LED verte: télescope parqué
  digitalWrite(LEDV, Park);
  // LED bleu: A DEFINIR
  digitalWrite(LEDB, !digitalRead(ALIMTEL));
}

//---------- Fonctions Timer ----------

void appuiLong()
{
    if (Bclef || Bvert) BappuiLong = true;
}

int dRead(int pin) {
  // Lecture des entrées "anti-parasitées"
  bool a=digitalRead(pin);
  delay(20);
  if (digitalRead(pin)==a) return a;
  delay(20);
  return digitalRead(pin);
}

void tpsInitMoteur() {
  sendMsg("Mot ready");
  MotReady=true;
}
