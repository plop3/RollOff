/*
  Pilotage automatique de l'abri du telescope
  Serge CLAUS
  GPL V3
  Version 6.0
  22/10/2018-20/01/2022
  /*********************************/

/***********/
/* MODULES */
/***********/

/**************/
/* PARAMETRES */
/**************/
#define BAUDRATE 	        9600    // Vitesse du port série
#define RON HIGH       		        // Etat On pour les relais (HIGH, LOW)
#define ROFF !RON
#define DELAIPORTES       40000L  // Durée d'ouverture/fermeture des portes (40000L)
#define INTERVALLEPORTES  12000   // Intervalle entre la fermeture de la porte 1 et de la porte 2
#define DELAIABRI         22000L  // Durée de déplacement de l'abri (15000L)
#define DELAIMOTEUR       40000L  // Délai d'initialisation du moteur abri
#define IMPMOT            500     // Durée d'impulsion moteur
#define BAPPUILONG        3000    //Durée en ms pour un appui long sur le bouton
#define TPSPARK           180000  // Temps de park du télescope

/*****************/
/* PERIPHERIQUES */
/*****************/
// Timer
#include <SimpleTimer.h>
SimpleTimer timer;

// EEPROM
#include <EEPROM.h>

// LEDs neopixel
#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
#include <avr/power.h> // Required for 16 MHz Adafruit Trinket
#endif
#define LEDPIN 13
#define NBLEDS 24  // Nombre total de LEDs (3 barrettes de 8 LEDs)
Adafruit_NeoPixel pixels(NBLEDS, LEDPIN, NEO_GRB + NEO_KHZ800);
/*
   0-7: Eclairage table
   8-15:Eclairage intérieur
   16-23: Eclairage extérieur
*/

// Afficheur OLED SSD1306
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET     -1
#define SCREEN_ADDRESS 0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Serveurs Telnet
#include <SPI.h>
#include <Ethernet.h>
byte mac[] = {
  0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xEE };
IPAddress ip(192, 168, 0, 16);  
IPAddress myDns(192, 168, 0, 254);
IPAddress gateway(192, 168, 0, 254);
IPAddress subnet(255, 255, 255, 0);

EthernetServer server(9998);
EthernetServer server2(9999);
EthernetServer serverD(23);
EthernetClient client;      // Client Telnet 9999
EthernetClient client2;     // Client Telnet 9999
EthernetClient clientD;     // Client Telnet console de debug
EthernetClient clientO;     // Client OnStep 
IPAddress onstep(192, 168, 0, 15);

boolean alreadyConnected = false; 

/**************/
/* CONSTANTES */
/**************/
//----------Sorties ----------
#define ALIMAUX A0    // Sortie auxiliaire
#define CMDMOT  A1  	// (R1) Ouverture/fermeture abri Commande moteur de porte de garage
#define ALIMMOT A2  	// (R2) Alimentation 220V moteur abri
#define ALIM12V A3    // Alimentation 12V (non utilisée)
#define ALIMTEL A4    // Alimentation du télescope (relais externe)
#define PLUIE   A6    // Capteur de pluie
#define P11     3   	// (R5) LM298 1 porte 1
#define P12     5   	// (R6) LM298 2 porte 1
#define P21     6   	// (R7) LM293 3 porte 2
#define P22     7   	// (R8) LM298 4 porte 2
#define SPARK   8     // Sortie Park
#define RESETMEGA	A13 // Reset de l'arduino
#define LEDV    2     // LED verte du shield
#define LEDB    9     // LED bleue du shield
// APA106      13     // Définie plus haut

//---------- Entrées ----------
// Capteurs
#define AO  49       // Capteur abri ouvert
#define AF  48       // Capteur abri fermé
#define Po1 24       // Capteur porte 1 ouverte
#define Po2 25       // Capteur porte 2 ouverte
#define PARK	A5   // Entrée Park: Etat du telescope 0: non parqué, 1: parqué
// Boutons
#define BCLEF   A12    // Bouton à clef d'ouverture/fermeture des portes (pos 1 & 2)
#define BVERT  	34 	   // Bouton intérieur d'ouverture/fermeture
#define BROUGE 	46 	// Bouton de sélection
#define BNOIR  	A7 	// Bouton noir	
#define BARU 	22     // Bouton d'arret d'urgence
#define BLUMT   A11    // Bouton d'éclairage de la table (rouge)  Interrupteur double
#define BLUMI   A10    // Bouton d'éclairage de l'abri   (rouge)

//---------- RollOffIno ----------
//  Maximum length of messages = 63                                               *|
const char* ERROR1 = "The controller response message was too long";
const char* ERROR2 = "The controller failure message was too long";
const char* ERROR3 = "Command input request is too long";
const char* ERROR4 = "Invalid command syntax, both start and end tokens missing"; 
const char* ERROR5 = "Invalid command syntax, no start token found";
const char* ERROR6 = "Invalid command syntax, no end token found";
const char* ERROR7 = "Roof controller unable to parse command";
const char* ERROR8 = "Command must map to either set a relay or get a switch";
const char* ERROR9 = "Request not implemented in controller";
const char* ERROR10 = "Abort command ignored, roof already stationary";

const char* VERSION_ID = "V1.2-0";
#define MAX_RESPONSE 127
#define MAX_INPUT 45
#define MAX_MESSAGE 63
#define ROOF_OPEN_MILLI 120000L

/**********/
/* MACROS */
/**********/
#define PARKREMOTE    // Si défini: park par le réseau, sinon park par Pin
#define PortesOuvert  (!digitalRead(Po1) && !digitalRead(Po2)) // && Alim12VStatus)
#define Porte1Ouvert  (!digitalRead(Po1))
#define Porte2Ouvert  (!digitalRead(Po2))
#define AbriFerme     (!digitalRead(AF))
#define AbriOuvert    (!digitalRead(AO))
#define CmdMotOff     digitalWrite(CMDMOT, ROFF)
#define CmdMotOn      digitalWrite(CMDMOT, RON)
#define MotOn 	  	  digitalWrite(ALIMMOT, RON)
#define MotOff 	      digitalWrite(ALIMMOT, ROFF)
#define TelOn          digitalWrite(ALIMTEL, RON)
#define TelOff        digitalWrite(ALIMTEL, ROFF)
#define OuvreP1       digitalWrite(P12,LOW);digitalWrite(P11,HIGH)
#define OuvreP2       digitalWrite(P22,LOW);digitalWrite(P21,HIGH)
#define FermeP1       digitalWrite(P11,LOW);digitalWrite(P12,HIGH)
#define FermeP2       digitalWrite(P21,LOW);digitalWrite(P22,HIGH)
#define Bclef         !digitalRead(BCLEF)
#define Bvert         !digitalRead(BVERT)
#define Brouge        !digitalRead(BROUGE)
#define Bnoir         !digitalRead(BNOIR)
#define Baru 		  !digitalRead(BARU) 			// Arret d'urgence
#define MoteurStatus  (digitalRead(ALIMMOT) == RON) // Alimentation du moteur abri
#define Park 		  digitalRead(PARK)
#define Pluie     !digitalRead(A6)

/**********************/
/* VARIABLES GLOBALES */
/**********************/
bool PortesFerme = true;
bool DEPL = false;    		// Abri en cours de déplacement
bool FERM = false;    		// Portes en cours de fermeture
bool OUVR = false;			  // Portes en cours d'ouverture
bool REMOTE = true;       // Mode auto (distant), manuel (boutons)
bool MotAbriOk = false;   // Moteur abri prêt (allumé depuis plus de DELAIMOTEUR secondes)
bool BLUMTO = !digitalRead(BLUMT);  // Dernier etat du bouton d'éclairage table
bool BLUMIO = !digitalRead(BLUMI);  // Dernier etat du bouton d'éclairage intérieur
bool BappuiLong = false;  // Appui long sur le bouton vert ou la clef
bool Tempo1=false;        // Temporisation à usages multiples
String Message = "";      // Message affiché sur l'écran OLED
bool AbriStop=true;       // Etat de l'abri (start=false, stop=true)
bool LOCK=false;		      // Abri locké: Sauvegardé en EEPROM
bool AUX=false;           // Sortie auxiliaire

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
  Serial.begin(BAUDRATE);  		// Connexion à AstroPi (port Indi)
  
  // LEDs APA106
  pixels.begin();
  pixels.clear();
  pixels.show();
  barre(0, 0); // Extinction des barres de LEDs
  barre(1, 0);
  barre(2, 0);

  //SSD1306
  display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS);
  //display.display();
  display.clearDisplay();
  display.setTextSize(3);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.cp437(true);
  display.display();
  
  // Initialisation des relais
  CmdMotOff; pinMode(CMDMOT, OUTPUT); // Coupure de la commande du moteur de déplacement
  stopMot; pinMode(ALIMMOT,OUTPUT);
  // Initialisation du LM298
  digitalWrite(P11, LOW); pinMode(P11, OUTPUT);
  digitalWrite(P12, LOW); pinMode(P12, OUTPUT);
  digitalWrite(P21, LOW); pinMode(P21, OUTPUT);
  digitalWrite(P22, LOW); pinMode(P22, OUTPUT);
    
  // Activation des entrées (capteurs...)
  pinMode(AO, INPUT_PULLUP);    // Capteur abri ouvert
  pinMode(AF, INPUT_PULLUP);    // Capteur abri fermé
  pinMode(Po1, INPUT_PULLUP);   // Capteur porte ouverte 1
  pinMode(Po2, INPUT_PULLUP);   // Capteur porte ouverte 2 
  pinMode(BCLEF, INPUT_PULLUP); // Bouton à clef
  pinMode(BVERT, INPUT_PULLUP); // Bouton vert
  pinMode(BROUGE, INPUT_PULLUP);// Bouton rouge  
  pinMode(BNOIR, INPUT_PULLUP); // Bouton noir
  pinMode(BARU, INPUT_PULLUP);  // Bouton d'arret d'urgence
  pinMode(BLUMI, INPUT_PULLUP); // Bouton éclairage intérieur
  pinMode(BLUMT, INPUT_PULLUP); // Bouton éclairage table
  pinMode(PARK, INPUT);         // TODO Mettre à INPUT
  pinMode(PLUIE,INPUT_PULLUP);  // Capteur de pluie
  // Abri fermé: moteur abri OFF, sinon ON
  if (!PortesOuvert && AbriFerme) {
	  arretAbri();
  }
  else {
	startMot();pinMode(ALIMMOT, OUTPUT);  
  AbriStop=false;
	if (!PortesOuvert) PortesFerme=false;
  }

// Etat initial des boutons d'éclairage
BLUMIO=!digitalRead(BLUMI);
BLUMTO=!digitalRead(BLUMT);

  // Ethernet
  Ethernet.begin(mac, ip, myDns, gateway, subnet);

	// Restauration de l'état de l'abri
	bool alim;
	EEPROM.get(2, alim);
	digitalWrite(AUX, alim ? RON: ROFF);
	EEPROM.get(0, LOCK);
	if (LOCK !=1) LOCK=false;
}

/*********************/
/* BOUCLE PRINCIPALE */
/*********************/
void loop() {
  pool(); // fonctions périodiques
}

/*************/
/* FONCTIONS */
/*************/
bool deplaceAbri() {
  // Déplace l'abri
  // Conditions: télescope parqué, portes ouvertes, pas de déplacement en cours
  if (!PortesOuvert || !telPark() || DEPL) 
  {
    sendMsg("Dep con2");
    return false;
  }
  bool avant=AbriFerme; // Etat de l'abri avant déplacement
  bool posok=((AbriOuvert && !AbriFerme) || (!AbriOuvert && AbriFerme));
  sendMsg("Depl abri");
  barre(0, 128);
  CmdMotOn;
  delay(IMPMOT);
  CmdMotOff;
  DEPL = true;
  attend(DELAIABRI, DEPL);
  // Attand le positionnement de l'abri ou l'annulation du déplacement
  while (!AbriOuvert && !AbriFerme && DEPL) pool(); // Attente des capteurs
  DEPL = false;
  barre(0, 0);
  if ((avant==AbriFerme && posok) || (!posok && !AbriOuvert && !AbriFerme)) {
    sendMsg("Pb depl");
    return false;
  }
  return true;
  sendMsg("Fin depl");
}

void deplaceAbriInsecure() {
  // Déplace l'abri sans sécurité
  // Conditions: aucunes
  sendMsg("Depl abri");
  barre(0, 128);
  if (!MoteurStatus) startMot();
  while (!MotAbriOk) pool();
  CmdMotOn;
  delay(IMPMOT);
  CmdMotOff;
  DEPL = true;
  attend(DELAIABRI, DEPL);
  barre(0, 0);
  DEPL=false;
  sendMsg("Fin depl");
}

bool ouvreAbri() {
	// Ouvre l'abri
  // Conditions: Pas de déplacement, pas de lock, télescope parqué
  if (DEPL || LOCK || !telPark()) {
    sendMsg("Dep con1");
    return false; 			// Abri en cours de déplacement ou locké ou télescope non parqué
  }
  if (AbriOuvert) return true;  	// Abri déjà ouvert
  sendMsg("Ouv abri");
  if (!MoteurStatus) startMot();	// Démarrage du moteur abri
  // Gestion appui long (clef et bouton vert)
  if (PortesOuvert) {
	  // Ferme les portes
	  fermePortes();
  }
  else {
	  if (ouvrePortes()) {
		  if (!BappuiLong) {
			  while (!MotAbriOk) pool(); 	// Attend que le moteur soit initialisé
			  if (deplaceAbri() && AbriOuvert) {
          TelOn;                    // Mise en marche du télescope
          sendMsg("Abri ouv");
			    return true;
			  }
		  }
		  else {
			  // Appui long: ouvre seulement les portes
        sendMsg("Portes O");
			  return false;
		  }
	  }
    else {
      sendMsg("Err O por");
	    return false;
    }
  }
  return true;
}

bool fermeAbri() {
  if (DEPL || (LOCK && Pluie)) {
    sendMsg("Cond FA");
    return false; // Abri en cours de déplacement
  }
  // Ferme l'abri
  // Si les portes sont fermées et l'abri ouvert on passe temporairement en auto pour ouvrir les portes
  bool mode=REMOTE;
  REMOTE=true;
  if (!PortesOuvert) ouvrePortes();
  REMOTE=mode;
  if (AbriFerme) return true; // Abri déjà fermé
  sendMsg("Ferm abri");
  if (!telPark()) {
    // Park du télescope
    parkTelescope();
  }
  if (!telPark()) {
    sendMsg("Err park");
    return false;
  }
  if (deplaceAbri() && AbriFerme) {
    if (fermePortes()) {
      sendMsg("Abri ferm");
      return true;
    }
  }
  sendMsg("Abri NF");
  return false;
}

void fermePortesInsecure() {
  // Ferme les portes sans vérification
  sendMsg("F portes");
  REMOTE=true;
  FERM = true;
  sendMsg("F P2");
  FermeP2;
  attend(INTERVALLEPORTES, FERM);
   sendMsg("F P1");
  FermeP1;
  FERM=false;
  sendMsg("Portes F");
}

bool ouvrePortes() {
	// Ouvre les portes
  AbriStop=false;
	sendMsg("Ouv portes");
  if (PortesOuvert) {
    // Portes ouvertes
    OuvreP1;
    OuvreP2;
  }
  else {
	OUVR=true;
  sendMsg("Ouv P1");
    OuvreP1;
    attend(INTERVALLEPORTES, OUVR);
	if (OUVR) {
    sendMsg("Ouv P2");
		OuvreP2;
    if (REMOTE) {
		  attend(DELAIPORTES, OUVR);
		  while (!PortesOuvert && OUVR) pool();
    }
    else {
      // Appui long, on termine (ouverture des portes sans conditions)
      if (!BappuiLong) {
        // Appui court, on attend les capteurs de porte ou le délai d'ouverture
        Tempo1=false;
        timer.setTimeout(DELAIPORTES,tempo1);
        while (!PortesOuvert && OUVR && !Tempo1) pool();
      }
    }
    OUVR=false;
    }
  }
  if (PortesOuvert) {
    PortesFerme = false;
    sendMsg("Portes O");
    return true;
  }
  sendMsg("Pb PO");
  return false;
}

bool fermePortes() {
	// Ferme les portes
  if (!AbriFerme || AbriOuvert) {
    sendMsg("Cond FP");
    return false;
  }
  sendMsg("Ferm P");
  FERM = true;
  sendMsg("Ferm P2");
  FermeP2;
  attend(INTERVALLEPORTES, FERM);
  if (FERM) {
    sendMsg("Ferm P1");
	  FermeP1;
	  attend(DELAIPORTES,FERM);
	  if (FERM) {
		  FERM = false;
		  PortesFerme = true;
		  stopMot();
      sendMsg("Portes F");
      timer.setTimeout(3000,arretAbri);
      return true;
	  }
  }
  stopMot();	// Arret du moteur de l'abri
  sendMsg("Err FP");
  return false;
}

void attend(unsigned long delai, bool &sortie) 
// Attend un délai déterminé ou une condition se sortie (passage à false pour sortir sans délai)
{
  unsigned long previousMillis = millis();
  unsigned long currentMillis;
  do {
    currentMillis = millis();
    pool();
  } while ((currentMillis - previousMillis <= delai) && sortie);
}

void readBoutons() {
  // Lecture des boutons de l'abri
  // Déplacement en cours, on ne fait rien
  if (DEPL || OUVR || FERM) return;
  if (!Bnoir) {
	  // Touches ou clef mode principal
	  if (Bclef || Bvert) {
        sendMsg("B vert");
		    BappuiLong=false;
        // Temporisation pour appui long
        timer.setTimeout(3000,appuiLong);
		  // Déplacement de l'abri
			if (!AbriOuvert) {
				// Ouverture abri (abri non fermé)
        REMOTE=false;
				ouvreAbri();
			}
			// Fermeture abri
			else if (AbriOuvert) {
        REMOTE=false;
				fermeAbri();
			}
		}
		else if (Brouge && Porte1Ouvert) {
		  // Ouverture/fermeture porte 2 (pour réglage panneau à flat)
      sendMsg("B rouge");
		  if (Porte2Ouvert) {FermeP2;sendMsg("Ferm P2");}  else {OuvreP2;sendMsg("Ouv P2");}
		}
	}
  else {
	  // Touche en mode secondaire
	  if (Brouge) {
		  // Déplacement de l'abri inconditionnel 
      REMOTE=false;
		  deplaceAbriInsecure();
	  }
	  if (Bvert) {
		  // Fermeture des portes inconditionnel
		  // /!\ Fonctionnement sans tests.
      REMOTE=false;
		  fermePortesInsecure();
	  }
  }
  if (Brouge || Bvert || Bclef) delay(200);
  while (Bvert || Brouge || Bclef) {timer.run();}	// Attente boutons relachés
}

void pool() {
  // Timers
  timer.run();  
  // Fonctions périodiques
  readBoutons();
  readIndi();     // Lecture des commandes Indi
  // Arret d'urgence
    if (Baru) ARU();
	// Surveillance
	surv();
	// Gestion des éclairages
	eclairages();
  // Afficheur OLED
  ssd1306Info();  // Info sur l'écran OLED
  // LEDs du shield
  gereLeds();
  // Sécurité météo
  meteo();
  // Console de debug
  clientD = serverD.available();
}

void startMot() {
	// Mise en marche du moteur de l'abri
	sendMsg("Start M");
	MotOn;
  timer.setTimeout(DELAIMOTEUR, initMotOk);
}

void stopMot() {
	// Arret du moteur de l'abri
	sendMsg("Stop M");
	MotOff;
	MotAbriOk=false;
}

void stopAbri() {
	// Arret de l'abri (pas en arret d'urgence)
		sendMsg("Stop abri");
		// Arret du moteur abri
		stopMot();
		// Arret des portes
		digitalWrite(P11, LOW);
		digitalWrite(P12, LOW);
		digitalWrite(P21, LOW);
		digitalWrite(P22, LOW);
		// Plus d'action en cours
		OUVR=false;
		FERM=false;
		DEPL=false;
}

void ARU() {
	// Arret d'urgence
	sendMsg("ARU");
	stopAbri();
	while (!Baru);      // Appel externe à ARU
    delay(500); 		// Anti rebonds
    while (Baru);
    pinMode(RESETMEGA, OUTPUT); // Reset de l'arduino
}

void sendMsg(String message)
{
	// Envoi des messages
  // Sortie sur l'écran Oled
  Message=message;
  ssd1306Info();
  // Sortie sur la console telnet
  clientD = serverD.available();
  if (clientD) {
    if (clientD.available() > 0) {
      // read the bytes incoming from the client:
      serverD.println(message);
    }
  }
}

void gereLeds() {
  // Gestion des LEDs du shield
  // LED verte: télescope parqué
  digitalWrite(LEDV, Park);
  // LED bleu: déplacement en cours
  digitalWrite(LEDB, (DEPL || OUVR || FERM));
}

void surv() {
  if (REMOTE) return;
  // Déplacement et le télescope perd le park
  if (DEPL && !telPark()) {
    // Arret d'urgence
    sendMsg("Dépl park");
    stopAbri();
  }
  // Fermeture des portes et le télescope perd le park
  if (FERM && !telPark()) {
    // Arret d'urgence
    sendMsg("F P park");
    stopAbri();
  }
  // Déplacement intenpestif de l'abri sauf si portes ouvertes et télescope parqué (pour réglages...)
  if (!DEPL && !AbriOuvert && !AbriFerme && (!telPark() || !PortesOuvert)) {
    sendMsg("Err depl");
    stopAbri;
  }
}

bool telPark() {
	// Etat du télescope park/unpark
	return Park;
}

void eclairages() {
  // Gestion des écairages
  bool Etat = digitalRead(BLUMI);
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
  Etat = !digitalRead(BLUMT);
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

void ssd1306Info() {
  // Affiche les infos sur l'état du télescope
  if (AbriStop) return;
  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextSize(2);
  display.println(Message);
  display.setTextSize(2);
  //display.setCursor(0,18);
  display.println(telPark() ? "Park" : "Non P.");
  // Affiche l'état de l'abri
  //Abri ouvert,  Abri fermé, Portes ouvertes, Alim télescope, Alim moteur,  Commande moteur
  display.setTextSize(1);
    if (AbriOuvert) display.print("AO ");
  if (AbriFerme) display.print("AF ");
  if (!digitalRead(Po1)) display.print("P1 ");
  if (!digitalRead(Po2)) display.print("P2 ");
  // if (Alim12VStatus) display.print("12V ");
  if (MoteurStatus) display.print("M ");
  if (digitalRead(CMDMOT)==RON) display.print("m ");
  if (DEPL || FERM || OUVR) display.print("*");
  display.display();
}

bool parkTelescope() {
  // Park du télescope
  #ifdef PARKREMOTE
  // Park du télescope par connexion réseau
  if (!clientO.connect(onstep, 9999))  {
    sendMsg("Err park");
    return false;
  }
  clientO.print(":Q#");    // Arret du mouvement
  bool vrai=true;
  attend(2200,vrai);
  clientO.print(":Te#");  // Tracking On)
  attend(2200,vrai);
  clientO.print(":hP#");   // Park du télescope)  
  clientO.stop();
  #else
  // Park du télescope par Pin
  digitalWrite(SPARK, HIGH);
  delay(300);
  digitalWrite(SPARK, LOW);
  #endif
  // On attend 3mn max que le télescope soit parqué
  unsigned long tpsdebut = millis();
  unsigned long tpsact;
  do {
    tpsact = millis();
  } while ((tpsact - tpsdebut) < TPSPARK && !telPark());
  attend(5000,vrai);  // Attente du park complet
    if (!telPark()) {
      sendMsg("Err park");
      return (false);
    }
    sendMsg("Park OK");
  return true;
}

void arretAbri() {
  sendMsg("Abri arret");
  // Arrête l'abri
  // Eteind l'écran
  display.clearDisplay();
  display.display();
  // Eteind les lumières
  barre(0,0);
  barre(1,0);
  barre(2,0);
  // Arret des alimentations
  AbriStop=true;
  PortesFerme=true;
  stopMot;
  TelOff;
}


bool meteo() {
  // Sécurité météo: pluie, (vent...)
  if (Pluie) {
    sendMsg("Pluie");
    /* TODO
      - Passer le capteur pluie à ON pour HASS
      - Lecture analogique du capteur ? (voir WeatherRadio)
    */
    fermeAbri();
  }
}

//---------- Fonctions Timer ----------
void initMotOk()
// Attend l'initialisation du moteur de l'abri
{
    sendMsg("Init M OK");
    // Moteur abri pret
    MotAbriOk = true;
}


void appuiLong()
{
    if (Bclef || Bvert)
    {
        BappuiLong = true;
        sendMsg("App long");
    }
}

void tempo1() {
  // Temporisation à usages multiples
  sendMsg("Tempo1");
  Tempo1=true;
}

/************************/
/* FONCTIONS ROLLOFFINO */
/************************/

void sendData(char* buffer) {
	// Envoi les données sur le port USB
  switch (TypeCon) {
    case 0:
    	Serial.println(buffer);
	    Serial.flush();
      break;
    case 1:
      client.println(buffer);
      client.flush();
      break;
    case 2:
      client2.println(buffer);
      client2.flush();
      client2.stop();
      break;
  }
}

void readIndi()
{   
  client2=server2.available();
  if (client2.available()>0) {
    TypeCon=2;
    readData();
  }
  if (Serial.available())
  {
    TypeCon=0;
    readData();
  }
  client = server.available();
  if (client.available()>0 ) {
    TypeCon=1;
    readData();
  }  
}

bool parseCommand() // (command:target:value)
{
  char inpBuf[MAX_INPUT+1];
  memset(inpBuf, 0, sizeof(inpBuf));
	if (TypeCon==0) Serial.readBytesUntil(')',inpBuf,MAX_INPUT);
  if (TypeCon==1) client.readBytesUntil(')',inpBuf,MAX_INPUT);
  if (TypeCon==2) client2.readBytesUntil(')',inpBuf,MAX_INPUT);
	strcpy(command, strtok(inpBuf, "(:"));
  strcpy(target, strtok(NULL, ":"));
  strcpy(value, strtok(NULL, ")"));
  if ((strlen(command) > 2) && strlen(target) && strlen(value))
  {
      return true;
  }
  sendNak(ERROR7);
  return false;
}

void readData()
{
    // Confirm there is input available, read and parse it.
    if (parseCommand())
    {
        const char *error = ERROR8;
        // On initial connection return the version
        if (strcmp(command, "CON") == 0)
        {
            strcpy(value, VERSION_ID); // Can be seen on host to confirm what is running
            sendAck(value);
			return;
        }
        // Map the general input command term to the local action
        // SET: OPEN, CLOSE, ABORT, LOCK, AUXSET
        else if (strcmp(command, "SET") == 0)	
        {
            // Prepare to OPEN
            if (strcmp(target, "OPEN") == 0)				// Ouverture de l'abri
            {
                sendAck(value);
                timeMove = millis();
                REMOTE=true;
                ouvreAbri();
            }
            // Prepare to CLOSE
            else if (strcmp(target, "CLOSE") == 0)			// Fermeture de l'abri
            {
                sendAck(value);
                timeMove = millis();
                REMOTE=true;
                fermeAbri();
            }
            else if (strcmp(target, "OPENDOORS") == 0)			// Fermeture de l'abri
            {
                sendAck(value);
                timeMove = millis();
                REMOTE=true;
                ouvrePortes();
            }
            else if (strcmp(target, "CLOSEDOORS") == 0)			// Fermeture de l'abri
            {
                sendAck(value);
                timeMove = millis();
                REMOTE=true;
                if (AbriFerme) fermePortes();
            }
            else if (strcmp(target, "OPENDOOR1") == 0)			// Fermeture de l'abri
            {
                sendAck(value);
                timeMove = millis();
                REMOTE=true;
                OuvreP1;
            }
            else if (strcmp(target, "CLOSEDOOR1") == 0)			// Fermeture de l'abri
            {
                sendAck(value);
                timeMove = millis();
                REMOTE=true;
                if (PortesOuvert) FermeP1;
            }
            // Prepare to ABORT								// Arret de l'abri
            else if (strcmp(target, "ABORT") == 0)
            {
                // Test whether or not to Abort
                if (!isStopAllowed())
                {
                    error = ERROR10;
                }
                else
                {
					        stopAbri();
                  sendAck(value);
                }
            }
            // Prepare for the Lock function
            else if (strcmp(target, "LOCK") == 0)			// Lock de l'abri
            {
       				LOCK=(value=="ON");
               sendMsg(value);
      				EEPROM.put(0, LOCK);
    	  			// Sauvegarde en EEprom
      				sendAck(value);
            }

            // Prepare for the Auxiliary function
            else if (strcmp(target, "AUXSET") == 0)			// Bascule de la sorite auxiliaire
            {
              AUX=(value=="ON");
              digitalWrite(ALIMAUX,AUX ? RON: ROFF);
      				EEPROM.put(0, AUX);
    	  			// Sauvegarde en EEprom
      				sendAck(value);
            }
						else if (strcmp(target, "RESET") == 0)			// RESET Arduino
            {
                sendAck(value);
                delay(300);
                pinMode(RESETMEGA, OUTPUT);
            }
            else sendNak(error);
        }
        // Handle requests to obtain the status of switches
        // GET: OPENED, CLOSED, LOCKED, AUXSTATE
        else if (strcmp(command, "GET") == 0)
        {
            if (strcmp(target, "OPENED") == 0)
				requestReceived(AbriOuvert);
            else if (strcmp(target, "CLOSED") == 0)
                requestReceived(AbriFerme);
            else if (strcmp(target, "LOCKED") == 0)
                requestReceived(0);
            else if (strcmp(target, "AUXSTATE") == 0)
				requestReceived(0);
            else sendNak(error);
        }
		else {
            sendNak(error); // Unknown input or Abort command was rejected
        }
    }   // end command parsed
}       // end Serial input found

void sendNak(const char *errorMsg)
{
    char buffer[MAX_RESPONSE];
    if (strlen(errorMsg) > MAX_MESSAGE)
        sendNak(ERROR2);
    else
    {
        strcpy(buffer, "(NAK:ERROR:");
        strcat(buffer, value);
        strcat(buffer, ":");
        strcat(buffer, errorMsg);
        strcat(buffer, ")");
		sendData(buffer);
    }
}

void sendAck(char* val)
{
  char response [MAX_RESPONSE];
  if (strlen(val) > MAX_MESSAGE)
    sendNak(ERROR1);
  else
  {  
    strcpy(response, "(ACK:");
    strcat(response, target);
    strcat(response, ":");
    strcat(response, val);
    strcat(response, ")");
	sendData(response);
  }
}

void requestReceived(int state)
{
	strcpy(value, state ? "ON":"OFF");
	sendAck(value);            // Send result of reading pin associated with "target" 
}

bool isStopAllowed()
{
  unsigned long timeNow = millis();
   // If the roof is either fully opened or fully closed, ignore the request.
  if ((AbriOuvert && PortesOuvert) || (AbriFerme && PortesFerme)) 
  {
    return false;
  }
  // If time since last open or close request is longer than the time for the roof travel return false
  if ((timeNow - timeMove) >= ROOF_OPEN_MILLI)
  {
    return false;
  }
  else
  // Stop will be attempted
  {
    return true;
  }
}

