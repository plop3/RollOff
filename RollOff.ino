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
#define DEBUG		true	   // Déboguage sur le port série	
#define BAUDRATE 	9600 	   // Vitesse du port série
#define RON HIGH       		   // Etat On pour les relais (HIGH, LOW)
#define ROFF !RON
#define DELAIPORTES       10000L //40000L     // Durée d'ouverture/fermeture des portes (40000L)
#define INTERVALLEPORTES  4000L  //12000 // Intervalle entre la fermeture de la porte 1 et de la porte 2
#define DELAIABRI         10000L //22000L       // Durée de déplacement de l'abri (15000L)
#define IMPMOT 500             // Durée d'impulsion moteur
#define BAPPUILONG 3000        //Durée en ms pour un appui long sur le bouton

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
#define LEDPIN 13
#define NBLEDS 24  // Nombre total de LEDs (3 barrettes de 8 LEDs)
Adafruit_NeoPixel pixels(NBLEDS, LEDPIN, NEO_GRB + NEO_KHZ800);
/*
   0-7: Eclairage table
   8-15:Eclairage intérieur
   16-23: Eclairage extérieur
*/

/**************/
/* CONSTANTES */
/**************/
//----------Sorties ----------
#define CMDMOT  A1  	// (R1) Ouverture/fermeture abri Commande moteur de porte de garage
#define ALIMMOT A2  	// (R2) Alimentation 220V moteur abri
#define P11     3   	// (R5) LM298 1 porte 1
#define P12     5   	// (R6) LM298 2 porte 1
#define P21     6   	// (R7) LM293 3 porte 2
#define P22     7   	// (R8) LM298 4 porte 2
#define RESETMEGA	A13 // Reset de l'arduino

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
#define PortesOuvert  (!digitalRead(Po1) && !digitalRead(Po2)) // && Alim12VStatus)
#define Porte1Ouvert  (!digitalRead(Po1))
#define Porte2Ouvert  (!digitalRead(Po2))
#define AbriFerme     (!digitalRead(AF))
#define AbriOuvert    (!digitalRead(AO))
#define CmdMotOff     digitalWrite(CMDMOT, ROFF)
#define CmdMotOn      digitalWrite(CMDMOT, RON)
#define MotOn 	  	  digitalWrite(ALIMMOT, RON)
#define MotOff 	      digitalWrite(ALIMMOT, ROFF)
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

/**********************/
/* VARIABLES GLOBALES */
/**********************/
bool PortesFerme = true;
bool DEPL = false;    		// Abri en cours de déplacement
bool FERM = false;    		// Portes en cours de fermeture
bool OUVR = false;			// Portes en cours d'ouverture
bool AUTO = true;       // Mode auto (distant), manuel (boutons)
bool MotAbriOk = false;   	// Moteur abri prêt (allumé depuis plus de DELAIMOTEUR secondes)
bool BLUMTO = !digitalRead(BLUMT);  // Dernier etat du bouton d'éclairage table
bool BLUMIO = !digitalRead(BLUMI);  // Dernier etat du bouton d'éclairage intérieur
bool DebugMode=DEBUG;		// Mode débug console	
bool BappuiLong = false;  	// Appui long sur le bouton vert ou la clef
bool Tempo1=false;        // Temporisation à usages multiples

//---------- RollOffIno ----------
const int cLen = 15;
const int tLen = 15;
const int vLen = MAX_RESPONSE;
char command[cLen+1];
char value[vLen+1];
char target[tLen+1];
unsigned long timeMove = 0;

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
  
  // Initialisation des relais
  CmdMotOff; pinMode(ALIMMOT, OUTPUT); // Coupure de la commande du moteur de déplacement
  // Initialisation du LM298
  digitalWrite(P11, LOW); pinMode(P11, OUTPUT);
  digitalWrite(P12, LOW); pinMode(P12, OUTPUT);
  digitalWrite(P21, LOW); pinMode(P21, OUTPUT);
  digitalWrite(P22, LOW); pinMode(P22, OUTPUT);
  
  // Activation des entrées (capteurs...)
  pinMode(AO, INPUT_PULLUP);
  pinMode(AF, INPUT_PULLUP);
  pinMode(Po1, INPUT_PULLUP);
  pinMode(Po2, INPUT_PULLUP);
  pinMode(BCLEF, INPUT_PULLUP);
  pinMode(BVERT, INPUT_PULLUP);
  pinMode(BROUGE, INPUT_PULLUP);
  pinMode(BNOIR, INPUT_PULLUP);
  pinMode(BARU, INPUT_PULLUP);
  pinMode(PARK, INPUT_PULLUP); // TODO Mettre à INPUT
  // Abri fermé: moteur abri OFF, sinon ON
  if (!PortesOuvert) {
	stopMot();pinMode(ALIMMOT, OUTPUT);
  }
  else {
	startMot();pinMode(ALIMMOT, OUTPUT);  
	PortesFerme=false;
  }
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
  if (!PortesOuvert) return false;
  sendMsg("Déplace abri");
  barre(0, 128);
  CmdMotOn;
  delay(IMPMOT);
  CmdMotOff;
  DEPL = true;
  attend(DELAIABRI, DEPL);
  // Attand le positionnement de l'abri ou l'annulation du déplacement
  while (!AbriOuvert && !AbriFerme && DEPL) pool; // Attente des capteurs
  DEPL = false;
  barre(0, 0);
  return true;
  sendMsg("Fin déplace abri");
}

bool deplaceAbriInsecure() {
  sendMsg("Déplace abri (insécure)");
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
  sendMsg("Fin déplace abri (insécure)");
}
bool ouvreAbri() {
	// Ouvre l'abri
  if (DEPL) return false; 			// Abri en cours de déplacement
  if (AbriOuvert) return true;  	// Abri déjà ouvert
  sendMsg("Ouvre abri");
  if (!MoteurStatus) startMot();	// Démarrage du moteur abri
  // Gestion appui long (clef et bouton vert)
  if (PortesOuvert) {
	  // Ferme les portes
	  fermePortes();
  }
  else {
	  if (ouvrePortes()) {
		  if (!BappuiLong) {
        DEPL=true;                  // bloque les nouvelles commandes le temps que MotAbriOK=true
			  while (!MotAbriOk) pool(); 	// Attend que le moteur soit initialisé
			  if (deplaceAbri() && AbriOuvert) {
          sendMsg("Abri ouvert");
			    return true;
			  }
		  }
		  else {
			  // Appui long: ouvre seulement les portes
			  return false;
		  }
	  }
	  return false;
  }
}

bool fermeAbri() {
  if (DEPL) return false; // Abri en cours de déplacement
  // Ferme l'abri
  // Si les portes sont fermées et l'abri ouvert on passe temporairement en auto pour ouvrir les portes
  bool mode=AUTO;
  AUTO=true;
  if (!PortesOuvert) ouvrePortes();
  AUTO=mode;
  if (AbriFerme) return true; // Abri déjà fermé
  sendMsg("Ferme abri");
  if (deplaceAbri() && AbriFerme) {
    if (fermePortes()) {
      sendMsg("Abri fermé");
      return true;
    }
  }
  sendMsg("Abri non fermé");
  return false;
}

void fermePortesInsecure() {
  sendMsg("Ferme portes (insécure)");
  AUTO=true;
  FERM = true;
  sendMsg("Ferme porte 2");
  FermeP2;
  attend(INTERVALLEPORTES, FERM);
   sendMsg("Ferme porte 1");
  FermeP1;
  FERM=false;
  sendMsg("Portes fermées (insécure)");
}

bool ouvrePortes() {
	// Ouvre les portes
	sendMsg("Ouvre portes");
  if (PortesOuvert) {
    // Portes ouvertes
    OuvreP1;
    OuvreP2;
  }
  else {
	OUVR=true;
  sendMsg("Ouvre porte 1");
    OuvreP1;
    attend(INTERVALLEPORTES, OUVR);
	if (OUVR) {
    sendMsg("Ouvre porte 2");
		OuvreP2;
    if (AUTO) {
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
    sendMsg("Portes ouvertes");
    return true;
  }
  return false;
}

bool fermePortes() {
	// Ferme les portes
  if (!AbriFerme || AbriOuvert) return false;
  sendMsg("Ferme portes");
  FERM = true;
  sendMsg("Ferme porte 2");
  FermeP2;
  attend(INTERVALLEPORTES, FERM);
  if (FERM) {
    sendMsg("Ferme porte 1");
	FermeP1;
	attend(DELAIPORTES,FERM);
	if (FERM) {
		FERM = false;
		PortesFerme = true;
		stopMot();
    sendMsg("Portes fermées");
		return true;
	}
  }
  stopMot();	// Arret du moteur de l'abri
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
        sendMsg("Bouton déplace abri");
		    BappuiLong=false;
        timer.setTimeout(3000,appuiLong);
		  // Temporisation pour appui long
		  // Déplacement de l'abri
			if (!AbriOuvert) {
				// Ouverture abri (abri non fermé)
        AUTO=false;
				ouvreAbri();
			}
			// Fermeture abri
			else if (AbriOuvert) {
        AUTO=false;
				fermeAbri();
			}
		}
		else if (Brouge && Porte1Ouvert) {
		  // Ouverture/fermeture porte 2 (pour réglage panneau à flat)
      sendMsg("Bouton déplace porte 2");
		  if (Porte2Ouvert) {FermeP2;sendMsg("Ferme porte 2");}  else {OuvreP2;sendMsg("Ouvre porte 2");}
		}
	}
  else {
	  // Touche en mode secondaire
	  if (Brouge) {
		  // Déplacement de l'abri inconditionnel (vérifie que les portes soient ouvertes)
		  // TODO à terminer
      AUTO=false;
		  deplaceAbriInsecure();
	  }
	  if (Bvert) {
		  // Fermeture des portes inconditionnel
		  // /!\ Fonctionnement sans tests.
		  // TODO à terminer
      AUTO=false;
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
}

void startMot() {
	// Mise en marche du moteur de l'abri
	sendMsg("Start moteur");
	MotOn;
  // FIXME timer.setTimeout(DELAIMOTEUR, initMotOk);
  timer.setTimeout(3000, initMotOk);
}

void stopMot() {
	// Arret du moteur de l'abri
	sendMsg("Stop moteur");
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
	sendMsg("Arret d'urgence");
	stopAbri();
	while (!Baru);      // Appel externe à ARU
    delay(500); 		// Anti rebonds
    while (Baru);
    pinMode(RESETMEGA, OUTPUT); // Reset de l'arduino
}

void sendMsg(String message)
{
	// Envoi des messages
	if (DebugMode) {
		Serial.println(message);
	}
}

void surv() {
  if (AUTO) return;
  // Déplacement et le télescope perd le park
  if (DEPL && !telPark()) {
    // Arret d'urgence
    sendMsg("Déplacement et le télescope perd le park");
    stopAbri();
  }
  // Fermeture des portes et le télescope perd le park
  if (FERM && !telPark()) {
    // Arret d'urgence
    sendMsg("Fermeture des portes et le télescope perd le park");
    stopAbri();
  }
  // Déplacement intenpestif de l'abri sauf si portes ouvertes et télescope parqué (pour réglages...)
  if (!DEPL && !AbriOuvert && !AbriFerme && (!telPark() || !PortesOuvert)) {
    sendMsg("Déplacement intenpestif de l'abri sauf si portes ouvertes et télescope parqué");
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

//---------- Fonctions Timer ----------
void initMotOk()
// Attend l'initialisation du moteur de l'abri
{
    sendMsg("Init moteur OK");
    // Moteur abri pret
    MotAbriOk = true;
}


void appuiLong()
{
    if (Bclef || Bvert)
    {
        BappuiLong = true;
        sendMsg("Appui long");
    }
}

void tempo1() {
  // Temporisation à usages multiples
  Tempo1=true;
}

/************************/
/* FONCTIONS ROLLOFFINO */
/************************/

void sendData(char* buffer) {
	// Envoi les données sur le port USB
	Serial.println(buffer);
	Serial.flush();
}

void readIndi()
{
    if (Serial.available())
    {
        readData();
    }
}

bool parseCommand() // (command:target:value)
{
    char inpBuf[MAX_INPUT+1];
    memset(inpBuf, 0, sizeof(inpBuf));
	Serial.readBytesUntil(')',inpBuf,MAX_INPUT);
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
                AUTO=true;
                ouvreAbri();
            }
            // Prepare to CLOSE
            else if (strcmp(target, "CLOSE") == 0)			// Fermeture de l'abri
            {
                sendAck(value);
                timeMove = millis();
                AUTO=true;
                fermeAbri();
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
            else if (strcmp(target, "LOCK") == 0)			// TODO Lock de l'abri
            {
				sendAck(value);
            }

            // Prepare for the Auxiliary function
            else if (strcmp(target, "AUXSET") == 0)			// TODO Bascule de la sorite auxiliaire
            {
				sendAck(value);
            }
			else if (strcmp(target, "DEBUG") == 0)			// Mode débug (ON/OFF)
			{
				sendMsg("Debug mode");
                sendMsg(value);
				DebugMode=(value=="ON");
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

