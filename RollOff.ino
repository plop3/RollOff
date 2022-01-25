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
#define TPSARRET          600000L // Temps avant l'arret de l'alimentation 12V

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
#define ALIM12V A3    // (R2) Alimentation 12V abri
#define ALIMTEL A4    // Alimentation du télescope (relais externe)
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
#define AO      49      // Capteur abri ouvert
#define AF      48      // Capteur abri fermé
#define Po1     24      // Capteur porte 1 ouverte
#define Po2     25      // Capteur porte 2 ouverte
#define PARK	  A5      // Entrée Park: Etat du telescope 0: non parqué, 1: parqué
#define PLUIE   A6      // Capteur de pluie
// Boutons
#define BCLEF   A12     // Bouton à clef d'ouverture/fermeture des portes (pos 1 & 2)
#define BNOIR  	A7 	    // Bouton noir	
#define BVERT   34      // Bouton vert
#define BROUGE  46      // Bouton rouge
#define BARU    22      // Bouton d'arret d'urgence
#define BLUMT   A11     // Bouton d'éclairage de la table (rouge)  Interrupteur double
#define BLUMI   A10     // Bouton d'éclairage de l'abri   (rouge)

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
#define PortesOuvert  (!dRead(Po1) && !dRead(Po2)) // && Alim12VStatus)
#define Porte1Ouvert  (!dRead(Po1))
#define Porte2Ouvert  (!dRead(Po2))
#define AbriFerme     (!dRead(AF))
#define AbriOuvert    (!dRead(AO))
#define Stop12V       digitalWrite(ALIM12V, RON)
#define Start12V      digitalWrite(ALIM12V, ROFF)
#define StopTel       digitalWrite(ALIMTEL, LOW)
#define StartTel      digitalWrite(ALIMTEL, HIGH)
#define CmdMotOff     digitalWrite(CMDMOT, ROFF)
#define CmdMotOn      digitalWrite(CMDMOT, RON)
#define MotOn 	  	  digitalWrite(ALIMMOT, RON)
#define MotOff 	      digitalWrite(ALIMMOT, ROFF)
#define OuvreP1       digitalWrite(P12,LOW);digitalWrite(P11,HIGH)
#define OuvreP2       digitalWrite(P22,LOW);digitalWrite(P21,HIGH)
#define FermeP1       digitalWrite(P11,LOW);digitalWrite(P12,HIGH)
#define FermeP2       digitalWrite(P21,LOW);digitalWrite(P22,HIGH)
#define Bclef         !dRead(BCLEF)
#define Bnoir         !dRead(BNOIR)
#define Bvert         !dRead(BVERT)
#define Brouge        !dRead(BROUGE)
#define MoteurStatus  (dRead(ALIMMOT) == RON) // Alimentation du moteur abri
#define Status12V     (dRead(ALIM12V) == RON) // Alimentation 12V
#define Park 		      true //dRead(PARK) //TODO 
#define Pluie         !dRead(A6)
#define Baru          !dRead(BARU)


/**********************/
/* VARIABLES GLOBALES */
/**********************/
int cmd = 0;                        // Commande a réaliser (0: pas de commande)
bool BLUMTO;                        // Dernier etat du bouton d'éclairage table
bool BLUMIO;                        // Dernier etat du bouton d'éclairage intérieur
bool BappuiLong = false;            // Appui long sur le bouton vert ou la clef
bool MotReady = false;              // Moteur abri pret (DELAIMOTEUR)

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
  Serial.begin(BAUDRATE);  		// Port Indi

  delay(1000);                // Attente d'initialisation du matériel

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
  pinMode(CMDMOT, OUTPUT);CmdMotOff;  // Coupure de la commande du moteur de déplacement
  MotOff; pinMode(ALIMMOT,OUTPUT);    // Coupure du moteur d'abri
  Start12V;pinMode(ALIM12V, OUTPUT);  // Mise en marche de l'alimentation 12V

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
  pinMode(BNOIR, INPUT_PULLUP); // Bouton noir
  pinMode(BLUMI, INPUT_PULLUP); // Bouton éclairage intérieur
  pinMode(BLUMT, INPUT_PULLUP); // Bouton éclairage table
  pinMode(PARK, INPUT);         // TODO Mettre à INPUT
  pinMode(PLUIE,INPUT_PULLUP);  // Capteur de pluie
  
  delay(1000);  // Attente d'initialisation des capteurs

  // Arret d'urgence appuyé, on attend
  while(Baru) {};
  
  // Portes fermées: moteur abri OFF, sinon ON
  if (PortesOuvert) startMot();
  // Abri ouvert, télescope alimenté
  if (AbriOuvert) StartTel;

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
  readIndi();                   // Lecture Indi (1: commandes actives)
  readBoutons();                // Lecture des boutons
  readARU();                    // Test du bouton ARU
  eclairages();                 // Gestion des éclairages
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
  }
}

bool deplaceAbri() {
  // Déplace l'abri
  // Conditions: télescope parqué, portes ouvertes, pas de déplacement en cours
  if (!PortesOuvert || !Park) 
  {
    return false;
  }
  if (!MoteurStatus) startMot();      // Mise en marche du moteur de l'abri si besoin
  barre(0, 128);
  while(!MotReady) attend(1000,1);
  CmdMotOn;
  delay(IMPMOT);
  CmdMotOff;
  attend(DELAIABRI,1);
  // Attend le positionnement de l'abri ou l'annulation du déplacement
  barre(0, 0);
  for (int i=0;i<10;i++) {
    if (AbriOuvert || AbriFerme) return true; // Attente des capteurs
    // Délai supplémentaire
    attend(1000,1);
   }
  return false;
}

bool ouvreAbri() {
	// Ouvre l'abri
  // Conditions: 
  if (AbriOuvert) return true;  	// Abri déjà ouvert
  // Gestion appui long (clef et bouton vert)
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
  if (AbriFerme) return true; // Abri déjà fermé
  if (!PortesOuvert) return false;
  if (deplaceAbri() && AbriFerme) {
    if (fermePortes()) {
      return true;
    }
  }
  return false;
}

bool ouvrePortes() {
	// Ouvre les portes
  if (PortesOuvert) {
    // Portes ouvertes
    OuvreP1;
    OuvreP2;
    return true;
  }
  else {
    OuvreP1;
    attend(INTERVALLEPORTES,0);
		OuvreP2;
    attend(DELAIPORTES,0);
    for (int i=0;i<10;i++) {
      if (PortesOuvert) return true;
      // Délai supplémentaire   
      attend(1000,0);
    }
  }
    return false;
}

bool fermePortes() {
	// Ferme les portes
  if (!AbriFerme) {
    return false;
  }
  FermeP2;
  attend(INTERVALLEPORTES,1);
  FermeP1;
	attend(DELAIPORTES,1);
  MotOff;
  StopTel;
  return true;
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
			cmd=1;
		}
		// Fermeture abri
		else if (AbriOuvert) {
			cmd=2;
		}
  }
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
}

void stopAbri() {
	// Arret de l'abri (pas en arret d'urgence)
		// Arret du moteur abri
		MotOff;
		// Arret des portes
		digitalWrite(P11, LOW);
		digitalWrite(P12, LOW);
		digitalWrite(P21, LOW);
		digitalWrite(P22, LOW);
		// Plus d'action en cours
    // Reset de l'arduino
    pinMode(RESETMEGA, OUTPUT);
}

void survDepl() {
  // Surveillance du déplacement intempestif de l'abri
  if (!AbriOuvert && !AbriFerme) stopAbri();
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
  // Park du télescope
  // Park du télescope par Pin
  digitalWrite(SPARK, HIGH);
  delay(300);
  digitalWrite(SPARK, LOW);
  // On attend 3mn max que le télescope soit parqué
  unsigned long tpsdebut = millis();
  unsigned long tpsact;
  do {
    tpsact = millis();
  } while ((tpsact - tpsdebut) < TPSPARK && Park);
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
  // Alimente le moteur de l'abri
  MotOn;
  MotReady=false;
  timer.setTimeout(DELAIMOTEUR,tpsInitMoteur);
}

//---------- Fonctions Timer ----------

void appuiLong()
{
    if (Bclef || Bnoir) BappuiLong = true;
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
  MotReady=true;
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
                cmd=1;
            }
            // Prepare to CLOSE
            else if (strcmp(target, "CLOSE") == 0)			// Fermeture de l'abri
            {
                sendAck(value);
                timeMove = millis();
                cmd=2;
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
					        cmd=3;
                  sendAck(value);
                }
            }
            // Prepare for the Lock function
            else if (strcmp(target, "LOCK") == 0)			// Lock de l'abri
            {
      				sendAck(value);
            }

            // Prepare for the Auxiliary function
            else if (strcmp(target, "AUXSET") == 0)			// Bascule de la sorite auxiliaire
            {
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
  if ((AbriOuvert && PortesOuvert) || (AbriFerme && !PortesOuvert)) 
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
