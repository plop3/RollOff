/*
  Pilotage automatique de l'abri du telescope
  Serge CLAUS
  GPL V3
  Version 5.0
  22/10/2018-16/01/2022
*/

/* TODO
Clavier i2c

*/

/***********/
/* MODULES */
/***********/

/**************/
/* PARAMETRES */
/**************/
#define BAUDRATE 	9600 	   // Vitesse du port série
#define RON HIGH       		   // Etat On pour les relais (HIGH, LOW)
#define ROFF !RON
#define DELAIPORTES 40000L     // Durée d'ouverture/fermeture des portes (40000L)
#define DELAIMOTEUR 30000L     // Durée d'initialisation du moteur (40000L)
#define DELAIABRI 22000L       // Durée de déplacement de l'abri (15000L)
#define INTERVALLEPORTES 12000 // Intervalle entre la fermeture de la porte 1 et de la porte 2
#define IMPMOT 500             // Durée d'impulsion moteur
#define BAPPUILONG 3000        //Durée en ms pour un appui long sur le bouton
#define PARKONSTEP false       // 0: Park par ESPServer, 1: Park par entrée OnStepX
#define CAPTEURPLUIE false     // Capteur de pluie présent (true)

/*****************/
/* PERIPHERIQUES */
/*****************/
// Timer
#include <SimpleTimer.h>
SimpleTimer timer;

// Serveur Telnet
#include <SPI.h>
#include <Ethernet.h>
byte mac[] = {
    0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};
IPAddress ip(192, 168, 0, 16);
IPAddress myDns(192, 168, 0, 254);
IPAddress gateway(192, 168, 0, 254);
IPAddress subnet(255, 255, 255, 0);

EthernetServer server(23);		// Console de debug
EthernetClient client = 0;
boolean alreadyConnected = false; // whether or not the client was connected previously

// EEPROM
#include <EEPROM.h>

/**************/
/* CONSTANTES */
/**************/
//----------Sorties ----------
#define ALIMAUX A0	  // (R0)Alimentation auxiliaire
#define CMDMOT  A1    // (R1) Ouverture/fermeture abri Commande moteur de porte de garage
#define ALIMMOT A2    // (R2) Alimentation 220V moteur abri
#define ALIM12V A3	  // (R3) Alimentation 12V (non utilisé actuelllement)	
#define ALIMTEL A4    // Alimentation télescope    Relais dans le boitier OnStep)
#define P11 3         // (R5) LM298 1 porte 1
#define P12 5         // (R6) LM298 2 porte 1
#define P21 6         // (R7) LM293 3 porte 2
#define P22 7         // (R8) LM298 4 porte 2
#define RESETMEGA A13 // Reset de l'arduino
#define SPARK   	8 // Sortie ordre de park vers OnStepX
// LEDs
#define LEDV		2 // Led verte (shield) 
#define LEDB		9 // Led bleue (shield)		
#define LEDAPA106  13 // LEDs APA106	
#define NBLEDS 	   24 // Nombre total de LEDs (3 barrettes de 8 LEDs)

//---------- Entrées ----------
// Capteurs
#define AO 		49  // Capteur abri ouvert
#define AF 		48  // Capteur abri fermé
#define PO1 	24 	// Capteur porte 1 ouverte
#define PO2 	25 	// Capteur porte 2 ouverte
#define PARK  	A5	// Entrée Park: Etat du telescope 0: non parqué, 1: parqué
#define PLUIE 	A6  // Capteur de pluie
// Boutons
#define BCLEF  A12	// Bouton à clef d'ouverture/fermeture
#define BVERT  	34 	// Bouton intérieur d'ouverture/fermeture
#define BROUGE 	46 	// Bouton de sélection
#define BNOIR  	A7 	// Bouton noir	
#define BARU 	22  // Bouton d'arret d'urgence
#define BLUMI  A10  // Bouton d'éclairage de l'abri   (rouge)
#define BLUMT  A11	// Bouton d'éclairage de la table (rouge)  Interrupteur double

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
#define CmdMotOff digitalWrite(CMDMOT, ROFF) // Commande déplacement abri
#define CmdMotOn digitalWrite(CMDMOT, RON)
#define PortesOuvert (!digitalRead(PO1) && !digitalRead(PO2)) // Portes ouvertes
#define AbriFerme (!digitalRead(AF))                          // Abri fermé
#define AbriOuvert (!digitalRead(AO))                         // Abri ouvert
#define OuvreP1  digitalWrite(P12, LOW);digitalWrite(P11, HIGH)
#define OuvreP2  digitalWrite(P22, LOW);digitalWrite(P21, HIGH)
#define FermeP1  digitalWrite(P11, LOW);digitalWrite(P12, HIGH)
#define FermeP2  digitalWrite(P21, LOW);digitalWrite(P22, HIGH)
#define Baru !digitalRead(BARU) // Arret d'urgence
#define StartMot digitalWrite(ALIMMOT, RON)
#define StopMot digitalWrite(ALIMMOT, ROFF)
#define MoteurStatus (digitalRead(ALIMMOT) == RON) // Alimentation du moteur abri
#define Park digitalRead(PARK)

/**********************/
/* VARIABLES GLOBALES */
/**********************/
bool AbriCours = false;   // Demande d'ouverture en cours (attend que les portes soient ouvertes)
bool ArretCours = false;  // Demande d'arret de l'abri en cours (attend que l'abri soit fermé)
bool PorteCours = false;  // Demande de déplacement des portes en cours
bool MotAbriOk = false;   // Moteur abri prêt (allumé depuis plus de DELAIMOTEUR secondes)
bool TempoDepl = false;   // Temporisation déplacement abri
bool TempoPortes = false; // Temporisation ouverture portes
bool REMOTE = true;       // Mode manuel/automatique (Manuel: commandes par boutons)
bool AUTO=true;			  // Mode automatique (surveillance active)
bool BappuiLong = false;  // Appui long sur le bouton vert ou la clef
bool LOCK=false;		  // Abri locké: Sauvegardé en EEPROM

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
void setup()
{
    // Connexion à AstroPi (port Indi)
    Serial.begin(BAUDRATE);
    // Coupure de la commande du moteur de déplacement
    CmdMotOff;
    pinMode(CMDMOT, OUTPUT);
    // Coupure alimentation moteur abri
    StopMot;
    pinMode(ALIMMOT, OUTPUT);

    // Coupure des vérins de portes
    digitalWrite(P11, LOW);pinMode(P11, OUTPUT);
    digitalWrite(P12, LOW);pinMode(P12, OUTPUT);
    digitalWrite(P21, LOW);pinMode(P21, OUTPUT);
    digitalWrite(P22, LOW);pinMode(P22, OUTPUT);
	digitalWrite(SPARK, ROFF);pinMode(SPARK, OUTPUT);
	digitalWrite(ALIMAUX, ROFF);pinMode(ALIMAUX, OUTPUT);
	digitalWrite(ALIMMOT, ROFF);pinMode(ALIMMOT, OUTPUT);
	digitalWrite(CMDMOT, ROFF);pinMode(P22, OUTPUT);
	digitalWrite(ALIM12V, ROFF);pinMode(P22, OUTPUT);
	digitalWrite(ALIMTEL, ROFF);pinMode(P22, OUTPUT);
	digitalWrite(LEDV, LOW);pinMode(LEDV, OUTPUT);
	digitalWrite(LEDB, LOW);pinMode(LEDB, OUTPUT);
    // Activation des entrées (capteurs...)
    pinMode(AO, INPUT_PULLUP);
    pinMode(AF, INPUT_PULLUP);
    pinMode(PO1, INPUT_PULLUP);
    pinMode(PO2, INPUT_PULLUP);
    pinMode(BCLEF, INPUT_PULLUP);
    pinMode(BVERT, INPUT_PULLUP);
    pinMode(BROUGE, INPUT_PULLUP);
	pinMode(BNOIR, INPUT_PULLUP);
    pinMode(BLUMI, INPUT_PULLUP);
	pinMode(BLUMT, INPUT_PULLUP);
	pinMode(BARU, INPUT_PULLUP);
	pinMode(PARK, INPUT_PULLUP); // TODO Mettre à INPUT
	pinMode(PLUIE, INPUT);
	
	
	// Restauration de l'état de l'abri
	bool alim;
	EEPROM.get(2, alim);
	digitalWrite(ALIMAUX, alim ? RON: ROFF);
	EEPROM.get(0, LOCK);
	//if (LOCK !=1) LOCK=false;// TODO Décommenter
    // Ethernet
    Ethernet.begin(mac, ip, myDns, gateway, subnet);
    Ethernet.init(10);
    server.begin();
}

/*********************/
/* BOUCLE PRINCIPALE */
/*********************/
void loop()
{
    // Timers
    timer.run();

    // Serveur Telnet
    telnetServer(); // Lecture/envoi des commandes Telnet

    // Gestion des étapes
    // ------------------
    // Demande de déplacement de l'abri en cours (attente de l'ouverture des portes)
    // Déplacement différé de l'abri
    if (AbriCours && PortesOuvert && !TempoPortes)
    {
        deplaceAbri();
    }
    // Demande d'arret de l'abri en cours (attente de l'abri en position fermé)
    // Fermeture différée des portes
    if (ArretCours && AbriFerme && !TempoDepl)
    {
        StopMot;       // Arret du moteur de l'abri
        fermePortes(); // Fermeture des portes
        ArretCours = false;
    }

    // Gestion de l'arret d'urgence
    if (Baru)
        ARU();

    // Gestion des appuis bouton
    // Appui long avec la clef: ouverture des portes
    // Portes ouvertes et clef: fermeture des portes
    if (!digitalRead(BCLEF) && !AbriOuvert) {
        REMOTE = false;
        if (!PortesOuvert) {
            BappuiLong=false;
            timer.setTimeout(3000,appuiLong);
            ouvreAbri();
        }
        else if (AbriFerme) fermePortes();
    }
    else if (!digitalRead(BVERT))
    {
        REMOTE = false;
        ouvreAbri();
    }
    else if (!digitalRead(BROUGE) || (!digitalRead(BCLEF) && AbriOuvert))
    {
        REMOTE = false;
        fermeAbri();
    }
    delay(100);

	// Abri locké ?
	if (LOCK && MoteurStatus) {
		StopMot;
		MotAbriOk = false;
	}
    // Gestion des commandes série (Indi)
    readIndi();
	
	// Surveillance de l'abri
	AUTO=false;// A SUPPRIMER
	if (AUTO) {
		if (AbriOuvert && AbriFerme) ARU();					// Problème de capteurs
		if (!AbriCours && !AbriOuvert && !AbriFerme && (!Park || !PortesOuvert)) ARU(); // Déplacement intempestif de l'abri
		if (PorteCours && !Park) ARU(); 					// Fermeture des portes et télescope plus parqué
		if (!AbriOuvert && !AbriFerme && !Park) ARU();		// Abri en cours de déplacement et télescope plus parqué
	}
}

/*************/
/* FONCTIONS */
/*************/

// Ouverture de l'abri
void ouvreAbri()
{
    // Abri déjà ouvert
    if (AbriOuvert || AbriCours || ArretCours || !Park || LOCK)
        return;
    sendMsg("Ouvre abri");
    if (!MotAbriOk || !MoteurStatus)
    {
        StartMot;
        // FIXME timer.setTimeout(DELAIMOTEUR, initMotOk);
        timer.setTimeout(3000, initMotOk);
    }
    ouvrePortes();
    AbriCours = true; // Ouverture différée de l'abri après ouverture des portes
}

// Fermeture de l'abri
void fermeAbri()
{
    if ( AbriCours || ArretCours || !Park || LOCK)
        return;
    if (AbriFerme && !AbriOuvert) {
        // Abri fermé et portes ouvertes, on ferme les portes
        sendMsg("Ferme portes");
        TempoDepl=false;
    }
    else {
        sendMsg("Ferme abri");
        deplaceAbri(); // Fermeture différée des portes après fermeture de l'abri
    }
    ArretCours = true;
}

void deplaceAbri()
{
    // Moteur pas pret, portes non ouvertes
    if (!MotAbriOk || !PortesOuvert || LOCK || !Park)
        return;
    // Commande du moteur de l'abri
    sendMsg("Deplace abri");
    CmdMotOn;
    delay(IMPMOT);
    CmdMotOff;
    if (REMOTE)
    {
        TempoDepl = true;
        timer.setTimeout(DELAIABRI, attendDepl);
    }
    AbriCours = false;
}

// Ouverture des portes
void ouvrePortes()
{
    if (PorteCours)
        return;
    sendMsg("Ouvre portes");
    // Ouverture de la porte 1
    ouvrePorte1();
    timer.setTimeout(INTERVALLEPORTES, ouvrePorte2);
    PorteCours = true;
    if (REMOTE)
    {
        TempoPortes = true;
        timer.setTimeout(DELAIPORTES, attendPortes);
    }
}

void fermePortes()
{
    if (PorteCours || LOCK || !Park)
        return; // Evite les commandes multiples
    // Fermeture des portes
    // Abri fermé on ferme les portes
    if (AbriFerme && !AbriOuvert)
    {
        sendMsg("Ferme portes");
        fermePorte2();
        PorteCours = true;
        timer.setTimeout(INTERVALLEPORTES, fermePorte1);
    }
}

void ouvrePorte1()
{
    sendMsg("Ouvre porte 1");
    // Ouverture de la porte 1
    OuvreP1;
}

void fermePorte1()
{
    sendMsg("Ferme porte 1");
    // Fermeture de la porte 1
    FermeP1;
    PorteCours = false;
}

void ouvrePorte2()
{
    sendMsg("Ouvre porte 2");
    // Ouverture de la porte 2
    OuvreP2;
    PorteCours = false;
    if (BappuiLong)
    {
        BappuiLong = false;
        AbriCours = false;
    }
}

void fermePorte2()
{
	if (!LOCK) {
		sendMsg("Ferme porte 2");
		// Fermeture de la porte 2
		FermeP2;
	}
}

//---------- Fonctions Timer ----------
void initMotOk()
{
    sendMsg("Init moteur OK");
    // Moteur abri pret
    MotAbriOk = true;
}

void attendDepl()
{
    sendMsg("Attend déplacement");
    // Temporisation temps de déplacement de l'abri
    TempoDepl = false;
}

void attendPortes()
{
    sendMsg("Attend portes");
    // Temporisation temps d'ouverture/fermeture des portes
    TempoPortes = false;
}

void appuiLong()
{
    if (!digitalRead(BCLEF))
    {
        BappuiLong = true;
        sendMsg("Appui long");
    }
}

//---------- ARU, surveillance ----------
void ARU()
{
    sendMsg("ARU");
    // Demande d'arret d'urgence
    StopMot;                // Coupure alimentation moteur abri
    digitalWrite(P11, LOW); // Arret des portes
    digitalWrite(P12, LOW);
    digitalWrite(P21, LOW);
    digitalWrite(P22, LOW);
	bool OldLock=LOCK;		// Sauvegarde du status lock
	LOCK=true;
    while (!Baru)
        ;       // Appel externe à ARU
    delay(500); // Anti rebonds
    while (Baru)
        ;
	LOCK=OldLock;		// restauration du status lock
    pinMode(RESETMEGA, OUTPUT); // Reset de l'arduino
}

//---------- Web, telnet ----------
void telnetServer()
{
    // Serveur Telnet
    EthernetClient client = server.available();
    if (client)
    {
        if (!alreadyConnected)
        {
            // clear out the input buffer:
            client.flush();
            alreadyConnected = true;
        }
    }
}

void sendMsg(String message)
{
    // Envoi des messages
    server.println(message);
}

/************************/
/* FONCTIONS ROLLOFFINO */
/************************/

void sendData(char* buffer) {
	// Envoi les données sur le port USB ou Telnet
	Serial.println(buffer);
	sendMsg(buffer);
	Serial.flush();
}

void readIndi()
{
    if (Serial.available())
    {
        readData(); // 0: port USB 1: port Telnet
    }
}

bool parseCommand() // (command:target:value)
{
    char inpBuf[MAX_INPUT+1];
    memset(inpBuf, 0, sizeof(inpBuf));
    //memset(command, 0, sizeof(command));
    //memset(target, 0, sizeof(target));
    //memset(value, 0, sizeof(value));
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
            if (strcmp(target, "OPEN") == 0)
            {
				REMOTE=true;
                ouvreAbri();
				sendAck(value);
                timeMove = millis();
            }
            // Prepare to CLOSE
            else if (strcmp(target, "CLOSE") == 0)
            {
				REMOTE=true;
                fermeAbri();
                sendAck(value);
                timeMove = millis();
            }
            // Prepare to ABORT
            else if (strcmp(target, "ABORT") == 0)
            {
                // Test whether or not to Abort
                if (!isStopAllowed())
                {
                    error = ERROR10;
                }
                else
                {
                    ARU();
                    sendAck(value);
                }
            }
            // Prepare for the Lock function
            else if (strcmp(target, "LOCK") == 0)
            {
                sendMsg("Lock...");
                sendMsg(value);
				LOCK=(value=="ON");
				EEPROM.put(0, LOCK);
				// Sauvegarde en EEprom
				sendAck(value);
            }

            // Prepare for the Auxiliary function
            else if (strcmp(target, "AUXSET") == 0)
            {
				digitalWrite(ALIMAUX,value=="1" ? RON: ROFF);
				EEPROM.put(2, (value=="1"));
				sendAck(value);
            }
            else if (strcmp(target, "RESET") == 0)
            {
                sendAck(value);
                delay(300);
                pinMode(RESETMEGA, OUTPUT);
            }
            else if (strcmp(target, "OPENDOOR1") == 0) {
                sendAck(value);
                ouvrePorte1();
            }
            else if (strcmp(target, "CLOSEDOOR1") == 0) {
                sendAck(value);
                if (AbriFerme && ! AbriOuvert) fermePorte1();
            }
			else if (strcmp(target, "OPENDOORS") == 0) {
				sendAck(value);
                ouvrePortes();
			}
			else if (strcmp(target, "CLOSEDOORS") == 0) {
				sendAck(value);
                if (AbriFerme && ! AbriOuvert) {
					fermePortes();
					REMOTE=true;
					sendAck(value);
				}
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
                requestReceived(LOCK);
            else if (strcmp(target, "AUXSTATE") == 0)
				requestReceived(digitalRead(ALIMAUX)==RON);
            else sendNak(error);
        }
		else {
            sendNak(error); // Unknown input or Abort command was rejected
        }
        // Command or Request not implemented
		/*
		strcpy(value, "OFF"); // Request Not implemented
        sendNak(ERROR9);
        sendAck(value);
        */
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
  if (AbriOuvert || AbriFerme) 
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
