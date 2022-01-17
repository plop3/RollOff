/*
  Pilotage automatique de l'abri du telescope
  Serge CLAUS
  GPL V3
  Version 5.0
  22/10/2018-16/01/2022
*/

/***********/
/* MODULES */
/***********/

/**************/
/* PARAMETRES */
/**************/
#define BAUDRATE 9600 // Vitesse du port série
#define RON HIGH       // Etat On pour les relais (HIGH, LOW)
#define ROFF !RON
#define DELAIPORTES 40000L     // Durée d'ouverture/fermeture des portes (40000L)
#define DELAIMOTEUR 30000L     // Durée d'initialisation du moteur (40000L)
#define DELAIABRI 22000L       // Durée de déplacement de l'abri (15000L)
#define INTERVALLEPORTES 12000 // Intervalle entre la fermeture de la porte 1 et de la porte 2
#define IMPMOT 500             // Durée d'impulsion moteur
#define BAPPUILONG 3000        //Durée en ms pour un appui long sur le bouton

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

EthernetServer server(23);
EthernetClient client = 0;
boolean alreadyConnected = false; // whether or not the client was connected previously

/**************/
/* CONSTANTES */
/**************/
//----------Sorties ----------
#define CMDMOT A1     // (R1) Ouverture/fermeture abri Commande moteur de porte de garage
#define ALIMMOT A2    // (R2) Alimentation 220V moteur abri
#define P11 3         // (R5) LM298 1 porte 1
#define P12 5         // (R6) LM298 2 porte 1
#define P21 6         // (R7) LM293 3 porte 2
#define P22 7         // (R8) LM298 4 porte 2
#define RESETMEGA A13 // Reset de l'arduino

//---------- Entrées ----------
// Capteurs
#define AO 49  // Capteur abri ouvert
#define AF 48  // Capteur abri fermé
#define PO1 24 // Capteur porte 1 ouverte
#define PO2 25 // Capteur porte 2 ouverte
// Boutons
#define BCLEF A12 // Bouton à clef d'ouverture/fermeture
#define BVERT 34  // Bouton intérieur d'ouverture/fermeture
#define BROUGE 46 // Bouton de sélection
#define BARU 22   // Bouton d'arret d'urgence

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
#define OuvreP1             \
    digitalWrite(P12, LOW); \
    digitalWrite(P11, HIGH)
#define OuvreP2             \
    digitalWrite(P22, LOW); \
    digitalWrite(P21, HIGH)
#define FermeP1             \
    digitalWrite(P11, LOW); \
    digitalWrite(P12, HIGH)
#define FermeP2             \
    digitalWrite(P21, LOW); \
    digitalWrite(P22, HIGH)
#define Baru !digitalRead(BARU) // Arret d'urgence
#define StartMot digitalWrite(ALIMMOT, RON)
#define StopMot digitalWrite(ALIMMOT, ROFF)
#define MoteurStatus (digitalRead(ALIMMOT) == RON) // Alimentation du moteur abri

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
bool BappuiLong = false;  // Appui long sur le bouton vert ou la clef

//---------- RollOffIno
const int cLen = 15;
const int tLen = 15;
const int vLen = MAX_RESPONSE;
char command[cLen+1];
char value[vLen+1];
char target[tLen+1];
unsigned long timeMove = 0;

enum cmd_input {
CMD_NONE,  
CMD_OPEN,
CMD_CLOSE,
CMD_STOP,
CMD_LOCK,
CMD_AUXSET
} command_input;

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
    digitalWrite(P11, LOW);
    pinMode(P11, OUTPUT);
    digitalWrite(P12, LOW);
    pinMode(P12, OUTPUT);
    digitalWrite(P21, LOW);
    pinMode(P21, OUTPUT);
    digitalWrite(P22, LOW);
    pinMode(P22, OUTPUT);
    // Activation des entrées (capteurs...)
    pinMode(AO, INPUT_PULLUP);
    pinMode(AF, INPUT_PULLUP);
    pinMode(PO1, INPUT_PULLUP);
    pinMode(PO2, INPUT_PULLUP);
    pinMode(BCLEF, INPUT_PULLUP);
    pinMode(BVERT, INPUT_PULLUP);
    pinMode(BROUGE, INPUT_PULLUP);
    pinMode(BARU, INPUT_PULLUP);

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
    // TODO Appui long sur les boutons
    if (!digitalRead(BVERT) || (!digitalRead(BCLEF) && !AbriOuvert))
    {
        //timer.setTimeout(3000,appuiLong);
        REMOTE = false;
        ouvreAbri();
    }
    if (!digitalRead(BROUGE) || (!digitalRead(BCLEF) && !AbriFerme))
    {
        REMOTE = false;
        fermeAbri();
    }
    delay(100);

    // Gestion des commandes série (Indi)
    readIndi();
}

/*************/
/* FONCTIONS */
/*************/

// Ouverture de l'abri
void ouvreAbri()
{
    sendMsg("Ouvre abri");
    // Abri déjà ouvert
    if (AbriOuvert || AbriCours || ArretCours)
        return;
    if (!MotAbriOk || !MoteurStatus)
    {
        StartMot;
        // FIXME timer.setTimeout(DELAIMOTEUR, initMotOk);
        timer.setTimeout(3000, initMotOk);
    }
    ouvrePortes();
    AbriCours = true; // Fermeture différée de l'abri après fermeture des portes
}

// Fermeture de l'abri
void fermeAbri()
{
    sendMsg("Ferme abri");
    if (AbriFerme || AbriCours || ArretCours)
        return;
    deplaceAbri(); // Fermeture différée des portes après fermeture de l'abri
    ArretCours = true;
}

void deplaceAbri()
{
    sendMsg("Deplace abri");
    // Moteur pas pret, portes non ouvertes
    if (!MotAbriOk || !PortesOuvert)
        return;
    // Commande du moteur de l'abri
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
    sendMsg("Ouvre portes");
    if (PorteCours)
        return;
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
    sendMsg("Ferme portes");
    if (PorteCours)
        return; // Evite les commandes multiples
    // Fermeture des portes
    // Abri fermé on ferme les portes
    if (AbriFerme)
    {
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
    if (BappuiLong)
    {
        BappuiLong = false;
        ArretCours = false;
    }
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
    sendMsg("Ferme porte 2");
    // Fermeture de la porte 2
    FermeP2;
}

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
    if (!digitalRead(BVERT) || !digitalRead(BCLEF))
    {
        BappuiLong = true;
        sendMsg("Appui long");
    }
}
void ARU()
{
    sendMsg("ARU");
    // Demande d'arret d'urgence
    StopMot;                // Coupure alimentation moteur abri
    digitalWrite(P11, LOW); // Arret des portes
    digitalWrite(P12, LOW);
    digitalWrite(P21, LOW);
    digitalWrite(P22, LOW);
    while (!Baru)
        ;       // Appel externe à ARU
    delay(500); // Anti rebonds
    while (Baru)
        ;
    pinMode(RESETMEGA, OUTPUT); // Reset de l'arduino
}

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

void readIndi()
{
    while (Serial.available() <= 0)
    {
        for (int cnt = 0; cnt < 60; cnt++)
        {
            if (Serial.available() > 0)
                break;
            else
                delay(100);
        }
    }
    readUSB();
}

void readUSB()
{
    // Confirm there is input available, read and parse it.
    if (Serial && (Serial.available() > 0))
    {
        if (parseCommand())
        {
            unsigned long timeNow = millis();
            //int hold = 0;
            int relay = -1; // -1 = not found, 0 = not implemented, pin number = supported
            int sw = -1;    //      "                 "                    "
            bool connecting = false;
            const char *error = ERROR8;

            // On initial connection return the version
            if (strcmp(command, "CON") == 0)
            {
                connecting = true;
                strcpy(value, VERSION_ID); // Can be seen on host to confirm what is running
                sendAck(value);
            }

            // Map the general input command term to the local action
            // SET: OPEN, CLOSE, ABORT, LOCK, AUXSET
            else if (strcmp(command, "SET") == 0)
            {
                // Prepare to OPEN
                if (strcmp(target, "OPEN") == 0)
                {
                    command_input = CMD_OPEN;
                    ouvreAbri();
                    relay = 1;
                    //hold = FUNC_OPEN_HOLD;
                    timeMove = timeNow;
                }
                // Prepare to CLOSE
                else if (strcmp(target, "CLOSE") == 0)
                {
                    command_input = CMD_CLOSE;
                    fermeAbri();
                    relay = 1;
                    //hold = FUNC_CLOSE_HOLD;
                    timeMove = timeNow;
                }
                // Prepare to ABORT
                else if (strcmp(target, "ABORT") == 0)
                {
                    command_input = CMD_STOP;

                    // Test whether or not to Abort
                    if (!isStopAllowed())
                    {
                        error = ERROR10;
                    }
                    else
                    {
                        ARU();
                        relay = 1;
                        //hold = FUNC_STOP_HOLD;
                    }
                }
                // Prepare for the Lock function
                else if (strcmp(target, "LOCK") == 0)
                {
                    command_input = CMD_LOCK;
                    //relay = FUNC_LOCK;
                    //hold = FUNC_LOCK_HOLD;
                }

                // Prepare for the Auxiliary function
                else if (strcmp(target, "AUXSET") == 0)
                {
                    command_input = CMD_AUXSET;
                    //relay = FUNC_AUX;
                    //hold = FUNC_AUX_HOLD;
                }
            }

            // Handle requests to obtain the status of switches
            // GET: OPENED, CLOSED, LOCKED, AUXSTATE
            else if (strcmp(command, "GET") == 0)
            {
                if (strcmp(target, "OPENED") == 0)
                    sw = AbriOuvert;
                else if (strcmp(target, "CLOSED") == 0)
                    sw = AbriFerme;
                else if (strcmp(target, "LOCKED") == 0)
                    sw = 0;
                else if (strcmp(target, "AUXSTATE") == 0)
                    sw = 0;
            }

            /*
       * See if there was a valid command or request 
       */
            if (!connecting)
            {
                if ((relay == -1) && (sw == -1))
                {
                    sendNak(error); // Unknown input or Abort command was rejected
                }

                // Command or Request not implemented
                /*
                else if ((relay == 0 || relay == -1) && (sw == 0 || sw == -1))
                {
                    strcpy(value, "OFF"); // Request Not implemented
                    sendNak(ERROR9);
                    sendAck(value);
                }
                */
                // Valid input received

                // A command was received
                // Set the relay associated with the command and send acknowlege to host
                
                else if (relay > 0) // Set Relay response
                {
                    commandReceived(relay, value);
                }
                
                // A state request was received
                else if (sw > -1) // Get switch response
                {
                    requestReceived(sw);
                }
                
            } // end !connecting
        }     // end command parsed
    }         // end Serial input found
}

bool parseCommand() // (command:target:value)
{
    bool start = false;
    bool eof = false;
    int recv_count = 0;
    int wait = 0;
    int offset = 0;
    char startToken = '(';
    char endToken = ')';
    const int bLen = MAX_INPUT;
    char inpBuf[bLen + 1];

    memset(inpBuf, 0, sizeof(inpBuf));
    memset(command, 0, sizeof(command));
    memset(target, 0, sizeof(target));
    memset(value, 0, sizeof(value));

    while (!eof && (wait < 20))
    {
        if (Serial.available() > 0)
        {
            Serial.setTimeout(1000);
            recv_count = Serial.readBytes((inpBuf + offset), 1);
            if (recv_count == 1)
            {
                offset++;
                if (offset >= MAX_INPUT)
                {
                    sendNak(ERROR3);
                    return false;
                }
                if (inpBuf[offset - 1] == startToken)
                {
                    start = true;
                }
                if (inpBuf[offset - 1] == endToken)
                {
                    eof = true;
                    inpBuf[offset] = '\0';
                }
                continue;
            }
        }
        wait++;
        delay(100);
    }

    if (!start || !eof)
    {
        if (!start && !eof)
            sendNak(ERROR4);
        else if (!start)
            sendNak(ERROR5);
        else if (!eof)
            sendNak(ERROR6);
        return false;
    }
    else
    {
        strcpy(command, strtok(inpBuf, "(:"));
        strcpy(target, strtok(NULL, ":"));
        strcpy(value, strtok(NULL, ")"));
        if ((strlen(command) >= 3) && (strlen(target) >= 1) && (strlen(value) >= 1))
        {
            return true;
        }
        else
        {
            sendNak(ERROR7);
            return false;
        }
    }
}

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
        Serial.println(buffer);
        Serial.flush();
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
    Serial.println(response);
    Serial.flush();
  }
}


void commandReceived(int relay, char* value)
{
  //setRelay(relay, hold, value);
  sendAck(value);         // Send acknowledgement that relay pin associated with "target" was activated to value requested
}

void requestReceived(int sw)
{
  getSwitch(sw, value);
  sendAck(value);            // Send result of reading pin associated with "target" 
}

/*

void setRelay(int id, int hold, char* value)
{
  if (strcmp(value, "ON") == 0)
  {
    digitalWrite(id, LOW);            // NO RELAY would normally already be in this condition (open)
    //delay(RELAY_PRIOR_DELAY);
    digitalWrite(id, HIGH);           // Activate the NO relay (close it) 
    if (hold == 0)
    {  
      //delay(RELAY_ON_DELAY);
      digitalWrite(id, LOW);          // Turn NO relay off
    }
  }
  else
  {
    digitalWrite(id, LOW);            // Turn NO relay off 
  }
  //delay(RELAY_POST_DELAY);
} 
*/
void getSwitch(int id, char* value)
{
  if (!id)
    strcpy(value, "OFF");
  else
    strcpy(value, "ON");  
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

/*
bool isSwitchOn(int id)
{
  char switch_value[16+1];
  getSwitch(id, switch_value);
  if (strcmp(switch_value, "ON") == 0)
  {
    return true;
  }   
  return false;
}
*/