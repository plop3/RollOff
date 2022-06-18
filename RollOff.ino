/*
  Pilotage automatique de l'abri du telescope
  Serge CLAUS
  GPL V3
  Version 8.0
  22/10/2018-16/06/2022
  /*********************************/

/***********/
/* MODULES */
/***********/
#include "infos.h";		// Informations de connexion
#include "RollOffIno.h";  // Fonctions rollOffIno
#include "Config.h";		// Fichier de configuration
#include "Pinmap.h";		// Pins Arduino Mega
#include "Constants.h";		// Constantes

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

// Serveurs Telnet
#include <SPI.h>
#include <Ethernet.h>
byte mac[] = {
  0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xEE };
IPAddress ip(192, 168, 0, 16);  
IPAddress myDns(192, 168, 0, 254);
IPAddress gateway(192, 168, 0, 254);
IPAddress subnet(255, 255, 255, 0);

EthernetServer server(9999);	// Serveur Indi
EthernetClient client;			// Client MQTT
boolean alreadyConnected = false; 

// MQTT
void callbackMQTT(char* topic, byte* payload, unsigned int length) {
  // handle message arrived
  // Demande de fermeture de l'abri
  if (strcmp(topic, "abri-in") == 0) {
	  switch ((char)payload[0]) {
		  case 'A':	// Ouverture abri
			cmd=1;
		  break;
		  case 'a':	//Fermeture abri
			cmd=2;
		  break;
		  case 'P': // Ouvre les portes
			cmd=8;
		  break;
		  case 'p':	// Ferme les portes
			cmd=9;
		  break;
		  case 'F':	// Ouvre porte 1
			cmd=10;
		  break;
		  case 'f':	// Ferme porte 1
			cmd=11;
		  break;
		  case 'l':	// Eteint les éclairages
			barre(0,0);
			barre(1,0);
			barre(2,0);
		  break;
		  case 'T':	// Allume l'alimentation télescope
			cmd=6;
		break;
		  case 't': // Eteint l'alimentation télescope
			cmd=7;
		  break;
	  }
  }
}


#include <PubSubClient.h>
IPAddress broker(192, 168, 0, 4);
EthernetClient mqttclient;
PubSubClient mqtt(broker, 1883, callbackMQTT, mqttclient);

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
  pinMode(BROUGE, INPUT_PULLUP);// Bouton rouge
  pinMode(BLUMI, INPUT_PULLUP); // Bouton éclairage intérieur
  pinMode(BLUMT, INPUT_PULLUP); // Bouton éclairage table
  pinMode(PARK, INPUT_PULLUP);  // TODO Inverser le signal (0: Télescope parqué)
  pinMode(PLUIE,INPUT);  	// Capteur de pluie analogique
  
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
	
  // Ethernet
  Ethernet.begin(mac, ip, myDns, gateway, subnet);
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
    mqtt.publish("abri-out/doors",PortesOuvert ? "ON": "OFF");
    mqtt.publish("abri-out/open",AbriFerme ? "OFF": "ON");
    break;
  case 2: // Ferme abri
    fermeAbri();
    mqtt.publish("abri-out/doors",PortesOuvert ? "ON": "OFF");
    mqtt.publish("abri-out/open",AbriFerme ? "OFF": "ON");
    break;
  case 3: // Arret de l'abri
    stopAbri();
    mqtt.publish("abri-out/stop","ON");
    break;
  case 4:
    lockAbri();
    mqtt.publish("abri-out/locked","ON");
    break;
  case 5:
    bougePorte2();
    break;
  case 6:
    StartTel;
    mqtt.publish("abri-out/alimtel","ON");
    break;  
  case 7:
    StopTel;
	  mqtt.publish("abri-out/alimtel","OFF");
    break;
  case 8:
	  ouvrePortes();
    mqtt.publish("abri-out/doors",PortesOuvert ? "ON": "OFF");
	  break;
  case 9:
    fermePortes();
    mqtt.publish("abri-out/doors",PortesOuvert ? "ON": "OFF");
	break;
  case 10:
	ouvrePorte1();
  mqtt.publish("abri-out/door1",Porte1Ouvert ? "ON": "OFF");
	break;
  case 11:
  fermePorte1();
  mqtt.publish("abri-out/door1",Porte1Ouvert ? "ON": "OFF");
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
  if (!Porte1Ouvert) {
	OuvreP1;
	attend(DELAIPORTES,1);
  }
  FermeP2;
  attend(INTERVALLEPORTES * 1.5,1);
  FermeP1;
	attend(DELAIPORTES,1);
  abriOff();
  return true;
}

void ouvrePorte1() {
		OuvreP1;
    int i=DELAIPORTES*1.2;
    while(!Porte1Ouvert && i>0 ) {
      attend(1000,0);
      i--;
    }
}

void fermePorte1() {
	if (!Porte2Ouvert) {
    FermeP1;
    attend(DELAIPORTES,0);
  }
}

void bougePorte2() {
  // Ouvre/ferme la porte 2 (La porte 1 doit être ouverte)
  if (!Porte1Ouvert) return;
  if (Porte2Ouvert) {
    FermeP2;
  }
  else {
    OuvreP2;
  }
}

void startTel() {
		StartTel;
		mqtt.publish("abri-out/alimtel", "ON");
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
  // Lecture MQTT
  if (!mqtt.connected()) connectMQTT();
  mqtt.loop();   
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
  if ((100 - 100*float(analogRead(PLUIE))/1023) > HUMMAX) {
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
  //StopTel; 
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

void connectMQTT() {
  if (mqtt.connect("abri",MQTTUSER,MQTTPASSWD)) {
    mqtt.publish("abri-out/open",AbriFerme ? "OFF": "ON");
    mqtt.publish("abri-out/locked", LOCK ? "ON": "OFF");
    mqtt.publish("abri-out/doors",PortesOuvert ? "ON": "OFF");
    mqtt.publish("abri-out/door1",Porte1Ouvert ? "ON": "OFF");
	  mqtt.publish("abri-out/alimtel", !digitalRead(ALIMTEL) ? "ON": "OFF");
    mqtt.subscribe("abri-in");
  }
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

/************************/
/* FONCTIONS ROLLOFFINO */
/************************/

char command[cLen+1];
char value[vLen+1];
char target[tLen+1];
unsigned long timeMove = 0;
int TypeCon=0;  // 0: USB, 1: Telnet 9999, 2: Telnet 9998

/*************/
/* FONCTIONS */
/*************/

void sendData(char* buffer) {
    // Envoi les données sur le port USB
  Serial.println(buffer);
  Serial.flush();
}

void readIndi()
{   
  if (Serial.available()) readData();
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
            if (strcmp(target, "OPEN") == 0)        // Ouverture de l'abri
            {
                sendAck(value);
                timeMove = millis();
                Remote=true;
                cmd=1;
            }
            // Prepare to CLOSE
            else if (strcmp(target, "CLOSE") == 0)      // Fermeture de l'abri
            {
                sendAck(value);
                timeMove = millis();
                Remote=true;
                cmd=2;
            }
            // Prepare to ABORT               // Arret de l'abri
            else if (strcmp(target, "ABORT") == 0)
            {
                // Test whether or not to Abort
                if (!isStopAllowed())
                {
                    error = ERROR10;
                }
                else
                {
                  Remote=true;
                  cmd=3;
                  sendAck(value);
                }
            }
            // Prepare for the Lock function
            else if (strcmp(target, "LOCK") == 0)     // Lock de l'abri
            {
              sendAck(value);
            }

            // Prepare for the Auxiliary function
            else if (strcmp(target, "AUXSET") == 0)     // Bascule de la sorite auxiliaire
            {
              sendAck(value);
              Remote=true;
              strcmp(value, "ON") ==0 ? cmd=6: cmd=7;
            }
            else if (strcmp(target, "RESET") == 0)      // RESET Arduino
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
        requestReceived(!digitalRead(ALIMTEL));
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
