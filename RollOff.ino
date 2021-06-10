/*
  Pilotage automatique de l'abri du telescope
  Serge CLAUS
  GPL V3
  Version 4.0
  22/10/2018-12/02/2021

  Bouton à clef pour ouverture de l'abri:
    Appui long: Ouverture, fermeture des portes seules
    Appui court: Ouverture de l'abri, fermeture de l'abri
  1 Bouton à l'intérieur:
    Commandes identiques au bouton à clef.
  2 boutons arrêt d'urgence (1 intérieur, 1 extérieur)
  2 capteurs de position de l'abri (ouvert, fermé)
  1 capteur de position du télescope (parqué: true)
  2 capteurs d'ouverture des portes
  OPTIONS:
    1 barrière IR de sécurité
  /*********************************/

//---------------------------------------Modules--------------------------------
#include "RollOffIndi.h" // Gestion de l'abri par Indi (https://github.com/wotalota/indi-rolloffino)

//---------------------------------------PERIPHERIQUES--------------------------
// Temps maxi de park en millisecondes
#define TPSPARK 180000

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

// Timer
#include <SimpleTimer.h>
SimpleTimer timer;

// EEPROM
#include <EEPROM.h>

// ESP-Link
#include <ELClient.h>
#include <ELClientSocket.h>
#include <ELClientCmd.h>
#include <ELClientMqtt.h>
ELClient esp(&Serial3, &Serial3);
//ELClientCmd cmd(&esp);
ELClientMqtt mqtt(&esp);


//---------------------------------------CONSTANTES-----------------------------
// Sorties
//#define ALIM12V A3  // (R3) Mise en marche de l'alimentation 12V de l'abri (vérins portes, capteurs)
#define ALIMTEL A4   // (R4) Alimentation télescope    Relais dans le boitier OnStep)
#define ALIMMOT A2  // (R2) Alimentation 220V moteur abri
#define MOTEUR  A1  // (R1) Ouverture/fermeture abri Commande moteur de porte de garage
#define P11     3   // (R5) LM298 1 porte 1
#define P12     5   // (R6) LM298 2 porte 1
#define P21     6   // (R7) LM293 3 porte 2
#define P22     7   // (R8) LM298 4 porte 2

#define RESET   A13 // Reset de l'arduino

// Entrées
#define PARK  A5     // Entrée Park: Etat du telescope 0: non parqué, 1: parqué

#define AO  49       // Capteur abri ouvert
#define AF  48       // Capteur abri fermé
#define Po1 24       // Capteur porte 1 ouverte
#define Po2 25       // Capteur porte 2 ouverte

// Boutons
#define B1CLEF   A12    // Bouton à clef d'ouverture/fermeture des portes (pos 1 & 2)
#define BARU     22    // Bouton d'arret d'urgence
#define BLUMT    A11    // Bouton d'éclairage de la table (rouge)  Interrupteur double
#define BLUMI    A10    // Bouton d'éclairage de l'abri   (rouge)

// Constantes globales
#define DELAIPORTES 50000L          // Durée d'ouverture/fermeture des portes (40000L)
// #define DELAIPORTESCAPTEUR  30000L  // Durée d'ouverture/fermeture des portes (40000L)
#define DELAIMOTEUR 15000L          // Durée d'initialisation du moteur (40000L)
#define DELAIABRI   20000L          // Durée de déplacement de l'abri (15000L)
#define INTERVALLEPORTES 12000       // Intervalle entre la fermeture de la porte 1 et de la porte 2
// #define INTERVALLECAPTEUR 6000      // Temps de fermeture/ouverture des portes pour tests capteurs
#define MOTOFF HIGH                 // Etat pour l'arret du moteur
#define MOTON !MOTOFF
#define IMPMOT 300                 // Durée d'impulsion moteur

#define DELAIMQTT 30000UL            // Rafraichissement MQTT

//---------------------------------------Macros---------------------------------
#define AlimTelStatus (!digitalRead(ALIMTEL))    // Etat de l'alimentation télescope
// #define Alim12VStatus (digitalRead(ALIM12V))  // Etat de l'alimentation 12V ATX
#define PortesOuvert  (!digitalRead(Po1) && !digitalRead(Po2)) // && Alim12VStatus)
#define AbriFerme     (!digitalRead(AF))
#define AbriOuvert    (!digitalRead(AO))
#define MoteurStatus  (!digitalRead(ALIMMOT))
#define StartTel      digitalWrite(ALIMTEL, LOW)
#define StopTel       digitalWrite(ALIMTEL, HIGH)
#define StartMot      digitalWrite(ALIMMOT, LOW)
#define StopMot       digitalWrite(ALIMMOT, MOTOFF)
#define CmDMotOff     digitalWrite(MOTEUR, HIGH)
#define CmDMotOn      digitalWrite(MOTEUR, LOW)
// #define Stop12V    digitalWrite(ALIM12V, LOW)
// #define Start12V   digitalWrite(ALIM12V, HIGH)
#define TelPark       (analogRead(PARK)>300)
//#define TelPark       digitalRead(PARK)
//#define TelPark true
#define OuvreP1       digitalWrite(P12,LOW);digitalWrite(P11,HIGH)
#define OuvreP2       digitalWrite(P22,LOW);digitalWrite(P21,HIGH)
#define FermeP1       digitalWrite(P11,LOW);digitalWrite(P12,HIGH)
#define FermeP2       digitalWrite(P21,LOW);digitalWrite(P22,HIGH)
#define Bclef         !digitalRead(B1CLEF)
#define Baru          !digitalRead(BARU)

#define BAPPUILONG  3000  //Durée en ms pour un appui long sur le bouton

// Telnet client
#define DEFAULT_SOCKET_TIMEOUT	5000


// ------------------------------------Variables globales------------------------------------

int countM;           // Nombre d'essais ouverture/fermeture

bool TEMPO = false; // Temporisation
bool TBOUTON = false; // Temporisation bouton
bool INIT = false;    // Abri initialisé
bool ENDSEQ = false;  // Séquence précédente complète
bool DEPL = false;    // Abri en cours de déplacement
bool FERM = false;    // Portes en cours de fermeture
bool SURV = true;    // Activation du graphe de surveillance
bool CMDARU = false;  // Commande interne, ou Indi d'arret d'urgence
bool AUTO = false;    // Abri en mode automatique (commande reçue à distance)
bool Bmem = false;    // Mémorisation du bouton

bool BLUMTO = !digitalRead(BLUMT);  // Dernier etat du bouton d'éclairage table
bool BLUMIO = !digitalRead(BLUMI);  // Dernier etat du bouton d'éclairage intérieur

String Message ="";                 // Message affiché sur l'écran OLED

int TimerID;                        // ID du timer attend

bool connected;                     // ESP-Link MQTT

// Telnet client
char * const tcpServer PROGMEM = "192.168.0.15";
uint16_t const tcpPort PROGMEM = 9999;
ELClientSocket tcp(&esp);

// MQTT
// Callback when MQTT is connected
void mqttConnected(void* response) {
  Serial3.println("MQTT connected!");
  mqtt.subscribe("esp-abri/set");
  //mqtt.subscribe("esp-abri/status");
  //mqtt.subscribe("/hello/world/#");
  //mqtt.subscribe("/esp-link/2", 1);
  //mqtt.publish("esp-abri/status", "Ok");
  connected = true;
}

// Callback when MQTT is disconnected
void mqttDisconnected(void* response) {
  Serial3.println("MQTT disconnected");
  connected = false;
}

// Callback when an MQTT message arrives for one of our subscriptions
void mqttData(void* response) {
  ELClientResponse *res = (ELClientResponse *)response;

  Serial3.print("Received: topic=");
  String topic = res->popString();
  Serial3.println(topic);

  Serial3.print("data=");
  String data = res->popString();
  Serial3.println(data);
  if (topic=="esp-abri/set") {
    //if (data=="ON") BoutonOpenState=true;
    if (data=="OFF") BoutonCloseState=true;
  }
}

void mqttPublished(void* response) {
  Serial3.println("MQTT published");
}

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
  Serial3.begin(115200);  

  // Initialisation des relais
  // digitalWrite(ALIM12V, HIGH); pinMode(ALIM12V, OUTPUT);
  StopTel; pinMode(ALIMTEL, OUTPUT);  // Coupure de l'alimentation 12V du télescope
  StopMot; pinMode(ALIMMOT, OUTPUT);  // Coupure alimentation moteur abri
  CmDMotOff; pinMode(MOTEUR, OUTPUT); // Coupure de la commande du moteur de déplacement
  // Initialisation du LM298
  digitalWrite(P11, LOW); pinMode(P11, OUTPUT);
  digitalWrite(P12, LOW); pinMode(P12, OUTPUT);
  digitalWrite(P21, LOW); pinMode(P21, OUTPUT);
  digitalWrite(P22, LOW); pinMode(P22, OUTPUT);
  digitalWrite(ALIMMOT, MOTOFF);

  // Activation des entrées (capteurs...)
  pinMode(AO, INPUT_PULLUP);
  pinMode(AF, INPUT_PULLUP);
  pinMode(Po1, INPUT_PULLUP);
  pinMode(Po2, INPUT_PULLUP);
  pinMode(B1CLEF, INPUT_PULLUP);
  pinMode(BARU, INPUT_PULLUP);
  pinMode(BLUMT, INPUT_PULLUP);
  pinMode(BLUMI, INPUT_PULLUP);
  //pinMode(PARK, INPUT_PULLUP); // Si le télescope n'est pas branché, considéré comme parqué
  pinMode(PARK, INPUT);
  //timer.setInterval(1000,debug);
  //timer.setInterval(DELAIMQTT,refreshMQTT);
  barre(0, 0); // Extinction des barres de LEDs
  barre(1, 0);
  barre(2, 0);

  // MQTT
  Serial3.print("ESPSYNC:");
  bool ok;
  do {
    ok=esp.Sync();
  } while(!ok);
  Serial3.println("EL-Client synced!");

  mqtt.connectedCb.attach(mqttConnected);
  mqtt.disconnectedCb.attach(mqttDisconnected);
  mqtt.publishedCb.attach(mqttPublished);
  mqtt.dataCb.attach(mqttData);
  mqtt.setup();

// Client TCP
  tcp.begin(tcpServer, tcpPort, SOCKET_TCP_CLIENT);

  // Initialisation de la position de l'abri
  INIT=false;
  if (!initAbri()) {
    // L'abri ne peut pas s'initialiser on passe en arrêt d'urgence
    CMDARU=true;
    pool();
  }
  refreshMQTT();
}

//---------------------------BOUCLE PRINCIPALE-----------------------------------------------

void loop() {
  pool(); // fonctions périodiques
  // Gestion de l'abri
  // Attente d'une commande
  Message="OK";
//  if (BoutonStopState) {
//    CMDARU = true;
//    return;
 // }

  if (Bclef) {
    // Commande manuelle
    Bmem = true;
    AUTO = false;
    Btempo();
  }
  else if (BoutonOpenState || BoutonCloseState) {
    // Commande auto
    Bmem = false;
    AUTO = true;
    TBOUTON = false;
  }

  if (AbriFerme && PortesOuvert && Bmem && !BoutonOpenState) {
	Bmem=false;
	fermePortes();
  } 
  else if(AbriFerme && (Bmem || BoutonOpenState)) {
    // Ouverture abri
    BoutonOpenState = false;
    Bmem = false;
    ouvreAbri();
  }
  // Fermeture abri
  else if (AbriOuvert  && (Bmem || BoutonCloseState)) {
    Bmem = false;
    BoutonCloseState = false;
    fermeAbri();
  }
  else {
    // Commande non conforme, on ignore (ouverture et abri ouvert...)
    Bmem = false;
    BoutonCloseState = false;
  }
}

//-----------------------------------FONCTIONS-----------------------------------------------
bool refreshMQTT() {
  // Mise à jour des infos MQTT
  //Serial3.println("plop");
  if (AbriOuvert && !AbriFerme) {
    mqtt.publish("esp-abri/status","ON");
  }
  else if (AbriFerme && !AbriOuvert) {
    mqtt.publish("esp-abri/status","OFF");
  }
  /*else {
  #  mqtt.publish("esp-abri/status","move");
  }*/
}

bool deplaceAbri() {
  if (!(TelPark) || !PortesOuvert) return(false);

  
  countM = 0;
  DEPL = true;
  Message="Depl abri";
  if (!AbriOuvert && !AbriFerme) {
    // Abri mal positionné, temporisation pour le moteur
    attend(DELAIMOTEUR);
    CmDMotOn;
    delay(IMPMOT);
    CmDMotOff;
  }
  else {
    // Attente du début de déplacement
    do {
      CmDMotOn;
      delay(IMPMOT);
      CmDMotOff;
      countM++;
      attend(5000);
    } while((AbriOuvert || AbriFerme) && countM<5);
  }
  // Déplacement en cours ?
  if (!AbriOuvert && ! AbriFerme) {
    barre(0, 128);
    if (AUTO) attend(DELAIABRI);
    while(!AbriOuvert && !AbriFerme);  // Attente des capteurs
    barre(0, 0);
    DEPL=false;
    return(true);
  }
  DEPL=false;
  return(false);
}

bool ouvreAbri() {
  mqtt.publish("esp-abri/status","ON");
  if (AbriOuvert) return(true);   // Abri déjà ouvert
  // Ouverture des portes si besoin
  if(ouvrePortes()) {
    if (deplaceAbri()) {
      StartTel;           // Mise en marche du télescope
      return(true);
    }
  }
  mqtt.publish("esp-abri/status","OFF");
  return(false);
}

bool fermeAbri() {
  mqtt.publish("esp-abri/msg","fermeture_abri");
  mqtt.publish("esp-abri/status","OFF");
  if (AbriFerme) return(true);
  if (!TelPark) {
    mqtt.publish("esp-abri/msg","park_telescope");
    // Tentative de parquer le télescope par les commandes OnStep
    tcp.send(":Q#");   // Arret du mouvement
    delay(2200);
    tcp.send(":Te#");   // Tracking On
    delay(2200);
    tcp.send(":hP#");   // Park du télescope
    // On attend 3mn max que le télescope soit parqué
    unsigned long tpsdebut=millis();
    unsigned long tpsact;
    do {
      tpsact=millis();
    } while ((tpsact-tpsdebut)<TPSPARK && !TelPark);
    delay(5000);  // Attente du park complet
    mqtt.publish("esp-abri/msg","telescope_parque");
    if (!TelPark) 
    {
      mqtt.publish("esp-abri/msg","park_impossible");
      mqtt.publish("esp-abri/status","ON");
      return(false);
    }
  }
  mqtt.publish("esp-abri/msg","arret_telescope");
  StopTel;      // Arret du télescope
  if (deplaceAbri()) {
    if (fermePortes()) {
      return(true);
    }
  }
  mqtt.publish("esp-abri/status","ON");
  return(true);
}

bool ouvrePortes() {
  if (PortesOuvert) {
    // Portes ouvertes
    OuvreP1;
    OuvreP2;
    attend(BAPPUILONG + 500);
  } 
  else {
    // Portes fermées
    ENDSEQ=false;
    EEPROM.put(0,ENDSEQ); // Début de séquence
    // Ouverture porte 1
    Message="Ouvre P";
    OuvreP1;
    attend(INTERVALLEPORTES);
    StartMot;
    OuvreP2;
    if (AUTO) attend(DELAIPORTES);
    while(!PortesOuvert) {
      attend(100);
    }
  }
  ENDSEQ=true;
  EEPROM.put(0,ENDSEQ); // Fin de séquence
  // Appui long sur le bouton
  if (TBOUTON) return(false); // Portes ouvertes seulement
  return(true);
}

bool fermePortes() {
  if (!AbriFerme) return(false);
  ENDSEQ=false;
  EEPROM.put(0,ENDSEQ); // Début de séquence
  Message="Ferme P";
  
  FERM=true;
  FermeP2;
Message="P2";
  attend(INTERVALLEPORTES/2);
  StopMot;
  attend(INTERVALLEPORTES/2);
  FermeP1;
Message="P1";
  attend(DELAIPORTES);
Message="P fermes";
  FERM=false;
  ENDSEQ=true;
  EEPROM.put(0,ENDSEQ); // Fin de séquence
  return(true);
}

void attend(unsigned long delai) {
  unsigned long previousMillis = millis();
  unsigned long currentMillis;
  do {
    currentMillis=millis();
    pool();
  } while (currentMillis-previousMillis<=delai);
  
/*
  tempo(delai);
  while(!TEMPO) {
    pool();
  }
*/
}

void testAbort() {
  if (BoutonStopState) {
    CMDARU = true;
    return;
  }
}

void pool() {
  // Fonctions périodiques
  esp.Process();  // Gestion de ESP-Link
  ARU();          // Gestion arret d'urgence
  Surv();         // Surveillance de l'abri (déplacement intempestif)
  readIndi();     // Lecture des commandes Indi
  //testAbort();
  timer.run();    // Gestion des timers
  ssd1306Info();  // Info sur l'écran OLED
  eclairages();   // Gestion des éclairages
}

bool initAbri() {
  // Initialisation de la position de l'abri
  Message="Init...";
  ssd1306Info();
  // Abri ouvert
  if (AbriOuvert && !AbriFerme) {
    OuvreP1;
    OuvreP2;
    StartTel;
    StartMot;
    Message="Ouvert";
  }
  else if (AbriFerme && !AbriOuvert) {
    // Abri fermé
    if (PortesOuvert) {
      // Portes ouvertes, on les ferme
      fermePortes();
    }
    else {
      // Portes fermées
      // Vérification de la fin de séquence
      EEPROM.get(0,ENDSEQ);
      if (!ENDSEQ) {
        // Séquence non terminée, on ferme les portes
        fermePortes();
        ENDSEQ=true;
        EEPROM.put(0,ENDSEQ); //Séquence terminée
      }
    }
    Message="Ferme";
  }
  else if (AbriOuvert && AbriFerme) {
    // Problème de capteur initialisation impossible
    Message="Pb capteur";
    ssd1306Info();
    return(false);
  }
  else if (!AbriOuvert && !AbriFerme) {
    // Abri en position intermédiaire (arr prématuré)
    if (!(TelPark)) {
      // Télescope non parqué, initialisation impossible
      Message="Pb park";
      ssd1306Info();
      return(false);
    }
    // Télescope parqué, initialisation de l'abri
    Message="Reposition";
    ssd1306Info();
    OuvreP1;
    OuvreP2;
    StartMot;
    deplaceAbri();
    if (AbriOuvert) {
      fermeAbri();
    }
    Message="Ferme";
  }
  else {
    // Autres cas, initialisation impossible
    Message="Probleme";
    ssd1306Info();
    return(false);
  }
  // Initialisation OK
  INIT=true;
  //SURV=true;
  return(true);
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

void tempo(long duree) {
  // Temporisateur
  TEMPO = false;
  TimerID=timer.setTimeout(duree, timertemp);
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

void ssd1306Info() {
  // Affiche les infos sur l'état du télescope
  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextSize(2);
  display.println(Message);
  display.setTextSize(2);
  //display.setCursor(0,18);
  display.println(TelPark ? "Park" : "Non P.");
  // Affiche l'état de l'abri
  //Abri ouvert,  Abri fermé, Portes ouvertes, Alim télescope, Alim moteur,  Commande moteur
  display.setTextSize(1);
  if (AbriOuvert) display.print("AO ");
  if (AbriFerme) display.print("AF ");
  if (!digitalRead(Po1)) display.print("P1 ");
  if (!digitalRead(Po2)) display.print("P2 ");
  // if (Alim12VStatus) display.print("12V ");
  if (MoteurStatus) display.print("M ");
  if (!digitalRead(MOTEUR)) display.print("*");
  display.display();
}

void ARU() {
  if (Baru || CMDARU || BoutonStopState) {  // Arret d'urgence
    // Mise à zéro de toutes les sorties
    //Stop12V;
    digitalWrite(ALIMMOT, MOTOFF); // Coupure alimentation moteur abri
    digitalWrite(ALIMTEL, HIGH);   // Coupure de l'alimentation du télescope
    //digitalWrite(MOTEUR, HIGH);    
    digitalWrite(P11, LOW);        // Arret des portes
    digitalWrite(P12, LOW);
    digitalWrite(P21, LOW);
    digitalWrite(P22, LOW);
    // Reset du timer
    //timer.deleteTimer(TimerID);
    if (BoutonStopState) {
      CMDARU=true;
      BoutonStopState=false;
    }
    if (Baru) {
      Message="ARU";
      ssd1306Info();
      delay(500);
    }
    INIT = false;
    //SURV = false;
    if (CMDARU) {
      // Attente d'appui sur le bouton ARU
      CMDARU=false;
      while (!Baru) {
        readIndi();     // Lecture des commandes Indi
      }
      delay(500);
    }
    // Attente de relachement du bouton ARU
    while(Baru) {
      readIndi();     // Lecture des commandes Indi
    }
    // Reset de l'arduino
    pinMode(RESET, OUTPUT);
    /*
    delay(5000);	// Attente de l'alimentation 12V
    // Initialisation de l'abri
    while (!initAbri()) {
      // Initialisation impossible on attend un appui du bouton ou de la clef
      Message="Init FAIL";
      ssd1306Info();
      // Attente du bouton start
      while(!Bclef) {
        readIndi();
        delay(200);
      }
    }
    */
  }
}

void Surv() {
  if (SURV) {
    // Surveillance active
    //if ((DEPL && !(TelPark)) || (!DEPL && !AbriOuvert && !AbriFerme)) {
    if (DEPL && !TelPark) {  
      // Arret d'urgence
      Message="Err park";
      CMDARU=true;
    }
    else if (!DEPL && !AbriOuvert && !AbriFerme) {
      // Arret d'urgence
      Message="Err depl";
      CMDARU=true;
    }
    else if (FERM && !TelPark) {
      // Arret d'urgence
      Message="Err park";
      CMDARU=true;
    }
  }
}

