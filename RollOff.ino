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
  TODO:
	- Arret de l'alimentation 12V
  - Sortie Park pour OnStepX
  - Entrée capteur pluiz
  OPTIONS:
  - Barrière(s) IR de sécurité
	- Clavier codé
  /*********************************/

//---------------------------------------Modules--------------------------------
#include "RollOffIndi.h" // Gestion de l'abri par Indi (https://github.com/wotalota/indi-rolloffino)

//---------------------------------------PARAMETRES-----------------------------
#define PARKONSTEP false    // 0: Park par ESPServer, 1: Park par entrée OnStepX
#define CAPTEURPLUIE false  // Capteur de pluie présent (true)

//---------------------------------------PERIPHERIQUES--------------------------

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
#define ALIM12V A3  // (R3) Mise en marche de l'alimentation 12V de l'abri (vérins portes, capteurs)
#define ALIMTEL A4   // (R4) Alimentation télescope    Relais dans le boitier OnStep)
#define ALIMMOT A2  // (R2) Alimentation 220V moteur abri
#define MOTEUR  A1  // (R1) Ouverture/fermeture abri Commande moteur de porte de garage
#define P11     3   // (R5) LM298 1 porte 1
#define P12     5   // (R6) LM298 2 porte 1
#define P21     6   // (R7) LM293 3 porte 2
#define P22     7   // (R8) LM298 4 porte 2
#define SPARK   8   // Sortie ordre de park vers OnStepX

#define RESET   A13 // Reset de l'arduino

// Entrées
#define PARK  A5     // Entrée Park: Etat du telescope 0: non parqué, 1: parqué

#define AO  49       // Capteur abri ouvert
#define AF  48       // Capteur abri fermé
#define Po1 24       // Capteur porte 1 ouverte
#define Po2 25       // Capteur porte 2 ouverte
#define PLUIE 26     // Capteur de pluie

// Boutons
#define B1CLEF   A12    // Bouton à clef d'ouverture/fermeture des portes (pos 1 & 2)
#define BARU     22    // Bouton d'arret d'urgence
#define BLUMT    A11    // Bouton d'éclairage de la table (rouge)  Interrupteur double
#define BLUMI    A10    // Bouton d'éclairage de l'abri   (rouge)

// Constantes globales
#define DELAIPORTES 50000L          // Durée d'ouverture/fermeture des portes (40000L)
#define DELAIMOTEUR 40000L          // Durée d'initialisation du moteur (40000L)
#define DELAIABRI   20000L          // Durée de déplacement de l'abri (15000L)
#define INTERVALLEPORTES 12000       // Intervalle entre la fermeture de la porte 1 et de la porte 2
#define MOTOFF HIGH                 // Etat pour l'arret du moteur
#define MOTON !MOTOFF
#define IMPMOT 300                 // Durée d'impulsion moteur

#define DELAIMQTT 30000UL            // Rafraichissement MQTT
// Temps maxi de park en millisecondes
#define TPSPARK 180000

//---------------------------------------Macros---------------------------------
#define AlimTelStatus (!digitalRead(ALIMTEL))    // Etat de l'alimentation télescope
#define Alim12VStatus (digitalRead(ALIM12V))  // Etat de l'alimentation 12V ATX
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
#define Stop12V       digitalWrite(ALIM12V, LOW)
#define Start12V      digitalWrite(ALIM12V, HIGH)
#define TelParkStateA (analogRead(PARK)>300)
#define TelParkStateN  digitalRead(PARK)
#define OuvreP1       digitalWrite(P12,LOW);digitalWrite(P11,HIGH)
#define OuvreP2       digitalWrite(P22,LOW);digitalWrite(P21,HIGH)
#define FermeP1       digitalWrite(P11,LOW);digitalWrite(P12,HIGH)
#define FermeP2       digitalWrite(P21,LOW);digitalWrite(P22,HIGH)
#define Bclef         !digitalRead(B1CLEF)
#define Baru          !digitalRead(BARU)
#define Pluie         digitalRead(PLUIE)

#define BAPPUILONG  3000  //Durée en ms pour un appui long sur le bouton

// Telnet client
#define DEFAULT_SOCKET_TIMEOUT	5000


// ------------------------------------Variables globales------------------------------------

int countM;           // Nombre d'essais ouverture/fermeture

bool PortesFerme = true;
bool TBOUTON = false; // Temporisation bouton
bool DEPL = false;    // Abri en cours de déplacement
bool FERM = false;    // Portes en cours de fermeture
bool CMDARU = false;  // Commande interne, ou Indi d'arret d'urgence
bool AUTO = false;    // Abri en mode automatique (commande reçue à distance)
bool Bmem = false;    // Mémorisation du bouton

bool BLUMTO = !digitalRead(BLUMT);  // Dernier etat du bouton d'éclairage table
bool BLUMIO = !digitalRead(BLUMI);  // Dernier etat du bouton d'éclairage intérieur

bool CapteurPluie = false;          // Etat du capteur de pluie pour HASS (MQTT)

String Message = "";                // Message affiché sur l'écran OLED

bool connected;                     // ESP-Link MQTT

char * const tcpServer PROGMEM = "192.168.0.15";
uint16_t const tcpPort PROGMEM = 9999;

// ------------------------------------ Client Telnet ------------------------------------
// Telnet client
ELClientSocket tcp(&esp);

// ------------------------------------  MQTT ------------------------------------
// MQTT
// Callback when MQTT is connected
void mqttConnected(void* response) {
  Serial3.println("MQTT connected!");
  mqtt.subscribe("esp-abri/set");
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
  if (topic == "esp-abri/set") {
    //if (data == "ON") BoutonOpenState = true;
    //if (data == "OFF") BoutonCloseState = true;
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
  Serial.begin(9600);  		// Connexion à AstroPi (port Indi)
  Serial3.begin(115200);  	// Connexion ESP-Link

  // Initialisation des relais
  // Stop12V; pinMode(ALIM12V, OUTPUT);	 // Coupure de l'alimentation 12 abri
  StopTel; pinMode(ALIMTEL, OUTPUT);  // Coupure de l'alimentation 12V du télescope
  StopMot; pinMode(ALIMMOT, OUTPUT);  // Coupure alimentation moteur abri
  CmDMotOff; pinMode(MOTEUR, OUTPUT); // Coupure de la commande du moteur de déplacement
  // Initialisation du LM298
  digitalWrite(P11, LOW); pinMode(P11, OUTPUT);
  digitalWrite(P12, LOW); pinMode(P12, OUTPUT);
  digitalWrite(P21, LOW); pinMode(P21, OUTPUT);
  digitalWrite(P22, LOW); pinMode(P22, OUTPUT);
  digitalWrite(ALIMMOT, MOTOFF);
  digitalWrite(SPARK, LOW); pinMode(SPARK, OUTPUT);

  // Activation des entrées (capteurs...)
  pinMode(AO, INPUT_PULLUP);
  pinMode(AF, INPUT_PULLUP);
  pinMode(Po1, INPUT_PULLUP);
  pinMode(Po2, INPUT_PULLUP);
  pinMode(PLUIE, INPUT); 
  pinMode(B1CLEF, INPUT_PULLUP);
  pinMode(BARU, INPUT_PULLUP);
  pinMode(BLUMT, INPUT_PULLUP);
  pinMode(BLUMI, INPUT_PULLUP);
  //pinMode(PARK, INPUT_PULLUP); // Si le télescope n'est pas branché, considéré comme parqué
  pinMode(PARK, INPUT);
  barre(0, 0); // Extinction des barres de LEDs
  barre(1, 0);
  barre(2, 0);

  // MQTT
  Serial3.print("ESPSYNC:");
  bool ok;
  do {
    ok = esp.Sync();
  } while (!ok);
  Serial3.println("EL-Client synced!");

  mqtt.connectedCb.attach(mqttConnected);
  mqtt.disconnectedCb.attach(mqttDisconnected);
  mqtt.publishedCb.attach(mqttPublished);
  mqtt.dataCb.attach(mqttData);
  mqtt.setup();

  // Client TCP
  tcp.begin(tcpServer, tcpPort, SOCKET_TCP_CLIENT);
  refreshMQTT();
  if (!AbriOuvert && !AbriFerme && PortesOuvert) DEPL = true; 	// Abri non positionné, considéré comme en déplacement
  //StartMot;
  mqtt.publish("esp-abri/msg", "initialisation_abri");
  delay(500); // Attente pour les capteurs

    // Abri ouvert, démarrage de l'alimentation télescope
    if (AbriOuvert) {
        StartTel;
        StartMot;
    }
}

//---------------------------BOUCLE PRINCIPALE-----------------------------------------------

void loop() {
  pool(); // fonctions périodiques
  // Gestion de l'abri
  // Attente d'une commande
  Message = "OK";
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
    Bmem = false;
    fermePortes();
  }
  else if (!AbriOuvert && (Bmem || BoutonOpenState)) {
    // Ouverture abri (abri non fermé)
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
void refreshMQTT() {
  // Mise à jour des infos MQTT
  //Serial3.println("plop");
  if (AbriOuvert && !AbriFerme) {
    mqtt.publish("esp-abri/status", "ON");
  }
  else if (AbriFerme && !AbriOuvert) {
    mqtt.publish("esp-abri/status", "OFF");
  }
  /*else {
    #  mqtt.publish("esp-abri/status","move");
    }*/
}

bool TelPark() {
  //Test du park télescope
  if (TelParkStateA) return (true);
  // 2e chance
  delay(100);
  return (TelParkStateA);
}

bool deplaceAbri() {
  if (!TelPark() || !PortesOuvert) return (false);
  barre(0, 128);
  if (!MoteurStatus) {StartMot; delay(DELAIMOTEUR);};
  Message = "Depl abri";
  CmDMotOn;
  delay(IMPMOT);
  CmDMotOff;
  DEPL = true;
  mqtt.publish("esp-abri/msg", "deplacement_abri");
  attend(DELAIABRI);
  while (!AbriOuvert && !AbriFerme); // Attente des capteurs
  barre(0, 0);
  DEPL = false;
  return (true);
}

bool ouvreAbri() {
  mqtt.publish("esp-abri/msg", "ouverture_abri");
  if (AbriOuvert) return (true);  // Abri déjà ouvert
  if (!PortesOuvert) StartMot; // Démarrage du moteur abri
  // Ouverture des portes si besoin
  if (ouvrePortes()) {
    if (TBOUTON) return (false);    // Ouverture seule des portes
    StartTel;           // Mise en marche du télescope
    if (deplaceAbri() && AbriOuvert) {
      //StartTel;       // Mise en marche du télescope (BUG Kstars démarre trop tôt le déplacement de la monture)
      refreshMQTT();
      return (true);
    }
    else {
      StopTel;
      StopMot;
    }
  }
  refreshMQTT();
  return (false);
}

bool fermeAbri() {
  mqtt.publish("esp-abri/msg", "fermeture_abri");
  if (AbriFerme) return (true);
  if (!TelPark()) {
    mqtt.publish("esp-abri/msg", "park_telescope");
    if (!PARKONSTEP) {
      // Tentative de parquer le télescope par les commandes OnStep
      tcp.send(":Q#");   // Arret du mouvement
      delay(2200);
      tcp.send(":Te#");   // Tracking On
      delay(2200);
      tcp.send(":hP#");   // Park du télescope
    }
    else {
      digitalWrite(SPARK, HIGH);  // Park du télescope par entrée OnStepX
      delay(500);
      digitalWrite(SPARK, LOW);
    }
    // On attend 3mn max que le télescope soit parqué
    unsigned long tpsdebut = millis();
    unsigned long tpsact;
    do {
      tpsact = millis();
    } while ((tpsact - tpsdebut) < TPSPARK && !TelPark());
    delay(5000);  // Attente du park complet
    if (!TelPark())
    {
      mqtt.publish("esp-abri/msg", "park_impossible");
      refreshMQTT();
      return (false);
    }
    mqtt.publish("esp-abri/msg", "telescope_parque");
  }
  mqtt.publish("esp-abri/msg", "arret_telescope");
  StopTel;      // Arret du télescope
  if (deplaceAbri() && AbriFerme) {
    StopMot;      // Arret du moteur abri
    if (fermePortes()) {
      refreshMQTT();
      return (true);
    }
  }
  refreshMQTT();
  return (false);
}

bool ouvrePortes() {
  if (PortesOuvert) {
    // Portes ouvertes
    OuvreP1;
    OuvreP2;
    //attend(BAPPUILONG + 500);
  }
  else {
    // Portes fermées
    // Ouverture porte 1
    Message = "Ouvre P";
    OuvreP1;
    attend(INTERVALLEPORTES);
    OuvreP2;
    attend(DELAIPORTES);
    while (!PortesOuvert) {
      attend(100);
    }
  }
  PortesFerme = false;
  return (true);
}

bool fermePortes() {
  if (!AbriFerme || AbriOuvert) return (false);
  Message = "Ferme P";
  FERM = true;
  FermeP2;
  Message = "P2";
  attend(INTERVALLEPORTES);
  FermeP1;
  Message = "P1";
  attend(DELAIPORTES);
  Message = "P fermes";
  FERM = false;
  PortesFerme = true;
  return (true);
}

void attend(unsigned long delai) {
  unsigned long previousMillis = millis();
  unsigned long currentMillis;
  do {
    currentMillis = millis();
    pool();
  } while (currentMillis - previousMillis <= delai);
}

void pool() {
  // Fonctions périodiques
  esp.Process();  // Gestion de ESP-Link
  ARU();          // Gestion arret d'urgence
  Surv();         // Surveillance de l'abri
  if (AbriOuvert) meteo();        // Détection de pluie (vent...)
  readIndi();     // Lecture des commandes Indi
  timer.run();    // Gestion des timers
  ssd1306Info();  // Info sur l'écran OLED
  eclairages();   // Gestion des éclairages
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
  display.println(TelPark() ? "Park" : "Non P.");
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
    StopMot; 						// Coupure alimentation moteur abri
    StopTel;   						// Coupure de l'alimentation du télescope
    digitalWrite(P11, LOW);        	// Arret des portes
    digitalWrite(P12, LOW);
    digitalWrite(P21, LOW);
    digitalWrite(P22, LOW);

    if (BoutonStopState) {
      CMDARU = true;
      BoutonStopState = false;
    }
    if (Baru) {
      Message = "ARU";
      ssd1306Info();
      delay(500);
    }
    if (CMDARU) {
      // Attente d'appui sur le bouton ARU
      CMDARU = false;
      while (!Baru) {
        readIndi();     // Lecture des commandes Indi
      }
      delay(1000);
    }
    // Attente de relachement du bouton ARU
    while (Baru) {
      readIndi();     // Lecture des commandes Indi
    }
    // Reset de l'arduino
    pinMode(RESET, OUTPUT);
  }
}

void Surv() {
  // Déplacement et le télescope perd le park
  if (DEPL && !TelPark()) {
    // Arret d'urgence
    Message = "Err park";
    CMDARU = true;
  }
  // Fermeture des portes et le télescope perd le park
  if (FERM && !TelPark()) {
    // Arret d'urgence
    Message = "Err park";
    CMDARU = true;
  }
  // Déplacement intenpestif de l'abri sauf si portes ouvertes et télescope parqué (pour réglages...)
  if (!DEPL && !AbriOuvert && !AbriFerme && (!TelPark || !PortesOuvert)) {
    Message = "Err depl";
    CMDARU = true;
  }
}

bool meteo() {
  // Sécurité météo: pluie, (vent...)
  if (Pluie) {
    mqtt.publish("esp-abri/msg", "alerte pluie");
    /* TODO
      - Passer le capteur pluie à ON pour HASS
      - Lecture analogique du capteur ? (voir WeatherRadio)
    */
    fermeAbri();
  }
}