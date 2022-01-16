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
#define BAUDRATE           38400       // Vitesse du port série
#define RON  HIGH                       // Etat On pour les relais (HIGH, LOW)
#define ROFF !RON
#define DELAIPORTES         40000L      // Durée d'ouverture/fermeture des portes (40000L)
#define DELAIMOTEUR         30000L      // Durée d'initialisation du moteur (40000L)
#define DELAIABRI           22000L      // Durée de déplacement de l'abri (15000L)
#define INTERVALLEPORTES    12000       // Intervalle entre la fermeture de la porte 1 et de la porte 2
#define IMPMOT              500         // Durée d'impulsion moteur
#define BAPPUILONG          3000        //Durée en ms pour un appui long sur le bouton

/******************/
/* PHERIPHERIQUES */
/******************/
// Timer
#include <SimpleTimer.h>
SimpleTimer timer;

/**************/
/* CONSTANTES */
/**************/
//----------Sorties ----------
#define CMDMOT      A1      // (R1) Ouverture/fermeture abri Commande moteur de porte de garage
#define P11         3       // (R5) LM298 1 porte 1
#define P12         5       // (R6) LM298 2 porte 1
#define P21         6       // (R7) LM293 3 porte 2
#define P22         7       // (R8) LM298 4 porte 2
//---------- Entrées ----------
// Capteurs
#define AO          49      // Capteur abri ouvert
#define AF          48      // Capteur abri fermé
#define PO1         24      // Capteur porte 1 ouverte
#define PO2         25      // Capteur porte 2 ouverte
// Boutons
#define BCLEF       A12     // Bouton à clef d'ouverture/fermeture 
#define BVERT       34      // Bouton intérieur d'ouverture/fermeture   
#define BROUGE      46      // Bouton de sélection
#define BARU        22      // Bouton d'arret d'urgence

/**********/
/* MACROS */
/**********/
#define CmdMotOff     digitalWrite(CMDMOT, ROFF)
#define CmdMotOn      digitalWrite(CMDMOT, RON)

/**********************/
/* VARIABLES GLOBALES */
/**********************/
bool AbriCours=false;   // Demande d'ouverture en cours (attend que les portes soient ouvertes)
bool ArretCours=false;  // Demande d'arret de l'abri en cours (attend que l'abri soit fermé)
bool PorteCours=false;  // Demande de déplacement des portes en cours
bool MotAbriOk=false;   // Moteur abri prêt (allumé depuis plus de DELAIMOTEUR secondes)

/*********/
/* SETUP */
/*********/
void setup() {
  // Connexion à AstroPi (port Indi)  
  Serial.begin(BAUDRATE);  		
  // Coupure de la commande du moteur de déplacement
  CmdMotOff; pinMode(CMDMOT, OUTPUT); 
  // Coupure des vérins de portes
  digitalWrite(P11, LOW); pinMode(P11, OUTPUT);
  digitalWrite(P12, LOW); pinMode(P12, OUTPUT);
  digitalWrite(P21, LOW); pinMode(P21, OUTPUT);
  digitalWrite(P22, LOW); pinMode(P22, OUTPUT);
  // Activation des entrées (capteurs...)
  pinMode(AO, INPUT_PULLUP);
  pinMode(AF, INPUT_PULLUP);
  pinMode(PO1, INPUT_PULLUP);
  pinMode(PO2, INPUT_PULLUP);
  pinMode(BCLEF, INPUT_PULLUP);
  pinMode(BVERT, INPUT_PULLUP);
  pinMode(BROUGE, INPUT_PULLUP);
  pinMode(BARU, INPUT_PULLUP);
  // Initialisation du moteur abri
  //timer.setTimeout(DELAIMOTEUR, initAbriOk); 
  timer.setTimeout(3000, initAbriOk);
}

/*********************/
/* BOUCLE PRINCIPALE */
/*********************/
void loop() {
    timer.run();
    // Demande de déplacement de l'abri en cours (attente de l'ouverture des portes)
    // Déplacement différé de l'abri
    if (AbriCours && !digitalRead(PO1) && !digitalRead(PO2)) {
        deplaceAbri(); 
        Serial.println("Demande deplacement abri effectif");
    }
    // Demande d'arret de l'abri en cours (attente de l'abri en position fermé)
    // Fermeture différée des portes
    if (ArretCours && !digitalRead(AF)) {
        fermePortes();
        // TODO arret de l'abri
        ArretCours=false;
    }
    if (!digitalRead(BVERT)) ouvreAbri();
    if (!digitalRead(BROUGE)) fermeAbri();
    delay(200);
}

/*************/
/* FONCTIONS */
/*************/

// Ouverture de l'abri
void ouvreAbri() {
    // Abri déjà ouvert
    if (!digitalRead(AO) || AbriCours || ArretCours) return;
    Serial.println("ouvre abri");
    ouvrePortes();
    AbriCours=true;     // Fermeture différée de l'abri après fermeture des portes
}

// Fermeture de l'abri
void fermeAbri() {
    if (!digitalRead(AF) || AbriCours || ArretCours) return;
    Serial.println("ferme abri");
    deplaceAbri();      // Fermeture différée des portes après fermeture de l'abri
    ArretCours=true;
}

void deplaceAbri() {
    // Moteur pas pret, portes non ouvertes
    if (!MotAbriOk || digitalRead(PO1) || digitalRead(PO2)) return;
    Serial.println("deplace abri");
    // Commande du moteur de l'abri
    CmdMotOn;
    delay(IMPMOT);
    CmdMotOff;
    AbriCours=false;
}

// Ouverture des portes
void ouvrePortes() {
    if (PorteCours) return;
    Serial.println("ouvre portes");
    // Ouverture de la porte 1
    if (!digitalRead(PO1)) {
        // Porte 1 déja ouverte
        if (digitalRead(PO2)) ouvrePorte2();
    }
    else {
        ouvrePorte1();
        // Porte 2 non ouverte
        if (digitalRead(PO2)) {
            timer.setTimeout(INTERVALLEPORTES, ouvrePorte2);    
            PorteCours=true;
        }
    }
}

void fermePortes() {
    if (PorteCours) return;
    Serial.println("ferme portes");
    // Fermeture des portes
    // Abri fermé on ferme les portes
    if (!digitalRead(AF)) {
        fermePorte2();
        PorteCours=true;
        timer.setTimeout(INTERVALLEPORTES, fermePorte1);
    }
}

void ouvrePorte1() {
    Serial.println("ouvre porte 1");
    // Ouverture de la porte 1
    digitalWrite(P12,LOW);
    digitalWrite(P11,HIGH);
}

void fermePorte1() {
    Serial.println("ferme porte 1");
    // Fermeture de la porte 1
    digitalWrite(P11,LOW);
    digitalWrite(P12,HIGH);
    PorteCours=false;
}

void ouvrePorte2() {
    Serial.println("ouvre porte 2");
    // Ouverture de la porte 2
    digitalWrite(P12,LOW);
    digitalWrite(P11,HIGH);
    PorteCours=false;
}

void fermePorte2() {
    Serial.println("ferme porte 2");
    // Fermeture de la porte 2
    digitalWrite(P21,LOW);
    digitalWrite(P22,HIGH);
}

void initAbriOk() {
    // Moteur abri pret
    MotAbriOk=true;
}

/************************/
/* FONCTIONS ROLLOFFINO */
/************************/
