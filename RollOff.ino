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
#define BARU        22      // Bouton d'arret d'urgence

/**********/
/* MACROS */
/**********/
#define CmdMotOff     digitalWrite(CMDMOT, ROFF)
#define CmdMotOn      digitalWrite(CMDMOT, RON)

/**********************/
/* VARIABLES GLOBALES */
/**********************/

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
  pinMode(BARU, INPUT_PULLUP);

// TEST
 ouvrePortes();
}

/*********************/
/* BOUCLE PRINCIPALE */
/*********************/
void loop() {

}

/*************/
/* FONCTIONS */
/*************/

// Ouverture de l'abri
void ouvreAbri() {
    ouvrePortes();
}

// Fermeture de l'abri
void fermeAbri() {
    fermePortes();
}
// Ouverture des portes
void ouvrePortes() {
    // Ouverture de la porte 1
    ouvrePorte1();
}

void fermePortes() {
    // Fermeture des portes
    fermePorte2();
}

void ouvrePorte1() {
    // Ouverture de la porte 1
    digitalWrite(P12,LOW);
    digitalWrite(P11,HIGH);
}

void fermePorte1() {
    // Fermeture de la porte 1
    digitalWrite(P11,LOW);
    digitalWrite(P12,HIGH);
}

void ouvrePorte2() {
    // Ouverture de la porte 2
}

void fermePorte2() {
    // Fermeture de la porte 2
}
/************************/
/* FONCTIONS ROLLOFFINO */
/************************/
