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

/*****************/
/* PERIPHERIQUES */
/*****************/
// Timer
#include <SimpleTimer.h>
SimpleTimer timer;

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

//---------- Entrées ----------
// Capteurs
#define AO  49       // Capteur abri ouvert
#define AF  48       // Capteur abri fermé
#define Po1 24       // Capteur porte 1 ouverte
#define Po2 25       // Capteur porte 2 ouverte
// Boutons
#define BCLEF   A12    // Bouton à clef d'ouverture/fermeture des portes (pos 1 & 2)
#define BVERT  	34 	   // Bouton intérieur d'ouverture/fermeture

/**********/
/* MACROS */
/**********/
#define PortesOuvert  (!digitalRead(Po1) && !digitalRead(Po2)) // && Alim12VStatus)
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

/**********************/
/* VARIABLES GLOBALES */
/**********************/
bool PortesFerme = true;

/*********/
/* SETUP */
/*********/
void setup() {
  // Initialisation des ports série
  Serial.begin(BAUDRATE);  		// Connexion à AstroPi (port Indi)
  
  
  // Initialisation des relais
  CmdMotOff; pinMode(CMDMOT, OUTPUT); // Coupure de la commande du moteur de déplacement
  MotOff; pinMode(ALIMMOT,OUTPUT);
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
  // Abri fermé: moteur abri OFF, sinon ON
  MotOn;
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
  CmdMotOn;
  delay(IMPMOT);
  CmdMotOff;
  // Attand le positionnement de l'abri ou l'annulation du déplacement
  return true;
}

bool ouvreAbri() {
	// Ouvre l'abri
  // Conditions: Pas de déplacement, pas de lock, télescope parqué
  if (AbriOuvert) return true;  	// Abri déjà ouvert
  // Gestion appui long (clef et bouton vert)
  if (ouvrePortes()) {
	  if (deplaceAbri() && AbriOuvert) {
      return true;
    }
  }  
}

bool fermeAbri() {
  // Ferme l'abri
  // Si les portes sont fermées et l'abri ouvert on passe temporairement en auto pour ouvrir les portes
  if (AbriFerme) return true; // Abri déjà fermé
  if (deplaceAbri() && AbriFerme) {
    if (fermePortes()) {
      return true;
    }
  }
  return false;
}

bool ouvrePortes() {
	// Ouvre les portes
  OuvreP1;
  delay(INTERVALLEPORTES);
	OuvreP2;
  delay(DELAIPORTES);
}

bool fermePortes() {
	// Ferme les portes
  FermeP2;
  delay(INTERVALLEPORTES);
  FermeP1;
	delay(DELAIPORTES);
}

void readBoutons() {
  // Lecture des boutons de l'abri
  // Déplacement en cours, on ne fait rien
  if (Bclef || Bvert) {
    // Déplacement de l'abri
	  if (!AbriOuvert) {
  	  // Ouverture abri (abri non fermé)
		  ouvreAbri();
	  }
	  // Fermeture abri
	  else {
		  fermeAbri();
    }
	}
}

void pool() {
  // Timers
  timer.run();  
  // Fonctions périodiques
  readBoutons();
}

//---------- Fonctions Timer ----------

