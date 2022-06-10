// Pinmap

//----------Sorties ----------
#define CMDMOT  A1  	// (R1) Ouverture/fermeture abri Commande moteur de porte de garage
#define ALIMMOT A2  	// (R2) Alimentation 220V moteur abri
#define ALIM12V A3    	// (R2) Alimentation 12V abri
#define ALIMTEL A4    	// Alimentation du télescope (relais externe)
#define P11     3   	// (R5) LM298 1 porte 1
#define P12     5   	// (R6) LM298 2 porte 1
#define P21     6   	// (R7) LM293 3 porte 2
#define P22     7   	// (R8) LM298 4 porte 2
#define SPARK  A8     	// Sortie Park
#define RESETMEGA	A13 // Reset de l'arduino
#define LEDV    2     	// LED verte du shield
#define LEDB    9     	// LED bleue du shield
#define LEDPIN 13		// APA106 
#define NBLEDS 24  		// Nombre total de LEDs (3 barrettes de 8 LEDs)

//---------- Entrées ----------
// Capteurs
#define AO      49      // Capteur abri ouvert
#define AF      48      // Capteur abri fermé
#define Po1     25      // Capteur porte 1 ouverte
#define Po2     24      // Capteur porte 2 ouverte
#define PARK	  A5      // Entrée Park: Etat du telescope 0: non parqué, 1: parqué
#define PLUIE   A6      // Capteur de pluie
// Boutons
#define BCLEF   A12     // Bouton à clef d'ouverture/fermeture des portes (pos 1 & 2)
#define BNOIR  	A7 	    // Bouton noir	
#define BVERT   34      // Bouton vert
#define BROUGE  46      // Bouton rouge
#define BARU    23      // Bouton d'arret d'urgence
#define BLUMT   A11     // Bouton d'éclairage de la table (rouge)  Interrupteur double
#define BLUMI   A10     // Bouton d'éclairage de l'abri   (rouge)
