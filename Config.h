// Configuration de l'abri

#define DEBUG             false   // Mode debug série
#define BAUDRATE 	        9600    // Vitesse du port série
#define RON HIGH       		        // Etat On pour les relais (HIGH, LOW)
#define ROFF !RON
#define DELAIPORTES       40000L  // Durée d'ouverture/fermeture des portes (40000L)
#define INTERVALLEPORTES  12000   // Intervalle entre la fermeture de la porte 1 et de la porte 2
#define DELAIABRI         22000L  // Durée de déplacement de l'abri (15000L)
#define DELAIMOTEUR       50000L  // Délai d'initialisation du moteur abri
#define IMPMOT            500     // Durée d'impulsion moteur
#define BAPPUILONG        3000    //Durée en ms pour un appui long sur le bouton
#define TPSPARK           180000  // Temps de park du télescope
#define TPSARRET          600000L // Temps avant l'arret de l'alimentation 12V
