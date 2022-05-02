/**************/
/* CONSTANTES */
/**************/
//---------- RollOffIno ----------
const int cLen = 15;
const int tLen = 15;
const int vLen = MAX_RESPONSE;

/**********/
/* MACROS */
/**********/
#define PortesOuvert  (!dRead(Po1) && !dRead(Po2))
//#define PortesFerme   (dRead(Po1) && dRead(Po2)) 
#define Porte1Ouvert  (!dRead(Po1))
#define Porte2Ouvert  (!dRead(Po2))
#define AbriFerme     (!dRead(AF))
#define AbriOuvert    (!dRead(AO))
#define Stop12V       digitalWrite(ALIM12V, RON)
#define Start12V      digitalWrite(ALIM12V, ROFF)
#define StopTel       digitalWrite(ALIMTEL, HIGH)
#define StartTel      digitalWrite(ALIMTEL, LOW)
#define CmdMotOff     digitalWrite(CMDMOT, ROFF)
#define CmdMotOn      digitalWrite(CMDMOT, RON)
#define MotOn 	  	  digitalWrite(ALIMMOT, RON)
#define MotOff 	      digitalWrite(ALIMMOT, ROFF)
#define OuvreP1       digitalWrite(P12,LOW);digitalWrite(P11,HIGH)
#define OuvreP2       digitalWrite(P22,LOW);digitalWrite(P21,HIGH)
#define FermeP1       digitalWrite(P11,LOW);digitalWrite(P12,HIGH)
#define FermeP2       digitalWrite(P21,LOW);digitalWrite(P22,HIGH)
#define Bclef         !dRead(BCLEF)
#define Bnoir         !dRead(BNOIR)
#define Bvert         !dRead(BVERT)
#define Brouge        !dRead(BROUGE)
#define MoteurStatus  (dRead(ALIMMOT) == RON) // Alimentation du moteur abri
#define Status12V     (dRead(ALIM12V) == RON) // Alimentation 12V
//#define Park 	      dRead(PARK) 	      // Télescope parqué 
#define Park          true
#define Pluie         !dRead(A6)
#define Baru          !dRead(BARU)
