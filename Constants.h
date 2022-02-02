/**************/
/* CONSTANTES */
/**************/

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
#define Park 	      dRead(PARK) 	      // Télescope parqué 
#define Pluie         !dRead(A6)
#define Baru          !dRead(BARU)
