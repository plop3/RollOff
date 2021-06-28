/*********************************/
#define BAUD_RATE 38400

#define MAX_INPUT 45
#define MAX_RESPONSE 127
#define MAX_MESSAGE 63

#define MAX_INACTIVE_TIME 5  // 20 minutes of inactivity from roof driver before auto shutdown (maximum is 16 hours)
/*********************************/
int BoutonOpenState      = false;
int BoutonCloseState     = false;
int BoutonStopState      = false;

//bool BoutonRelaisState   = HIGH;    // Changed

//les états possible

enum {Repos, Ouverture, Ouvert, Fermeture, Fermer};  // At Rest, Opening. Open, Closing, Closed
int etat = Repos;

//bool estAlimente = false;

/*********************************/
const int cLen = 15;
const int tLen = 15;
const int vLen = MAX_RESPONSE;
char command[cLen + 1];
char target[tLen + 1];
char value[vLen + 1];
bool remotePowerRequest = false;
bool timerActive = false;
unsigned long t_seconds = 0;    // seconds accumulated since last activity
unsigned long t_millisec = 0;   // milli-seconds accumulated since last checked
unsigned long t_prev = 0;       // milli-second count when last checked

// Maximum length of messages = 63
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
const char* ERROR11 = "Observatory power is off, command ignored";

const char* INFO_1 = "V1.1-1";