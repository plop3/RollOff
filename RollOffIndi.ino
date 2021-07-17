void readIndi() {
  readUSB();
}

void sendAck(char* val)
{
  char response [MAX_RESPONSE];
  if (strlen(val) > MAX_MESSAGE)
    sendNak(ERROR1);
  else
  {
    strcpy(response, "(ACK:");
    strcat(response, target);
    strcat(response, ":");
    strcat(response, val);
    strcat(response, ")");
    if (Serial.availableForWrite() > 0)
    {
      Serial.println(response);
      Serial.flush();
    }
  }
}

void sendNak(const char* errorMsg)
{
  char buffer[MAX_RESPONSE];
  if (strlen(errorMsg) > MAX_MESSAGE)
    sendNak(ERROR2);
  else
  {
    strcpy(buffer, "(NAK:ERROR:");
    strcat(buffer, value);
    strcat(buffer, ":");
    strcat(buffer, errorMsg);
    strcat(buffer, ")");
    if (Serial.availableForWrite() > 0)
    {
      Serial.println(buffer);
      Serial.flush();
    }
  }
}

bool parseCommand()           // (command:target:value)
{
  bool start = false;
  bool eof = false;
  int recv_count = 0;
  int wait = 0;
  int offset = 0;
  char startToken = '(';
  char endToken = ')';
  const int bLen = MAX_INPUT;
  char inpBuf[bLen + 1];

  memset(inpBuf, 0, sizeof(inpBuf));
  memset(command, 0, sizeof(command));
  memset(target, 0, sizeof(target));
  memset(value, 0, sizeof(value));

  while (!eof && (wait < 20))
  {
    if (Serial.available() > 0)
    {
      Serial.setTimeout(1000);
      recv_count = Serial.readBytes((inpBuf + offset), 1);
      if (recv_count == 1)
      {
        offset++;
        if (offset >= MAX_INPUT)
        {
          sendNak(ERROR3);
          return false;
        }
        if (inpBuf[offset - 1] == startToken)
        {
          start = true;
        }
        if (inpBuf[offset - 1] == endToken)
        {
          eof = true;
          inpBuf[offset] = '\0';
        }
        continue;
      }
    }
    wait++;
    delay(100);
  }

  if (!start || !eof)
  {
    if (!start && !eof)
      sendNak(ERROR4);
    else if (!start)
      sendNak(ERROR5);
    else if (!eof)
      sendNak(ERROR6);
    return false;
  }
  else
  {
    strcpy(command, strtok(inpBuf, "(:"));
    strcpy(target, strtok(NULL, ":"));
    strcpy(value, strtok(NULL, ")"));
    if ((strlen(command) >= 3) && (strlen(target) >= 1) && (strlen(value) >= 1))
    {
      return true;
    }
    else
    {
      sendNak(ERROR7);
      return false;
    }
  }
}

#define estAlimente true
void readUSB()
{
  // See if there is input available from host, read and parse it.
  if (Serial && (Serial.available() > 0))
  {
    if (parseCommand())
    {
      bool connecting = false;
      bool powerOff = false;
      bool found = true;
      t_seconds = 0;
      t_millisec = 0;
      t_prev = millis();

      if (strcmp(command, "CON") == 0)
      {
        if (!estAlimente)
        {
          remotePowerRequest = true;
        }
        timerActive = true;  // Whether power turned on manually, from a prior session or auto, When connected timer will run
        connecting = true;
        strcpy(value, INFO_1); // To indicate the version of the Arduino code
      }

      // Map the general input command term to the local action to be taken
      // SET: OPEN, CLOSE, ABORT, AUXSET
      else if (strcmp(command, "SET") == 0)
      {
        if (strcmp(target, "AUXSET") == 0)      // Relais Alimentation
        {
          if (strcmp(value, "ON") == 0)
          {
            timerActive = true;
            if (!estAlimente)
              remotePowerRequest = true;
          }
          else
          {
            timerActive = false;
            if (estAlimente)
              remotePowerRequest = true;
          }
        }
        else
        {
          if (!estAlimente)
          {
            powerOff = true;
          }

          // Power is on
          else
          {
            if (strcmp(target, "OPEN") == 0)            // BoutonOpenState
            {
              BoutonOpenState = true;
            }
            else if (strcmp(target, "CLOSE") == 0)      // BoutonCloseState
            {
              BoutonCloseState = true;
            }
            else if (strcmp(target, "ABORT") == 0)      // BoutonStopState
            {
              BoutonStopState = true;
            }
            else
            {
              found = false;
            }
          }   // End power is on
        }     // End roof movement commands
      }       // End set commands
      else
      {
        // Handle requests to obtain the status of switches
        // GET: OPENED, CLOSED, LOCKED, AUXSTATE
        strcpy(value, "OFF");

        if (strcmp(command, "GET") == 0)
        {
          if (strcmp(target, "OPENED") == 0)          // FinOuvertureState
          {
            if (AbriOuvert && !AbriFerme)
              strcpy(value, "ON");
          }
          else if (strcmp(target, "CLOSED") == 0)     // FinFermetureState
          {
            if (AbriFerme && !AbriOuvert && PortesFerme)
              strcpy(value, "ON");
          }
          else if (strcmp(target, "LOCKED") == 0)     // Telescope_ParcState
          {
            if ((!TelPark) || !estAlimente)   // Unsafe telescope or power off, either could indicate a lock preventing open/close commands
              strcpy(value, "ON");
          }
          else if (strcmp(target, "AUXSTATE") == 0)   // Relais Alimentation
          {
            if (estAlimente)
              strcpy(value, "ON");
          }
          else
          {
            found = false;
          }
        } // End GET command
        else
        {
            found = false;                             // Not a known command
        }
      } // End looking for commands

      /*
         See if the command was recognized
      */
      if (!connecting && !found)
      {
        sendNak(ERROR8);
      }
      else if (powerOff)
      {
        sendNak(ERROR11);
      }
      else
      {
        sendAck(value);       // Found a CON, SET or GET
      }
    }   // end of parseCommand
  }     // end look for USB input
}
