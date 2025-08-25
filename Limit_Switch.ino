

void limitSwitchSetup()
{
pinMode(LimitSWTop, INPUT_PULLUP);
pinMode(LimitSWBot, INPUT_PULLUP);
}

void pollLimitSwitches()
{
  
pollTopLimitSW();
pollBotLimitSW();
//Serial.print("limit sw feedback: ");
//Serial.println(limitSWfeedback, BIN);
}


//Read the limit switch, return 1 if not pressed, 0 if pressed. 
void pollTopLimitSW()
{
  
  boolean topSWRawState = digitalRead(LimitSWTop);
//Serial.println(topSWRawState);
  if (topSWRawState != topSWLastButtonState) {
    // reset the debouncing timer
    topSWLastDebounceTime = millis();
  }
    if ((millis() - topSWLastDebounceTime) > debounceDelay) {
    if ((topSWRawState != topSWState)|(initLimitSW)) {
      topSWState = topSWRawState;
      //Serial.println("A");

      if (topSWState == LOW) {
      //Serial.println("top switch pushed");
      limitSWfeedback = (limitSWfeedback & B11111110); //top switch signal is low
      limitSWfeedback = (limitSWfeedback | B00000100); // top switch transition is falling
      limitSWfeedback = (limitSWfeedback | B01000000); // a top switch transition occured
      }
      
      if (topSWState == HIGH) {
      //Serial.println("top switch released");
      limitSWfeedback = (limitSWfeedback | B00000001); // top switch signal is high
      limitSWfeedback = (limitSWfeedback | B00001000); // top switch transition is rising
      limitSWfeedback = (limitSWfeedback | B01000000); // a top switch transition ocurred
      }
    }
  }
topSWLastButtonState = topSWRawState;
}

void pollBotLimitSW()
{
boolean botSWRawState = digitalRead(LimitSWBot);

  if (botSWRawState != botSWLastButtonState) {
    // reset the debouncing timer
    botSWLastDebounceTime = millis();
  }
    if ((millis() - botSWLastDebounceTime) > debounceDelay) {
    if ((botSWRawState != botSWState) | (initLimitSW)) {
      initLimitSW = 0; //runs once at start of program
      botSWState = botSWRawState;

      if (botSWState == LOW) {
      //Serial.println("bottom switch pushed");
      limitSWfeedback = (limitSWfeedback & B11111101); //bottom switch signal is low
      limitSWfeedback = (limitSWfeedback | B00010000); // bottom switch transition is falling
      limitSWfeedback = (limitSWfeedback | B10000000); // abottomp switch transition occured
      }
      
      if (botSWState == HIGH) {
      //Serial.println("bottom switch released");
      limitSWfeedback = (limitSWfeedback | B00000010); // bottom switch signal is high
      limitSWfeedback = (limitSWfeedback | B00100000); // bottom switch transition is rising
      limitSWfeedback = (limitSWfeedback | B10000000); // a bottom switch transition ocurred
      }
    }
  }
  botSWLastButtonState = botSWRawState;
}

//clears the transition bits. 
void clearLSTransitionFeedback()
{
  limitSWfeedback = limitSWfeedback & B00000011; // clear the transtion indicator bits
}
