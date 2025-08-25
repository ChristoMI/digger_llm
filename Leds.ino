#define HeartbeatPin A2 //pin 95
#define LED1Pin  A1 //96
#define LED2Pin A0

//#define DIAG1Pin 41 //pin 51
//#define DIAG2Pin 37 //pin 53
//#define DIAG3Pin 40//pin 52
//#define DIAG4Pin 7 //pin 16
//#define DIAG5Pin 6 //pin 15

bool DIAG1PinStatus = LOW;
bool DIAG2PinStatus = LOW;
bool DIAG3PinStatus = LOW;
bool DIAG4PinStatus = LOW;
bool DIAG5PinStatus = LOW;


bool heartbeatStatus = LOW;

void setupLeds()
{
  pinMode(HeartbeatPin, OUTPUT);
  pinMode(LED1Pin, OUTPUT);
  pinMode(LED2Pin, OUTPUT);

  digitalWrite(LED2Pin, HIGH);
  digitalWrite(LED2Pin, HIGH);
  digitalWrite(HeartbeatPin, HIGH);
  delay(200);
  digitalWrite(LED2Pin, LOW);
  digitalWrite(LED2Pin, LOW);
  digitalWrite(HeartbeatPin, LOW);

  pinMode(DIAG1Pin, OUTPUT);
  pinMode(DIAG2Pin, OUTPUT);
  pinMode(DIAG3Pin, OUTPUT);
  pinMode(DIAG4Pin, OUTPUT);
  pinMode(DIAG5Pin, OUTPUT);

  digitalWrite(DIAG1Pin, LOW);
  digitalWrite(DIAG2Pin, LOW);
  digitalWrite(DIAG3Pin, LOW);
  digitalWrite(DIAG4Pin, LOW);
  digitalWrite(DIAG5Pin, LOW);

}

void heartbeatToggle()
{
  if(heartbeatStatus == LOW)
  {
      digitalWrite(HeartbeatPin, HIGH);
      heartbeatStatus = HIGH;
  }
  else if (heartbeatStatus == HIGH)
   {
      digitalWrite(HeartbeatPin, LOW);
      heartbeatStatus = LOW;
  }
}


void toggleDIAGPin(int pinNum)
{
  if(pinNum == 1)
  {    
    DIAG1PinStatus = !DIAG1PinStatus;
    digitalWrite(DIAG1Pin, DIAG1PinStatus);
  }
    if(pinNum == 2)
  {    
    DIAG2PinStatus = !DIAG2PinStatus;
    digitalWrite(DIAG2Pin, DIAG2PinStatus);
  }
      if(pinNum == 3)
  {    
    DIAG3PinStatus = !DIAG3PinStatus;
    digitalWrite(DIAG3Pin, DIAG3PinStatus);
  }
      if(pinNum == 4)
  {    
    DIAG4PinStatus = !DIAG4PinStatus;
    digitalWrite(DIAG4Pin, DIAG4PinStatus);
  }
      if(pinNum == 5)
  {    
    DIAG5PinStatus = !DIAG5PinStatus;
    digitalWrite(DIAG5Pin, DIAG5PinStatus);
  }
}
