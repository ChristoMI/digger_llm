#include <LSS.h>
#include <SoftwareSerial.h>

#define LSS_ID    (0)
#define LSS_BAUD  (LSS_DefaultBaud)

const int retracted_position = 975; //was 975
const int extended_position  = 75;

SoftwareSerial servoSerial(10,9); // RX, TX, IC(23, 18)
LSS lss = LSS(LSS_ID);

//int32_t servoPos;
//uint8_t servoRpm;
//uint16_t servoCurrent;

char input_char;
char mode_char;
bool extend_servo = false;

void servoRetract()
{
    lss.move(retracted_position);
}

void servoExtend()
{
    lss.move(extended_position);
}

void servoPartialRetract()
{
    lss.move(retracted_position - 300);
}

/*
void read_lss()
{
    servoPos = lss.getPosition();
    delay(5);
    servoCurrent = lss.getCurrent();

    Serial.print("Position  (1/10 deg) = "); Serial.println(servoPos);
    //Serial.print("Speed          (rpm) = "); Serial.println(servoRpm);
    Serial.print("Current         (mA) = "); Serial.println(servoCurrent);
}*/
void read_lss() //11.2ms
{
  servoPos = getServoPosition();
  delay(1);
  servoCurrent = getServoCurrent();
//  Serial.print("Servo Pos: ");
//   Serial.println(servoPos);
//   Serial.print("Servo Cur: ");
//   Serial.println(servoCurrent);
}

long getServoPosition()
{
int k = 0;
for(k = 0; k<100; k++) //clear the buffer
{
  char c = servoSerial.read();
  if (c == -1)
  {
    //Serial.println("buffer cleared ");
    break;
  }
}
char servoString[] = "000000000";
int msgSize = 10;
int sizeOfServoMessage = 0;
servoSerial.print("#0QD\r");
delay(4);
int i = 0;
  for(i=0; i<msgSize; i++)
  {
    char c = servoSerial.read();
      if(c == 13)
      {
        //Serial.println("carriage return ");
        //Serial.println(servoString);
        sizeOfServoMessage = i;
        //Serial.print("sizeOfServoMessage ");
        //Serial.println(sizeOfServoMessage);
        break;
      }
    servoString[i] = c;
    //Serial.print(c);
  }

int j = 0;
  for(j=0; j<(sizeOfServoMessage - 4);j++)
  {
  servoString[j] = servoString[j+4];
  }
  servoString[sizeOfServoMessage - 4] = '\0';
   //Serial.print("revised ");
   //Serial.println(servoString);
   long servoIntReading = atol(servoString);
    //Serial.print("servo int value ");
   //Serial.println(servoIntReading);
   return(servoIntReading);
}

long getServoCurrent()
{
int k = 0;
for(k = 0; k<100; k++) //clear the buffer
{
  char c = servoSerial.read();
  if (c == -1)
  {
    //Serial.println("buffer cleared ");
    break;
  }
}
char servoString[] = "000000000";
int msgSize = 10;
int sizeOfServoMessage = 0;
servoSerial.print("#0QC\r");
delay(4);
int i = 0;
  for(i=0; i<msgSize; i++)
  {
    char c = servoSerial.read();
      if(c == 13)
      {
        //Serial.println("carriage return ");
        //Serial.println(servoString);
        sizeOfServoMessage = i;
        //Serial.print("sizeOfServoMessage ");
        //Serial.println(sizeOfServoMessage);
        break;
      }
    servoString[i] = c;
    //Serial.print(c);
  }

int j = 0;
  for(j=0; j<(sizeOfServoMessage - 4);j++)
  {
  servoString[j] = servoString[j+4];
  }
  servoString[sizeOfServoMessage - 4] = '\0';
   //Serial.print("revised ");
   //Serial.println(servoString);
   long servoIntReading = atol(servoString);
    //Serial.print("servo int value ");
   //Serial.println(servoIntReading);
   return(servoIntReading);
}

 void servoSetup() 
{
    LSS::initBus(servoSerial, LSS_BAUD);
    delay(100);
    servoRetract();
    //delay(2000);
    //servoExtend();
}
