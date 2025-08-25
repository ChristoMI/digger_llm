//Digger system

//#include <SoftwareSerial.h>
#include <ODriveArduino.h>

#include "digger.h"


long ElevatorZero = 0;

long superLoopTime = 0;
bool fsm = false; //disabled by default
//uint8_t global_sensor_trigger = 0;
long previousDataGetTime = 0;

extern volatile byte limitSWfeedback;
int AugerRPMDefault = 200; //Set default auger speed here


void setup() 
{
  Serial.begin(115200);   // Serial to PC
  Serial1.begin(115200);  // Serial to oDrive
  Serial2.begin(115200);  // Serial to Rpi
  Serial3.begin(57600);  // Serial to Xbee

  
  message("Initializing components... Digger V4.2, Alta Variant w/ surface placement");
  //strainGaugeSetup();s
  limitSwitchSetup();
  message("Sending motor configs...");
  sendMotorControlConfigs(); //write and save motor commands
  message("Setting up servo control...");
  servoSetup();
  message("Setting up Teensy interface...");
  setupTeensyInterface();
  setAccelFlag();
  
  //IMUSetup();

  //grab the initial position of elevator
  //Elevator Encoder Position in counts
  getData(NO_PRINT);
  
  groundLvl = mCtlEnc1;
  startLvl = groundLvl;
  deepestDepth = groundLvl;
  message("Initial ground level encoder count: " + String(groundLvl,4));

  if (mode == 0)
    message("Mode: Adaptive Digging");
  if (mode == 1)
    message("Mode: Constant Rate Digging");
  if (mode == 2)
    message("Mode: Surface Placement");
    
  //message("Initial data from sensors:");
  getData(PRINT);

  //heartbeat starting
  UISerial.println("h");
  heartbeat = millis();

  Serial2.println("Test S2");
}


void loop() {

  //Check for serial commands
  checkComms();

  //Check for Odrive limits
  checkSuperLoop();

  //Update accelerometer info
  updateAccel();

  if (alwaysStreamData)
    getData(PRINT);

  if (fsm)
  {
    runState();
    //delay(1000);
  }

  //send heartbeat at 4 hz
  if ( (millis() - heartbeat) > 250 )
  {
    heartbeat = millis();
    UISerial.println("h");
  }


}
