//Adaptive Digging Finite State Machine
//testing the file

void runState()
{

  //Check the simming flag, if simming
  //populate sensor values with sim values that
  //have been sent from the gui

  if (simming)
  {
    
  }

  //message("currentState: " + String(currentState));
  
  switch (currentState)
  {
    case START:
      runStart();
      break;

    case CRA:
      runCRA();
      break;

    case PECK_UP_LONG:
      runPeckUpLong();
      break;
      
    case PECK_UP_SHORT:
      runPeckUpShort();
      break;
      
    case PECK_RETURN:
      runPeckReturn();
      break;
      
    case BITE_UP:
      runBiteUp();
      break;
      
    case BITE_RETURN:
      runBiteReturn();
      break;
      
    case SLOW_RPM:
      break;
      
    case SLOW_ELEVATOR:
      break;

    case STOP:
      message("Stopping state machine: " + currentState);
      fsm = false;
      break;

    case SUCCEED:
      isOpen = false;
      dt = millis();
      fsm = false;

      //Open the auger clamps
      servoExtend();
      isOpen = true;
      delay(1000);

      //Raise the elevator to the start position
      upVel(65000);
      while (mCtlEnc1 >= startLvl)
      {
        getData(NO_PRINT);

        //Close the auger clamps after 1 second
        if (isOpen)
        {
          if ((millis() - dt) > 1000)
          {
            servoRetract();
            isOpen = false;
          }
        }
      }
      stopAllMotors();
      currentState = STOP;
      break;

    case FAIL:
      fsm = false;
      break;
      
    case START_CRA_CONTROL:
      runSTART_CRA_CONTROL();
      break;

    case CRA_CONTROL:
      runCRA_CONTROL();
      break;

    case 14:
      runStartSFC();
      break;

    case 15:
      runSFC();
      break;

    case PLACE_SENSOR:
      if (WOB > 2)
      {
        fsm = false;
        stopAllMotors();
        delay(500);
        servoExtend();
        isOpen = true;
        delay(500);
        upVel(20000);
        while (mCtlEnc1 >= startLvl)
        {
          getData(NO_PRINT);
        }
        stopAllMotors();
        servoRetract();
        currentState = SUCCEED;
      }
      break;
  }
  
}


void runStartSFC()
{
//  //Find the surface.  Advance down
//  //until 5 lbs of pressure...
//  while (WOB < 5)
//  {
//    downVel(7500);
//    getData(PRINT);
//  }
//
//  stopAllMotors();
//
//  //Save the "ground level"
//  groundLvl = mCtlEnc1;
//
//  //Reset the deepest depth
//  deepestDepth = mCtlEnc1; //track the bottom of the hole
//  depthCm = 0.0;
//
//  //Come back up a bit
//  long currentLvl = mCtlEnc1;
//  target = mCtlEnc1 - 5000;
//  
//
//  
//  while (currentLvl > target)
//  {
//    upVel(7500);
//    getData(PRINT);
//    currentLvl = mCtlEnc1; 
//  }

  stopAllMotors();


  //Set final depth target of 11 cm down for screw
  depthTarget = -11.0;

  //Switch to constant rate augering
  currentState = SFC;
  motorCommandSent = false;
  checkingDegraded = false;
  

  //Grab start time
  digStartTime = mCtlSeconds;
  checkDegradedSeconds = mCtlSeconds;
  lastSoilCheck = millis();
  message("Starting surface sensor insertion at: " + String(digStartTime,3));
}


void runStart()
{
  //Find the surface.  Advance down
  //until 5 lbs of pressure...
  while (WOB < 5)
  {
    downVel(7500);
    getData(PRINT);
  }

  stopAllMotors();

  //Save the "ground level"
  groundLvl = mCtlEnc1;

  //Reset the deepest depth
  deepestDepth = mCtlEnc1; //track the bottom of the hole
  depthCm = 0.0;

  //Come back up a bit
  long currentLvl = mCtlEnc1;
  target = mCtlEnc1 - 5000;
  while (currentLvl > target)
  {
    upVel(7500);
    getData(PRINT);
    currentLvl = mCtlEnc1; 
  }

  stopAllMotors();

  //Switch to constant rate augering
  currentState = CRA;
  motorCommandSent = false;
  checkingDegraded = false;
  

  //Grab start time
  digStartTime = mCtlSeconds;
  checkDegradedSeconds = mCtlSeconds;
  lastSoilCheck = millis();
  message("Starting dig at: " + String(digStartTime,3));
}

void runSTART_CRA_CONTROL()
{
  //This combines START and CRA with no adapative responses
    //Find the surface.  Advance down
  //until 5 lbs of pressure...
  while (WOB < 5)
  {
    downVel(7500);
    getData(PRINT);
  }

  stopAllMotors();

  //Save the "ground level"
  groundLvl = mCtlEnc1;

  //Reset the deepest depth
  deepestDepth = mCtlEnc1; //track the bottom of the hole
  depthCm = 0.0;

  //Come back up a bit
  long currentLvl = mCtlEnc1;
  target = mCtlEnc1 - 5000;
  while (currentLvl > target)
  {
    upVel(7500);
    getData(PRINT);
    currentLvl = mCtlEnc1; 
  }

  stopAllMotors();

  //Switch to constant rate augering
  currentState = CRA_CONTROL;
  motorCommandSent = false;

  //Grab start time
  digStartTime = mCtlSeconds;
}

void runCRA_CONTROL()
{
  if (motorCommandSent == false)
  {
    constantRateAuger(5000, 200); //elevator speeed is 5000, auger is 200 rpm
    motorCommandSent = true;
  }

//Wait 10 seconds, then check for failures
  if (mCtlSeconds - digStartTime > 10)
  {
    //If WOB, auger motor current, or auger RPM are out of limits, then fail.
    if ( (WOB > 17.9) || (mCtlCur0 > 10.0) || (mCtlVelRPM0 < 170.0) )
    {
      currentState = FAIL;
      stopAllMotors();
      
      message("Control Auger Fail:");
      if (WOB > 21.9)
        message("WOB - " + String(WOB, 2));
  
      if (mCtlCur0 > 12.0)
        message("Auger Motor Current High - " + String(mCtlCur0,2));
  
      if (mCtlVelRPM0 < 110.0)
        message("Auger RPM Low - " + String(mCtlVelRPM0,2));
    }
  }

  //calculate depth in cm
  depthCm = (groundLvl - mCtlEnc1) * cmPerCount;

  //check to see if success depth reached
  if (depthCm < depthTarget)
  {
    currentState = SUCCEED;
    stopAllMotors();
    double totalTime = mCtlSeconds - digStartTime;
    message("Surface sensor insertion success.");
    message("Total Dig Time: " + String(totalTime, 2));
  }

  //Every 5 seconds, check soil conditions
//  if (millis() - lastSoilCheck > soilCheckDelay)
//  {
//    getSoilConditions(soilMoisture, soilStrength);
//    lastSoilCheck = millis();
//  }

  
}

void runSFC()
{
  if (motorCommandSent == false)
  {
    constantRateAuger(3000, 100); //elevator speeed is 5000, auger is 200 rpm
    motorCommandSent = true;
  }

//Wait 10 seconds, then check for failures
//  if (mCtlSeconds - digStartTime > 10)
//  {
//    //If WOB, auger motor current, or auger RPM are out of limits, then fail.
//    if ( (WOB > 17.9) || (mCtlCur0 > 10.0) || (mCtlVelRPM0 < 170.0) )
//    {
//      currentState = FAIL;
//      stopAllMotors();
//      
//      message("Surface Sensor Auger Fail:");
//      if (WOB > 21.9)
//        message("WOB - " + String(WOB, 2));
//  
//      if (mCtlCur0 > 12.0)
//        message("Auger Motor Current High - " + String(mCtlCur0,2));
//  
//      if (mCtlVelRPM0 < 110.0)
//        message("Auger RPM Low - " + String(mCtlVelRPM0,2));
//    }
//  }

  //calculate depth in cm
  depthCm = (startLvl - mCtlEnc1) * cmPerCount;

  //check to see if success depth reached
  if (depthCm < depthTarget)
  {
    currentState = SUCCEED;
    stopAllMotors();
    double totalTime = mCtlSeconds - digStartTime;
    message("Total Dig Time: " + String(totalTime, 2));
  }
  
}

void runCRA()
{
  if (motorCommandSent == false)
  {
    constantRateAuger(5000, 200); //elevator speeed is 5000, auger is 200 rpm
    motorCommandSent = true;
  }

  //calculate depth in cm
  depthCm = (groundLvl - mCtlEnc1) * cmPerCount;

  //check to see if success depth reached
  if (depthCm < depthTarget)
  {
    currentState = SUCCEED;
    stopAllMotors();
  }  

  //Check for no progress
  if (mCtlSeconds - checkDegradedSeconds > 5)
  {

    if (checkingDegraded == false)
    {
      message("Checking dedraded operations at: " + String(mCtlSeconds, 4));
      message("Low RPM trigger is at: " + String((rpmTarget-40)));
      checkingDegraded = true;
    }
    
    checkDegradedProgress();
  }
  
  
}

void checkDegradedProgress()
{

  if (simming)
  {
    //set some params based on sim.*
    WOB = sim.wob;
    mCtlVelRPM0 = sim.rpm0;
    depthCm = sim.deepest;
    
    if (sim.moist !=0)
      soilMoisture = sim.moist;

    if (sim.strength !=0)
      soilStrength = sim.strength;
  }
  
  if ((WOB) > 21.9  || (mCtlVelRPM0 < (rpmTarget-35)) || mCtlCur0 > 18.0  ) //need to add amperage on main motor check
  {
    message("Detecting a halt in progress:");
    if ((WOB/2.025) > 9.0) 
      message("WOB: " + String((WOB/2.025),2));
    if (mCtlVelRPM0 < (rpmTarget-40))
      message("RPM: " + String(mCtlVelRPM0,2));
    if (mCtlCur0 > 18.0)
      message("Amps: " + String(mCtlCur0, 2));
      
    if (depthCm < - 3.0) // DEEP
    {
      if ( (soilMoisture > 50.0) || (soilStrength > 2.5) ) // DEEP AND MOIST or DEEP AND COMPACT
      {
        //Long Peck up
        upVel(peckSpeed);
        runAugerMotor(rpmTarget);
        currentState = PECK_UP_LONG;
        cmPerSecond = 0.0;
      }
      else //DEEP AND DRY or DEEP AND LOOSE
      {
       //Short peck up
        upVel(peckSpeed);
        runAugerMotor(rpmTarget);
        target = deepestDepth - 36000;
        currentState = PECK_UP_SHORT;
        cmPerSecond = 0.0; 
      }
    }
    else // Just shallow, need short peck up
    {
      //Short peck up
      upVel(peckSpeed);
      target = deepestDepth - 36000;
      currentState = PECK_UP_SHORT;
      cmPerSecond = 0.0; 
    }
  }

 
  if (currentState == CRA) //if a peck is started, then ignore this stuff
  {//Every 5 seconds, check soil conditions
    if (millis() - lastSoilCheck > soilCheckDelay)
    {
  
      message("Checking soil conditions.");
      getSoilConditions(soilMoisture, soilStrength);
      lastSoilCheck = millis();
  
      
      //Check for slow progress after 15 seconds of digging
      if ((mCtlSeconds - digStartTime) > 15.0)
      {
        
        if (forceSlow)
          cmPerSecond = 0.05;
          
        if (cmPerSecond < 0.1) //slow progress
        {
          if (rpmTarget == 200 && elevatorTarget == 5000)
            message("Slow progress detected. cm/second = " + String(cmPerSecond));
         
          //if moist, slow the drill
          if (soilMoisture > 45.0 && rpmTarget > 100)
          {
            rpmTarget = 100;
            message("Slowing RPM to 100.");
          }
  
          if (((soilStrength > 2.5) || (soilMoisture <= 35.0)) && elevatorTarget > 3000 )
          {
            elevatorTarget = 2000;
            message("Slowing elevator advance."); 
          }
  
          constantRateAuger(elevatorTarget, rpmTarget); //elevator speeed is 5000, auger is 200 rpm
          motorCommandSent = true;
        }
      
     }
    }

  }
}

void runPeckUpLong()
{
  while (mCtlEnc1 >= groundLvl)
  {
    getData(PRINT);
  }
   //Peck back to deepestDepth
  downVel(peckSpeed);
  currentState = PECK_RETURN;
}

void runPeckUpShort()
{
  while (mCtlEnc1 >= target)
  {
    getData(PRINT);
  }

  //Peck back to deepestDepth
  downVel(peckSpeed);
  currentState = PECK_RETURN;
}

void runPeckReturn()
{
  while (mCtlEnc1 < deepestDepth) 
  {
    getData(PRINT);   
  }

  currentState = CRA;
  motorCommandSent = false;

  //reset timer on progress checking
  message("Peck return.");
  lastSoilCheck = millis()-4500;
  checkDegradedSeconds = mCtlSeconds - 4.0;
}

void runBiteUp()
{
  while (mCtlEnc1 >= groundLvl)
  {
    getData(PRINT);
  }
   //Peck back to deepestDepth
  downVel(biteSpeed);
  currentState = BITE_RETURN;
}

void runBiteReturn()
{
  while (mCtlEnc1 < deepestDepth) 
  {
    getData(PRINT);   
  }

  currentState = CRA;
  motorCommandSent = false;

  //reset timer on progress checking
  lastSoilCheck = millis()-4500;
}
