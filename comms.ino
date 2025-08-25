//Communications section

//Serial character commands for system
/*
 * a: Auger motor run, ramped vel mode,
 * A: Auger motor stop, control system to idle. 
 * u: up slow
 * d down slow
 * r: run dig routine
 * f: begin finite state machine for adaptive digging
 * q = quit, control system off
 * 
 */

void checkComms()
{
    //Serial2 is the command line from the RPi
    if (Serial2.available()) {
      char c = Serial2.read();
      if (c != '\n')
        message("S2: " + String(c));
        
      if (RCDiggingEnabled)
      {
        if (c == 'A') {
          //Abort and return to top
          message("Aborting sensor insertion.");
          upVel(65000);
          while (mCtlEnc1 >= startLvl)
          {
            getData(NO_PRINT);
          }
          stopAllMotors();
          fsm = false;
          currentState = STOP;
          cmPerSecond = 0.0;
        }
  
        if (c == 'P') {
          fsm = true;
          stopAllMotors();
          delay(750);
          currentState = SUCCEED;
        }
  
        if (c == 'f') 
        {
          //Check mode...
          if (mode == ADAPT)
          {
            //start adaptive fsm
            if (fsm == false)
            {
              message("Beginning surface sensor insertion.");
              fsm = true;
              delay(1000);
              currentState = START_SFC;
            }
          }
          else if (mode == SURFACE)
          {
              downVel(20000);
              currentState = PLACE_SENSOR;
              fsm = true;    
          }
        }
       
      }

      //The pi will send an R when the ROS code is up and runnning.
      if (c == 'R') 
      {
        message("RPi online.");  
      }

      
      if (c == 'r') //re-send query
      {
        message("Re-sending soil conditions query.");
        getSoilConditions(soilMoisture, soilStrength);
        lastSoilCheck = millis();
      }

      if (c == 'm') //moisture prediction
      {
        String m = Serial2.readStringUntil('\n');
        //message(m);
        soilMoisture = m.toDouble();
        if (simming && sim.moist != 0) //both non-zero, then use sim values
        {
          soilMoisture = sim.moist;
        }
      }

      if (c == 's') //strength prediction
      {
        String s = Serial2.readStringUntil('\n');
        soilStrength = s.toDouble();

        if (simming && sim.strength != 0) //both non-zero, then use sim values
        {
          soilStrength = sim.strength;
        } 
      }
    } //end RPi section

    
    if (UISerial.available()) {
      char c = UISerial.read();
      //UISerial.print("rcvd: ");UISerial.println(c);

       //Check for leading x which is the mark for simulated data coming in.
       if (c == 'x')
        {
          if (simming)
          {
            while (UISerial.available() > 0)
            {
              simData_str = UISerial.readStringUntil('\n');
            }

            //parse sim data into component values
            parseSimData();

            
            c = '-';
          }
        }

      //Raise Z axis, 'key u = up'
      if (c == 'u') {
        upVel(7500);
      }
      
      //lower z axis, 'key d = down'
      if (c == 'd') {
        downVel(7500);
      }

      //Down fast
      if (c == 'D') {
        downVel(80000);
      }

      //Up fast
      if (c == 'U') {
        upVel(80000);
      }

      //You can use this as a general query for the system state
      if (c == '?') {
        UISerial.println(mCtlVelRPM0);
        message("RPM: " + String(mCtlVelRPM0, 5) + "  -  Low Target: " + String(rpmTarget-25));
      }

      //Manually initialize the elevator start position
      if (c == '*') {
        startLvl = mCtlEnc1;
        message("Elevator start position initialized.");
      }

      //Enable digging with the RC switch
      if (c == '^') {
        if (RCDiggingEnabled)
        {
          RCDiggingEnabled = false;
          message("RC digging disabled.");
        }
        else
        {
          RCDiggingEnabled = true;
          
          //flush serial2 input buffer
          while (Serial2.available())
            Serial2.read();
            
          message("RC digging enabled.");
        }
      }

      if (c == '!')
      {
        if (DEBUG == false)
        {
          DEBUG = true;
          message("Set debug.");
        }
        else
        {
          DEBUG = false;
          message("Unset debug.");
        }
      }

      //Set mode to adaptive digging
      if (c =='@') {
        //set mode to ADAPT
        mode = ADAPT;
        message("Mode: Adaptive Digging");
      }

      //Set mode to surface placement
      if (c =='_') {
        //set mode to surface
        mode = SURFACE;
        message("Mode: Surface Placement");
      }

      //Set mode to constant rate digging
      if (c =='=') {
        //set mode to CR
        mode = CR;
        message("Mode: Constant Rate Digging");
      }

      //Manually start the surface placement mode
      if (c == '/') {
        if (mode == SURFACE)
          {
              downVel(20000);
              currentState = PLACE_SENSOR;
              fsm = true;    
          }
      }
      
      if (c == '3') {   
        message("Depth Test:");
        Serial1.println("w axis1.requested_state 8"); //closed loop control enable
        delay(10);
        long now = millis();
        long dt = now;
        
        getData(NO_PRINT);
        depthCm = (groundLvl - mCtlEnc1) * cmPerCount;
        lastDepth = depthCm;
        message("Start millis: " + String(now) + " Start depth: " + String(depthCm));
        Serial1.print("w axis1.controller.config.control_mode 1\n"); //pos mode is mode #1
        Serial1.print("w axis1.trap_traj.config.vel_limit "); // 15000\n");
        Serial1.print(5000);
        Serial1.println();
        Serial1.println("t 1 550000");
        while (millis() - now < 5000)
        {
          if (millis() - dt > 1000)
          {
            getData(NO_PRINT);
            depthCm = (groundLvl - mCtlEnc1) * cmPerCount;
            double dd = depthCm - lastDepth;
            message("depth: " + String(dd));
            lastDepth = depthCm;
            dt = millis();
          }
        }
        long tTime = millis() - now;
        getData(NO_PRINT);
        depthCm = (groundLvl - mCtlEnc1) * cmPerCount;
        message("End millis: " + String(tTime) + " End depth: " + String(depthCm));
        message("Ave cm/sec: " + String(depthCm / (tTime / 1000.0)));
        
        stopAllMotors();
        lastDepth = 0;
        
        //UISerial.println("Sending yeet.");
        //Serial3.println("2.24,180.81,57.88,-4.29, 2.0,0.1,-0.17,0.6");
      }

      if (c == '4') {
        
        //set force slow progress flag
        if (forceSlow == false)
        {
          forceSlow = true;
          message("Faking slow progress.");
        }
        else
        {
          forceSlow = false;
          message("Removing faked slow progress flag.");
        }
      }

      if (c == '5')
      {
        //force high moisture
        if (forceHighMoisture == false)
        {
          forceHighMoisture = true;
          message("Setting moisture to 60.");
        }
        else
        {
          forceHighMoisture = false;
          message("Removing fake moisture flag");
        }
      }

      if (c == '6')
      {
        //force high strength
        if (forceHighStrength == false)
        {
          forceHighStrength = true;
          message("Setting strength to 4.");
        }
        else
        {
          forceHighStrength = false;
          message("Removing fake strength flag");
        }
      }

      //get data
      if (c == 'g') {
       request_event();
       getData(PRINT);
      }

      //Reset motors / recover from frozen state
      if (c == 'r') {
        currentState = STOP;

        Serial1.println("w axis0.error 0");
        Serial1.println("w axis1.error 0");
        sendMotorControlConfigs();    
        runAugerMotor(100);
        upVel(7500);  
      }

      //retreat to zero
      if (c == 'e') {
        Serial1.println("w axis0.requested_state 8"); //closed loop control enable
        Serial1.println("w axis1.requested_state 8"); //closed loop control enable
        Serial1.println("t 1 0");
      }
      
      //auger
      if (c == 'a') {
        runAugerMotor(200);      
      }

      if (c == 'A') {
        //Abort and return to top
        upVel(65000);
        while (mCtlEnc1 >= startLvl)
        {
          getData(NO_PRINT);
        }
        stopAllMotors();
        fsm = false;
        currentState = STOP;
        cmPerSecond = 0.0;
        
        //Serial1.println("v 0 0");
        //delay(5);
        //Serial1.println("w axis0.requested_state 1"); //closed loop control disable      
      }
      
      //servo commands
      //Extend
      if (c == 'E') {
        servoExtend();
      }

      if (c == 'R') {
        servoRetract();
      }

      if (c == 'v') {
        if (alwaysStreamData == false)
        {
          message("Setting continuous stream of data.");
          alwaysStreamData = true;
        }
        else
        {
          alwaysStreamData = false;
          message("Stopping continuous stream of data.");
        }
      }
      
      //quit      
      if (c == 'q') {
        Serial1.println("w axis0.requested_state 1");
        delay(10);
        Serial1.println("w axis1.requested_state 1");
        delay(10);
        Serial1.println("v 0 0");
        delay(10);
        Serial1.println("v 1 0");
        delay(5);
        fsm = false;
        currentState = STOP;
        cmPerSecond = 0.0;
      }

      //start adaptive digging
      if (c == 'f')
      {
        // strainGaugeSetup();
        fsm = true;
        delay(1000);
        currentState = START;
      }

      //start surface sensor insertion
      if (c == 'c')
      {
        message("Starting surface sensor insertion.");
        fsm = true;
        delay(1000);
        currentState = START_SFC;
        depthTarget = -11.0;
        
      }

      if (c == 'T') //test
      {
        getSoilConditions(soilMoisture, soilStrength);
       
      }

      //This was used to debug some items...
      if (c == 'r')
      {
        UISerial.print("Bin_pack RPM: ");UISerial.println(bin_pack.a.M0_Output_RPM);
      }

      //Pecking commands
      //Long peck
      if (c == 'P') {

        //Peck up
        upVel(peckSpeed);
        currentState = PECK_UP_LONG;
        cmPerSecond = 0.0;
        fsm = true;
      }  

      //Short peck
      if (c == 'p') {
        //Peck up
        upVel(peckSpeed);
        target = deepestDepth - 36000;
        currentState = PECK_UP_SHORT;
        cmPerSecond = 0.0;
        fsm = true;
      }

      //Bite
      if (c == 'b')
      {
        upVel(biteSpeed);
        currentState = BITE_UP;
        cmPerSecond = 0.0;
        fsm = true;
      }

      if (c == 's') //spin slow
      {
        runAugerMotor(100);
      }

      if (c == 'S') //spin fast
      {
        runAugerMotor(200);
      }

      //Simulation stuff
      if (c == 'X') //sim on/off
      {
        if (simming)
        {
          message("Sim mode off.");
          simming = false;
        }
        else
        {
          message("Sim mode on.");
          simming = true;
        }

      }
      
   }

 
}


//CheckSuperLoop checks to see if limit switches (if installed) have been tripped.
void checkSuperLoop()
{
  long checkSuperLoopTime = millis();
  if(checkSuperLoopTime - previousDataGetTime > 50) // originally 100
  {
    getData(PRINT); //get motor paramaters from Odrive motor controller over serial
    previousDataGetTime = checkSuperLoopTime;
  }
  
  pollLimitSwitches(); //update global variable limitSWfeedback
  
  if(limitSWfeedback & B00010000)
  {
    //the bottom limit switch has been reached
    stopAllMotors();
  }
  if(limitSWfeedback & B00000100)
  {
    //the top limit switch has been reached
    stopAllMotors();
  }
  
  clearLSTransitionFeedback();

  //Emergency stop check on auger current 
  //This happens regardles of state
  if (mCtlCur0 > 18.0)
  {
    currentState = FAIL;
    stopAllMotors();
  }
}
