//serial 1 wiring
//odrv gpio 1(odrv tx) - ard 19 (ard rx)
//odrv gpio 2(odrv rx) - ard 18 (ard tx)

#define odriveSerial Serial1

//from jh serial_io
String odrive_read_line()
{
  //String resp_str = altUISerial.readStringUntil('\n');
  String resp_str = odriveSerial.readStringUntil('\n');
  return resp_str;
}

bool odrive_read_bin_bytes(char * od_bin_arr, int od_bin_size)
{
  int rtn_num_bytes = odriveSerial.readBytes(od_bin_arr, od_bin_size);
  
  
  if( od_bin_arr[od_bin_size-2] == 13 &&
      od_bin_arr[od_bin_size-1] == 10)
  {
//    backupUISerial.println("VALID ODRIVE RESPONSE");
    return(false);
  }
  else
  {
//    backupUISerial.println("BAD ODRIVE RESPONSE");
    return(true); 
  }
//  char temp_arr[1];
//  if( odriveSerial.readBytes(temp_arr, 1) )
//  {
//    backupUISerial.print("LAST BYTE: ");
//    backupUISerial.println(od_bin_arr[od_bin_size-1], DEC);
//    backupUISerial.print("NEXT BYTE: ");
//    backupUISerial.println(temp_arr[0], DEC);
//  }
//  if( odriveSerial.available())
//  {
//    backupUISerial.println("ODRIVE PRINTED MORE THAN 50 BYTES!!!");
//    while( odriveSerial.available() )
//    {
//      odriveSerial.readBytes(temp_arr, 1);
//      backupUISerial.print(temp_arr[0], DEC);
//      backupUISerial.print(", ");
//    }
//    backupUISerial.print("\n");
//  }
//  return rtn_num_bytes;
}

String get_odrive_status()
{
  String odrive_str = "";
  odriveSerial.println(F("ns"));
  odrive_str += odrive_read_line();
  odrive_str.trim();
  odrive_str += odrive_read_line();
  return odrive_str;
}

bool get_odrive_status_bin(char* bin_array, int od_bin_size)
{
  
  String odrive_str = "";
  odriveSerial.println(F("nb"));

  // odrive_read_bin_bytes returns true if error, and false if no error
  return odrive_read_bin_bytes(bin_array, od_bin_size);
}


void updateAccel()
{

  xa = acc_x;
  ya = acc_y;
  za = acc_z;
  
}

//Get data pulls the telemetry data and gets it ready to stream
void getData(int printCheck)
{
  //startTime = millis();

  copy_teensy_status();
  copy_odrive_status();
  copy_arduino_status();
  
  //Get voltage
  Serial1.print("r vbus_voltage\n");
  delay(serialDelay);
  while (Serial1.available() > 0)
  {
    volt_str = Serial1.readStringUntil('\n');
  }
  mCtlVolts = volt_str.toDouble();
  
  delay(serialDelay);

  //Get time
  Serial1.print("r system_stats.uptime\n");
  delay(serialDelay);
  while (Serial1.available() > 0)
  {
    time_str = Serial1.readStringUntil('\n');
  }
  mCtlSeconds = (time_str.toInt())/1000.0;
 
  //Get auger motor current
  cur_str0 = "NULL";
  Serial1.print("r axis0.motor.current_control.Iq_measured\n");
  delay(serialDelay);
  while (Serial1.available() > 0)
  {
    cur_str0 = Serial1.readStringUntil('\n');
  }
  mCtlCur0 = cur_str0.toDouble();
  if (mCtlCur0 > 20.0)
  {
    //emergency stop over amp
    currentState = STOP;
    fsm = false;
    message("EmerencyStop - Amps: " + String(mCtlCur0, 2));
  }
  

  //Get elevator motor current
  cur_str1 = "NULL";
  Serial1.print("r axis1.motor.current_control.Iq_measured\n");
  delay(serialDelay);
  while (Serial1.available() > 0)
  {
    cur_str1 = Serial1.readStringUntil('\n');
  }
  mCtlCur1 = bin_pack.a.M1_Current;//cur_str1.toDouble();

  //Get Auger RPM
  Serial1.print("r axis0.encoder.vel_estimate\n");
  delay(serialDelay);
  while (Serial1.available() > 0)
  {
    vel_str = Serial1.readStringUntil('\n');
  }
  //mCtlVelRPM0 = (((vel_str.toInt()) * (-60.0/8124.0))/6.0);
  lowpass_auger_RPM();
  mCtlVelRPM0 = auger_RPM_filtered;
  
  //Get elevator motor RPM
  Serial1.print("r axis1.encoder.vel_estimate\n");
  delay(serialDelay);
  while (Serial1.available() > 0)
  {
    vel_str = Serial1.readStringUntil('\n');
  }
  mCtlVelRPM1 = ((vel_str.toInt()) * (-60.0/8192.0));
    
  //Elevator Encoder Position in counts
  Serial1.print("r axis1.encoder.shadow_count\n");
  delay(serialDelay);
  while (Serial1.available() > 0)
  {
    vel_str = Serial1.readStringUntil('\n');
  }
  mCtlEnc1 = vel_str.toInt();
  if (mCtlEnc1 > deepestDepth)
    deepestDepth = mCtlEnc1; //track the bottom of the hole
/*
//Pos Setpoint
Serial1.print("r axis1.controller.pos_setpoint\n");
delay(serialDelay);
while (Serial1.available() > 0)
    {
        vel_str = Serial1.readStringUntil('\n');
    }
    mCtlPosSet1 = vel_str.toInt();
    UISerial.print("M1PosSP, ");
    UISerial.print(mCtlPosSet1); 
    UISerial.print(", ");*/

/*    //Velocity Setpoint
Serial1.print("r axis1.controller.vel_setpoint\n");
delay(serialDelay);
while (Serial1.available() > 0)
    {
        vel_str = Serial1.readStringUntil('\n');
    }
    mCtlVelSet1 = vel_str.toInt();
    UISerial.print("M1VelSP, ");
    UISerial.print(mCtlVelSet1); 
    UISerial.print(", ");*/

  //read errors
  Serial1.print("r axis0.error\n");
  delay(serialDelay);
  while (Serial1.available() > 0)
  {
    error_str = Serial1.readStringUntil('\n');
  }
  mCtlError0 = (error_str.toInt());
  
  Serial1.print("r axis1.error\n");
  delay(serialDelay);
  while (Serial1.available() > 0)
  {
    error_str = Serial1.readStringUntil('\n');
  }
  mCtlError1 = (error_str.toInt());
   

/*
UISerial.print("SW, ");
  UISerial.print(digitalRead(LimitSWTop));
  UISerial.print(digitalRead(LimitSWBot));
      UISerial.print(", ");

    */

  //Get weight on bit
  WOB = bin_pack.a.WOB;     

/*
limitSWTopVal = digitalRead(LimitSWTop);
limitSWBotVal = digitalRead(LimitSWBot);
UISerial.print("LS, ");
  UISerial.print(limitSWTopVal);
  UISerial.print(limitSWBotVal);
    UISerial.print(", "); 

  //UISerial.print(", "); 
  UISerial.print("TOPLS, ");
  UISerial.print(ElevatorZero);
  UISerial.print(", "); 
  UISerial.print("SOIL, ");
  UISerial.println(soilZero);
  //UISerial.println("");*/

 

  if (printCheck == PRINT)
    printData();
}

void printData()
{

  if (millis() - lastPrint > printDelay)
  {
    //Volts
    UISerial.print("V");
    UISerial.print(mCtlVolts);
    UISerial.print(", ");
  
    //Time
    UISerial.print("t");
    UISerial.print (mCtlSeconds);
    UISerial.print(", ");
  
    //Auger Motor Current
    UISerial.print("C");
    UISerial.print(mCtlCur0);
    UISerial.print(", ");
  
    //Elevator Motor Current
    UISerial.print("c");
    UISerial.print(mCtlCur1);
    UISerial.print(", ");
  
    //Auger RPM
    UISerial.print("R");
    UISerial.print(mCtlVelRPM0); 
    UISerial.print(", ");
  
    //Elevator motor RPM
    UISerial.print("r");
    UISerial.print(mCtlVelRPM1); 
    UISerial.print(", ");
  
    //Elevator encoder count, i.e. depth count
    UISerial.print("M");
    UISerial.print(mCtlEnc1); 
    UISerial.print(", ");
  
    //Error 1
    UISerial.print("E");
    UISerial.print(mCtlError0); 
    UISerial.print(", ");
  
    //Error 2
    UISerial.print("e");
    UISerial.print(mCtlError1); 
    UISerial.print(", ");    
  
    //WOB
    UISerial.print("W");
    UISerial.print(WOB, 1);
    UISerial.print(", ");
  
    //x, y, and z accelerometer data
    //xa.replace("\n","");
    //xa.replace("\r","");
    UISerial.print("x"); UISerial.print(xa);
    UISerial.print(", ");
  
    //ya.replace("\n","");
    //ya.replace("\r","");
    UISerial.print("y"); UISerial.print(ya);
    UISerial.print(", ");
  
    //za.replace("\n","");
    //za.replace("\r",""); 
    UISerial.print("z"); UISerial.print(za);
    UISerial.print(", ");

    //current state
    UISerial.print("S");UISerial.print(currentState);
    UISerial.print(", ");

    //moisture
    if (forceHighMoisture)
      soilMoisture = 60.0;
    UISerial.print("m");UISerial.print(soilMoisture);
    UISerial.print(", ");

    //strength
    if (forceHighStrength)
      soilStrength = 4.0;
    UISerial.print("s");UISerial.print(soilStrength);
    UISerial.print(", ");

    //dept in cm
    UISerial.print("d");UISerial.print(depthCm);
    UISerial.print(", ");
    
    //5 second progress
    UISerial.print("p");UISerial.println(cmPerSecond);
    

    //debug
    //UISerial.print("WOB / 2.025: ");UISerial.println(WOB / 2.025);

    if (simming)
      getData(NO_PRINT);
      
    lastPrint = millis();
  }
}

void getSoilConditions(double &moisture, double &strength)
{

   
  if (simming)
  {
    mCtlCur0 = sim.current0;
    mCtlCur1 = sim.current1;
    mCtlVelRPM0 = sim.rpm0;
    mCtlVelRPM1 = sim.rpm1;
    WOB = sim.wob;
    deepestDepth = sim.deepest;
    xa = sim.xacc;
    ya = sim.yacc;
    za = sim.zacc;
  }
  
  //Query RPi - serial 2 for Alta system upgrade - AP, Dec 2022
  //Amps
  Serial2.print(mCtlCur0);Serial2.print(',');

  //Auger RPM
  Serial2.print(mCtlVelRPM0);Serial2.print(',');

  //Time - sending elapsed dig time
  Serial2.print(mCtlSeconds - digStartTime);Serial2.print(',');

  //Depth - must convert encoder count to cm prior to sending
  
  Serial2.print(depthCm);Serial2.print(',');

  //WOB - must convert to kg for query
  Serial2.print(WOB / 2.025);Serial2.print(',');

  //remove g calculation as JH code already outputs accells in g
  //float g = 16384.0; // mpu6050 calibration to get g's
  
  Serial2.print(xa);Serial2.print(',');

  Serial2.print(ya);Serial2.print(',');

  Serial2.println(za);

  //calculate current depth progress
  

  if (simming)
  {
    cmPerSecond = sim.progress;

    if (sim.moist != 0)
      soilMoisture = sim.moist;

    if (sim.strength !=0)
      soilStrength = sim.strength;
  }
  else
    cmPerSecond = lastDepth / 5.0;
 
  

  lastDepth = depthCm;

  //check_comms reads responses...
}

//message prepends a string with the letter x so the gui will 
//know to display the remainder of the string in the text box.
void message(String s)
{
  UISerial.println("x" + s);
}


//If sim data is sent back to the system, then it gets parsed here:
void parseSimData()
{
  int commaCount = 0;
  String t = simData_str;
  char *strtokIndx;

  for (uint8_t i = 0; i < t.length(); i++)
  {
    if (t[i] == ',')
      commaCount++;

    if (commaCount == 11)
    {
      strtokIndx = strtok(t.c_str(), ",");
      if (atof(strtokIndx) != 0)
        sim.current0 = atof(strtokIndx);

      strtokIndx = strtok(NULL, ",");
      if (atof(strtokIndx) != 0)
        sim.current1 = atof(strtokIndx);

      strtokIndx = strtok(NULL, ",");
      if (atof(strtokIndx) != 0)
        sim.rpm0 = atof(strtokIndx);

      strtokIndx = strtok(NULL, ",");
      if (atof(strtokIndx) != 0)
        sim.rpm1 = atof(strtokIndx);
      
      strtokIndx = strtok(NULL, ",");
      if (atof(strtokIndx) != 0)
        sim.wob = atof(strtokIndx);
      
      strtokIndx = strtok(NULL, ",");
      if (atof(strtokIndx) != 0)
        sim.deepest = atof(strtokIndx);

      strtokIndx = strtok(NULL, ",");
      if (atol(strtokIndx) != 0)
        sim.xacc = atol(strtokIndx);
      
      strtokIndx = strtok(NULL, ",");
      if (atol(strtokIndx) != 0)
        sim.yacc = atol(strtokIndx);
      
      strtokIndx = strtok(NULL, ",");
      if (atol(strtokIndx) != 0)
        sim.zacc = atol(strtokIndx);

      strtokIndx = strtok(NULL, ",");
      if (atof(strtokIndx) != 0.0)
        sim.moist = atof(strtokIndx);

      strtokIndx = strtok(NULL, ",");
      if (atof(strtokIndx) != 0)
        sim.strength = atof(strtokIndx);

      strtokIndx = strtok(NULL, ",");
      if (atof(strtokIndx) != 0)
        sim.progress = atof(strtokIndx);

      //String temp = String(sim.current0,2)+","+String(sim.current1,2)+","+String(sim.wob,2)+","+String(sim.strength,2)+","+String(sim.progress,2);
      //message(temp);
 
    }
    else
    {
      //message("Bad sim data rcvd.");
    }
  }
}
