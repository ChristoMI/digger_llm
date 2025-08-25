//These are strings that are sent to the odrive board over Serial1.
//Refer to odrive documentation for the specifics of each command, if needed.s


void drillToPos(long pos, long velocity)
{   
    //String str = "w axis1.trap_traj.config.vel_limit ";
    //Test
    //
    //String StringData = String(velocity);
    //str += StringData;
    //Serial1.println(str);
    //Serial.println(str);
    //delay(1);
    Serial1.println("w axis1.requested_state 8"); //closed loop control enable
    //delay(1);
    //String str2 = "q 1 "; //q motor position velocity_lim current_lim
//    String StringData2 = String(pos);
//    str2 += StringData2;
    //Serial1.println(str2);
    
    //Serial1.println();//add current command?
    //Serial.println("drilling");
}

//void downVel(long vel)
//{
//  Serial1.println("w axis1.requested_state 8"); //closed loop control enable
//  delay(5);
//  Serial1.print("w axis1.controller.config.control_mode 1\n"); //pos mode is mode #1
//  delay(5);
//  Serial1.print("w axis1.trap_traj.config.vel_limit "); // 15000\n");
//  Serial1.print(vel);
//  Serial1.println();
//  delay(5);
//  Serial1.println("t 1 100000000");
//}

void downVel(long vel)
{
    Serial1.println(F("w axis1.requested_state 8")); //closed loop control enable, control system must be on first
    Serial1.print(F("w axis1.controller.config.control_mode 2\n")); //vel mode is mode 2
    Serial1.print(F("w axis1.controller.config.vel_ramp_rate 300000\n"));
    Serial1.print(F("w axis1.controller.vel_ramp_enable 1\n"));
    Serial1.print(F("w axis1.controller.current_setpoint 8\n"));
    Serial1.print(F("w axis1.controller.vel_ramp_target ")); 
    Serial1.println(vel);
}

void upVel(long vel)
{
//  Serial1.println("w axis1.requested_state 8"); //closed loop control enable
//  delay(5);
//  Serial1.print("w axis1.controller.config.control_mode 1\n"); //pos mode is mode #1
//  delay(5);
//  Serial1.print("w axis1.trap_traj.config.vel_limit "); // 15000\n");
//  Serial1.print(vel);
//  Serial1.println();
//  delay(5);
//  Serial1.println("t 1 -100000000");
    Serial1.println(F("w axis1.requested_state 8")); //closed loop control enable, control system must be on first
    Serial1.print(F("w axis1.controller.config.control_mode 2\n")); //vel mode is mode 2
    Serial1.print(F("w axis1.controller.config.vel_ramp_rate 300000\n"));
    Serial1.print(F("w axis1.controller.vel_ramp_enable 1\n"));
    Serial1.print(F("w axis1.controller.current_setpoint -8\n"));
    Serial1.print(F("w axis1.controller.vel_ramp_target ")); 
    Serial1.println(-vel); 
}

void constantRateAuger(int elevatorSpeed, int augerSpeed)
{
    //Serial.println("Starting");
    Serial1.println("w axis0.requested_state 8"); //closed loop control enable
    delay(10);
    Serial1.println("w axis1.requested_state 8"); //closed loop control enable
    delay(10);
    runAugerMotor(augerSpeed);
    //runAugerMotor(0);

//    Serial1.print("w axis1.controller.config.control_mode 1\n"); //pos mode is mode #1
//    Serial1.print("w axis1.trap_traj.config.vel_limit "); // 15000\n");
//    Serial1.print(elevatorSpeed);
//    Serial1.println();
//    Serial1.println("t 1 550000");

      downVel(elevatorSpeed);
  
}

bool checkArrived(long desiredPos)
{
  //Serial.println("checking");
  long posError = abs(desiredPos - mCtlEnc1);
  if ((mCtlVelRPM1 < 3)&&(posError < 150))
  {
    return 1;
  }
  else
 {
  return 0;
 }
}

void runAugerMotor(int RPMs) //prototyped 4/19, test this funcn.
{
long encCountsPerSecond = (( (RPMs) /60.0) * 8124 * 6); //8124 enc counts/sec. 6:1 gearbox
Serial1.print("w axis0.controller.vel_ramp_enable 1\n");
Serial1.println("w axis0.requested_state 8"); //closed loop control enable
Serial1.print("w axis0.controller.vel_ramp_target ");
Serial1.print(-encCountsPerSecond);
Serial1.print("\n");
}

void runVertMotor(int IPS) //prototyped 4/19, test this funcn.
{/*
long encCountsPerSecond = ((RPMs/60.0) * 8124 * 6); //8124 enc counts/sec. 6:1 gearbox
Serial1.print("w axis0.controller.vel_ramp_enable 1\n");
Serial1.println("w axis0.requested_state 8"); //closed loop control enable
Serial1.print("w axis0.controller.vel_ramp_target ");
Serial1.print(-encCountsPerSecond);
Serial1.print("\n");*/
}

void stopAllMotors()
{
  Serial1.println("w axis0.requested_state 1");
  delay(5);
  Serial1.println("w axis1.requested_state 1");
  delay(5);
  Serial1.println("v 0 0");
  delay(5);
  Serial1.println("v 1 0");
  delay(5);
}
