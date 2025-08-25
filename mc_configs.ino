// Renamed Serial Ports
//extern HardwareSerial odriveSerial = Serial1;

//Configure the odrive board and motors for use.


#define odriveSerial Serial1

void sendMotorControlConfigs()
{
  
/*
 *        AXIS 0 CONFIG
 */
    //odriveSerial.print("se\n"); //erase config //see getting started guide notes on microcoontroller hanging for several seconds.
    //delay(100);

    //Serial.println("reboot");
    //odriveSerial.print("sr\n"); //reboot, other versions of firmware use "sb" 
    delay(1000);
    int t = 2;
    
    //checked reflected in odrivetool
    odriveSerial.print(F("w axis0.encoder.config.cpr 8192\n"));
    delay(t);
    
   
    //checked reflected in odrivetool
    odriveSerial.print(F("w axis0.motor.config.pole_pairs 21\n"));
    delay(t);

   
    //was 260000 4/18/20
    odriveSerial.print(F("w axis0.controller.config.vel_limit 400000\n"));
    delay(t);

    
    odriveSerial.print(F("w axis0.motor.config.resistance_calib_max_voltage 1.6\n"));
    delay(t);

   
    //checked in odrivetool
    odriveSerial.print(F("w axis0.motor.config.calibration_current 10\n"));
    delay(t);

  
    //checked in odrivetool
    odriveSerial.print(F("w axis0.motor.config.requested_current_range 60\n"));
    delay(t);

   
    //checked in odrivetool
    odriveSerial.print(F("w axis0.motor.config.current_lim 25\n"));
    delay(t);

    
    odriveSerial.print(F("w axis0.motor.config.current_control_bandwidth 500\n")); //started at 50 3/17/21
    delay(t);

    
    //MOTOR 0 PID TUNINGS
    odriveSerial.print(F("w axis0.controller.config.pos_gain 00.0f\n"));
    delay(t);

    odriveSerial.print(F("w axis0.controller.config.vel_gain 0.00016\n")); //was .00025
    delay(t);

    //checked in odrivetool
    odriveSerial.print(F("w axis0.controller.config.vel_integrator_gain 0.000f\n"));
    delay(t);
    
    //checked in odrivetool
    odriveSerial.print(F("w axis0.config.startup_motor_calibration 1\n"));
    delay(t);
    
    //checked in odrivetool
    odriveSerial.print(F("w axis0.config.startup_encoder_offset_calibration 1\n"));
    delay(t);

    //checked in odrivetool
    odriveSerial.print(F("w axis0.config.startup_closed_loop_control 0\n"));
    delay(t);
    
    //100,000 = 750ish rpm?, 40000 = 300rpm //checked in odrivetool
    odriveSerial.print(F("w axis0.trap_traj.config.vel_limit 400000\n"));
    delay(t);

    //checked in odrivetool
    odriveSerial.print(F("w axis0.trap_traj.config.accel_limit 60000\n"));
    delay(t);
    
    //checked in odrivetool
    odriveSerial.print(F("w axis0.trap_traj.config.decel_limit 60000\n"));
    delay(t);
    
    //checked in odrivetool
    odriveSerial.print(F("w axis0.trap_traj.config.A_per_css 0\n"));
    delay(t);
    
    //vel mode is mode #2 //checked in odrivetool
    odriveSerial.print(F("w axis0.controller.config.control_mode 2\n"));
    delay(t);
   
    //velocity ramp mode
    odriveSerial.print(F("w axis0.controller.config.vel_ramp_rate 300000\n")); //200000 original
    delay(t);
    odriveSerial.print(F("w axis0.controller.vel_ramp_enable 1\n"));
    delay(t);

/*
 *        AXIS 1 CONFIG
 */
    odriveSerial.print(F("w axis1.encoder.config.cpr 8192\n"));
    delay(t);
    odriveSerial.print(F("w axis1.motor.config.pole_pairs 7\n"));
    delay(t);
    odriveSerial.print(F("w axis1.controller.config.vel_limit 375000\n"));
        //odriveSerial.print(F("w axis1.controller.config.vel_limit 12000\n"));

    delay(t);
    odriveSerial.print(F("w axis1.controller.config.vel_limit_tolerance 1.2\n"));
    delay(t);
    odriveSerial.print(F("w axis1.encoder.config.calib_range 0.05\n"));
    delay(t);
    odriveSerial.print(F("w axis1.motor.config.resistance_calib_max_voltage 2.5\n"));
    delay(t);
    odriveSerial.print(F("w axis1.motor.config.calibration_current 15\n"));
    delay(t);
    odriveSerial.print(F("w axis1.motor.config.requested_current_range 60\n"));
    delay(t);
    
    //was 35
    odriveSerial.print(F("w axis1.motor.config.current_lim 30\n"));
    delay(t);

    odriveSerial.print(F("w axis1.motor.config.current_control_bandwidth 50\n")); //started at 50 2/9/21
    delay(t);

    //f?
    odriveSerial.print(F("w axis1.controller.config.pos_gain 18.0\n")); //not used in vel mode
    delay(t);

    odriveSerial.print(F("w axis1.controller.config.vel_gain .0019\n")); //was .0024 
    delay(t); //was 0.18
    odriveSerial.print(F("w axis1.controller.config.vel_integrator_gain 0.0000\n"));
    delay(t);
    odriveSerial.print(F("w axis1.config.startup_motor_calibration 1\n"));
    delay(t);
    odriveSerial.print(F("w axis1.config.startup_encoder_offset_calibration 1\n"));
    delay(t);
    odriveSerial.print(F("w axis1.config.startup_closed_loop_control 0\n"));
    delay(t);

    odriveSerial.print(F("w axis1.trap_traj.config.accel_limit 400000\n"));
    delay(t);
    
    odriveSerial.print(F("w axis1.trap_traj.config.decel_limit 400000\n"));
    delay(t);

}
