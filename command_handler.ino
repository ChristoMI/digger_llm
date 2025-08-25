/***********************************************************************************/
/*  Title:        command_handler
     Author:      WMT
     Date:        3/3/2020
     Description: The command_handler takes in a single char that acts as the command
                  for processing. It then executes the intended code operation based
                  on series of if-statements. Note that this function was originally
                  written in 20200225_P4Elevator within the main loop, but was moved
                  here to improve readability. This was a particular concern when
                  the digger code was upgraded to respond to commands from both the
                  USB (desktop development, diagnostic) and the Xbee (field use)
                  serial ports.
*/
/***********************************************************************************/
//#include "teensy_interface.h"

// See constants included from command_handler.h!!!
//#include "command_handler.h"
//#include "subsystem_states.h"

unsigned int BINARY_STATUS_PACKET_SZ = 136; // This might be a duplicate of BIN_STATUS_SIZE above.

unsigned int SPECTRA_PACKET_SZ = 128;
unsigned int FLT_MSG_PACKET_SZ = 128;

// Gear Ratios for Odrive's M0 and M1.
float M0_GEAR_RATIO = (60.0 / 8192.0) / 6.0;
float M1_GEAR_RATIO = 60.0 / 8192.0;
//float busCurrent = 0;
//String busCurrent_str = "";

float ROP = 0;
//extern enum State_enum;
// Renamed Serial Ports
//extern HardwareSerial &backupSerial;
#define odriveSerial Serial1



double prev_auger_current_filtered;
double auger_current_filtered;

double prev_elev_current_filtered;
double elev_current_filtered;
double prev_elev_RPM_filtered;
double elev_RPM_filtered;
double prev_accel_x_filtered;
double accel_x_filtered;
double prev_accel_y_filtered;
double accel_y_filtered;
double prev_accel_z_filtered;
double accel_z_filtered;


bool fsm_auto_decisions = false; //flag to control when the drill auto-reacts to conditions. 



int od_bin_size = 58;             // 56 bytes of data and 2 for the CR and LF
char bin_array[58];
int od_data_size = 56;            // 56 bytes of data without the CR and LF
float odrive_time;
String ardHeader = ",Dig Time,StrainX,StrainY,WOB,LSW Top,LSW Bot,FSM State,Rel Servo Current,M0 Torq,M0 Work,MSE,ACCx,ACCy,ACCz,Roll,Pitch,Yaw,StrainS,ACCS,RPYS,FFTS";
float odrive_volt;
int swt;
int swb;

float M0_Torque;
float M0_Work;
float MSE;
unsigned long dig_time;

long oldPos = 0; //used for ROP calcs, line ~360
long prevTime = 0; //used in ROP calcs, line ~360

// Default Teensy values
const uint8_t spectra_buf[128] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31,
                                  32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63,
                                  64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95,
                                  96, 97, 98, 99, 100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 125, 126, 127
                                 };

/****Buffer Indices: UPDATE WHEN ALLVARS CHANGES****/
int spectra_buf_index = 136;    //  The start index of the spectra buffer, in bytes, into the char array of the AllVars struct.
int flt_buff_index = 264;       //  The start index of the flight buffer, in bytes, into the char array of the AllVars struct.

// These variables are used in the flight message as a character array.
String flt_ctrl_msg = "";
char flt_buff[256];             //  A char array buffer capable of holding the entire 256 byte flight message.
int flt_buff_count = 128;       //  This needs to be reset when a real flight message arrives.
int flt_msg_offset = 0;         //  Offset into the 256 byte arrray, flt_buff, depending on which message is loaded (equal to 0 or 1, for an index of 0 or 128)
String temp_str_msg;            //  A string that is used to load a default message into the flight message buffer when in DUMMY mode.


// copy_teensy_status
//  Loads variables from teensy_interface into the bin_pack global.
void copy_teensy_status()
{
  if ( DUMMY_TEENSY )
  {
    ready_flags.spectra = true;
    ready_flags.accel = true;
    ready_flags.orientation = true;
    ready_flags.axis = true;

    yaw = .5 * cos(.0005 * millis());
    pitch = .5 * sin(.0005 * millis());
    roll = 1.3 * sin(.0005 * millis());
    acc_x = .5 * cos(.0005 * millis());
    acc_y = .5 * sin(.0005 * millis());
    acc_z = 1.3 * sin(.0005 * millis());
    axis_one = .5 * cos(.0005 * millis());
    axis_two = .5 * sin(.0005 * millis());
    normal = 1.3 * sin(.0005 * millis());
  }

  if ( ready_flags.spectra )
  {
    // Example of memcpy from: https://forum.arduino.cc/index.php?topic=274173.0
    if ( DUMMY_TEENSY )
    {
      // Copy the constant, spectra_buff
      memcpy(bin_pack.a.spectra_buf, spectra_buf, sizeof(spectra_buf[0]) * 128);
    }
    else
    {
      // Copy the teensy buffer, power_spectra
      memcpy(bin_pack.a.spectra_buf, power_spectra, sizeof(power_spectra[0]) * 128);
    }

  }
  if ( ready_flags.accel )
  {
    // Floats 26, 27, and 28 are the accelerometer values
    bin_pack.f[26] = acc_x;
    bin_pack.f[27] = acc_y;
    bin_pack.f[28] = acc_z;
  }
  if ( ready_flags.orientation )
  {
    // Floats 29, 30, and 31 are the orientation values
    bin_pack.f[29] = roll;
    bin_pack.f[30] = pitch;
    bin_pack.f[31] = yaw;
  }
  if ( ready_flags.axis)
  {
    // Floats 15, 16, and 17 are for the strain gauges monitored by the teensy_interface.
    bin_pack.f[15] = axis_one;
    bin_pack.f[16] = axis_two;
    bin_pack.f[17] = normal;
  }

  // booleans, 'b', indexed at 128, 129, 130, and 131 hold the ready flags.
  bin_pack.b[128] = ready_flags.axis;
  bin_pack.b[129] = ready_flags.accel;
  bin_pack.b[130] = ready_flags.orientation;
  bin_pack.b[131] = ready_flags.spectra;

  // unset the flags as we just read the latest data.
  ready_flags.spectra = false;
  ready_flags.accel = false;
  ready_flags.orientation = false;
  ready_flags.axis = false;
  ready_flags.sem = false;

  //lowpass accel data
  lowpass_accel_data(); //update global data with lowpass filtered 
}

// load_flt_msg
//  Loads the Flight Controller Message data into the global variable. To be replaced with actual
//  Flight Controller Messages later
void load_flt_msg(String msg)
{
  // Convert the flight controller message string to a char array.
  msg.toCharArray(flt_buff, msg.length() );
}

// pack_and_send_spectra
//  Copies bytes from the spectra buffer into the binary packed char array and
//  sends those bytes via wireless serial
void pack_and_send_spectra( char packet_num )
{
  // The function, copy_teensy_status, takes care of updating the bin_pack variable.
  copy_teensy_status();

  // Write the characters of bin_pack to the wireless port.
 // xbee_write_char(&bin_pack.c[spectra_buf_index], SPECTRA_PACKET_SZ, packet_num);
}

// pack_and_send_flt_msg
//  Copies bytes from the Flight Control Message buffer into the binary packed char array and
//  sends those bytes via wireless serial
//  Input: packet_num will be 1 or 2
void pack_and_send_flt_msg( char packet_num )
{
  // Calculate the offset into the flight message buffer for the input packet_num.
  flt_msg_offset = ((int)(packet_num - 1)) * SUB_PACKET_SZ_CHAR;
  
  // Copy the flight controller message chars into the array.
  for ( int i = 0; i < SUB_PACKET_SZ_CHAR; i++)
  {
    // Check if only using dummy flight message data
    if ( DUMMY_FLT_MSG )
    {
      // Store this default message's length
      flt_buff_count = 24;

      // If so, then create a default char array
      char default_flt_msg[flt_buff_count];
      if( flt_msg_offset==0 )
      {
        temp_str_msg = "#NIM# DUMMY MESSAGE...1";
        temp_str_msg.toCharArray(default_flt_msg, flt_buff_count);

      }
      else
      {
        temp_str_msg = "#NIM#...DUMMY MESSAGE 2";
        temp_str_msg.toCharArray(default_flt_msg, flt_buff_count);
      }

      // Copy the default flight message into the actual buffer
      for ( int j = 0; j < flt_buff_count; j++)
      {
        flt_buff[j] = default_flt_msg[j];
      }
    }

      flt_buff_count = strlen(flt_buff);
      //Serial.print("flt buff count: ");
      //Serial.println(flt_buff_count);

    // The loop should continue with real flight message buffer values, or the dummy from above
    if ( i < flt_buff_count )
    {
      // Offset into the flight buffer by packet_num * packet size, in char iterations
      bin_pack.c[i + flt_buff_index + flt_msg_offset] = flt_buff[i];
    }
    else
    {
      // If the message is not long enough for the buffer, pad with periods.
      bin_pack.c[i + flt_buff_index + flt_msg_offset] = ' ';
    }
  }

  // Write the characters of flight message from bin_pack to the wireless port.
  //xbee_write_char(&bin_pack.c[flt_buff_index + flt_msg_offset], FLT_MSG_PACKET_SZ, packet_num);
}


// copy_odrive_status
//  Runs the function, , to retrieve the status as a sequence of bytes
//  from Odrive. The bin_pack Union has been upgraded to include a struct for all the variables
//  within the Odrive message.
void copy_odrive_status()
{
  if ( get_odrive_status_bin(bin_array, od_bin_size) )
  {
    // Error - Write 0 into the 60 bytes from the Odrive
    for (int i = 0; i < od_data_size; i++)
    {
      bin_pack.c[i] = 0;
    }
  }
  
  else
  {
    // No Error, copy the voltage and time stamp from Odrive string
    for (int i = 0; i < 8; i++)
    {
      odrive_conv.c[i] = bin_array[i];
    }

    // Check for good Odrive data output
    if (odrive_conv.f[1] == 0)
    {
      //      backupSerial.println("ODRIVE TIME STAMP FAIL!");
      // If the Odrive time stamp is zero, then it is bad data and use the old instead.
      odrive_conv.f[0] = odrive_volt;
      odrive_conv.f[1] = odrive_time;

      // Leave the remaining Odrive (odrive_conv) data as-is.
    }
    else
    {
      // Else the Odrive time is fine, and we update the global backup var.
      odrive_volt = odrive_conv.f[0];
      odrive_time = odrive_conv.f[1];

      // Copy the bytes as chars from bin_array to the bin_pack, one at a time
      for (int i = 0; i < od_bin_size; i++)
      {
        bin_pack.c[i] = bin_array[i];
      }
    }

    // Calibrate the RPM value from the Odrive for the gearing ratios.
    bin_pack.a.M0_Output_RPM = bin_pack.a.M0_Output_RPM * M0_GEAR_RATIO * -1.0; //invert
    bin_pack.a.M1_Output_RPM = bin_pack.a.M1_Output_RPM * M1_GEAR_RATIO;
    bin_pack.a.M1_Current = -1.0 * bin_pack.a.M1_Current;
    bin_pack.a.M0_Current = -1.0 * bin_pack.a.M0_Current;
       
lowpass_auger_current();//update global variable with filterd current value
lowpass_auger_RPM(); //update global variable with filterd RPM value
lowpass_elevator_current();//update global variable with filterd current value
lowpass_elevator_RPM(); //update global variable with filterd RPM value

//    // DEBUG - Print out the results of the bin_pack struct's floats.
//    for(int i=0; i<15; i++)
//    {
//      backupSerial.print("bin_pack.f[");
//      backupSerial.print(i);  
//      backupSerial.print("] = ");
//      backupSerial.println(bin_pack.f[i]);
//    } 
  }
} // end copy_odrive_status

void lowpass_accel_data()
{
  float abs_acel_x = abs(acc_x);
  float abs_acel_y = abs(acc_y);
  float abs_acel_z = (abs(acc_z - 1)); 
    
    float alpha = .65;
    accel_x_filtered =  ((alpha * abs_acel_x) + ((1 - alpha) * prev_accel_x_filtered));
    prev_accel_x_filtered =  accel_x_filtered;  
    

    accel_y_filtered =  ((alpha * abs_acel_y) + ((1 - alpha) * prev_accel_y_filtered));
    prev_accel_y_filtered =  accel_y_filtered;

    accel_z_filtered =  ((alpha * abs_acel_z) + ((1 - alpha) * prev_accel_z_filtered));
    prev_accel_z_filtered =  accel_z_filtered;
    /*
    FCSerial.print(accel_x_filtered);
    FCSerial.print(", ");
    FCSerial.print(accel_y_filtered);
    FCSerial.print(", ");
    FCSerial.print(accel_z_filtered);
    FCSerial.print(", ");
    FCSerial.print(abs_acel_x);
    FCSerial.print(", ");
    FCSerial.print(abs_acel_y);
    FCSerial.print(", ");
    FCSerial.println(abs_acel_z); */
}
//update global variable with filtered value
void lowpass_auger_current()
{
    float alpha = 0.3;
    auger_current_filtered =  (alpha * bin_pack.a.M0_Current + (1 - alpha) * prev_auger_current_filtered);
    prev_auger_current_filtered = auger_current_filtered;
//    backupSerial.print(bin_pack.a.M0_Current);
//    backupSerial.print(", ");
//    backupSerial.print(auger_current_filtered);
//    backupSerial.print(", ");
}
void lowpass_auger_RPM()
{
    float alpha = 0.3;
    auger_RPM_filtered =  (alpha * bin_pack.a.M0_Output_RPM + (1 - alpha) * prev_auger_RPM_filtered);
    prev_auger_RPM_filtered = auger_RPM_filtered;
//    backupSerial.print( bin_pack.a.M0_Output_RPM);
//    backupSerial.print(", ");
//    backupSerial.print(auger_RPM_filtered);
//    backupSerial.print(", ");

}
void lowpass_elevator_current()
{
    float alpha = 0.3;
    elev_current_filtered =  (alpha * bin_pack.a.M1_Current + (1 - alpha) * prev_elev_current_filtered);
    prev_elev_current_filtered = elev_current_filtered;
//    backupSerial.print(bin_pack.a.M1_Current);
//    backupSerial.print(", ");
//    backupSerial.print(elev_current_filtered);
//    backupSerial.print(", ");
}
void lowpass_elevator_RPM()
{
    float alpha = 0.3;
    elev_RPM_filtered =  (alpha * bin_pack.a.M1_Output_RPM + (1 - alpha) * prev_elev_RPM_filtered);
    prev_elev_RPM_filtered = elev_RPM_filtered;
//    backupSerial.print( bin_pack.a.M1_Output_RPM);
//    backupSerial.print(", ");
//    backupSerial.println(elev_RPM_filtered);
}
// check_odrive_errors
//  Runs the function, get_odrive_status_bin, to retrieve the status as a sequence of bytes
//  from Odrive. The error values within bin_pack's Union are then printed to the backupSerial
//  port.
void check_odrive_errors()
{
  if ( get_odrive_status_bin(bin_array, od_bin_size) )
  {
    // Error - Write 0 into the 60 bytes from the Odrive
    for (int i = 0; i < od_data_size; i++)
    {
      bin_pack.c[i] = 0;
    }
  }
  else
  {
    // No Error, copy the voltage and time stamp from Odrive string
    for (int i = 0; i < 8; i++)
    {
      odrive_conv.c[i] = bin_array[i];
    }

    // Check for good Odrive data output
    if (odrive_conv.f[1] == 0)
    {
      //      backupSerial.println("ODRIVE TIME STAMP FAIL!");
      // If the Odrive time stamp is zero, then it is bad data and use the old instead.
      odrive_conv.f[0] = odrive_volt;
      odrive_conv.f[1] = odrive_time;

      // Leave the remaining Odrive (odrive_conv) data as-is.
    }
    else
    {
      // Else the Odrive time is fine, and we update the global backup var.
      odrive_volt = odrive_conv.f[0];
      odrive_time = odrive_conv.f[1];

      // Copy the bytes as chars from bin_array to the bin_pack, one at a time
      for (int i = 0; i < od_bin_size; i++)
      {
        bin_pack.c[i] = bin_array[i];
      }
    }

    // Print out the error info
    uint32_t error_code = 0;

    // Pull the packed error bits for M1 errors...
    uint32_t error_bits = bin_pack.a.M0_Error;

    //  float M0_Axis_Error;
    uint32_t error_mask = 0x00000FFF; // bits 0:11, lsb
    error_code = error_bits & error_mask;
    //backupSerial.print(F("M0_Axis_Error:\t\t"));
    //backupSerial.println(error_code);

    //  float M0_Controller_Error;
    error_mask = 0x00001000; // bits 12, lsb
    error_code = error_bits & error_mask;
    //backupSerial.print(F("M0_Controller_Error:\t"));
    //backupSerial.println(error_code >> 12);

    //  float M0_Encoder_Error;
    error_mask = 0x000DE000; // bits 13:18, lsb
    error_code = error_bits & error_mask;
    //backupSerial.print(F("M0_Encoder_Error:\t"));
    //backupSerial.println(error_code >> 13);

    //  float M0_Motor_Error;
    error_mask = 0xFFFA0000; // bits 19:31, lsb
    error_code = error_bits & error_mask;
    //backupSerial.print(F("M0_Motor_Error:\t\t"));
    //backupSerial.println(error_code >> 19);

    // Pull the packed error bits for M1 errors...
    error_bits = bin_pack.a.M1_Error;

    //  float M1_Axis_Error;
    error_mask = 0x00000FFF; // bits 0:11, lsb
    error_code = error_bits & error_mask;
    //backupSerial.print(F("M1_Axis_Error:\t\t"));
    //backupSerial.println(error_code);

    //  float M1_Controller_Error;
    error_mask = 0x00001000; // bits 12, lsb
    error_code = error_bits & error_mask;
    //backupSerial.print(F("M1_Controller_Error:\t"));
    //backupSerial.println(error_code >> 12);

    //  float M1_Encoder_Error;
    error_mask = 0x000DE000; // bits 13:18, lsb
    error_code = error_bits & error_mask;
   // backupSerial.print(F("M1_Encoder_Error:\t"));
    //backupSerial.println(error_code >> 13);

    //  float M1_Motor_Error;
    error_mask = 0xFFFA0000; // bits 19:31, lsb
    error_code = error_bits & error_mask;
    //backupSerial.print(F("M1_Motor_Error:\t\t"));
    //backupSerial.println(error_code >> 19);
  }
}

// copy_arduino_status
//  Reads the various sensors/variables on the Arduino and copies to the bin_pack Union.
//  The bin_pack Union has been upgraded to include a struct for all the variables
//  within the Arduino message.
void copy_arduino_status()
{
  dig_time = millis();

  // Dummy Switch data
  if ( DUMMY_SWITCH_FLAG )
  {
    // Switches may not be present in system, set values
    swt = 1;
    swb = 0;
  }
  else
  {
    // Switches are present in system, read pins
    swt = !digitalRead(LimitSWTop); // Invert the active low signal
    swb = !digitalRead(LimitSWBot); // Invert the active low signal
  }
  
  M0_Torque = ((bin_pack.a.M0_Current * 8.27 * 6.0) / (100.0)) * .738 ; //units: ft-lbs: (current * 8.27 * gear_ratio) / (KV)
  M0_Work = ((M0_Torque/.738) * 2.0 * 3.14159 * bin_pack.a.M0_Output_RPM) / 60.0; // is actually power in Watts, not work
  
  ROP = (((bin_pack.a.M1_Position - oldPos)/8124.0)*(12.0/16)*(.1)) / ((dig_time - prevTime)/1000.0); // (((bin_pack.a.M1_Position - oldPos)/enc_steps_per_rot)*(pulley radtios)*(leadscrew pitch)). Units: inches per second.
  //MSE = -1;
  //Serial.print("ROP: ");
  //Serial.println(ROP);

  float MSE_torque_term = (480 * M0_Torque * bin_pack.a.M0_Output_RPM)/(3.0*3.0*ROP);
  float MSE_wob_term = (4.0 * normal)/(3.141*3*3); //normal is WOB from teensy
  MSE = MSE_torque_term + MSE_wob_term;

  //Serial.print("MSE: ");
  //Serial.println(MSE);
  // Dummy Servo Release Mechanism Data
  if ( DUMMY_SERVO_FLAG )
  {
    servoCurrent = 2123; // 2123 mA, or 2.123 A, to test magnitude and precision on GUI
    servoPos = -1234; // -1234 counts to test magnitude and sign on GUI
  }

  // Dummy Depth Measurement Data
  if ( DUMMY_DEPTH_FLAG )
  {
    bin_pack.a.M1_Position = 655360 + (655360 * sin(.0001 * millis()) ); // Max 12" distance to travel (about 2*655360 counts)
    if( bin_pack.a.M1_Position >= 655360)
    {
      // Simulate that auger has touched the ground by reporting the ground level position
      bin_pack.a.Ground_Position = 655360; // 6" to ground (estimated)
    }
    else
    {
      // Simulate that auger has touched the ground by reporting the ground level position
      bin_pack.a.Ground_Position = -1; // 6" to ground (estimated)
    }
  }
  else
  {
    // Only pass real ground position as the the real M1 position comes from Odrive. 
    bin_pack.a.Ground_Position = ground_pos; // provided in teensy_interface
  }
  
  // There are 14 floats in the Odrive portion, meaning that the next float is stored at index 14.
  bin_pack.ul[14] = dig_time;

  // Floats 15, 16, and 17 are for the strain gauges monitored by the teensy_interface.

  // Start packing bytes at index 18
  bin_pack.ul[18] = swt;
  bin_pack.ul[19] = swb;
  bin_pack.ul[20] = 0; //was fsm_state, but not needed in my version -AP
  bin_pack.ul[21] = (unsigned long) servoCurrent;
  bin_pack.f[22] = M0_Torque;
  bin_pack.f[23] = M0_Work;
  bin_pack.f[24] = MSE;
  bin_pack.l[25] = (long) servoPos;

  prevTime = dig_time;
  oldPos = bin_pack.a.M1_Position;
}
  
