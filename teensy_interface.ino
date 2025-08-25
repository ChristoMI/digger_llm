/***********************************************************************************/
/*  Title:        teensy_interface
 *   Author:      EP
 *   Date:        <9/24/2020
 *   Description: Asynchronous interface to separate Teensy board monitoring the: 
 *                accelerometers, compass, strain gauges, and spectral power 
 *                measurement.
 */
/***********************************************************************************/
//#include "teensy_interface.h"
#include <Wire.h>


void setupTeensyInterface()
{
    Wire.begin(slave_address);
    Wire.onReceive(receive_event);
    Wire.onRequest(request_event);

    // Unset all the flags
    ready_flags.spectra = false;
    ready_flags.accel = false;
    ready_flags.orientation = false;
    ready_flags.axis = false;
    ready_flags.sem = false;

    // Initial Values
    yaw = 0;
    pitch = 0;
    roll = 0;
    acc_x = 0;
    acc_y = 0;
    acc_z = 0;
    normal = 0;
    axis_one = 0;
    axis_two = 0;
}

void request_event(void)
{
    sync.time = millis();
    Wire.write(sync.buffer, 8);
    Wire.write('z');
}

void receive_event(int data_size) {
    char flag = Wire.read();
    float alpha = 0.3;  //for yaw LP filter. 
    //Serial.print("Getting data from teensy: ");Serial.println(flag);
            
    switch(flag)
    {
        case 'l':
            UISerial.println(F("[WARNING] Accelerometer values have exceeded the designated threshold"));
            break;
        case 'a':
            for (i = 0; i < 30; i++)
            {
                power_spectra[i] = Wire.read();
                //Serial.println(power_spectra[i]);
            }
            ready_flags.spectra = false;
            newFFT = LOW;
            break;
        case 'b':
            for (i = 30; i < 60; i++)
            {
                power_spectra[i] = Wire.read();
                //Serial.println(power_spectra[i]);
            }
            ready_flags.spectra = false;
            break;
        case 'c':
            for (i = 60; i < 90; i++)
            {
                power_spectra[i] = Wire.read();
                //Serial.println(power_spectra[i]);
            }
            ready_flags.spectra = false;
            break;
        case 'd':
            for (i = 90; i < 120; i++)
            {
                power_spectra[i] = Wire.read();
                //Serial.println(power_spectra[i]);
            }
            ready_flags.spectra = false;
            break;
        case 'e':
            for (i = 120; i < 128; i++)
            {
                power_spectra[i] = Wire.read();
                //Serial.println(power_spectra[i]);
            }
            ready_flags.spectra = true;
            newFFT = HIGH;

//            for(j = 0; j<128; j++)
//            {
//            Serial.print(power_spectra[j]);
//            }
//            Serial.println();
            break;
        case 'f':
            //Serial.print("Got teensy 'f'. Size of buffer is: ");Serial.println(sizeof(converter.buffer));
            for (i = 0; i < sizeof(converter.buffer); i++)
            {
                converter.buffer[i] = Wire.read();
                //Serial.print(converter.buffer[i]);
            }
            acc_x = converter.floater;
            //Serial.print("\nacc_x: ");Serial.println(acc_x);
            for (i = 0; i < sizeof(converter.buffer); i++)
            {
                converter.buffer[i] = Wire.read();
            }
            acc_y = converter.floater;
            for (i = 0; i < sizeof(converter.buffer); i++)
            {
                converter.buffer[i] = Wire.read();
            }
            acc_z = converter.floater;
            ready_flags.accel = true;
            break;
        case 'v':
            UISerial.println(F("[WARNING] Vibration threshold exceeded."));
            for (i = 0; i < 4; i++)
            {
                converter.buffer[i] = Wire.read();
            }
            
            magnitude = Wire.read();
            UISerial.println(converter.floater);
            UISerial.println(magnitude);
            break;
        case 'y':
            for (i = 0; i < sizeof(converter.buffer); i++)
            {
                converter.buffer[i] = Wire.read();
            }
            yaw = converter.floater;
            new_yaw = yaw;
            new_yaw_time = millis();
            relative_yaw = (new_yaw - prev_yaw)/((new_yaw_time - prev_yaw_time)/1000.0);

            relative_yaw_filtered =  (alpha * relative_yaw + (1 - alpha) * prev_relative_yaw_filtered);
            prev_relative_yaw_filtered = relative_yaw_filtered;

    
            yaw = relative_yaw_filtered;
            prev_yaw_time = new_yaw_time;
            prev_yaw = new_yaw;
            
            //Serial.print("Yaw: "); Serial.print(converter.floater); Serial.print("\t");
            for (i = 0; i < sizeof(converter.buffer); i++)
            {
                converter.buffer[i] = Wire.read();
            }
            roll = converter.floater; //this used to be pitch, it was changed to correct the reversal inside the teensy
            //Serial.print("Pitch: "); Serial.print(converter.floater); Serial.print("\t");
    
            for (i = 0; i < sizeof(converter.buffer); i++)
            {
                converter.buffer[i] = Wire.read();
            }
            pitch = converter.floater; //this used to be roll,
            //Serial.print("Roll: "); Serial.print(converter.floater); Serial.println();
            ready_flags.orientation = true;
            break;
        case 's':
            for (i = 0; i < sizeof(converter.buffer); i++)
            {
                converter.buffer[i] = Wire.read();
            }
            normal = converter.floater;
            //Serial.print("Normal: "); Serial.print(converter.floater); Serial.print("\t");
            for (i = 0; i < sizeof(converter.buffer); i++)
            {
                converter.buffer[i] = Wire.read();
            }
            axis_one = converter.floater;
            //Serial.print("Axis One: "); Serial.print(converter.floater); Serial.print("\t");
    
            for (i = 0; i < sizeof(converter.buffer); i++)
            {
                converter.buffer[i] = Wire.read();
            }
            axis_two = converter.floater;
            //Serial.print("Axis Two: "); Serial.print(converter.floater); Serial.println();
            ready_flags.axis = true;
            break;
    }

    char term_char = Wire.read();
    if (term_char != 'z')
    {
        UISerial.print(F("[ERROR] Invalid data has been received. Termination character received: ")); Serial.println(term_char);
    }
}
