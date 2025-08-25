// digger.h
//
// Global variables for the digging controller
//

//Pin defs:
#define DIAG1Pin 41 //pin 51
#define DIAG2Pin 37 //pin 53
#define DIAG3Pin 40//pin 52
#define DIAG4Pin 7 //pin 16
#define DIAG5Pin 6 //pin 15


//Comms configuration
//Serial2 is always the Rpi, Serial1 is always to oDrive
//UISerial is the serial line for control of system - "Serial3" means you 
//use the Xbee for controlling the system via radio, "Serial" means you
//use the programmer cable for the interface
#define UISerial Serial3 //Serial for cable, Serial3 for xbee operations


//Initial telemetry values
String volt_str = "";
String time_str = "";
String cur_str0 = "";
String cur_str1 = "";
String vel_str = "";
String error_str = "";
String simData_str = "";

//Initial accelerometer values
float xa = 0;
float ya = 0;
float za = 0;

//Initial soil composition values, these are calcuated and 
//returned by the Raspberry pi attached to the system
double soilMoisture = 0.0;
double soilStrength = 0.0;

//cm per encoder count for depth measurment
double cmPerCount = ((16.0/18.0) * 0.254) / 8192.0;

int serialDelay = 5; //was 10 1-8-20
double startTime = 0;
double endTime = 0;

//delay variables for printing data
long lastPrint = 0;
int printDelay = 100; //100 ms print delay

//variables for checking progress
long lastDepth = 0;
int depthDelay = 5000; //5 second encoder count
double cmPerSecond = 0.0;
double depthCm = 0.0;

//Auger RPM variables
double prev_auger_RPM_filtered;
double auger_RPM_filtered;

//heartbeat delay variable
long heartbeat;

//Vars related to checking the accelerometer data
long lastAccel = 0;
int accelDelay = 0;

//delay variables for getting moisture conditions
long lastSoilCheck = 0;
int soilCheckDelay = 5000; //5 second check on soil conditions

//Motor command boolean
bool motorCommandSent = false;

//Peck speed - speeds are references to Odrive speeds
int peckSpeed = 27000;
long biteSpeed = 80000;

//Variables related to pecking
int numPecks = 0;
double peckDepthCheck = 0.0;
double peckAdvance = 0.0;

//Target variables - names are self-explanatory
int rpmTarget = 200;
int elevatorTarget = 5000;
double depthTarget = -14.0;

double zposInches = 0;

//testing flags - these are for testing the functionality of the software
//and forcing conditions.  
bool forceSlow = false;
bool forceHighMoisture = false;
bool forceHighStrength = false;
bool DEBUG = false;
bool simming = false;

bool checkingDegraded = false;
bool alwaysStreamData = false;
bool RCDiggingEnabled = false;

//Main telemetry data
volatile double mCtlVolts= 0; //global storage of the voltage reading from odrive.
volatile double mCtlSeconds = 0;
volatile double checkDegradedSeconds = 0;
volatile double digStartTime = 0;
volatile double mCtlCur0 = 0;
volatile double mCtlCur1 = 0;
volatile double mCtlVelRPM0 = 0;
volatile double mCtlVelRPM1 = 0;
volatile double mCtlError0 = 0;
volatile double mCtlError1 = 0;
volatile double mCtlEnc1 = 0;
volatile double groundLvl = 0;
volatile double startLvl = 0;
volatile double deepestDepth = 0;
volatile long mCtlPosSet1 = 0;
volatile long mCtlVelSet1 = 0;
volatile bool limitSWTopVal = 0;
volatile bool limitSWBotVal = 0;
volatile double WOB = 0;
volatile long target = 0;



//Limit Switches
#define LimitSWTop 2 //external interrupt pin
#define LimitSWBot 3 //external interrupt pin

int topSWState = 1;          // the current reading from the input pin
int botSWState = 1;          // the current reading from the input pin

int topSWLastButtonState = HIGH;   // the previous reading from the input pin
int botSWLastButtonState = HIGH;   // the previous reading from the input pin

unsigned long topSWLastDebounceTime = 0;  // the last time the output pin was toggled
unsigned long botSWLastDebounceTime = 0;  // the last time the output pin was toggled

unsigned long debounceDelay = 50;    // the debounce time; increase if the output flickers

bool initLimitSW = 1;
/*
 * Limit Switch Feedback Variable
 * how to clear transition variables?
Bit0 top switch voltage
Bit1 bottom switch voltage
Bit2 top switch falling transition
Bit3 top switch rising transition
Bit4 bot switch falling transition
Bit5 bot switch rising transition
Bit6 a top switch transition occured
Bit7 a bottom switch transition occured
*/
volatile byte limitSWfeedback = B00000000;

//HX711 stuff
//#define DOUT  26
//#define CLK  27
//HX711 scale;
//float calibration_factor = -50000; //started at -27000

//States in the FSM
enum State_enum {
  START, //0
  CRA, //1 - "CRA" = Constant Rate Augering
  PECK_UP_LONG,//2
  PECK_UP_SHORT,//3
  PECK_RETURN,//4
  BITE_UP,//5
  BITE_RETURN,//6,
  SLOW_RPM,//7
  SLOW_ELEVATOR,//8
  STOP,//9
  SUCCEED,//10
  FAIL,//11
  START_CRA_CONTROL,//12 
  CRA_CONTROL,//13
  START_SFC, //14
  SFC, //15
  PLACE_SENSOR //16 - This will place a sensor on the surface without spinning the auger
};

int currentState = STOP;
bool isOpen = false;
long dt;

enum Print_enum {
  NO_PRINT,
  PRINT
};

enum Mode_enum {
  ADAPT, //Adaptive Augering
  CR, // Constand Rate Augering 
  SURFACE //Surface sensor placement
};

int mode = ADAPT;

//Simulation stuff - these variables are used when the
//ground control station is injecting fake environment
//data back into the system in order to test the system's
//adaptive response.
typedef struct {
    double current0;
    double current1;
    double rpm0;
    double rpm1;
    double wob;
    double deepest;
    long xacc;
    long yacc;
    long zacc;
    double moist;
    double strength;
    double progress;
} Telem;

Telem sim;

// Teensy stuff for teensy_interface - this is all for using the teensy board

// Constants
#define BIN_STATUS_SIZE 136 //must be divisible by 4.
#define SUB_PACKET_SZ_CHAR 128 //must be divisible by 4
// MAX_FLT_MSG_SIZE must be a multiple of SUB_PACKET_SZ_CHAR
#define MAX_FLT_MSG_SIZE 256 //must be divisible by 4
// ALL_VARS_SIZE must be equal to sum of BIN_STATUS_SIZE, SUB_PACKET_SZ_CHAR, and MAX_FLT_MSG_SIZE
#define ALL_VARS_SIZE 516

#define SUB_PACKET_SZ_FLOAT 32
#define DUMMY_TEENSY 0
#define DUMMY_FLT_MSG 0 //JH 3/8/21
#define DUMMY_SWITCH_FLAG 0
#define DUMMY_SERVO_FLAG 0
#define DUMMY_STRAIN_FLAG 0
#define DUMMY_DEPTH_FLAG 0

//Globals for teensy data grab

struct AllVars
{
  float supply_voltage;
  unsigned long system_time;
  float M0_Current;
  float M1_Current;
  float M0_Output_RPM;
  float M1_Output_RPM;
  long M1_Position;
  unsigned long M0_Error;
  unsigned long M1_Error;
  float M0_Ibus_Current;
  float M1_Ibus_Current;
  long Encoder0_Shadow_Count;
  float Controller0_Velocity_SetPoint;
  float Controller1_Velocity_SetPoint;
  unsigned long Dig_Time;
  float StrainX;
  float StrainY;
  float WOB;
  unsigned long Limit_Switch_Top;
  unsigned long Limit_Switch_Bottom;
  unsigned long State_Num; 
  unsigned long Release_servo_current;
  float M0_Torque;
  float M0_work;
  float MSE;
  long Release_servo_position;
  float acc_x;
  float acc_y;
  float acc_z;
  float roll;
  float pitch;
  float yaw;
  bool strain_stale;
  bool acc_stale;
  bool rpy_stale;
  bool fft_stale;
  long Ground_Position;
  uint8_t spectra_buf[SUB_PACKET_SZ_CHAR];
  char flt_buff[MAX_FLT_MSG_SIZE];
};

// For packing NIMBUS data into a char array
/* typedef union 
{
    float f[BIN_STATUS_SIZE/4];
    unsigned long ul[BIN_STATUS_SIZE/4];
    long l[BIN_STATUS_SIZE/4];
    char c[BIN_STATUS_SIZE];
    uint8_t u8[BIN_STATUS_SIZE];
    bool b[BIN_STATUS_SIZE];
    AllVars a;
}Bin_pack_t; */

typedef union 
{
    float f[ALL_VARS_SIZE/4];
    unsigned long ul[ALL_VARS_SIZE/4];
    long l[ALL_VARS_SIZE/4];
    char c[ALL_VARS_SIZE];
    uint8_t u8[ALL_VARS_SIZE];
    bool b[ALL_VARS_SIZE];
    AllVars a;
}Bin_pack_t;

Bin_pack_t bin_pack;

union Odrive_convert
{
  float f[14];                  // 56 bytes of data in 14 single floating point numbers
  char c[56];                   // 56 bytes of data in 56 signed char's
};

// Internal Command Handler Variables
Odrive_convert odrive_conv;

#define TIMEOUT 500


const int slave_address = 10;

char flag;
char value;
char term_char;
bool newFFT; //1 when new FFT has arrived

int no_progress_counter;
char stateStr[30];
long ground_pos;
bool WOBVelCtlFlag;

long soilZero;

long depthDelta;

int32_t servoPos;
uint8_t servoRpm;
uint16_t servoCurrent;

int STATUS_BUFF_SZ;

uint8_t power_spectra[128];
uint8_t magnitude;

float yaw, pitch, roll;
float acc_x, acc_y, acc_z;
float normal, axis_one, axis_two;

long prev_yaw_time = 0;
float prev_yaw = 0;
float new_yaw = 0;
long new_yaw_time = 0;
float relative_yaw = 0;
float relative_yaw_filtered = 0;
float prev_relative_yaw = 0;
float prev_relative_yaw_filtered = 0;

typedef struct
{
  bool spectra;
  bool accel;
  bool orientation;
  bool axis;
  int sem;
} ReadyFlags_t;

//extern ReadyFlags_t ready_flags;

ReadyFlags_t ready_flags;

uint8_t i;
uint8_t j;

volatile union CONVERTER{
    byte  buffer[4];
    float floater;
} converter;

union SYNC
{
    byte buffer[8];
    long int time;
} sync;





/*  Function: isTeensyLocked
 *  Author: WMT
 *  Description:  Returns the status of the ready_flag semaphore. The ready_flag.sem is used to manage access to the global variable.
 */
bool isTeensyLocked()
{
    return( ready_flags.sem );
}

/*  Function: lockTeensySem
 *  Author: WMT
 *  Description:  Sets the ready_flag semaphore to 1 in order to lock out the global variable.
 */
void lockTeensySem()
{
    ready_flags.sem = 1;
}

/*  Function: releaseTeensySem
 *  Author: WMT
 *  Description:  Sets the ready_flag semaphore to 0 in order to unlock out the global variable - allowing other code to write.
 */
void releaseTeensySem()
{
    ready_flags.sem = 0;
}

/*  Function: waitOnTeensySem
 *  Author: WMT
 *  Description:  Halts execution of the caller until the timeout value is reached. Returns -1 to indicate that timeout reached.
 */
int waitOnTeensySem()
{
    // Assume that the semaphore is locked, 1.
    int rtn = 1;

    // Timer values
    unsigned long cur_time, dur_time;
    cur_time = millis();
    dur_time = 0;

    // Loop until semaphore goes false, 0, or timeout is elapsed.
    while( ready_flags.sem && (dur_time < TIMEOUT) )
    {
      dur_time = millis() - cur_time;
    }

    // Check if While ended due to timeout, -1.
    if( dur_time < TIMEOUT )
    {
      rtn = -1;
    }
    else  //If it did not end due to timeout, then write it is open, 0.
    {
      rtn = 0;
    }

    return( rtn );
}

/*  Function: setSpectraFlag
 *  Author: WMT
 *  Description:  Sets (1) the ready_flags.spectra flag using the semaphore.
 */
bool setSpectraFlag()
{
  // Wait on the semaphore to set to 0.
  if( waitOnTeensySem() == 0)
  {
    ready_flags.spectra = 1;
    return( true );
  }
  else // Semaphore denied write, return false
  {
    return( false );
  }
}

/*  Function: clearSpectraFlag
 *  Author: WMT
 *  Description:  Clears (0) the ready_flags.spectra flag using the semaphore.
 */
bool clearSpectraFlag()
{
  // Wait on the semaphore to set to 0.
  if( waitOnTeensySem() == 0)
  {
    ready_flags.spectra = 0;
    return( true );
  }
  else // Semaphore denied write, return false
  {
    return( false );
  }
}

/*  Function: setAccelFlag
 *  Author: WMT
 *  Description:  Sets (1) the ready_flags.accel flag using the semaphore.
 */
bool setAccelFlag()
{
  // Wait on the semaphore to set to 0.
  if( waitOnTeensySem() == 0)
  {
    ready_flags.accel = 1;
    return( true );
  }
  else // Semaphore denied write, return false
  {
    return( false );
  }
}

/*  Function: clearAccelFlag
 *  Author: WMT
 *  Description:  Clears (0) the ready_flags.accel flag using the semaphore.
 */
bool clearAccelFlag()
{
  // Wait on the semaphore to set to 0.
  if( waitOnTeensySem() == 0)
  {
    ready_flags.accel = 0;
    return( true );
  }
  else // Semaphore denied write, return false
  {
    return( false );
  }
}

/*  Function: setOriFlag
 *  Author: WMT
 *  Description:  Sets (1) the ready_flags.orientation flag using the semaphore.
 */
bool setOriFlag()
{
  // Wait on the semaphore to set to 0.
  if( waitOnTeensySem() == 0)
  {
    ready_flags.orientation = 1;
    return( true );
  }
  else // Semaphore denied write, return false
  {
    return( false );
  }
}

/*  Function: clearOriFlag
 *  Author: WMT
 *  Description:  Clears (0) the ready_flags.orientation flag using the semaphore.
 */
bool clearOriFlag()
{
  // Wait on the semaphore to set to 0.
  if( waitOnTeensySem() == 0)
  {
    ready_flags.orientation = 0;
    return( true );
  }
  else // Semaphore denied write, return false
  {
    return( false );
  }
}

/*  Function: setAxisFlag
 *  Author: WMT
 *  Description:  Sets (1) the ready_flags.axis flag using the semaphore.
 */
bool setAxisFlag()
{
  // Wait on the semaphore to set to 0.
  if( waitOnTeensySem() == 0)
  {
    ready_flags.orientation = 1;
    return( true );
  }
  else // Semaphore denied write, return false
  {
    return( false );
  }
}

/*  Function: clearAxisFlag
 *  Author: WMT
 *  Description:  Clears (0) the ready_flags.axis flag using the semaphore.
 */
bool clearAxisFlag()
{
  // Wait on the semaphore to set to 0.
  if( waitOnTeensySem() == 0)
  {
    ready_flags.axis = 0;
    return( true );
  }
  else // Semaphore denied write, return false
  {
    return( false );
  }
}
