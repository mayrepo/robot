/*
 * This program send a message to the PC throught the serial port.
 *
 * Note that :
 *   - You !!!SHOULD!!! define the __CM5_STARTUP__ symbol before
 *     including libCM5.h, if you declare your program entry point
 *
 *   - You should call CM5_init() before doing anything else in main !!!!!!!!
 *
 *   - To terminate a program do an infinite loop
 *
 * If you forget any of those rules, then you are bound to a hellish
 * nightmare made of bricked CM5 and jamed terminal display.
 */

#define __CM5_STARTUP__
#include <libCM5.h>
#include <util/delay.h>

#include <avr/io.h>
#include <avr/interrupt.h>



////////////////////////////////////////////////////////////
///////////////////// value definitions ////////////////////
////////////////////////////////////////////////////////////

// assigning each motor's dyna_id to its position will
// make it easier for you to design your algorithm

#define down_right 2
#define down_left 3
#define up_left 4
#define up_right 1

#define INIT 0
#define GO_TO_CENTER 1
#define SEEKING 2
#define CHASING 3
#define GO_FORWARD 4
#define STOP -1


#define speed_ini 400
#define speed_max 512

/////////////////// CURSED CODE, DON'T TOUCH !!! ////////////

// global variable to count the number of overflows
volatile uint8_t tot_overflow;
volatile uint8_t finished ;
int nrOverflows;
  
// initialize timer, interrupt and variable
// delay is in millisceonds (ms)
void timer1_init(int delay)
{
    nrOverflows = delay / 33;
    // set up timer with prescaler = 8
    TCCR1B |= (1 << CS11);
  
    // initialize counter
    TCNT1 = 0;
  
    // enable overflow interrupt
    TIMSK |= (1 << TOIE1);
  
    // enable global interrupts
    sei();
  
    // initialize overflow counter variable
    tot_overflow = 0;
    finished=0 ;
}
  
// TIMER1 overflow interrupt service routine
// called whenever TCNT1 overflows
ISR(TIMER1_OVF_vect)
{
    // keep a track of number of overflows
    tot_overflow++;
  
    // check for number of overflows here itself
    if (tot_overflow >= nrOverflows) // NOTE: '>=' used instead of '=='
    {
        finished=1;
        // no timer reset required here as the timer
        // is reset every time it overflows
  
        tot_overflow = 0;   // reset overflow counter
    }
}

///////////////////////////////////////////////////////////




////////////////////////////////////////////////////////////
///////////////////// CM-5  ////////////////////////////////
////////////////////////////////////////////////////////////

// robot initialisation
// IMPORTANT to call this BEFORE NAY OTHER FUNCTION
// at the start of your main()
void init(void) {
  CM5_init();
  CM5_RS232_writeString("\nCM5 init complete\n");
}

// moderately precise timer using internal CM-5 chips. The function will return after having waited a specified time
// I believe precise up to 0.03s
// parameter delay: time to wait in milliseconds
void wait_ms(int delay) 
{
    // initialize timer
    timer1_init(delay);
    
    int counter=0;
    while(finished!=1)
    {
      counter+=1;
    }
}

//////////////////////////////////////
/////////////// AX 12 ////////////////
//////////////////////////////////////

// infinite turn mode activation, see technical docu
// parameter: ID of motor
void infiniteTurn(dyna_id motor) {
  gbpParameter[0] = AX12_CTAB_ID_CWAngleLimitLo;
  *((unsigned int*)(gbpParameter + 1)) = 0;
  *((unsigned int*)(gbpParameter + 3)) = 0;
  CM5_RS485_send(motor, INST_ID_WriteData, 5);
  CM5_RS485_receive(6);
  CM5_RS232_writeString("\nCM5 infinite rotation mode completed\n");
}

// infinite turn mode desactivation, see technical docu
// parameter: ID of motor
void normalTurn(dyna_id motor) {
  gbpParameter[0] = AX12_CTAB_ID_CWAngleLimitLo;
  *((unsigned int*)(gbpParameter + 1)) = 0;
  *((unsigned int*)(gbpParameter + 3)) = 1023;
  CM5_RS485_send(motor, INST_ID_WriteData, 5);
  CM5_RS485_receive(6);
  CM5_RS232_writeString("\nCM5 Normal rotation mode completed\n");
}


// set rotation speed of a single motor, only works in infinite turn mode!
// speed is an integer between -1023 and 1023
// parameter motor: ID of motor
// parameter speed: rotation speed, between -1024 and 1024, sign controls direction
// speed 1 = no ratation, speed 0 = maximal speed
void setSpeed(dyna_id motor, int speed) {
  int order;
  gbpParameter[0] = AX12_CTAB_ID_MovingSpeedLo;
  if(speed >= 0)
    order = speed;
  else
    order = 1024 - speed;
  *((unsigned int*)(gbpParameter + 1)) = order;
  CM5_RS485_send(motor, INST_ID_WriteData, 3);
  CM5_RS485_receive(6);
}

// move motor to a given angle, only works when nOt in infinite turn mode
// parameter motor: ID of motor
// parameter: angle is an integer between -1023 and 1023
// no angle should be between 300 and 360 degrees
void setAngle(dyna_id motor, int angle, int speed) {
  setSpeed(motor, speed);
  int angle_norm;
  gbpParameter[0] = AX12_CTAB_ID_GoalPositionLo;
  
  if (angle >=0)
    angle_norm = angle;
  
  else
    angle_norm = 1024 + angle;
  
  *((unsigned int*)(gbpParameter + 1)) = angle_norm;
  CM5_RS485_send(motor, INST_ID_WriteData, 3);
  CM5_RS485_receive(6);
}


// turns on motor light
// parameter motor: ID of motor
void lightOn(dyna_id motor) {
  gbpParameter[0] = AX12_CTAB_ID_Led;
  *(unsigned int *)(gbpParameter + 1) = 1;
  CM5_RS485_send(motor, INST_ID_WriteData, 2);
  CM5_RS485_receive(6);
}

// turns off motor light
// parameter motor: ID of motor
void lightOff(dyna_id motor) {
  gbpParameter[0] = AX12_CTAB_ID_Led;
  *(unsigned int *)(gbpParameter + 1) = 0;
  CM5_RS485_send(motor, INST_ID_WriteData, 2);
  CM5_RS485_receive(6);
}

// returns the current motor's speed
// This functions does not return anything but stores the speed in its 2nd parameter which lust be a pointer to int
// parameter inId: ID of motor
// parameter outSpeed: pointer to which the speed will be stored
void getSpeed(dyna_id inId, unsigned int* outSpeed) {
  gbpParameter[0] = AX12_CTAB_ID_MovingSpeedLo;
  gbpParameter[1] = 2;
  CM5_RS485_send(inId, INST_ID_ReadData, 2);
  CM5_RS485_receive(8);
  *outSpeed = *((unsigned int*)(gbpRxBuffer + 5));
}


// returns the current motor's angle,  infinite turn must be disabled to use this function
// This functions does not return anything but stores the speed in its 2nd parameter which lust be a pointer to int
// parameter inId: ID of motor
// parameter outSpeed: pointer to which the speed will be stored
void getAngle(dyna_id inId, unsigned int* outAngle) {
  gbpParameter[0] = AX12_CTAB_ID_PresentPosLo;
  gbpParameter[1] = 2;
  CM5_RS485_send(inId, INST_ID_ReadData, 2);
  CM5_RS485_receive(8);
  *outAngle = *((unsigned int*)(gbpRxBuffer + 5));
}

/////////////////////////////////////////////////////
////////////////////////////// AX S1 ////////////////
/////////////////////////////////////////////////////


// returns the obstacle detection flag (using infrared sensors), see technical documentation
// parameter sensor: Id of AX-S1
// parameter boolLight: pointer to store data read from AX-S1
void checkObstacle(dyna_id sensor, unsigned int* infoObst) {
  gbpParameter[0]=AXS1_CTAB_ID_ObstacleDetectionFlag;
  gbpParameter[1] = 2; 
  CM5_RS485_send(sensor, INST_ID_ReadData, 2);
  int tmp = CM5_RS485_receive(8) ;
  if ( tmp== 8) {
    *infoObst = *((gbpRxBuffer + 5));
  }
 
  else {
    CM5_RS232_writeString("\nWrong length of received message\n");
  }
}


// returns the light detection flag (using visual light sensors), see technical documentation
// parameter sensor: Id of AX-S1
// parameter boolLight: pointer to store data read from AX-S1
void checkLuminosity(dyna_id sensor, unsigned int* boolLight)  {
  gbpParameter[0]=AXS1_CTAB_ID_LuminosityDetectionFlag;
  gbpParameter[1] = 2;
  CM5_RS485_send(sensor, INST_ID_ReadData, 2);
  int tmp = CM5_RS485_receive(8) ;
  if ( tmp== 8)  
    *boolLight = *((unsigned int*)(gbpRxBuffer + 5));
  else
    CM5_RS232_writeString("\nWrong length of received message\n");
    
} 
// returns the left infrared reading. Is a numerical value not just a flag!
// parameter sensor: Id of AX-S1
// parameter sideIR: pointer to store data read from AX-S1
void leftInfraRed(dyna_id sensor, unsigned int* sideIR) {
  gbpParameter[0]=AXS1_CTAB_ID_LeftIRSensorData;
  gbpParameter[1]=2;
  CM5_RS485_send(sensor, INST_ID_ReadData, 2);
  int tmp = CM5_RS485_receive(8) ;
  if ( tmp== 8)  
    *sideIR = *((unsigned int*)(gbpRxBuffer + 5));
  else
    CM5_RS232_writeString("\nWrong length of received message\n");
}

// returns the center infrared reading. Is a numerical value not just a flag!
// parameter sensor: Id of AX-S1
// parameter sideIR: pointer to store data read from AX-S1
void centerInfraRed(dyna_id sensor, unsigned int* sideIR) {
  gbpParameter[0]=AXS1_CTAB_ID_CenterIRSensorData;
  gbpParameter[1]=2;
  CM5_RS485_send(sensor, INST_ID_ReadData, 2);
  int tmp = CM5_RS485_receive(8) ;
  if ( tmp== 8)  
    *sideIR = *((unsigned int*)(gbpRxBuffer + 5));
  else
    CM5_RS232_writeString("\nWrong length of received message\n");
}

// returns the right infrared reading. Is a numerical value not just a flag!
// parameter sensor: Id of AX-S1
// parameter sideIR: pointer to store data read from AX-S1
void rightInfraRed(dyna_id sensor, unsigned int* sideIR) {
  gbpParameter[0]=AXS1_CTAB_ID_RightIRSensorData;
  gbpParameter[1]=2;
  CM5_RS485_send(sensor, INST_ID_ReadData, 2);
  int tmp = CM5_RS485_receive(8) ;
  if ( tmp== 8)  
    *sideIR = *((unsigned int*)(gbpRxBuffer + 5));
  else
    CM5_RS232_writeString("\nWrong length of received message\n");
}


// returns the left leight sensor reading. Is a numerical value not just a flag!
// parameter sensor: Id of AX-S1
// parameter leftLum: pointer to store data read from AX-S1
void leftLuminosity(dyna_id sensor, unsigned int* leftLum) {
  gbpParameter[0]=AXS1_CTAB_ID_LeftLuminosity ;
  gbpParameter[1] = 2;
  CM5_RS485_send(sensor, INST_ID_ReadData, 2);
  int tmp = CM5_RS485_receive(8) ;
  if ( tmp== 8)  
    *leftLum = *((unsigned int*)(gbpRxBuffer + 5));
  else
    CM5_RS232_writeString("\nWrong length of received message\n");
}

// returns the central light sensor reading. Is a numerical value not just a flag!
// parameter sensor: Id of AX-S1
// parameter centerLum: pointer to store data read from AX-S1
void centerLuminosity(dyna_id sensor, unsigned int* centerLum) {
  gbpParameter[0]=AXS1_CTAB_ID_CenterLuminosity ;
  gbpParameter[1] = 2;
  CM5_RS485_send(sensor, INST_ID_ReadData, 2);
  int tmp = CM5_RS485_receive(8) ;
  if ( tmp== 8)  
    *centerLum = *((unsigned int*)(gbpRxBuffer + 5));
  else
    CM5_RS232_writeString("\nWrong length of received message\n");
    
}

// returns the right light sensor reading. Is a numerical value not just a flag!
// parameter sensor: Id of AX-S1
// parameter rightLum: pointer to store data read from AX-S1
void rightLuminosity(dyna_id sensor, unsigned int* rightLum) {
  gbpParameter[0]=AXS1_CTAB_ID_RightLuminosity ;
  gbpParameter[1] = 2;
  CM5_RS485_send(sensor, INST_ID_ReadData, 2);
  int tmp = CM5_RS485_receive(8) ;
  if ( tmp== 8)  
    *rightLum = *((unsigned int*)(gbpRxBuffer + 5));
  else
    CM5_RS232_writeString("\nWrong length of received message\n");
}



// returns the amount of sound detected
// untested, see documentation of AX-S1!!
void dataSound(dyna_id sensor, unsigned int* dataSound) {
  gbpParameter[0] = AXS1_CTAB_ID_SoundData;
  gbpParameter[1] = 2;
  CM5_RS485_send(sensor, INST_ID_ReadData, 2);
  int tmp = CM5_RS485_receive(8) ;
  if ( tmp== 8) {
    *dataSound = *((gbpRxBuffer + 5));
    CM5_RS232_writeString("\nDetected Sound Intensity : ");
    CM5_RS232_writeHexInt(*dataSound);
    CM5_RS232_writeString("\n");
  } 

  else
    CM5_RS232_writeString("\nWrong length of received message\n");
}


//helper function
char noteBuzz(dyna_id sensor, int note) {
  gbpParameter[0] = AXS1_CTAB_ID_BuzzerIndex;
  gbpParameter[1] = note;
  CM5_RS485_send(sensor, INST_ID_WriteData, 2);
  CM5_RS485_receive(6);
  return 0;
}

//helper function
char timeBuzz(dyna_id sensor, int time) {
  gbpParameter[0] = AXS1_CTAB_ID_BuzzerTime;
  gbpParameter[1] = time;
  CM5_RS485_send(sensor, INST_ID_WriteData, 2);
  CM5_RS485_receive(6);
  return 0;
}

// play a note of given duration on the AX-S1.
// minimum duration: 0.3s, max is 5s
// intTime encodes duration, see technical docu
// intTime must be <50
void buzzWithDelay(dyna_id sensor, int note, unsigned char intTime) {
  timeBuzz(sensor,intTime);
  noteBuzz(sensor, note); 
  unsigned char noteFinished = 0 ;

  while (noteFinished==0)
  {
    gbpParameter[0] = AXS1_CTAB_ID_BuzzerTime;
    gbpParameter[1] = 1;
    CM5_RS485_send(sensor, INST_ID_ReadData, 2);
    unsigned int rec = CM5_RS485_receive(7);
    if (rec==7)
    {
      CM5_RS232_writeHexChar (gbpRxBuffer [5]) ;
      if( gbpRxBuffer [5]==0)
        noteFinished = 1 ;
    }
  }
}

/////////////////////////////////////////////////////////////////////////
//////////////////////////   M A I N   L O O P   ////////////////////////
/////////////////////////////////////////////////////////////////////////

// Warnings : do pay a lot of attention to the infinite turns




int main(void) {

  init();
  CM5_RS232_writeString("Starting!!\n");
  unsigned int a;
  int STATUS;
  // STATUS will define the robot's attitude towards its environment  

  
  STATUS= INIT;
  infiniteTurn(1);
  infiniteTurn(2);
  infiniteTurn(3);
  infiniteTurn(4);
  unsigned int field;
  unsigned int obs;
  
  // STATUS should equal INIT only at the beginning of each match
  
  while(STATUS!=STOP)
  {

     while (STATUS==INIT) { 
     // play some music
       buzzWithDelay(100, 30, 10);
       buzzWithDelay(100, 40, 10);
       buzzWithDelay(100, 50, 10);

       // blink some lights
       lightOn(1);
       //lightOff(1);

       lightOn(2);
       //lightOff(2);

       lightOn(3);
       //lightOff(3);

       //lightOn(4);
       lightOff(4);
  
       STATUS=GO_TO_CENTER;

        //eventually after waiting for a while
        // the robot will engage its first attitude

     }
  

     while (STATUS==GO_TO_CENTER) {  // the temporisation should be adapted  
   
       // go straight assuming its a 4_wheeled robot to the center of the field
       setSpeed(up_left, speed_ini);
       setSpeed(down_left, speed_ini);
       // /!\ since the motors are set in opposite directions, the speeds should 
       //     be opposite for each side
       setSpeed(up_right, -speed_ini);
       setSpeed(down_right, -speed_ini); 
     
       wait_ms(3000);
       STATUS=SEEKING;

     }

  

     // begin the "seeking for an opponent" phase
      
     while (STATUS==SEEKING) {
       setSpeed(up_left, speed_ini);
       setSpeed(up_right, speed_ini);
       setSpeed(down_left, speed_ini);
       setSpeed(down_right, speed_ini);
       checkObstacle(100, &obs);
       // the robot starts spinning around
   

       // opponent detection will result in an attitude change
       if (obs == 2)  // indeed this condition should be explicit
         STATUS = CHASING;
   
     }

    // the robot will focus the opponent and try to push him away,
    // as hard as possible 
     while (STATUS==CHASING) {
       setSpeed(up_left, speed_max);
       setSpeed(down_left, speed_max);
       setSpeed(up_right, -speed_max);
       setSpeed(down_right, -speed_max);
       checkObstacle(100, &obs);
       centerLuminosity(100, &field);

       // if, for whatever reason, the robot does not detect any obstacle anymore
       // it returns to its seeking opponent phase
       if (obs!=2)
         STATUS=SEEKING;
     
       if (field==2)  // condition should also be explicit, using light sensor
         STATUS = GO_FORWARD;

     }

     // if the robot sensor indicates that it is about to cross the fields limit
     // robot starts to move back to the center  
     while (STATUS == GO_FORWARD) {
       setSpeed(up_left, speed_ini);
       setSpeed(up_right, -speed_ini);
       setSpeed(down_left, speed_ini);
       setSpeed(down_right, -speed_ini);
     }
   
 
 
  
  }
  
  while (1) {} ;
}

