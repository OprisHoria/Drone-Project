
/* For gyroscope communication */
#include "I2Cdev.h"  
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

/* DEFINES */
#define FALSE     0
#define TRUE    !False
#define BAUD_RATE 115200

#define BLT_ZERO  48
#define BLT_ONE   49
#define BLT_TWO   50
#define BLT_THREE   51
#define BLT_FOUR    52
#define BLT_FIVE  53
#define BLT_SIX   54
#define BLT_SEVEN   55
#define BLT_EIGHT 56
#define BLT_NINE  57
#define BLT_TEN   58
#define BLT_ELEVEN  59


#define INTERRUPT_PIN 2  
#define LED_PIN 13 


#define OUTPUT_READABLE_YAWPITCHROLL

#define ROLL	2
#define PITCH	1
#define YAW		0

/* Data types clarifier */
typedef unsigned char u8;
typedef unsigned short int u16;
typedef unsigned long int u32;



/* Enum type for state machine */
typedef enum{
  State_PowerSaving,
  State_TemperatureMeasurement,
  State_ManualDroneControl,
  State_AutomaticDroneTakeOff
}EnStates;





/* Variable definitions */
static EnStates En_StateMachineGlobal;
// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

MPU6050 mpu;
bool blinkState = false;
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

unsigned char GyroReady;
unsigned char maxRear, maxFront;





/* Function prototypes */
void GetBluetoothCommand(void);

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}



/* ESC values */


#include <Servo.h>

unsigned short int fl,fr,rl,rr;

#define BaseValueFRONT   (short int)1101U
#define BaseValueREAR    (short int)1100U
unsigned char a,b;
Servo FrontLeft,FrontRight,RearLeft,RearRight;

unsigned int throttle;





/* PID CONTROLLER VARIABLES*/

byte last_channel_1, last_channel_2, last_channel_3, last_channel_4;
int receiver_input_channel_1, receiver_input_channel_2, receiver_input_channel_3, receiver_input_channel_4;
int counter_channel_1, counter_channel_2, counter_channel_3, counter_channel_4, loop_counter;
int esc_1, esc_2, esc_3, esc_4;

unsigned long timer_channel_1, timer_channel_2, timer_channel_3, timer_channel_4, esc_timer, esc_loop_timer;
unsigned long timer_1, timer_2, timer_3, timer_4, current_time;
int cal_int, start;
unsigned long loop_timer;
double gyro_pitch, gyro_roll, gyro_yaw;
double gyro_roll_cal, gyro_pitch_cal, gyro_yaw_cal;
byte highByte, lowByte;

float pid_error_temp;
float pid_i_mem_roll, pid_roll_setpoint, gyro_roll_input, pid_output_roll, pid_last_roll_d_error;
float pid_i_mem_pitch, pid_pitch_setpoint, gyro_pitch_input, pid_output_pitch, pid_last_pitch_d_error;
float pid_i_mem_yaw, pid_yaw_setpoint, gyro_yaw_input, pid_output_yaw, pid_last_yaw_d_error;

float pid_p_gain_roll = 100;
float pid_i_gain_roll = 40;
float pid_d_gain_roll = 30;
int   pid_max_roll    = 400;

float pid_p_gain_pitch = pid_p_gain_roll;  //Gain setting for the pitch P-controller.
float pid_i_gain_pitch = pid_i_gain_roll;  //Gain setting for the pitch I-controller.
float pid_d_gain_pitch = pid_d_gain_roll;  //Gain setting for the pitch D-controller.
int pid_max_pitch = pid_max_roll;          //Maximum output of the PID-controller (+/-)


float pid_p_gain_yaw = 3;                //Gain setting for the pitch P-controller. //4.0
float pid_i_gain_yaw = 0.02;               //Gain setting for the pitch I-controller. //0.02
float pid_d_gain_yaw = 0;                //Gain setting for the pitch D-controller.
int pid_max_yaw = 400;                     //Maximum output of the PID-controller (+/-)


int x;
/* Variable initializer */
void setup(void)
{
	Wire.begin();								/* Starting gyroscope reading */
	#define BAUD_RATE 115200
	Serial.begin(BAUD_RATE);					/* Start UART communication */
	En_StateMachineGlobal = State_PowerSaving; 	/* Initializing first state of state machine*/ 
	GyroInit();									/* Gyroscope initializing */

	#define MS_DELAY 	3						/* Time between gyroscope runs */
	#define NO_OF_RUNS	500						/* Number of times the gyro run function gets called */

	for( x=0; x<NO_OF_RUNS; x++ )				/* Calibration for gyroscope yaw pitch and roll*/
	{
	GyroRun();									
	gyro_roll_cal  += ypr[ROLL];                               
	gyro_pitch_cal += ypr[PITCH];                             
	gyro_yaw_cal   += ypr[YAW];  
	delay(MS_DELAY);     
	}

	#define ENGINE_OFF 1100				/* 1100 is Engine off, maximum speed off engine is 1800 value */
	#define FL_MOTOR_PIN 3				/* PWM pin for front left MOTOR */
	#define FR_MOTOR_PIN 6				/* PWM pin for front right MOTOR */
	#define RL_MOTOR_PIN 10				/* PWM pin for rear left MOTOR */
	#define RR_MOTOR_PIN 11				/* PWM pin for rear right MOTOR */
	
	/*SETUP ESC*/
	throttle = MOTOR_OFF;				/* MOTOR speed */
	
	FrontLeft.attach(FL_MOTOR_PIN);		/* Notifying the board that the pins in FL_MOTOR_PIN is used for motor speed control */
	FrontRight.attach(FR_MOTOR_PIN);	/* Notifying the board that the pins in FR_MOTOR_PIN is used for motor speed control */
	RearLeft.attach(RL_MOTOR_PIN);		/* Notifying the board that the pins in RL_MOTOR_PIN is used for motor speed control */
	RearRight.attach(RR_MOTOR_PIN);		/* Notifying the board that the pins in RR_MOTOR_PIN is used for motor speed control */

	FrontLeftSpeed  = throttle;
	FrontRightSpeed = throttle;
	RearLeftSpeed   = throttle;
	RearRightSpeed  = throttle;
}

/* All operation is done here. */
void loop(void)
{
  GyroRun();
  StateMachine();
}

void StateMachine(void)
{
  switch(En_StateMachineGlobal)			/* State machine state is decided by bluetooth transmission */
  {
    case State_PowerSaving:				/* Go to power saving state */
    { 
		PowerSaving();
    }break;    
    case State_TemperatureMeasurement:	/* Go to measure temperature state */
    { 
		MeasureTemperature();
    }break;
    case State_ManualDroneControl:		/* Go to manual quadcopter control */
    { 
		ManualDroneControl();
    }break;
    case State_AutomaticDroneTakeOff:	/* Go to automatic quadcopter take-off */
    { 
		AutoTakeOff();
    }break;
	case State_AutomaticDroneLand:		/* Go to automatic quadcopter land */
	{
		AutoLand();
	}break;
  }
  UpdateMotorSpeed();					/* Write motor speed value */
}

void UpdateMotorSpeed(void);
{
  FrontLeft.writeMicroseconds(fl);
  FrontRight.writeMicroseconds(fr);
  RearLeft.writeMicroseconds(rl);
  RearRight.writeMicroseconds(rr);	
}
void GetBluetoothCommand(void)
{
  auto u8 U8_BluetoothMessage; 			/* Automatic variable declaration that will store received blt value */	
  U8_BluetoothMessage = Serial.read();	/* Get bluetooth transmission value */
  
  if( U8_BluetoothMessage == BLT_ZERO )	
  {
    En_StateMachineGlobal = State_TemperatureMeasurement;
	Serial.write("Temperature reading state");
  }
  else if( U8_BluetoothMessage == BLT_ONE )
  {
    En_StateMachineGlobal = State_ManualDroneControl;
    Serial.write("Manual control state");
  }
  else if( U8_BluetoothMessage == BLT_TWO )
  {
	En_StateMachineGlobal = State_AutomaticDroneTakeOff;
    Serial.write("Automatic take off state");
  }
  else if( U8_BluetoothMessage == BLT_THREE )
  {
	En_StateMachineGlobal = State_AutomaticDroneLand;
    Serial.write("Automatic landing state");  
  }
}

void ManualDroneControl(void)
{  
    u8 Message;
	
	gyro_roll_input  = (gyro_roll_input  * 0.8) + ((ypr[2] / 57.14286) * 0.2); 
	gyro_pitch_input = (gyro_pitch_input * 0.8) + ((ypr[1] / 57.14286) * 0.2);    
	gyro_yaw_input   = (gyro_yaw_input   * 0.8) + ((ypr[0] / 57.14286) * 0.2);   
     
    Message = Serial.read();


	if(Message == BLT_ONE)
	{
		throttle++;
	}
	else if(Message == BLT_ZERO)
	{
		throttle--;
	}

	calculatePid();


	fr = throttle - pid_output_pitch + pid_output_roll - pid_output_yaw;
	rr = throttle + pid_output_pitch + pid_output_roll + pid_output_yaw;
	rl = throttle + pid_output_pitch - pid_output_roll - pid_output_yaw;
	fl = throttle - pid_output_pitch - pid_output_roll + pid_output_yaw;

	Serial.print("Front left: ");
	Serial.print(fl);
	Serial.print("\t");

	Serial.print("Front right: ");
	Serial.print(fr);
	Serial.print("\t");

	Serial.print("Rear left: ");
	Serial.print(rl);
	Serial.print("\t");

	Serial.print("Rear right: ");
	Serial.print(rr);

	Serial.println();

	delay(10);

}


void AutoLeveling(void)
{
  
}





	#define GYRO_X_OFFSET 	134
	#define GYRO_Y_OFFSET 	45
	#define GYRO_Z_OFFSET 	35
	#define ACC_Z_OFFSET 	1258
	
	
void GyroInit(void)
{
    Serial.println(F("Initializing Gyro..."));	
    mpu.initialize();				/* Initializing device */
    pinMode(INTERRUPT_PIN, INPUT);	/* Notify the board that the pin defined in INTERRUPT_PIN is input */
  
    Serial.println(F("\nPress any button to start Quadcopter"));	/* Wait for user to start quadcopter */	
    while (Serial.available() && Serial.read()); 					/* Empty buffer */
    while (!Serial.available());                					/* Wait for data */
    while (Serial.available() && Serial.read()); 					/* Empty buffer again */

    Serial.println(F("Initializing Gyroscope"));
    devStatus = mpu.dmpInitialize();								/* Initializing gyroscope */

    mpu.setXGyroOffset(GYRO_X_OFFSET);	/* Gyroscope offset that calibrator decided for X axis */ 
    mpu.setYGyroOffset(GYRO_Y_OFFSET);	/* Gyroscope offset that calibrator decided for Y axis */ 
    mpu.setZGyroOffset(GYRO_Z_OFFSET);	/* Gyroscope offset that calibrator decided for Z axis */ 
    mpu.setZAccelOffset(ACC_Z_OFFSET); 	/* Accelerometer offset that calibrator decided for Z axis */ 

    if (devStatus == 0) 
	{
        Serial.println(F("Enabling Gyroscope..."));		
        mpu.setDMPEnabled(true);		/* Enable gyroscope */

        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        Serial.println(F("Gyroscope ready! Waiting for first interrupt..."));
        dmpReady = true;
		packetSize = mpu.dmpGetFIFOPacketSize();	/* get expected Gyro packet size for later comparison*/
    } 
	else 
	{
        Serial.print(F("Gyro Initialization failed (code "));	
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
}

void GyroRun(void)
{
    if (!dmpReady)									  /* If programming failed leave the function */
	{
		return;
	}

    mpuInterrupt = false;							  /* Reset interrupt flag */
    mpuIntStatus = mpu.getIntStatus();				  /* Get interrupt status byte */
    fifoCount = mpu.getFIFOCount();					  /* Get the current FIFO/stack count */

	#define INT_OVERFLOW 	0x10
	#define MAX_FIFO_SIZE 	1024
	#define INT_DATA		0x02
	
    if ( 											  /* Check if there is FIFO overflow */
		(mpuIntStatus & INT_OVERFLOW) 
		|| 
		(fifoCount == MAX_FIFO_SIZE)
	   ) 
	{
        mpu.resetFIFO();							  /* Reset FIFO */
        Serial.println("FIFO overflow!");			  /* Send error */
    } 
	else if ( (mpuIntStatus & INT_DATA) == TRUE )	  /* Check for interrupt */
	{
        while (fifoCount < packetSize) 				  /* While the data count is correct store it*/
		{	
			fifoCount = mpu.getFIFOCount();
		}

        mpu.getFIFOBytes(fifoBuffer, packetSize);	  /* Read a packet from FIFO */
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);	  /* Save data in ypr global variable */
    }
}

void CalculatePid(void)
{
	/* Roll calculations */
	pid_error_temp  = (gyro_roll_input - pid_roll_setpoint);
	pid_i_mem_roll += (pid_i_gain_roll * pid_error_temp);

	if(pid_i_mem_roll > pid_max_roll)
	{
		pid_i_mem_roll = pid_max_roll;
	}
	else if(pid_i_mem_roll < pid_max_roll * -1)
	{  
		pid_i_mem_roll = pid_max_roll * -1;
	}
  
	pid_output_roll = ((pid_p_gain_roll * pid_error_temp)   \
						+ pid_i_mem_roll + pid_d_gain_roll) \
						* (pid_error_temp - pid_last_roll_d_error);
	
	if(pid_output_roll > pid_max_roll)
	{
		pid_output_roll = pid_max_roll;
	}
	else if(pid_output_roll < pid_max_roll * -1)
	{
		pid_output_roll = pid_max_roll * -1;
	}
	
	pid_last_roll_d_error = pid_error_temp;
  
	/* Pitch calculations */
	pid_error_temp   = gyro_pitch_input - pid_pitch_setpoint;
	pid_i_mem_pitch += pid_i_gain_pitch * pid_error_temp;
	
	if(pid_i_mem_pitch > pid_max_pitch)
	{
		pid_i_mem_pitch = pid_max_pitch;
	}
	else if(pid_i_mem_pitch < pid_max_pitch * -1)
	{
		pid_i_mem_pitch = pid_max_pitch * -1;
	}
	pid_output_pitch = pid_p_gain_pitch * pid_error_temp + pid_i_mem_pitch + pid_d_gain_pitch * (pid_error_temp - pid_last_pitch_d_error);
	if(pid_output_pitch > pid_max_pitch)pid_output_pitch = pid_max_pitch;
	else if(pid_output_pitch < pid_max_pitch * -1)pid_output_pitch = pid_max_pitch * -1;

	pid_last_pitch_d_error = pid_error_temp;

	//Yaw calculations
	pid_error_temp = gyro_yaw_input - pid_yaw_setpoint;
	pid_i_mem_yaw += pid_i_gain_yaw * pid_error_temp;
	if(pid_i_mem_yaw > pid_max_yaw)pid_i_mem_yaw = pid_max_yaw;
	else if(pid_i_mem_yaw < pid_max_yaw * -1)pid_i_mem_yaw = pid_max_yaw * -1;

	pid_output_yaw = pid_p_gain_yaw * pid_error_temp + pid_i_mem_yaw + pid_d_gain_yaw * (pid_error_temp - pid_last_yaw_d_error);
	if(pid_output_yaw > pid_max_yaw)pid_output_yaw = pid_max_yaw;
	else if(pid_output_yaw < pid_max_yaw * -1)pid_output_yaw = pid_max_yaw * -1;

	pid_last_yaw_d_error = pid_error_temp;
}
