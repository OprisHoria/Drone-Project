
/* For gyroscope communication */
#include "I2Cdev.h"	
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

/* DEFINES */
#define FALSE 		0
#define TRUE 		!False
#define BAUD_RATE	115200

#define BLT_ZERO 	48
#define BLT_ONE  	49
#define BLT_TWO	 	50
#define BLT_THREE 	51
#define BLT_FOUR  	52
#define BLT_FIVE	53
#define BLT_SIX 	54
#define BLT_SEVEN  	55
#define BLT_EIGHT	56
#define BLT_NINE 	57
#define BLT_TEN  	58
#define BLT_ELEVEN	59


#define INTERRUPT_PIN 2  
#define LED_PIN 13 


#define OUTPUT_READABLE_YAWPITCHROLL

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

unsigned int throttle = 1100;





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

float pid_p_gain_roll = 1.4;
float pid_i_gain_roll = 2;
float pid_d_gain_roll = 15.0;
int   pid_max_roll 	  = 400;

float pid_p_gain_pitch = pid_p_gain_roll;  //Gain setting for the pitch P-controller.
float pid_i_gain_pitch = pid_i_gain_roll;  //Gain setting for the pitch I-controller.
float pid_d_gain_pitch = pid_d_gain_roll;  //Gain setting for the pitch D-controller.
int pid_max_pitch = pid_max_roll;          //Maximum output of the PID-controller (+/-)


float pid_p_gain_yaw = 4.0;                //Gain setting for the pitch P-controller. //4.0
float pid_i_gain_yaw = 0.02;               //Gain setting for the pitch I-controller. //0.02
float pid_d_gain_yaw = 0.0;                //Gain setting for the pitch D-controller.
int pid_max_yaw = 400;                     //Maximum output of the PID-controller (+/-)


int x;
/* Variable initializer */
void setup(void)
{
	pinMode(7,OUTPUT);
 // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
	
	
	Serial.begin(BAUD_RATE);					/* Start UART communication */
	En_StateMachineGlobal = State_PowerSaving;	
	
	GyroInit();
		
	for(x=0;x<500;x++)
	{
		GyroRun();
		gyro_roll_cal  += ypr[2];                                //Ad roll value to gyro_roll_cal.
		gyro_pitch_cal += ypr[1];                              //Ad pitch value to gyro_pitch_cal.
		gyro_yaw_cal   += ypr[0];  
		delay(3);  		
	}
	
	
/*SETUP ESC*/

  FrontLeft.attach(3);
  FrontRight.attach(6);
  RearLeft.attach(10);
  RearRight.attach(11);

  fl = throttle;
  fr = throttle;
  rl = throttle;
  rr = throttle;
}

/* All operation is done here. */
void loop(void)
{
	GyroRun();
	/* State Machine here */
	gyro_roll_input  = (gyro_roll_input  * 0.8) + ((ypr[2] / 57.14286) * 0.2); 
	gyro_pitch_input = (gyro_pitch_input * 0.8) + ((ypr[1] / 57.14286) * 0.2);    
	gyro_yaw_input   = (gyro_yaw_input   * 0.8) + ((ypr[0] / 57.14286) * 0.2);   
	
	switch(En_StateMachineGlobal)
	{
		case State_PowerSaving:
		{	
			//GetBluetoothCommand();
				u8 Message; 
			
			Message = Serial.read();
			
			
			/*			Serial.print("ypr\t");
            Serial.print(ypr[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(ypr[1] * 180/M_PI);
            Serial.print("\t");
            Serial.println(ypr[2] * 180/M_PI);
			*/
			
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
 
			Serial.print("Front Lelft \t");
			Serial.println(fl);
			
			Serial.print("Rear right \t");
			Serial.println(rr);
			
			delay(10);
		}
		break;
		
		case State_TemperatureMeasurement:
		{	
		}
		break;
		
		case State_ManualDroneControl:
		{	
			ManualDroneControl();
		}
		break;
		
		case State_AutomaticDroneTakeOff:
		{	
		}
		break;
	}
	
	
	FrontLeft.writeMicroseconds(fl);
	FrontRight.writeMicroseconds(fr);
	RearLeft.writeMicroseconds(rl);
	RearRight.writeMicroseconds(rr);
  
}


void GetBluetoothCommand(void)
{
	u8 U8_BluetoothMessage; 
	U8_BluetoothMessage = Serial.read();
	
	if( U8_BluetoothMessage == BLT_ZERO )
	{
		En_StateMachineGlobal = State_TemperatureMeasurement;
	}
	else if( U8_BluetoothMessage == BLT_ONE )
	{
		En_StateMachineGlobal = State_ManualDroneControl;
		Serial.write("Manual State");
	}
}

void ManualDroneControl(void)
{
	u8 U8_BluetoothMessage; 
	U8_BluetoothMessage = Serial.read();
	
	if(U8_BluetoothMessage == BLT_ZERO)
	{
		Serial.write("Up 1");

	}

}


void AutoLeveling(void)
{
	
}






void GyroInit(void)
{
	    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);
	
	// wait for ready
    Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    while (Serial.available() && Serial.read()); // empty buffer
    while (!Serial.available());                 // wait for data
    while (Serial.available() && Serial.read()); // empty buffer again

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(128);  //220
    mpu.setYGyroOffset(49);   //76
    mpu.setZGyroOffset(33);    // -85
    mpu.setZAccelOffset(1255); // 1788//1688 factory default for my test chip

	
	// make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
}

void GyroRun(void)
{
	// if programming failed, don't try to do anything
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
        // other program behavior stuff here
        // .
        // .
        // .
        // if you are really paranoid you can frequently test in between other
        // stuff to see if mpuInterrupt is true, and if so, "break;" from the
        // while() loop to immediately process the MPU data
        // .
        // .
        // .
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;


		
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);


    }
}

void calculatePid(void)
{
	//Roll calculations
  pid_error_temp = gyro_roll_input - pid_roll_setpoint;
  pid_i_mem_roll += pid_i_gain_roll * pid_error_temp;
  if(pid_i_mem_roll > pid_max_roll)pid_i_mem_roll = pid_max_roll;
  else if(pid_i_mem_roll < pid_max_roll * -1)pid_i_mem_roll = pid_max_roll * -1;
  
  pid_output_roll = pid_p_gain_roll * pid_error_temp + pid_i_mem_roll + pid_d_gain_roll * (pid_error_temp - pid_last_roll_d_error);
  if(pid_output_roll > pid_max_roll)pid_output_roll = pid_max_roll;
  else if(pid_output_roll < pid_max_roll * -1)pid_output_roll = pid_max_roll * -1;
  
  pid_last_roll_d_error = pid_error_temp;
  
  //Pitch calculations
  pid_error_temp = gyro_pitch_input - pid_pitch_setpoint;
  pid_i_mem_pitch += pid_i_gain_pitch * pid_error_temp;
  if(pid_i_mem_pitch > pid_max_pitch)pid_i_mem_pitch = pid_max_pitch;
  else if(pid_i_mem_pitch < pid_max_pitch * -1)pid_i_mem_pitch = pid_max_pitch * -1;
  
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
