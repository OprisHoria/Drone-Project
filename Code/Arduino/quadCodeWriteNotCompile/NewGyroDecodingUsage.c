

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


/* ESC values */


#include <Servo.h>

unsigned short int fl,fr,rl,rr;

#define BaseValueFRONT   (short int)1101U
#define BaseValueREAR    (short int)1100U
unsigned char a,b;
Servo FrontLeft,FrontRight,RearLeft,RearRight;

unsigned int throttle = 1100;





/* PID CONTROLLER VARIABLES*/


#include <Wire.h>

//Declaring some global variables
int gyro_x, gyro_y, gyro_z;
long acc_x, acc_y, acc_z, acc_total_vector;
int temperature;
long gyro_x_cal, gyro_y_cal, gyro_z_cal;
long loop_timer;
int Serial_loop_counter;
float angle_pitch, angle_roll;
int angle_pitch_buffer, angle_roll_buffer;
boolean set_gyro_angles;
float angle_roll_acc, angle_pitch_acc;
float angle_pitch_output, angle_roll_output;




int x;
/* Variable initializer */
void setup(void)
{
	Wire.begin();
	Serial.begin(115200);
	
	GyroInit();
	
	delay(1500);                                                         //Delay 1.5 second to display the text
 
  for (int cal_int = 0; cal_int < 2000 ; cal_int ++){                  //Run this code 2000 times
    if(cal_int % 125 == 0)Serial.print(".");                              //Print a dot on the Serial every 125 readings
    GyroRun();                                            //Read the raw acc and gyro data from the MPU-6050
    gyro_x_cal += gyro_x;                                              //Add the gyro x-axis offset to the gyro_x_cal variable
    gyro_y_cal += gyro_y;                                              //Add the gyro y-axis offset to the gyro_y_cal variable
    gyro_z_cal += gyro_z;                                              //Add the gyro z-axis offset to the gyro_z_cal variable
    delay(3);                                                          //Delay 3us to simulate the 250Hz program loop
  }
  gyro_x_cal /= 2000;                                                  //Divide the gyro_x_cal variable by 2000 to get the avarage offset
  gyro_y_cal /= 2000;                                                  //Divide the gyro_y_cal variable by 2000 to get the avarage offset
  gyro_z_cal /= 2000;                                                  //Divide the gyro_z_cal variable by 2000 to get the avarage offset

  loop_timer = micros();    

  //Reset the loop timer
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

  switch(En_StateMachineGlobal)
  {
    case State_PowerSaving:
    { 
      //GetBluetoothCommand();
        u8 Message; 
      
      Message = Serial.read();
      /*
      
            Serial.print("ypr\t");
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
      
      
      fr = 0;//throttle - pid_output_pitch + pid_output_roll - pid_output_yaw;
      rr = 0;//throttle + pid_output_pitch + pid_output_roll + pid_output_yaw;
      rl = 0;//throttle + pid_output_pitch - pid_output_roll - pid_output_yaw;
      fl = 0;//throttle - pid_output_pitch - pid_output_roll + pid_output_yaw;
 
      Serial.print("Front left: ");
      Serial.print(angle_pitch_output);
      Serial.print("\t");
      
      Serial.print("Front right: ");
      Serial.print(angle_roll_output);
      Serial.print("\t");

      Serial.print("Rear left: ");
      Serial.print(rl);
      Serial.print("\t");
       
      Serial.print("Rear right: ");
      Serial.print(rr);

      Serial.println();
    
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
  
  
 // FrontLeft.writeMicroseconds(fl);
 // FrontRight.writeMicroseconds(fr);
  //RearLeft.writeMicroseconds(rl);
  //RearRight.writeMicroseconds(rr);
  
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
	//Activate the MPU-6050
	Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
	Wire.write(0x6B);                                                    //Send the requested starting register
	Wire.write(0x00);                                                    //Set the requested starting register
	Wire.endTransmission();                                              //End the transmission
	//Configure the accelerometer (+/-8g)
	Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
	Wire.write(0x1C);                                                    //Send the requested starting register
	Wire.write(0x10);                                                    //Set the requested starting register
	Wire.endTransmission();                                              //End the transmission
	//Configure the gyro (500dps full scale)
	Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
	Wire.write(0x1B);                                                    //Send the requested starting register
	Wire.write(0x08);                                                    //Set the requested starting register
	Wire.endTransmission();                                              //End the transmission
}

void GyroRun(void)
{
	Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
	Wire.write(0x3B);                                                    //Send the requested starting register
	Wire.endTransmission();                                              //End the transmission
	Wire.requestFrom(0x68,14);                                           //Request 14 bytes from the MPU-6050
	while(Wire.available() < 14);                                        //Wait until all the bytes are received
	acc_x = Wire.read()<<8|Wire.read();                                  //Add the low and high byte to the acc_x variable
	acc_y = Wire.read()<<8|Wire.read();                                  //Add the low and high byte to the acc_y variable
	acc_z = Wire.read()<<8|Wire.read();                                  //Add the low and high byte to the acc_z variable
	temperature = Wire.read()<<8|Wire.read();                            //Add the low and high byte to the temperature variable
	gyro_x = Wire.read()<<8|Wire.read();                                 //Add the low and high byte to the gyro_x variable
	gyro_y = Wire.read()<<8|Wire.read();                                 //Add the low and high byte to the gyro_y variable
	gyro_z = Wire.read()<<8|Wire.read();                                 //Add the low and high byte to the gyro_z variable
}

void calculatePid(void)
{
 
  gyro_x -= gyro_x_cal;                                                //Subtract the offset calibration value from the raw gyro_x value
  gyro_y -= gyro_y_cal;                                                //Subtract the offset calibration value from the raw gyro_y value
  gyro_z -= gyro_z_cal;                                                //Subtract the offset calibration value from the raw gyro_z value
  
  //Gyro angle calculations
  //0.0000611 = 1 / (250Hz / 65.5)
  angle_pitch += gyro_x * 0.0000611;                                   //Calculate the traveled pitch angle and add this to the angle_pitch variable
  angle_roll += gyro_y * 0.0000611;                                    //Calculate the traveled roll angle and add this to the angle_roll variable
  
  //0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The Arduino sin function is in radians
  angle_pitch += angle_roll * sin(gyro_z * 0.000001066);               //If the IMU has yawed transfer the roll angle to the pitch angel
  angle_roll -= angle_pitch * sin(gyro_z * 0.000001066);               //If the IMU has yawed transfer the pitch angle to the roll angel
  
  //Accelerometer angle calculations
  acc_total_vector = sqrt((acc_x*acc_x)+(acc_y*acc_y)+(acc_z*acc_z));  //Calculate the total accelerometer vector
  //57.296 = 1 / (3.142 / 180) The Arduino asin function is in radians
  angle_pitch_acc = asin((float)acc_y/acc_total_vector)* 57.296;       //Calculate the pitch angle
  angle_roll_acc = asin((float)acc_x/acc_total_vector)* -57.296;       //Calculate the roll angle
  
  //Place the MPU-6050 spirit level and note the values in the following two lines for calibration
  angle_pitch_acc -= 0.0;                                              //Accelerometer calibration value for pitch
  angle_roll_acc -= 0.0;                                               //Accelerometer calibration value for roll

  if(set_gyro_angles){                                                 //If the IMU is already started
    angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;     //Correct the drift of the gyro pitch angle with the accelerometer pitch angle
    angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;        //Correct the drift of the gyro roll angle with the accelerometer roll angle
  }
  else{                                                                //At first start
    angle_pitch = angle_pitch_acc;                                     //Set the gyro pitch angle equal to the accelerometer pitch angle 
    angle_roll = angle_roll_acc;                                       //Set the gyro roll angle equal to the accelerometer roll angle 
    set_gyro_angles = true;                                            //Set the IMU started flag
  }
  
  //To dampen the pitch and roll angles a complementary filter is used
  angle_pitch_output = angle_pitch_output * 0.9 + angle_pitch * 0.1;   //Take 90% of the output pitch value and add 10% of the raw pitch value
  angle_roll_output = angle_roll_output * 0.9 + angle_roll * 0.1;      //Take 90% of the output roll value and add 10% of the raw roll value
  
  
  while(micros() - loop_timer < 4000);                                 //Wait until the loop_timer reaches 4000us (250Hz) before starting the next loop
  loop_timer = micros();                                               //Reset the loop timer
}
