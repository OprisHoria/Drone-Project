/*All Quadcopter code under*/


/* Data types clarifier */
typedef unsigned char u8;
typedef unsigned short int u16;
typedef unsigned long int u32;

#define FALSE 0
#define TRUE !False

/* Enum type for state machine */
typedef enum{
  State_PowerSaving,
  State_TemperatureMeasurement,
  State_ManualDroneControl,
  State_AutomaticDroneTakeOff
}EnStates;

static EnStates En_StateMachineGlobal;

#define BAUD_RATE 115200

/* Function prototypes */
void GetBluetoothCommand(void);


/* Variable initializer */
void setup(void)
{
  Serial.begin(BAUD_RATE);          /* Start UART communication */
  En_StateMachineGlobal = State_PowerSaving;  
}

/* All operation is done here. */
void loop(void)
{

  /* State Machine here */
  switch(En_StateMachineGlobal)
  {
    case State_PowerSaving:
    { 
      GetBluetoothCommand();
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
}

#define BLT_ZERO 48
#define BLT_ONE  49
#define BLT_TWO  50

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


