#include <EEPROM.h>

#include <AccelStepper.h>

//String CONTANTS
#define SEPERATOR ":"


//Motors
#define DEVICE_MOTORS_INSTALLED 2
AccelStepper motors[DEVICE_MOTORS_INSTALLED];

//Control LED
#define LED 13

//Memory Addresses
#define MEMORY_UART_SPEED 0
#define MEMORY_DEVICE_ADDRESS 4


//Global vars
byte deviceAddress=0;
byte actDevice=0;
byte actMotor=0;


//Constants for Memory allocation
#define INPUT_SIZE 30

void setup() {
  //Init motors
  motors[0] = AccelStepper(AccelStepper::FULL4WIRE, 2, 3, 4, 5, true);
  motors[1] = AccelStepper(AccelStepper::FULL4WIRE, 6, 7, 8, 9, true);

  
  
  //Get UART speed out of memory and set it
  // defaults to 9600
  long baudrate=0;
  EEPROM.get( MEMORY_UART_SPEED, baudrate );
    switch (baudrate) {
    case    300:
    case    600:
    case   1200:
    case   2400:
    case   4800:
    case   9600:
    case  14400:
    case  19200:
    case  28800:
    case  38400:
    case  57600:
    case 115200:
      Serial.begin(baudrate);
      break;
    default: 
      Serial.begin(9600);
      Serial.print("Using default Baudrate!");
  }
  

  EEPROM.get( MEMORY_DEVICE_ADDRESS, deviceAddress );
  
  Serial.print( deviceAddress );
  Serial.print( ":*:OK\n" );

}

//Protokoll:
//Command to Arduino
//x:y:command:par1:par2\n
//x: Devicenummer *==All devices
//y: Motornummer *==all motors, is ignored if command applies to no motor
//command: Command
//par_: parameter
//\n: Linefeed
//Response:
//x:y:command:OK


void error(char * c)
{
  Serial.print(deviceAddress);
  Serial.print(":");
  Serial.print((int)actMotor);
  Serial.print(":");
  Serial.println(c);
}

void loop() {
  // Get next command from Serial (add 1 for final 0)
  char input[INPUT_SIZE + 1];
  
  
  //Get stuff from uart
  while (Serial.available()==0){}
  byte size = Serial.readBytes(input, INPUT_SIZE);
    
  //The position in the command:
  //0:Devicenumber
  //1:Motornumber
  //3:command
  //4,...:parameters
  byte actPosition=0;
  
  // Add the final 0 to end the C string
  input[size] = 0;

  // Read each command  
  char* part = strtok(input, SEPERATOR);

  //Get the device number, if not the correct one or 0, we skip the rest
  if (*part != 0)
  {
    actDevice = atoi(part);
    
    if ((actDevice==deviceAddress)||(actDevice==0))
    {
      //Get Motor number
      part = strtok(NULL, SEPERATOR);
      if ((part != 0))
      {
        actMotor = atoi(part);
        if ((actMotor>=0)&&(actMotor<=DEVICE_MOTORS_INSTALLED))
        {
          //Get the command
          part = strtok(NULL, SEPERATOR);
        
          //Finally, we can evaluate the commands with a if then else
          
          //Address A
          if (strcmp(part,"A")==0)    //ADDRESS
          {
            //Get the address
            part = strtok(NULL, SEPERATOR);
            deviceAddress = atoi(part);
            EEPROM.update( MEMORY_DEVICE_ADDRESS, deviceAddress );
            Serial.print(deviceAddress);
            Serial.println(":*:A:OK");
          }else if (strcmp(part,"B")==0){        //BAUD
            //Get the address
            part = strtok(NULL, SEPERATOR);
            long baudRate = atol(part);
            EEPROM.update( MEMORY_UART_SPEED, baudRate);
            //Serial.begin(baudRate);
            Serial.print(deviceAddress);
            Serial.print(":*:B:");
            Serial.print(baudRate);
            Serial.println(":OK");        
          }else if (strcmp(part,"S")==0){      //SPEED
            //Get the speed
            part = strtok(NULL, SEPERATOR);
            float speed = atof(part);
            if (actDevice>0)
            {
              motors[actDevice-1].setSpeed(speed);
            }else{
              for (int i=0;i<DEVICE_MOTORS_INSTALLED;i++) motors[actDevice].setSpeed(speed);
            }
            Serial.print(deviceAddress);
            Serial.print(":");
            Serial.print(actMotor);
            Serial.print(":S:");
            Serial.print(speed);
            Serial.println(":OK");                    
          }else
          {
            error("UnknownCommand");
          } 
        }else{
          error("UnknownMotor");
        }
      }else{
        error("UnknownFormat2");
      }
    }else{
      error("WrongDevice");
    }
  }else{
    error("UnknownFormat1");
  }


   
}
