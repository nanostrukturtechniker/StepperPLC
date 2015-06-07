#include <EEPROM.h>

#include <AccelStepper.h>

//String CONTANTS
#define SEPERATOR ":"


//Motors
#define DEVICE_MOTORS_INSTALLED 2
AccelStepper motor[DEVICE_MOTORS_INSTALLED];
enum MotorStates{stop=0, constantSpeed=1, toPosition=2};
MotorStates motorState[DEVICE_MOTORS_INSTALLED];


//Control LED
#define LED 13

//Memory Addresses
#define MEMORY_UART_SPEED 0
#define MEMORY_DEVICE_ADDRESS 4
#define MEMORY_MOTORSETTINGS_BASE 200
#define MEMORY_MOTORSETTINGS_OFFSET_PER_MOTOR 20
#define MEMORY_MOTORSETTINGS_ACC 0
#define MEMORY_MOTORSETTINGS_SPEED 4


//Global vars
byte deviceAddress=0;
byte actDevice=0;
byte actMotor=0;


//Constants for Memory allocation
#define INPUT_SIZE 30

void setup() {
  //Init motors
  motor[0] = AccelStepper(AccelStepper::DRIVER, 2, 3,  true);
  motor[1] = AccelStepper(AccelStepper::DRIVER, 4, 5, true);


  //Set maximum Speed for motors
  motor[0].setMaxSpeed(100000);
  motor[1].setMaxSpeed(100000);
  
  
  
  //Get UART speed out of memory and set it
  // defaults to 9600
  long baudrate=0;
  EEPROM.get( MEMORY_UART_SPEED, baudrate );
  Serial.println((long)baudrate);
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
      Serial.println("Using Baudrate 9600!");
  }
  

  EEPROM.get( MEMORY_DEVICE_ADDRESS, deviceAddress );
  
  Serial.print( deviceAddress );
  Serial.print( ":0:OK\n" );
  
  
  cli();//stop interrupts
  //set timer2 
  TCCR2A = 0;// set entire TCCR2A register to 0
  TCCR2B = 0;// same for TCCR2B
  TCNT2  = 0;//initialize counter value to 0
  OCR2A = 125; // -> 16kHz
  // turn on CTC mode
  TCCR2A |= (1 << WGM21);
  // Set CS21 bit for 8 prescaler
  TCCR2B |= (1 << CS21);   
  // enable timer compare interrupt
  TIMSK2 |= (1 << OCIE2A);
  sei();//allow interrupts
}

//Protokoll:
//Command to Arduino
//x:y:command:par1:par2\n
//x: Devicenummer 0==All devices
//y: Motornummer 0==all motors, is ignored if command applies to no motor
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

ISR(TIMER2_COMPA_vect)
{
  cli(); //Stop interrupts
  for (int i=0;i<DEVICE_MOTORS_INSTALLED;i++) 
  {
    if (motorState[i]==toPosition)
    {
      if (motor[i].distanceToGo() == 0) motorState[i] = stop; else motor[i].run();
    } else if (motorState[i]==constantSpeed) 
    {
      motor[i].runSpeed();
    }
  }
  sei();//allow interrupts
}




void loop() {
  // Get next command from Serial (add 1 for final 0)
  char input[INPUT_SIZE + 1];
  
  
  //Get stuff from uart
  while (Serial.available()==0){
    //doSteppers();
  }
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
            Serial.println(":0:A:OK");
          }else if (strcmp(part,"B")==0){        //BAUD
            //Get the address
            part = strtok(NULL, SEPERATOR);
            long baudRate = atol(part);
            EEPROM.put( MEMORY_UART_SPEED, baudRate);
            //Serial.begin(baudRate);
            Serial.print(deviceAddress);
            Serial.print(":0:B:");
            Serial.print(baudRate);
            Serial.println(":RESET");        
          }else if (strcmp(part,"S")==0){      //SPEED
            //Get the speed
            part = strtok(NULL, SEPERATOR);
            float speed = atof(part);
            if (actMotor>0)
            {
              motor[actMotor-1].setSpeed(speed);
            }else{
              for (int i=0;i<DEVICE_MOTORS_INSTALLED;i++) motor[i].setSpeed(speed);
            }
            Serial.print(deviceAddress);
            Serial.print(":");
            Serial.print(actMotor);
            Serial.print(":S:");
            Serial.print(speed);
            Serial.println(":OK");                    
          }else if (strcmp(part,"X")==0){      //Acceleration
            //Get the Acceleration
            part = strtok(NULL, SEPERATOR);
            float acc = atof(part);
            if (actMotor>0)
            {
              motor[actMotor-1].setAcceleration(acc);
            }else{
              for (int i=0;i<DEVICE_MOTORS_INSTALLED;i++) motor[i].setAcceleration(acc);
            }
            Serial.print(deviceAddress);
            Serial.print(":");
            Serial.print(actMotor);
            Serial.print(":X:");
            Serial.print(acc);
            Serial.println(":OK");                    
          }else if (strcmp(part,"R")==0){      //RUN
            if (actMotor>0)
            {
              motorState[actMotor-1]=constantSpeed;
            }else{
              for (int i=0;i<DEVICE_MOTORS_INSTALLED;i++) motorState[i]=constantSpeed;
            }
            Serial.print(deviceAddress);
            Serial.print(":");
            Serial.print(actMotor);
            Serial.println(":R:OK");   
          }else if (strcmp(part,"H")==0){      //Halt
            if (actMotor>0)
            {
              motor[actMotor-1].stop();
              motorState[actMotor-1]=stop;
            }else{
              for (int i=0;i<DEVICE_MOTORS_INSTALLED;i++)
             {
               motor[i].stop();
               motorState[i]=stop;
             }
            }
            Serial.print(deviceAddress);
            Serial.print(":");
            Serial.print(actMotor);
            Serial.println(":H:OK");   
          }else if (strcmp(part,"P")==0){      //Position
            //Get the Position to set the motor to
            part = strtok(NULL, SEPERATOR);
            long pos = atol(part);
            if (actMotor>0)
            {
              motor[actMotor-1].setCurrentPosition(pos);
            }else{
              for (int i=0;i<DEVICE_MOTORS_INSTALLED;i++)
              {
                motor[i].setCurrentPosition(pos);
              }
            }
            Serial.print(deviceAddress);
            Serial.print(":");
            Serial.print(actMotor);
            Serial.print(":P:");
            Serial.print(pos);
            Serial.println(":OK");   
          }else if (strcmp(part,"M")==0){      //Move to positon
            //Get the Position to set the motor to
            part = strtok(NULL, SEPERATOR);
            long pos = atol(part);
            if (actMotor>0)
            {
              motor[actMotor-1].moveTo(pos);
              motorState[actMotor-1]=toPosition;
            }else{
              for (int i=0;i<DEVICE_MOTORS_INSTALLED;i++)
              {
                motor[i].moveTo(pos);
                motorState[i]=toPosition;
              }
            }

            Serial.print(deviceAddress);
            Serial.print(":");
            Serial.print(actMotor);
            Serial.print(":M:");
            Serial.print(pos);
            Serial.println(":OK");   
          }else if (strcmp(part,"G")==0){      //Move this far
            //Get the Position to set the motor to
            part = strtok(NULL, SEPERATOR);
            long pos = atol(part);
            if (actMotor>0)
            {
              motor[actMotor-1].move(pos);
              motorState[actMotor-1]=toPosition;
            }else{
              for (int i=0;i<DEVICE_MOTORS_INSTALLED;i++)
              {
                motor[i].move(pos);
                motorState[i]=toPosition;
              }
            }

            Serial.print(deviceAddress);
            Serial.print(":");
            Serial.print(actMotor);
            Serial.print(":G:");
            Serial.print(pos);
            Serial.println(":OK");   
          }else if (strcmp(part,"N")==0){      //Save
            //Save everything

            for (int i=0;i<DEVICE_MOTORS_INSTALLED;i++)
            {
//              EEPROM.put(MEMORY_MOTORSETTINGS_BASE+i*MEMORY_MOTORSETTINGS_OFFSET_PER_MOTOR+ MEMORY_MOTORSETTINGS_ACC,  motor[i].??);
              EEPROM.put(MEMORY_MOTORSETTINGS_BASE+i*MEMORY_MOTORSETTINGS_OFFSET_PER_MOTOR+ MEMORY_MOTORSETTINGS_SPEED,  motor[i].speed());
            }
            Serial.print(deviceAddress);
            Serial.print(":0:N:OK"); 
          }else if (strcmp(part,"L")==0){      //LOAD
            //Save everything

            for (int i=0;i<DEVICE_MOTORS_INSTALLED;i++)
            {
              float v=0;
  //            EEPROM.put(MEMORY_MOTORSETTINGS_BASE+i*MEMORY_MOTORSETTINGS_OFFSET_PER_MOTOR+ MEMORY_MOTORSETTINGS_ACC,  motor[i].??);
              EEPROM.get(MEMORY_MOTORSETTINGS_BASE+i*MEMORY_MOTORSETTINGS_OFFSET_PER_MOTOR+ MEMORY_MOTORSETTINGS_SPEED, v);
              motor[i].setSpeed(v);
            }
            Serial.print(deviceAddress);
            Serial.print(":0:L:OK");   
          }else

          {
            error("UnknownCommand"); 
          } 
        }else{
          error("UnknownMotor"); //Motor does not exist
        }
      }else{
        error("UnknownFormat2"); //Parameter missing/wrong
      }
    }else{
      //error("WrongDevice");    //Not for several devices on one bus!
    }
  }else{
    error("UnknownFormat1");  //Whole line seems wierd
  }


   
}
