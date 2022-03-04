#include <Arduino.h>
#include <SoftwareWire.h>
#include <EEPROM.h>
#include <avr/wdt.h>
#include <PinChangeInterrupt.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include "LowPower.h"
#include "ADS1114.h"
#include "WSSFM10RAT.h"
/**************************************************Miscellaneous I/O Defines*******************************************************/
#define DBG_LED                         7           //Debug LED Pin           
#define VSENSOR_ARR_EN                  5           //Enable analogue voltage supply Pin
#define ALERT_RDY                       8           //ADS11114 ALERT/RDY Pin
#define WSSFM10RAT_RESET_Pin            9           //WSSFMR10RAT Reset Pin
#define WSSFM10RAT_WAKEUP_Pin           13          //WSSFM10RAT  Wakeup Pin
#define MAX_CT_SENSORS                  16          //Maximum number of CT sensors per unit
#define SIGFOX_MSG_SIZE                 12          //Number of bytes in sigfox message
#define SENSOR_ARRAY_MESSAGE_SIZE       24          //Two 12-bytes messages are required to transmit sensor array readings
#define ACQUISITIONS_CYCLE_PER_HOUR     5          //Number of acquisition cycles completed in an hour
#define SAMPLES_PER_ACQUISITION_CYCLE   210         //Number of time each CT sensor will be sampled in each acquisition cycle           
/**************************************************Delay Defines*******************************************************************/
#define PWR_STABILIZATION_DELAY         100         //Delay used when pwr rails are switched on to allow stabilization
/**************************************************ADS1114 Defines*****************************************************************/
#define I2C_SCL_Pin                     2          //Software SCL Pin
#define I2C_SDA_Pin                     3          //Software SDA Pin
#define I2C_CLK_SPEED                   400000U    //I2C Clock Speed
#define ADS1114_I2C_ADD                 0x48       //ADS1114 I2C address
#define ADS1114_LOW_THRESH_DRDY_VAL     0x0000     //ADS1114 Low Threshold value required for ALERT Pin to generate interrupt after every conversion
#define ADS1114_HIGH_THRESH_DRDY_VAL    0x8000     //ADS1114 High Threshold value required for ALERT Pin to generate interrupt after every conversion
/**************************************************E-Monitor Config Defines********************************************************/
#define E_MONITOR_CONFIG_START_ADDR     0x00
/*********************************************CD74HC4067SM 16-to-1 Mux Defines*****************************************************/
#define CD74HC4067SM_MUX0               A3
#define CD74HC4067SM_MUX1               A2
#define CD74HC4067SM_MUX2               A1
#define CD74HC4067SM_MUX3               A0

#define CD74HC4067SM_ENABLE_PIN         2
#define CD74HC4067SM_ENABLE_PORT        PORTE

#define CD74HC4067SM_MUXSEL_PORT        PORTF

#define _HWB_H() (CD74HC4067SM_ENABLE_PORT |= (1<<CD74HC4067SM_ENABLE_PIN))
#define _HWB_L() (CD74HC4067SM_ENABLE_PORT &= ~(1<<CD74HC4067SM_ENABLE_PIN))

void CD74HC4067SM_Init();
void CD74HC4067SM_Enable();
void CD74HC4067SM_Disable();
void CD74HC4067SM_SelectChannel(uint8_t ChannelNo);
void CD74HC4067SM_Reset(uint8_t ChannelNo);
/******************************************************Enums***********************************************************************/
/*Enum used to describe three types of SCT current sensors used by the device*/
typedef enum
{
  SCT_10A  = 0,
  SCT_25A  = 1,
  SCT_50A  = 2,
  SCT_100A = 3
} CT_SENSOR_TYPE;

const char *CT_SensorTypes[4] = {"SCT_10A","SCT_25A","SCT_50A","SCT_100A"};
/***************************************************Structs & Typedefs*************************************************************/
/*Struct used to describe SCT current sensors*/
struct CT_SENSOR
{
  unsigned IrmsSteps : 10;
  unsigned SensorType : 2;
  bool     Enabled;
  float    CalibrationConstant;
  float    Irms;
  float    Isigma;
};

/*Struct used to configure each CT Sensor in Array*/
struct CT_Sensor_Config
{
  unsigned SensorType:2;
  float    CalibrationConstant;
  bool     Enabled;  
};

/*24-bit variable that will be used to bit-pack Irms readings from pairs of CT sensors*/
typedef union {
  uint32_t bits : 24;
} __attribute__((packed)) uint24_t;
/*************************************************EnergyMonitor Defines************************************************************/
#define V_BIAS                         1652U                                                //DC bias applied to CT sensors
/*************************************************Linearization Constants**********************************************************/
#define  GRADIENT                     1.036979f
#define  Y_INTERCEPT                  0.3153813f
/************************************************Global Variables******************************************************************/
ADS1114 myADC(ADS1114_I2C_ADD, ALERT_RDY);                                                  //Create ADS1114 Instance
WSSFM10RAT myWisol(&Serial1, WSSFM10RAT_RESET_Pin, WSSFM10RAT_WAKEUP_Pin);                  //Create instance of WSSFM10RAT module

CT_Sensor_Config  ctSensor_Config[MAX_CT_SENSORS] = {0};                                    //Array of 16 CT sensor config objects
CT_SENSOR ctSensors[MAX_CT_SENSORS] = {0};                                                  //Array of 16 CT sensor objects
uint24_t  ctSensorPairs[MAX_CT_SENSORS / 2] = {0};                                          //Array of eight 24-bit variables
uint8_t  transmitArr[SENSOR_ARRAY_MESSAGE_SIZE] = {0};                                      //Array used to hold data to be transmitted to backend
uint16_t AcquisitionCount = 0;                                                              //Used to track number of acquisition cycles completed
/*************************************************Private function prototypes******************************************************/
void    ADS1114_AlertRdy_ISR();                                                             //ISR callback function for  ADS1114
uint8_t Irms_Calculate(float *Irms, uint16_t SamplesToTake, float CalibrationConstant);     //Calculate RMS current of SCT current sensors
float   Irms_OffsetCorrection(float Irms);                                                  //Used to linearize CT sensor response

void    E_Monitor_CreateSigfoxMsg(CT_SENSOR *ctSensorArr, uint8_t *transmitArr);            //Used to packetize RMS Current readings for transmission to backend
void    E_Monitor_CT_Array_Isigma_Update(CT_SENSOR *ctSensorArr);                           //Calculate sum of Irms values
void    E_Monitor_WriteConfig(CT_Sensor_Config  *ctSensor_Config);                          //Write config details for each CT Sensor to EEPROM
void    E_Monitor_ReadConfig(CT_Sensor_Config  *ctSensor_Config);                           //Read config details for each CT Sensor from EEPROM
void    E_Monitor_PrintConfig(CT_Sensor_Config  *ctSensor_Config);                          //Print configuration details of each sensor

void    E_Monitor_Print_Menu();
char    E_Monitor_MainMenu_ProcessInput();
char    E_Monitor_GetChar();
void    E_Monitor_Print_RTC_Menu();
char    E_Monitor_RTC_Menu_ProcessInput();
void    E_Monitor_Print_SensorConfig_Menu();
void    E_Monitor_SetTime();
void    E_Monitor_SetDate();
void    E_Monitor_EnableSensor(CT_Sensor_Config  *ctSensor_Config,uint8_t CT_Sensor_Number);
void    E_Monitor_DisableSensor(CT_Sensor_Config  *ctSensor_Config,uint8_t CT_Sensor_Number);
/**********************************************************************************************************************************/

void setup()
{
  //Configure the debug LED I/O
  pinMode(DBG_LED, OUTPUT);

  //Configure enable pin for analogue voltage supply
  pinMode(VSENSOR_ARR_EN, OUTPUT);

  //Enable analogue voltage regulator
  digitalWrite(VSENSOR_ARR_EN, HIGH);

  //Power stabilization delay
  delay(PWR_STABILIZATION_DELAY);

  //Enable Serial1 for comms without WSSFM10RAT
  Serial1.begin(9600);

  //Enable Serial out for debug
  Serial.begin(115200);

  /*Wait for Serial terminal to open for 30 seconds on power-up*/
  while (!Serial);

   /*Copy config details to CT sensor array*/
  for(uint8_t i = 0 ; i  < MAX_CT_SENSORS; i++)
  {
      ctSensors[i].Enabled = true;
      ctSensors[i].CalibrationConstant = 0.09;
      ctSensors[i].SensorType = SCT_100A;
  }


  //Power stabilization delay
  delay(PWR_STABILIZATION_DELAY);

  /*Initialize Wisol Module*/
  if (myWisol.begin() == 0)
  {
    LED_Blink(5,500);
  }

  /*Perform reset & wake-up*/
  myWisol.Reset();

  /*Initialize ADS1114 & configure with desired settings*/
  if (myADC_ConfigureForAcquisition() == 0)
  {
    LED_Blink(10, 250);
  }

  /*Put Wisol to sleep*/
  if (myWisol.Sleep() == 0)
  {
    LED_Blink(5, 1000);
  }

 

  /*Initialize multiplexer*/
  CD74HC4067SM_Init();

  /*Enable Multiplexer*/
  CD74HC4067SM_Enable();

  /*Switch off debug LED*/
  digitalWrite(DBG_LED, LOW);
  
  /*Disable USB*/
   E_Monitor_USB_Disable();
}

void loop()
{
  /*Update Sigma(sum of Irms)*/
  E_Monitor_CT_Array_Isigma_Update(ctSensors);

  /*Increment acquisition count*/
  AcquisitionCount++;

  /*Check if required number of cycles per hour have been reached*/
  if (AcquisitionCount ==  ACQUISITIONS_CYCLE_PER_HOUR)
  {
    /*Compute average RMS current at the end of every hour*/
    E_Monitor_Irms_Update(ctSensors);
    
    /*Create Sigfox message from measured current*/
    E_Monitor_CreateSigfoxMsg(ctSensors, transmitArr);
       
    pinMode(WSSFM10RAT_RESET_Pin,OUTPUT);
    pinMode(WSSFM10RAT_WAKEUP_Pin,OUTPUT);
    
    /*Enable Power to USART1 for comms with WSSFM10R*/
    power_usart1_enable();
 
    /*Wake wisol module up*/
    myWisol.Wakeup();
    myWisol.Reset();

    /*Transmit Data to backend*/
    if (myWisol.TransmitData(transmitArr) == 0)
    {
      LED_Blink(5, 250);
    }
    
    /*Put Wisol to sleep*/
    if (myWisol.Sleep() == 0) {}

    /*Set acquisition count to zero*/
    AcquisitionCount = 0;

    /*Clear transmitArr*/
    memset(transmitArr, 0, SENSOR_ARRAY_MESSAGE_SIZE * sizeof(uint8_t));
  }

  /*Put E-monitor to sleep*/
  E_Monitor_Sleep();
  
  /*Wake-up from sleep*/
  E_Monitor_Wake();
}


void E_Monitor_CT_Array_Isigma_Update(CT_SENSOR *ctSensorArr)
{
  /*Acquire Irms measurement from each sensor in array*/
  for (uint8_t  i = 0 ; i < MAX_CT_SENSORS ; i++)
  {
      if(ctSensorArr[i].Enabled)
      {
        /*Select multiplexer channel*/
        CD74HC4067SM_SelectChannel(i);
  
        /*Clear conversion buffer*/
        Irms_Calculate(&ctSensorArr[i].Irms, 1, ctSensorArr[i].CalibrationConstant);
  
        /*Set Irms to zero before performing actual measurement*/
        ctSensorArr[i].Irms= 0;
  
        /*Compute Irms*/
        Irms_Calculate(&ctSensorArr[i].Irms, 800, ctSensorArr[i].CalibrationConstant);;
  
        /*Sum measured current*/
        ctSensorArr[i].Isigma += ctSensorArr[i].Irms;
     }
  }
}


void E_Monitor_Irms_Update(CT_SENSOR *ctSensorArr)
{
  /*Compute average RMS current over the hour*/
  for (uint8_t  i = 0 ; i < MAX_CT_SENSORS ; i++)
  {
    /*Calculate average Irms current over the hour*/
    ctSensors[i].Irms = ctSensors[i].Isigma/ ACQUISITIONS_CYCLE_PER_HOUR;
    
    /*Reset total in preparation for next window/hour*/
    ctSensors[i].Isigma = 0;
  }
}

uint8_t Irms_Calculate(float *I_rms, uint16_t SamplesToTake, float CalibrationConstant)
{
  /*Local variable used to track number of samples collected*/
  uint16_t SampleCount = 0;

  /*Local variables used to compute RMS current*/
  float    Isquared = 0;
  float    Isample = 0;
  float    Ifiltered = 0;
  float    Isum = 0;

  /*Collect specified number of samples*/
  while (SampleCount < SamplesToTake)
  {
    if (myADC.GetConversionResult(&Isample) == 0)
    {
      Ifiltered = (Isample - V_BIAS);
      Isquared = Ifiltered * Ifiltered;
      Isum += Isquared;
      SampleCount++;
    }
  }

  /*Compute RMS Current*/
  *I_rms = (sqrt(Isum / SampleCount)) * CalibrationConstant;

  return 0;
}

float Irms_OffsetCorrection(float Irms)
{
  return (GRADIENT * Irms) - Y_INTERCEPT;
}
/************************************************************************************************************ADC_FUNCTIONS**********************************************************************************/
uint8_t myADC_ConfigureForAcquisition()
{
  /*Enable I2C bus*/
  Wire.begin();

  /*Set clock speed of I2C bus as fast as possile,lowest prescaler*/
  TWBR = 1;

  /*Allow i2c bus to initialize*/
  delay(10);

  /*Initialize ADS1114*/
  if (myADC.begin() != 0)
  {
    return 1;
  }

  /*Set sample rate to 2400SPS*/
  if (myADC.SetSampleRate(DR_2400SPS) != 0)
  {
    return 1;
  }

  /*Set PGA setting to 2, allows input voltage range from 0-4.096v with a resolution of 125 μV*/
  if (myADC.SetPGA(PGA2) != 0)
  {
    return 1;
  }


  /*Set Mode to continuous conversion mode*/
  if (myADC.SetMode(ADS1114_CONTINUOUS_CONVERSION) != 0)
  {
    return 1;
  }

  /*Set comparator to generate positive going pulse after every conversion*/
  if (myADC.SetComparator(ASSERT_ONE_CONV) != 0)
  {
    return 1;
  }

  /*Set Low threshold & high threshold values*/
  if (myADC.SetHighThreshold(0x8000) != 0)
  {
    return 1;
  }

  if (myADC.SetLowThreshold(0x0000) != 0)
  {
    return 1;
  }

  /*Set callback function to handle Alert Ready callback */
  myADC.SetAlertRdy_ISR_Callback(ADS1114_AlertRdy_ISR);

  return 0;
}

void ADS1114_AlertRdy_ISR()
{
  /*Data ready flag to true*/
  myADC.AlertRdy_ISR_Handler();
}
/**************************************************************************************************MULTIPLEXER FUNCTIONS**************************************************************************************/
void CD74HC4067SM_SelectChannel(uint8_t ChannelNo)
{
  /*Shift by 4 bits to the right to correspond to physical pin mapping*/
  uint8_t ChannelNumber = (ChannelNo << 4);

  /*Set required port pins*/
  PORTF = ChannelNumber;
}

void CD74HC4067SM_Reset(uint8_t ChannelNo)
{
  /*Shift by 4 bits to the right to correspond to physical pin mapping*/
  uint8_t ChannelNumber = (ChannelNo << 4);

  /*Bring PORTF back to its initial state*/
  PORTF &= (~ChannelNumber);
}

void CD74HC4067SM_Init()
{

  //Set enable pin high before initializing it
  _HWB_H();
  digitalWrite(CD74HC4067SM_MUX0, LOW);
  digitalWrite(CD74HC4067SM_MUX1, LOW);
  digitalWrite(CD74HC4067SM_MUX2, LOW);
  digitalWrite(CD74HC4067SM_MUX3, LOW);


  //Initialize enable and channel select pins
  DDRE |= (1 << CD74HC4067SM_ENABLE_PIN);
  pinMode(CD74HC4067SM_MUX0, OUTPUT);
  pinMode(CD74HC4067SM_MUX1, OUTPUT);
  pinMode(CD74HC4067SM_MUX2, OUTPUT);
  pinMode(CD74HC4067SM_MUX3, OUTPUT);

  //Set channel to zero by default
  digitalWrite(CD74HC4067SM_MUX0, LOW);
  digitalWrite(CD74HC4067SM_MUX1, LOW);
  digitalWrite(CD74HC4067SM_MUX2, LOW);
  digitalWrite(CD74HC4067SM_MUX3, LOW);
}

void CD74HC4067SM_Enable()
{
  _HWB_L();
}

void CD74HC4067SM_Disable()
{
  _HWB_H();
}
/***************************************DEBUG_FUNCTIONS**************************************************************************/
void LED_Blink(uint8_t NoOfBlinks, uint16_t BlinkTime_Ms)
{
  for (uint8_t i = 0 ; i < NoOfBlinks ; i++)
  {
    digitalWrite(DBG_LED, !digitalRead(DBG_LED));
    delay(BlinkTime_Ms);
  }
}
/*******************************************************************SLEEP_FUNCTIONS**********************************************/
void E_Monitor_Sleep()
{
  /*Disable Multiplexer*/
  CD74HC4067SM_SelectChannel(0);
  CD74HC4067SM_Enable();
    
  /*Dettach pinChange interrupt used for Alert/Data Ready functionality of ADS1114*/
  myADC.DettachAlertRdy_Interrupt();

  /*Set ALERT RDY Pin as input to prevent ISR from interfering with device before it goes to sleep*/
  pinMode(ALERT_RDY, INPUT);
  delay(2);

  /*Turn of Analogue voltage regulator as a precautionary measure*/
  digitalWrite(VSENSOR_ARR_EN, LOW);
  delay(2);

  /*Set WSSFM10RAT RESET & WAKEUP Pins as inputs to prevent current draw through I/Os during sleep*/
  pinMode(WSSFM10RAT_RESET_Pin, INPUT);
  pinMode(WSSFM10RAT_WAKEUP_Pin, INPUT);

  /*Turn off DBG_LED as a precautionary measure*/
  digitalWrite(DBG_LED, LOW);
  delay(2);

  /*Put device to sleep for 8s*/
  LowPower.powerDown(SLEEP_4S, ADC_OFF, BOD_OFF);

}

void E_Monitor_Wake()
{
  /*Enable Analogue Voltage regulator*/
  digitalWrite(VSENSOR_ARR_EN, HIGH);

  /*Power stabilization delay*/
  delay(PWR_STABILIZATION_DELAY);

  /*Configure ADS1114 for acquisition after sleep*/
  if (myADC_ConfigureForAcquisition() == 0)
  {
    //LED_Blink(2,250);
  }
}

void E_Monitor_USB_Disable()
{
   USBCON &= ~(1 << USBE); 
   USBCON &= ~(1 << VBUSTE); 
   USBCON &= ~(1 << OTGPADE); 
   USBCON &= ~(1 << FRZCLK);  
   UHWCON &= ~(1 << UVREGE); 
   USBINT &= ~(1 << VBUSTI); 
   UDCON |= (1 << DETACH);
}
/*******************************************************************BIT_PACKING & MSG CREATION****************************************/
void E_Monitor_CreateSigfoxMsg(CT_SENSOR *ctSensorArr, uint8_t *transmitArr)
{
  /*Convert rms current readings from sensor array to digital steps*/
  for (uint8_t i = 0 ; i < MAX_CT_SENSORS ; i++)
  {
    /*Variable to store maximum current based on sensor type*/
    uint16_t MaxCurrent = 0;

    switch (ctSensorArr[i].SensorType)
    {
      case SCT_10A:
        MaxCurrent = 10;
        break;

      case SCT_25A:
        MaxCurrent = 25;
        break;

      case SCT_50A:
        MaxCurrent = 50;
        break;

      case SCT_100A:
        MaxCurrent = 100;
        break;

      default:
        break;
    }

    /*Determine Resolution based on stepcount*/
    float Resolution = MaxCurrent / (pow(2, 10) - 1);

    /*Compute digitalSteps*/
    ctSensorArr[i].IrmsSteps =  (ctSensorArr[i].Irms) / Resolution;
  }

  /*Bit pack into 24-bit variables & split into 8-byte array for transmission*/
  for (uint8_t  i = 0 ; i < MAX_CT_SENSORS / 2 ; i++)
  {
    ctSensorPairs[i].bits  =   ctSensorArr[2 * i].IrmsSteps;
    ctSensorPairs[i].bits  =  (ctSensorPairs[i].bits << 2);
    ctSensorPairs[i].bits |=   ctSensorArr[2 * i].SensorType;
    ctSensorPairs[i].bits = ctSensorPairs[i].bits << 12;

    ctSensorPairs[i].bits |= (ctSensorArr[2 * i + 1].IrmsSteps << 2);
    ctSensorPairs[i].bits |=   ctSensorArr[2 * i + 1].SensorType;

    transmitArr[3 * i + 2] = ctSensorPairs[i].bits ;
    transmitArr[3 * i + 1] = ctSensorPairs[i].bits >> 8 ;
    transmitArr[3 * i] =   ctSensorPairs[i].bits >> 16;
  }

}
/***********************************************************E_Monitor_Config_Functions********************************************************/
void E_Monitor_WriteConfig(CT_Sensor_Config *ctSensor_Config)
{
   /*Set location where config data should be stored*/
   int E_Monitor_ConfigStartAdd = E_MONITOR_CONFIG_START_ADDR;

   /*Write Updated Config to EEPROM*/
   for(uint8_t i = 0 ; i < MAX_CT_SENSORS ; i++)
   {
     EEPROM.put(E_Monitor_ConfigStartAdd, ctSensor_Config[i]);
     E_Monitor_ConfigStartAdd += sizeof(CT_Sensor_Config);
   }
}

void E_Monitor_ReadConfig(CT_Sensor_Config  *ctSensor_Config)
{
    /*Set location where config data IS stored*/
    int E_Monitor_ConfigStartAdd = E_MONITOR_CONFIG_START_ADDR;

    /*Read config details from EEPROM*/
    for(uint8_t  i = 0 ; i < MAX_CT_SENSORS ; i++)
    {
      EEPROM.get(E_Monitor_ConfigStartAdd, ctSensor_Config[i]);
      E_Monitor_ConfigStartAdd += sizeof(CT_Sensor_Config);
    }
}

void E_Monitor_PrintConfig(CT_Sensor_Config  *ctSensor_Config)
{
  for(uint8_t i = 0 ; i < MAX_CT_SENSORS ; i++)
  {
    Serial.println("");
    Serial.print("CT SENSOR [");
    Serial.print(i);
    Serial.println("] Configuration Details");
    Serial.print("Sensor Type:");
    Serial.println(CT_SensorTypes[ctSensor_Config[i].SensorType]);
    Serial.print("Sensor Calibration Constant:");
    Serial.println(ctSensor_Config[i].CalibrationConstant);
    Serial.print("Sensor Enabled:");
    Serial.println(ctSensor_Config[i].Enabled);
    Serial.println("");
  }
}

void E_Monitor_Print_Menu()
{
  /*Variable to store user input*/
  char UserProcessInput = '0';

  do
  {
    /*Print Config Menu for User*/
    Serial.println("/*********************E-Monitor Config Menu***********************/");
    Serial.println("(1) Edit SCT Sensor Array Config");
    Serial.println("(2) Print SCT Sensor Array Config");
    Serial.println("(3) RTC Menu");
    Serial.println("(4) WSSFM10RAT M");
    Serial.println("(e) Return");
  
    /*Process User Input*/
    UserProcessInput = E_Monitor_MainMenu_ProcessInput();
    
  }while(UserProcessInput != 'e');
  
}

char E_Monitor_MainMenu_ProcessInput()
{
  /*Get Input from User*/
  char UserInput = E_Monitor_GetChar();

  switch(UserInput)
  {
    case '1':
    /*Print Sensor Config Menu*/
    E_Monitor_Print_SensorConfig_Menu();
    break;

    case '2':
    E_Monitor_PrintConfig(ctSensor_Config);
    break;

    case '3':
    /*Print RTC Menu*/
    E_Monitor_Print_RTC_Menu();
    break;

    case '4':
    /*Send Sigfox Test Message*/
    break;

    case 'e':
    break;

    default:
    Serial.println("Invalid Option Selected");
    break;
  }

  Serial.println(UserInput);

  return UserInput;
}

void E_Monitor_Print_SensorConfig_Menu()
{
  /*Variable to store user input*/
  uint8_t UserProcessInput = 0;
  
  /*Print Sensor Config Menu*/
  do
  {
    Serial.println("/**********************CT Sensor Config Menu**********************/");
    Serial.println("Please enter the number [1 - 16] of the CT Sensor you wish to configure:");

    /*Process User Input*/
    UserProcessInput = E_Monitor_SensorConfigMenuProcessInput();

  }while(UserProcessInput != 0);

}

uint8_t E_Monitor_SensorConfigMenuProcessInput()
{
    /*Wait for serial input*/
    while(Serial.available() == 0);
    
   /*Get Input from User*/
   char UserInput[3] = {0};
   
   /*Get number of bytes to be read*/
   uint8_t bytesAvailable = Serial.available();

   /*Read bytes into array*/
   for(uint8_t  i = 0 ; i <  bytesAvailable ; i++)
   {
      UserInput[i] = Serial.read();
   }

   /*Convert  User Input buffer to an integer*/
   uint8_t CT_Sensor_Number = atoi(UserInput);
   
   /*Determine if user has entered a valid integer*/
    if((CT_Sensor_Number > 16) || (CT_Sensor_Number < 1))
   {
     Serial.println("Please enter a valid CT Sensor Number between 1-16");
     return 0;
   }

   /*Print Sensor Configuration Sub-Menu*/
   E_Monitor_Print_SensorConfigSubMenu(CT_Sensor_Number - 1);
   
  return 0;
}

void E_Monitor_Print_SensorConfigSubMenu(uint8_t CT_Sensor_Number)
{
  char UserInput = '0';

  do
  {
    Serial.print("/********************Sensor [");
    Serial.print(CT_Sensor_Number);
    Serial.println("] Configuration Sub-Menu********************/");
    Serial.println("(1) Enter Sensor Type");
    Serial.println("(2) Enter Calibration Constant");
    Serial.println("(3) Enable Sensor");
    Serial.println("(4) Disable Sensor");
    Serial.println("(e) Return");

    UserInput = E_Monitor_SensorConfigSubMenuProcessInput(CT_Sensor_Number);
    
  }while(UserInput != 'e');
}

void E_Monitor_SensorConfigSubMenu_SetCalibrationConstant(CT_Sensor_Config  *ctSensor_Config,uint8_t CT_Sensor_Number)
{   
  /*Variable to store user input*/
  char UserInput[6] = {0};

  /*float to hold calibration constant*/
  float CalibrationConstant = 0;
  
   do
   {
    /*Prompt User to enter calibration constant*/
    Serial.println("Please enter CT Sensor Calibration constant");
    
    /*Wait for User Input*/
    while(Serial.available() == 0);
      
    /*Get Input from User*/
    uint8_t bytesAvailable = Serial.available();

    /*Read out bytes from array*/
    E_Monitor_GetString(UserInput,bytesAvailable);

    /*Convert user input to float*/
    CalibrationConstant = atof(UserInput);

    /*Check if calibration constant is valid*/                                         
    if(CalibrationConstant == 0.0)
    {
      Serial.println("Invalid Calibration Constant Entered");
    }
  }while(CalibrationConstant == 0.0);

  Serial.println("Valid Calibration Constant");
  
  ctSensor_Config[CT_Sensor_Number].CalibrationConstant = CalibrationConstant;
}


char E_Monitor_SensorConfigSubMenuProcessInput(uint8_t CT_Sensor_Number)
{
  /*Get User Input*/
  char UserInput = E_Monitor_GetChar();

  switch(UserInput)
  {
    case '1':
    E_Monitor_GetSensorType(ctSensor_Config,CT_Sensor_Number);
    break;

    case '2':
    E_Monitor_SensorConfigSubMenu_SetCalibrationConstant(ctSensor_Config,CT_Sensor_Number);
    break;

    case '3':
    E_Monitor_EnableSensor(ctSensor_Config,CT_Sensor_Number);
    break;

    case '4':
    E_Monitor_DisableSensor(ctSensor_Config,CT_Sensor_Number);
    break;

    case 'e':
    break;

    default:
    Serial.println("Invalid Input");
    break;
  }

  return UserInput;
}

void E_Monitor_GetSensorType(CT_Sensor_Config  *ctSensor_Config,uint8_t CT_Sensor_Number)
{
  char UserInput = '0';

  do
  {
    Serial.print("/**************Select Sensor[");
    Serial.print(CT_Sensor_Number);
    Serial.println("] Type********************/");
    Serial.println("(1) SCT_10A ");
    Serial.println("(2) SCT_25A ");
    Serial.println("(3) SCT_50A");
    Serial.println("(4) SCT_100A");
    Serial.println("(e) Return"); 

    UserInput = E_Monitor_SetSensorType(ctSensor_Config,CT_Sensor_Number);

  }while(UserInput != 'e');
  
}

char E_Monitor_SetSensorType(CT_Sensor_Config  *ctSensor_Config,uint8_t CT_Sensor_Number)
{
    /*Wait for user to enter serial input*/
    while(Serial.available() != 0);
  
    /*Get User Input*/
    char UserInput = E_Monitor_GetChar();

    switch(UserInput)
    {
      case '1':
      ctSensor_Config[CT_Sensor_Number].SensorType = SCT_10A;
      Serial.println("Type set to SCT_10A");
      break;

      case '2':
      ctSensor_Config[CT_Sensor_Number].SensorType = SCT_25A;
      Serial.println("Type set to SCT_25A");
      break;

      case '3':
      ctSensor_Config[CT_Sensor_Number].SensorType = SCT_50A;
      Serial.println("Type set to SCT_50A");
      break;

      case '4':
      ctSensor_Config[CT_Sensor_Number].SensorType = SCT_100A;
      Serial.println("Type set to SCT_100A");
      break;

      case 'e':
      break;

      default:
      Serial.println("Invalid Input Entered");
      break;
    }

    return UserInput;
}

void E_Monitor_EnableSensor(CT_Sensor_Config  *ctSensor_Config,uint8_t CT_Sensor_Number)
{
  ctSensor_Config[CT_Sensor_Number].Enabled = true;
  Serial.println("Sensor Enabled");
}

void E_Monitor_DisableSensor(CT_Sensor_Config  *ctSensor_Config,uint8_t CT_Sensor_Number)
{
  ctSensor_Config[CT_Sensor_Number].Enabled = false;
  Serial.println("Sensor Disabled");
}

void E_Monitor_Print_RTC_Menu()
{
  /*Variable to store user input*/
  char UserInput  = '0';

  do
  {
    /*Print RTC Menu*/
    Serial.println("/*************************RTC Menu******************************/");
    Serial.println("(1) Set Time");
    Serial.println("(2) Set Date");
    Serial.println("(3) Get Time");
    Serial.println("(4) Get Date");

    /*Wait for user input*/
    UserInput = E_Monitor_RTC_Menu_ProcessInput();
    
  }while(UserInput != 'e');
}

char E_Monitor_RTC_Menu_ProcessInput()
{
  /*Get input from user*/
  char UserInput = E_Monitor_GetChar();

  switch(UserInput)
  {
    case '1':
    E_Monitor_SetTime();
    break;

    case '2':
    E_Monitor_SetDate();
    break;

    case '3':
    break;

    case '4':
    break;

    case 'e':
    break;

    default:
    Serial.println("Invalid Input");
    break;
  }  

  return UserInput;
}

void E_Monitor_SetTime()
{
  /*Sentinel value used to check if time is set correctly*/
  bool isTimeEnteredValid = false;

  /*String to store user input*/
  char myTime[9]={0};

   /*Variables to extract*/
   uint8_t hour[3] = {0};
   uint8_t minute[3] = {0};
   uint8_t second[3] = {0};

  do
  {
    /*Prompt User to enter time*/
    Serial.println("Please Enter Time in the following format hh:mm:ss");

    /*Wait for user to enter input*/
    while(Serial.available()== 0);

    /*Get number of bytes to be read*/
    uint8_t bytesAvailable = Serial.available();

    /*If bytes available is greater than 9 then input is invalid*/
    if(bytesAvailable < 9)
    {
      /*read out bytes from serial port*/
      for(uint8_t i = 0 ; i < bytesAvailable ; i++)
      {
        myTime[i] = Serial.read();
      }
    }
    else
    {
      Serial.println("Time entered is invalid");
    }

    /*Copy hour from time string*/
    strncpy(hour,myTime,2);

    /*Copy minute from time string*/
    strncpy(minute,(myTime + 3),2);

    /*Copy second from time string*/
    strncpy(second,(myTime+6),2);

    /*Convert char array to integers*/
    uint8_t hh = atoi(hour);
    uint8_t mm = atoi(minute);
    uint8_t ss = atoi(second);
    
    if((hh < 24) && (hh >= 0) && (mm < 60) && (mm >= 0) && (ss < 60) && (ss >= 0 ))
    {
      isTimeEnteredValid = true;
      
      Serial.print("Time is set to ");
      Serial.print(hh);
      Serial.print(":");
      Serial.print(mm);
      Serial.print(":");
      Serial.println(ss);

      /*Call RTC set time function here*/
    }
    else
    {
      Serial.println("Time entered is invalid");
    }
  }while(!isTimeEnteredValid);
}

void E_Monitor_SetDate()
{
  /*Sentinel value used to check if date is set correctly*/
  bool isDateEnteredValid = false;

  /*String to store user input*/
  char myDate[9]={0};

   /*Variables to extract*/
   uint8_t day[3] = {0};
   uint8_t month[3] = {0};
   uint8_t year[3] = {0};

  do
  {
    /*Prompt User to enter time*/
    Serial.println("Please Enter Date in the following format dd/mm/yy");

    /*Wait for user to enter input*/
    while(Serial.available()== 0);

    /*Get number of bytes to be read*/
    uint8_t bytesAvailable = Serial.available();

    /*If bytes available is greater than 9 then input is invalid*/
    if(bytesAvailable < 9)
    {
      /*read out bytes from serial port*/
      for(uint8_t i = 0 ; i < bytesAvailable ; i++)
      {
        myDate[i] = Serial.read();
      }
    }
    else
    {
      Serial.println("Date entered is invalid");
    }

    /*Copy hour from time string*/
    strncpy(day,myDate,2);

    /*Copy minute from time string*/
    strncpy(month,(myDate + 3),2);

    /*Copy second from time string*/
    strncpy(year,(myDate+6),2);

    /*Convert char array to integers*/
    uint8_t dd = atoi(day);
    uint8_t mm = atoi(month);
    uint8_t yy = atoi(year);
    
    if((dd <= 31) && (dd > 0) && (mm <= 12) && (mm > 0) && (yy < 100 ) && (yy >= 0 ))
    {
      isDateEnteredValid = true;
      
      Serial.print("Date is set to ");
      Serial.print(dd);
      Serial.print("/");
      Serial.print(mm);
      Serial.print("/");
      Serial.println(yy);
    }
    else
    {
      Serial.println("Date entered is invalid");
    }
  }while(!isDateEnteredValid);
}


void E_Monitor_GetString(char *cString,uint16_t cStringLength)
{
  for(uint16_t i = 0 ; i < cStringLength ; i++)
  {
    cString[i] = Serial.read();
  }
}

char E_Monitor_GetChar()
{
   /*Wait for serial input*/
   while(Serial.available() == 0);

   /*Read received byte*/
   char receivedByte = Serial.read();

   return receivedByte;
}
