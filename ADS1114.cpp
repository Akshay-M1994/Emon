/**
* @file  	ADS1114.c
* @brief 	Driver for Texas ADS1114 16-Bit Precision Analog-to-digital converter 
* @author	Akshay Maharaj
* @date 	14/04/2021
*/
/* Private defines -----------------------------------------------------------*/
#define CONVERSION_RESULT_SIZE         2U                         /*!<Size of each adc conversion result is 24-bits or 3-bytes*/
#define TWOS_COMPLEMENT_MASK           0x00800000                 /*!<This mask is used to identify 24-bit unsigned integers that are in two's complement format*/
#define MSB_CONVERSION_RESULT_MASK     0x00ffffff                 /*!<This mask is used to remove the MSB in each uint32_t variable that holds a conversion result*/
#define ADC_RESOLUTION                 12U                        /*!<Resolution of ADS1114*/

#define SAMPLE_RATE_MASK              0xFF1F
#define PGA_MASK                      0xF1FF      
#define COMP_QUEUE_MASK               0xFFFC
#define CONVERSION_MODE_MASK          0xFEFF
#define OS_BIT_MASK                   0xEFFF

#define OS_BIT_CONV_REQ               0x8000

#define MIN_LSB_RESOLUTION            0.0078125f

     

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include <Arduino.h>
#include <PinChangeInterrupt.h>
#include "ADS1114.h"
/* Private typedef -----------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/*!Macro used to validate ADS1114 sample rate selection*/
#define IS_SPS_VALID(FREQ) 	((FREQ == DR_5SPS)   || (FREQ == DR_10SPS)   || \
										(FREQ == DR_20SPS) 	|| (FREQ == DR_40SPS)   || \
										(FREQ == DR_80SPS) 	|| (FREQ == DR_160SPS)  || \
										(FREQ == DR_320SPS)  || (FREQ == DR_640SPS)  || \
										(FREQ == DR_1000SPS) || (FREQ == DR_2000SPS))

/*!Macro used to validate ADS1114 PGA selection*/
#define IS_PGA_VALID(PGA)		 		 ((PGA == PGA1) || (PGA == PGA2)  || \
								 	 	 (PGA == PGA4)  || (PGA == PGA8)  || \
									 	 (PGA == PGA16) || (PGA == PGA32) || \
									 	 (PGA == PGA64) || (PGA == PGA128))

/* Private variables ---------------------------------------------------------*/
uint32_t count = 0;
/*Private function prototypes-------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
ADS1114::ADS1114(uint8_t ADS1114_I2C_Address,uint8_t ALERT_RDY_Pin)
{
        
  /*Store pin being used for Alert/Rdy pin for for use in other methods*/
  Alert_Rdy_Pin = ALERT_RDY_Pin;

  /*Set Alert Rdy flag to false by default*/
  Alert_Rdy =  false;

  /*Set value of all registers to zero*/
  ADS1114_RegisterMap.Register[CONVERSION_REG_ADD] = 0;
  ADS1114_RegisterMap.Register[CONFIG_REG_ADD] = 0;
  ADS1114_RegisterMap.Register[LO_THRESH_REG_ADD] = 0;
  ADS1114_RegisterMap.Register[HIGH_THRESH_REG_ADD] = 0;

  /*Set i2c address(shifting by 1 bit to the left as i2c library uses 8-bit addressing*/
  I2C_Address = ADS1114_I2C_Address ;  
}

uint8_t ADS1114::begin()
{

  /*Configure I/O that will be used as alert pin*/
  pinMode(Alert_Rdy_Pin,INPUT_PULLUP);

  /*Check if ADS1114 is present on i2c bus*/
  if(!IsDevicePresent())
  {
    return 1;
  }

  /*Read configuration, low threshold & high threshold registers on startup and ensure that they have default values specified in datasheet*/
  ADS1114_RegisterMap.Register[CONFIG_REG_ADD] = ReadRegister(CONFIG_REG_ADD);
  ADS1114_RegisterMap.Register[LO_THRESH_REG_ADD] = ReadRegister(LO_THRESH_REG_ADD);
  ADS1114_RegisterMap.Register[HIGH_THRESH_REG_ADD] = ReadRegister(HIGH_THRESH_REG_ADD);
  
  /*If register values match default startup values then initialization has completed successfully*/
  if((ADS1114_RegisterMap.Register[CONFIG_REG_ADD] == CONFIG_REG_RST_VAL) && (ADS1114_RegisterMap.Register[LO_THRESH_REG_ADD] == LOW_THRESH_REG_RST_VAL) && (ADS1114_RegisterMap.Register[HIGH_THRESH_REG_ADD] == HIGH_THRESH_REG_RST_VAL))
  {
    return 0;
  }

  return 1;
}

uint16_t ADS1114::ReadRegister(uint8_t RegAddress)
{
  Wire.beginTransmission(I2C_Address);
  Wire.write(RegAddress);
  Wire.endTransmission();

  int rv = Wire.requestFrom(I2C_Address, (uint8_t) 2);
  if (rv == 2) 
  {
    uint16_t value = (Wire.read() << 8);
    value += Wire.read();
    return value;
  }
  return 0;
}

uint8_t ADS1114::WriteRegister(uint8_t RegisterAddress,uint16_t RegisterValue)
{
    Wire.beginTransmission(I2C_Address);
    Wire.write((uint8_t)RegisterAddress);
    Wire.write((uint8_t)(RegisterValue >> 8));
    Wire.write((uint8_t)(RegisterValue & 0xFF));
    return (Wire.endTransmission() == 0);
}


void ADS1114::SetAlertRdy_ISR_Callback(void (*userDefinedCallback)())
{
   /*Configure interrupt to be used to receive data ready events*/
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(Alert_Rdy_Pin), userDefinedCallback, FALLING);   
}

void ADS1114::DettachAlertRdy_Interrupt()
{
  /*Dettach alert rdy interrupt*/
    detachPinChangeInterrupt(digitalPinToPinChangeInterrupt(Alert_Rdy_Pin)); 
}

bool ADS1114::IsDevicePresent()
{
  Wire.beginTransmission(I2C_Address);
  if(Wire.endTransmission())
  { 
    return false;
  }
  else return true;
}
bool ADS1114::SetPGA(ADS1114_PGA PGA)
{
  /*Update internal register map with desired PGA settings*/
  ADS1114_RegisterMap.Register[CONFIG_REG_ADD] = (PGA_MASK & ADS1114_RegisterMap.Register[CONFIG_REG_ADD]);

  ADS1114_RegisterMap.Register[CONFIG_REG_ADD] = (PGA | ADS1114_RegisterMap.Register[CONFIG_REG_ADD]);

  /*Write updated config register settings to device*/
  if(WriteRegister(CONFIG_REG_ADD,ADS1114_RegisterMap.Register[CONFIG_REG_ADD]) != 0)
  {
    return false;
  }

  /*Read back config register to determine if register settings were updated successfully*/
  ADS1114_RegisterMap.Register[CONFIG_REG_ADD] = ReadRegister(CONFIG_REG_ADD);

  if((ADS1114_RegisterMap.Register[CONFIG_REG_ADD] & (~PGA_MASK)) == PGA)
  {
    return true;
  }

  return false;
}

bool ADS1114::SetSampleRate(ADS1114_SPS SampleRate)
{
  /*Update internal register map with desired sample rate settings*/
  ADS1114_RegisterMap.Register[CONFIG_REG_ADD] = (SAMPLE_RATE_MASK & ADS1114_RegisterMap.Register[CONFIG_REG_ADD]);
  ADS1114_RegisterMap.Register[CONFIG_REG_ADD] = (SampleRate | ADS1114_RegisterMap.Register[CONFIG_REG_ADD]);

  /*Write updated config register settings to device*/
  if(WriteRegister(CONFIG_REG_ADD,ADS1114_RegisterMap.Register[CONFIG_REG_ADD]) != 0)
  {
    return false;
  }

  /*Read back config register to determine if register settings were updated successfully*/
  ADS1114_RegisterMap.Register[CONFIG_REG_ADD] = ReadRegister(CONFIG_REG_ADD);

   if((ADS1114_RegisterMap.Register[CONFIG_REG_ADD] & (~SAMPLE_RATE_MASK)) == SampleRate)
  {
    return true;
  }

  return false;
}

bool ADS1114::SetMode(ADS1114_CONVERSION_MODE Mode)
{
  /*Update internal register map with desired conversion mode settings*/
  ADS1114_RegisterMap.Register[CONFIG_REG_ADD] = (CONVERSION_MODE_MASK & ADS1114_RegisterMap.Register[CONFIG_REG_ADD]);
  ADS1114_RegisterMap.Register[CONFIG_REG_ADD] = (Mode | ADS1114_RegisterMap.Register[CONFIG_REG_ADD]);

  

  /*Write updated config register settings to device*/
  if(WriteRegister(CONFIG_REG_ADD,ADS1114_RegisterMap.Register[CONFIG_REG_ADD]) != 0)
  {
    return false;
  }


  /*Read back config register to determine if register settings were updated successfully*/
  ADS1114_RegisterMap.Register[CONFIG_REG_ADD] = ReadRegister(CONFIG_REG_ADD);



   if((ADS1114_RegisterMap.Register[CONFIG_REG_ADD] & (~CONVERSION_MODE_MASK)) == Mode)
  {
    return true;
  }

  return false;

}

bool ADS1114::SetComparator(ADS1114_COMP_QUEUE COMP_QUEUE)
{
  /*Update internal register map with desired conversion mode settings*/
  ADS1114_RegisterMap.Register[CONFIG_REG_ADD] = (COMP_QUEUE_MASK & ADS1114_RegisterMap.Register[CONFIG_REG_ADD]);
  ADS1114_RegisterMap.Register[CONFIG_REG_ADD] = (COMP_QUEUE | ADS1114_RegisterMap.Register[CONFIG_REG_ADD]);

  /*Write updated config register settings to device*/
  if(WriteRegister(CONFIG_REG_ADD,ADS1114_RegisterMap.Register[CONFIG_REG_ADD]) != 0)
  {
    return false;
  }

  /*Read back config register to determine if register settings were updated successfully*/
  ADS1114_RegisterMap.Register[CONFIG_REG_ADD] = ReadRegister(CONFIG_REG_ADD);

   if((ADS1114_RegisterMap.Register[CONFIG_REG_ADD] & (~COMP_QUEUE_MASK)) == COMP_QUEUE)
  {
    return true;
  }

  return false;

}

bool ADS1114::SetLowThreshold(uint16_t LowThresholdValue)
{
    /*Update internal register map with desired conversion mode settings*/
  ADS1114_RegisterMap.Register[LO_THRESH_REG_ADD] = LowThresholdValue;

  /*Write updated config register settings to device*/
  if(WriteRegister(LO_THRESH_REG_ADD,ADS1114_RegisterMap.Register[LO_THRESH_REG_ADD]) != 0)
  {
    return false;
  }

  /*Read back config register to determine if register settings were updated successfully*/
  ADS1114_RegisterMap.Register[LO_THRESH_REG_ADD] = ReadRegister(LO_THRESH_REG_ADD);

  if(ADS1114_RegisterMap.Register[LO_THRESH_REG_ADD] == LowThresholdValue)
  {
    return true;
  }

  return false;
}

bool ADS1114::SetHighThreshold(uint16_t HighThreshValue)
{
  /*Update internal register map with desired conversion mode settings*/
  ADS1114_RegisterMap.Register[HIGH_THRESH_REG_ADD] = HighThreshValue;

  /*Write updated config register settings to device*/
  if(WriteRegister(HIGH_THRESH_REG_ADD,ADS1114_RegisterMap.Register[HIGH_THRESH_REG_ADD]) != 0)
  {
    return false;
  }

  /*Read back config register to determine if register settings were updated successfully*/
  ADS1114_RegisterMap.Register[HIGH_THRESH_REG_ADD] = ReadRegister(HIGH_THRESH_REG_ADD);

  if(ADS1114_RegisterMap.Register[HIGH_THRESH_REG_ADD] == HighThreshValue)
  {
    return true;
  }

    return false;

}

uint8_t ADS1114::StartSingleConversion()
{
    /*Read out config register*/
     ADS1114_RegisterMap.Register[CONFIG_REG_ADD] = ReadRegister(CONFIG_REG_ADD);

    /*Clear OS Bit field*/
    ADS1114_RegisterMap.Register[CONFIG_REG_ADD] = (OS_BIT_MASK & ADS1114_RegisterMap.Register[CONFIG_REG_ADD]);

    /*Set OS bit to one to start conversion*/
    ADS1114_RegisterMap.Register[CONFIG_REG_ADD] = (OS_BIT_CONV_REQ | ADS1114_RegisterMap.Register[CONFIG_REG_ADD]);

    /*Write updated config register settings to device*/
   if(WriteRegister(CONFIG_REG_ADD,ADS1114_RegisterMap.Register[CONFIG_REG_ADD]) != 0)
   {
      return 1;
   }

   return 0;
}

uint8_t ADS1114::GetConversionResult(float *ConversionResult)
{
    if(Alert_Rdy)
    {      
      /*Read out result from conversion register*/
      ADS1114_RegisterMap.Register[CONVERSION_REG_ADD] = ReadRegister(CONVERSION_REG_ADD);

      /*Variable to hold gain multiplier*/
      uint16_t PGA_Gain = (ADS1114_RegisterMap.Register[CONFIG_REG_ADD] & 0x0E00);
      PGA_Gain = (PGA_Gain >> 8);

      /*Float to hold LSB value*/
      float LSB_Value = 0;

      /*Convert digital code/steps to value in volts*/
      switch(PGA_Gain)
      {
        case 0:
        LSB_Value = 3;
        break;

        case 2:
        LSB_Value = 2;
        break;

        case 4:
        LSB_Value = 1;
        break;

        case 6:
        LSB_Value = 0.5;
        break;

        case 8:
        LSB_Value = 0.25;
        break;

        case 10:
        case 12:
        case 14:
        LSB_Value = 0.125;
        break;

        default:
        break;
      }

      uint16_t ConversionResultSign = 1;

      if((ADS1114_RegisterMap.Register[CONVERSION_REG_ADD] & 0x8000) == 0x8000)
      {
        ADS1114_RegisterMap.Register[CONVERSION_REG_ADD] = (~ADS1114_RegisterMap.Register[CONVERSION_REG_ADD]);
        ConversionResultSign = -1;
      }

      /*Shift result by 4-bits*/
      ADS1114_RegisterMap.Register[CONVERSION_REG_ADD] = (ADS1114_RegisterMap.Register[CONVERSION_REG_ADD] >> 4);
       
      /*Compute conversion result in millivolts*/
      *ConversionResult = ADS1114_RegisterMap.Register[CONVERSION_REG_ADD] * LSB_Value * ConversionResultSign;

      /*Set Alert Rdy flag to false after reading*/
      Alert_Rdy = false;

      return 0;
    }

    return 1;
}

uint8_t ADS1114::GetConversionResult(int16_t *ConversionResult)
{
   if(Alert_Rdy)
    {      
      /*Read out result from conversion register*/
      ADS1114_RegisterMap.Register[CONVERSION_REG_ADD] = ReadRegister(CONVERSION_REG_ADD);

       /*Shift result by 4-bits*/
      ADS1114_RegisterMap.Register[CONVERSION_REG_ADD] = (ADS1114_RegisterMap.Register[CONVERSION_REG_ADD] >> 4);

      /*If negative determine magnitude first and then multiply by -1*/
      if((ADS1114_RegisterMap.Register[CONVERSION_REG_ADD] & 0x0800) == 0x0800)
      {
        ADS1114_RegisterMap.Register[CONVERSION_REG_ADD] = (~ADS1114_RegisterMap.Register[CONVERSION_REG_ADD]);
        *ConversionResult  =  ADS1114_RegisterMap.Register[CONVERSION_REG_ADD] * -1 ;
      }
      else
      {
        *ConversionResult  =  ADS1114_RegisterMap.Register[CONVERSION_REG_ADD];
      }
      
      /*Set Alert Rdy flag to false after reading*/
      Alert_Rdy = false;

      return 0;
    }

    return 1;
}

void ADS1114::AlertRdy_ISR_Handler(void)
{
  Alert_Rdy = true;
}
/* Private functions -----------------------------------------------*/
/***********************************END OF FILE***************************************/
