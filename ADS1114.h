/**
* @file  	ADS1114.h
* @brief 	This header file contains all required definitions to implement the
* 			driver a Texas ADS1114 16-Bit Precision Analog-to-digital converter for STM32 Platform
* @author	Akshay Maharaj
* @date 	12/04/2021
*/

 /* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _ADS1114_H_
#define _ADS1114_H_
//-----------------------------------------------ADS1114_REGISTER_ADDRESSES-------------------------------------------------------//
#define REG_MAP_SIZE                      4U               /*!<Number of registers within ADS1114*/

#define CONVERSION_REG_ADD                0x00             /*!<CONVERSION Register Address*/
#define CONFIG_REG_ADD                    0x01             /*!<CONFIGURATION Register Address*/
#define LO_THRESH_REG_ADD                 0x02             /*!<LO THRESHOLD Register Address*/
#define HIGH_THRESH_REG_ADD               0x03             /*!<HIGH THRESHOLD Register Address*/
/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include <SoftwareWire.h>

 /*Exported_Enums/Typedefs----------------------------------------------------------------------------------------------------------*/
  /**
   * @brief These typedef enums are used to indicate/set the state of the DRDY_Pin
   */
  typedef enum
 {
 	DRDY_LOW = 0,						   /*!<Used to indicate DRDY Pin is at logic '0' i.e. a new conversion result is available*/
 	DRDY_HIGH,							   /*!<Used to indicate DRDY Pin is at logic '1' i.e. no conversion result is available*/
 }ADS1114_DRDY_STATE;

 //--------------------------------------------------ADS1114_CONVERSION_MODE---------------------------------------*/
 /**
  * @brief These typedef enums are used to specify the conversion mode when using ADS1248_GetConversionResult function.
  */
 typedef enum
 {
    ADS1114_CONTINUOUS_CONVERSION = 0x0000,             /*!<Get conversion results continuously on DRDY Interrupts*/
    ADS1114_SINGLE_SHOT = 0x0100,                       /*!<Get a single conversion result and put device in power down mode*/   
 }ADS1114_CONVERSION_MODE;
 //---------------------------------------------------ADS1114_PGA_SETTINGS-----------------------------------------*/
 /**
   * @brief These typedef enums are used specify/set the programmable gain of the ADS1114
  */
 typedef enum
  {
      PGA1   = 0x0000,						/*!< Set Programmable gain to 1*/
      PGA2   = 0x0200,						/*!< Set Programmable gain to 2*/
      PGA4   = 0x0400,						/*!< Set Programmable gain to 4*/
      PGA8   = 0x0600,						/*!< Set Programmable gain to 8*/
      PGA16  = 0x0800,						/*!< Set Programmable gain to 16*/
      PGA32  = 0x0A00,						/*!< Set Programmable gain to 32*/
      PGA64  = 0x0C00,						/*!< Set Programmable gain to 64*/
      PGA128 = 0x0E00,						/*!< Set Programmable gain to 128*/
  }ADS1114_PGA;
  //---------------------------------------------------ADS1114_DATARATE_SETTINGS----------------------------------------*/
  /**
    * @brief These typedef enums are used specify/set the sample/data rate of the ADS1114
   */
  typedef enum
  {
      DR_128SPS     = 0x0000,					/*!< Set Data/Sample rate to 128Hz*/
      DR_250SPS    = 0x0020,					/*!< Set Data/Sample rate to 250Hz*/
      DR_490SPS    = 0x0040,					/*!< Set Data/Sample rate to 490Hz*/
      DR_920SPS    = 0x0060,					/*!< Set Data/Sample rate to 920Hz*/
      DR_1600SPS   = 0x0080,					/*!< Set Data/Sample rate to 1600Hz*/
      DR_2400SPS   = 0x00A0,					/*!< Set Data/Sample rate to 2400Hz*/
      DR_3300SPS   = 0X00C0,					/*!< Set Data/Sample rate to 3300Hz*/
  }ADS1114_SPS;
  //--------------------------------------------ADS1114_COMPARATOR_QUEUE_SETTINGS---------------------------------------------------//
  typedef enum
  {
    ASSERT_ONE_CONV = 0,
    ASSERT_TWO_CONV = 1,
    ASSERT_FOUR_CONV = 2,
    DISABLE_COMPARATOR =3,
  }ADS1114_COMP_QUEUE;
  //-----------------------------------------------ADS1114_DEFAULT_REG_VALUES-------------------------------------------------------//
  /**
    * @brief These typedef enums are used specify the default values of the all registers on the ADS1114 on startup(except FSC registers,they vary and are internally controlled)
   */
  enum ADS1248_DEFAULT_REG_VALUES
  {
      CONVERSION_REG_RST_VAL  =  0x0000,	  /*!<Default value of CONVERSION register*/
      CONFIG_REG_RST_VAL 	  =  0x8583,	  /*!<Default value of CONFIGURATION register*/
      LOW_THRESH_REG_RST_VAL  =  0x8000,	  /*!<Default value of LOW THRESHOLD register*/
      HIGH_THRESH_REG_RST_VAL =  0x7FFF,	  /*!<Default value of HIGH THRESHOLD register*/
  };
  /*-----------------------------------------------Public typedef -------------------------------------------------*/
  /**
   * @brief This union is used to set bit values of the registers within the ADS1248
   */
  typedef union
  {
      struct
      {

        unsigned CONV_REG:16;		    /*!<Register contains ADC conversion result*/
    
        unsigned OS:1;					    /*!<Determines operational status of device i.e. if it is actively converting or needs to start a conversion should the device be in single shot mode*/
        unsigned MUX:3;					    /*!<Determins active differential channels*/
        unsigned PGA:3;					    /*!<Determines programmable gain amplifier settings*/
        unsigned MODE:1;			      /*!<Determines devices mode of operation i.e. single shot mode or continuous conversion*/
        
        unsigned DR:3;			        /*!<Determines the rate at which applied signals are converted/sampled*/
        unsigned COMP_MODE:1;			  /*!<Determines mode of operation of comparator i.e. Windowed or Traditional*/
        unsigned COMP_POL:1;			  /*!<Determines if comparator output is active high or low*/
        unsigned COMP_LAT:1;			  /*!<Determines latching behaviour of ALERT/RDY Pin*/
		    unsigned COMP_QUE:2;				/*!<Controls operation of ALERT/RDY pin i.e. number of conversions after which to generate interrupt*/

        unsigned LO_THRESH:16;			/*!<Register that holds lower threshold of comparator*/
        unsigned HIGH_THRESH:16;	  /*!<Register that holds higher threshold of comparator*/
      }bits;
      uint16_t Register[REG_MAP_SIZE];
  }_ADS1114_RegisterMap;

/**@brief Typedef used to create instances of ADS1114 on the same i2c bus*/
class ADS1114{
  public:
  
  ADS1114(uint8_t ADS1114_I2C_Address,uint8_t ALERT_RDY_Pin);
  
  uint8_t begin();

  void AlertRdy_ISR_Handler(void);

  void SetAlertRdy_ISR_Callback(void (*userDefinedCallback)());
  
  void DettachAlertRdy_Interrupt();

  bool SetSampleRate(ADS1114_SPS SampleRate);

  bool SetPGA(ADS1114_PGA PGA);
  
  bool SetMode(ADS1114_CONVERSION_MODE MODE);

  bool SetComparator(ADS1114_COMP_QUEUE COMP_QUEUE);
  
  bool SetHighThreshold(uint16_t HighThreshValue);

  bool SetLowThreshold(uint16_t LowThresholdValue);

  uint8_t GetConversionResult(float *ConversionResult);
  
  uint8_t GetConversionResult(int16_t *ConversionResult);

  uint8_t StartSingleConversion();

  _ADS1114_RegisterMap ADS1114_RegisterMap;

  private:
  
  volatile bool    Alert_Rdy;

  uint8_t Alert_Rdy_Pin;
  
  uint8_t I2C_Address;

  uint16_t ReadRegister(uint8_t RegAddress);  

  uint8_t WriteRegister(uint8_t RegisterAddress,uint16_t RegisterValue);

  bool IsDevicePresent();
};
 /* Define the public group ************************************/
 /* Define the private group ***********************************/
 #endif /* _ADS1114_H_ */
 /*********************END OF FILE******************************/
