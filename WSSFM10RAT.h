/**
* @file  	WSSFM10RAT.h
* @brief 	Header file for WSSFM10RAT Sigfox module driver
* @author	Akshay Maharaj
* @date 	12/04/2021
*/

 /* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _WSSFM10RAT_H_
#define _WSSFM10RAT_H_
/*-----------------------------------------------WSSFM10RAT COMMANDS---------------------------------------------*/
#define AT_OK              "AT"                                     //AT OK Command
#define AT_GET_TEMP        "AT$T?"                                  //Get WSSFM1OR Temperature
#define AT_SEND_FRAME      "AT$SF="                                 //Transmit 12-Byte Frame
#define AT_SLEEP           "AT$P=1"                                 //Enter Sleep Mode
#define AT_GET_ID          "AT$I=10"                                //Get DeviceID
#define AT_GET_PAK         "AT$I=11"                                //Get Device Pak

#define WSSFM10RAT_ID_LEN   20U                                     //WSSFM10RAT ID MAX LENGTH
#define WSSFM10RAT_PAK_LEN  20U                                     //WSSFM10RAT PAK MAX LENGTH
#define RXBUFFER_SIZE       60U
/* Includes -----------------------------------------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include <Arduino.h>
/*Exported_Enums/Typedefs----------------------------------------------------------------------------------------*/
/*-----------------------------------------------Public typedef -------------------------------------------------*/
class WSSFM10RAT{
  public:

  WSSFM10RAT(HardwareSerial *Serial,uint8_t Reset_n,uint8_t Wakeup_n);

  uint8_t begin();

  uint8_t Sleep();

  uint8_t SetTxPower();

  uint8_t TransmitData(uint8_t *bytes);

  uint8_t GetTemperature(float *Temperature);

  String getData();
  
  uint8_t SendTestMsg();

  void Reset();

  void Wakeup();

  char ID[WSSFM10RAT_ID_LEN];
  
  char PAK[WSSFM10RAT_PAK_LEN];

  private:
  
  uint8_t GetID();

  uint8_t GetPAK();

  void print_hex(int v, int num_places);
  
  HardwareSerial *WSSFM10RAT_Serial;
  
  uint8_t Reset_Pin;

  uint8_t  Wakeup_Pin;
};
 /* Define the public group ************************************/
 /* Define the private group ***********************************/
 #endif /* _WSSFM10RAT_H_ */
 /*********************END OF FILE******************************/
