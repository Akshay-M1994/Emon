/**
* @file  	WSSFM10RAT.cpp
* @brief 	Driver for WSSFM10RAT module
* @author	Akshay Maharaj
* @date 	14/04/2021
*/
/* Private defines -----------------------------------------------------------*/
#define SERIAL_DELAY        100U                                   //Delay to allow time for WSSFM10R to respond
#define RESET_DELAY         250U                                    //Delay required after pulling reset line low
#define WAKEUP_DELAY        250U                                    //Delay required after pulling wakeup line low


#define AT_OK              "AT"                                     //AT OK Command
#define AT_GET_TEMP        "AT$T?"                                  //Get WSSFM1OR Temperature
#define AT_SEND_FRAME      "AT$SF="                                 //Transmit 12-Byte Frame
#define AT_SLEEP           "AT$P=1\r\0"                             //Enter Sleep Mode
#define AT_GET_ID          "AT$I=10\r\0"                            //Get DeviceID
#define AT_GET_PAK         "AT$I=11\r\0"                            //Get Device Pak
/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include <Arduino.h>
#include "WSSFM10RAT.h"
/* Private typedef -----------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/*Private function prototypes-------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
WSSFM10RAT::WSSFM10RAT(HardwareSerial *Serial,uint8_t Reset_n,uint8_t Wakeup_n)
{
  /*Set internal pointer to serial port used for comms with WSSFM10RAT*/
  WSSFM10RAT_Serial = Serial;

  /*Set internal variables that store Reset & Wakeup pins*/
  Reset_Pin = Reset_n;
  Wakeup_Pin = Wakeup_n;

}

uint8_t WSSFM10RAT::begin()
{
  /*Set I/Os to high before configuring them*/
  digitalWrite(Reset_Pin,HIGH);
  digitalWrite(Wakeup_Pin,HIGH);

  /*Initialize I/Os to be used for wakeup and reset of Wisol module  */
  pinMode(Wakeup_Pin,OUTPUT);
  pinMode(Reset_Pin,OUTPUT);  

  /*Clear internal variables used to store ID & PAK*/
  memset(ID,0,WSSFM10RAT_ID_LEN);
  memset(PAK,0,WSSFM10RAT_PAK_LEN);

  /*Get ID*/
  if(GetID() != 0)
  {
    return 1;
  }

  /*Get PAK*/
  if(GetPAK() != 0)
  {
    return 1;
  }
  
  return 0;
}

uint8_t WSSFM10RAT::Sleep()
{

  /*Send Test GET_ID Command*/
  WSSFM10RAT_Serial->print("AT$P=1\r");

  /*Acquire Response from WSSFM10RAT*/
  String WSSFM10RAT_RESPONSE = getData();

  /*Check if message was sent successfully*/
  if(WSSFM10RAT_RESPONSE.indexOf("OK") >= 0)
  {
    return 0;
  }
   
   return 1;
} 

uint8_t WSSFM10RAT::SetTxPower(){}

uint8_t WSSFM10RAT::SendTestMsg()
{
  /*Buffer to hold response from sigfox module*/  
  char RxBuffer[RXBUFFER_SIZE]={0};

  /*Send Test Data*/
  WSSFM10RAT_Serial->print(AT_SEND_FRAME);
  WSSFM10RAT_Serial->print("123456123456");
  WSSFM10RAT_Serial->println("");

  /*Acquire Response from WSSFM10RAT*/
  String WSSFM10RAT_RESPONSE = getData();

  /*Check if message was sent successfully*/
  if(WSSFM10RAT_RESPONSE.indexOf("OK") >= 0)
  {
    return 0;
  }
  
  return 1;
}

void WSSFM10RAT::print_hex(int v, int num_places)                                    //Send Bytes In HEX format to WSSFM1OR
{
    int mask=0, n, num_nibbles, digit;
    for (n=1; n<=num_places; n++)
    {
        mask = (mask << 1) | 0x0001;
    }
    v = v & mask; // truncate v to specified number of places
    num_nibbles = num_places / 4;
    if ((num_places % 4) != 0)
    {
        ++num_nibbles;
    }
    do
    {
        digit = ((v >> (num_nibbles-1) * 4)) & 0x0f;
        WSSFM10RAT_Serial->print(digit, HEX);
    } while(--num_nibbles);
}

uint8_t WSSFM10RAT::TransmitData(uint8_t *bytes)
{
      /*Transmit 12-byte array*/
      WSSFM10RAT_Serial->print(AT_SEND_FRAME);
      print_hex(bytes[0],8);
      print_hex(bytes[1],8);
      print_hex(bytes[2],8);
      print_hex(bytes[3],8);
      print_hex(bytes[4],8);
      print_hex(bytes[5],8);
      print_hex(bytes[6],8);
      print_hex(bytes[7],8);
      print_hex(bytes[8],8);
      print_hex(bytes[9],8);
      print_hex(bytes[10],8);
      print_hex(bytes[11],8);
      WSSFM10RAT_Serial->println("");

      /*Acquire Response from WSSFM10RAT*/
      String WSSFM10RAT_RESPONSE = getData();
      
    /*Check if message was sent successfully*/
    if(WSSFM10RAT_RESPONSE.indexOf("OK") >= 0)
    {
      return 0;
    }
  
     return 1;
}


String WSSFM10RAT::getData(){
  
  String data = "";
  char output;

  while (!WSSFM10RAT_Serial->available()){}

  while(WSSFM10RAT_Serial->available()){
    output = WSSFM10RAT_Serial->read();
    if ((output != 0x0A) && (output != 0x0D)){//0x0A Line feed | 0x0D Carriage return
      data += output;
      }

    delay(10);
  } 

  return data;
}

uint8_t WSSFM10RAT::GetID()
{
  /*Send Test GET_ID Command*/
  WSSFM10RAT_Serial->print("AT$I=10\r");

  /*Acquire Response from WSSFM10RAT*/
  String WSSFM10RAT_RESPONSE = getData();

  /*Convert Response to c string*/
   strcpy(ID, WSSFM10RAT_RESPONSE.c_str());

  return 0;
}

uint8_t WSSFM10RAT::GetPAK()
{
   /*Send Test GET_PAK Command*/
  WSSFM10RAT_Serial->print("AT$I=11\r");

  /*Acquire Response from WSSFM10RAT*/
  String WSSFM10RAT_RESPONSE = getData();

  /*Convert Response to c string*/
  strcpy(PAK, WSSFM10RAT_RESPONSE.c_str());

  return 0;
}

void WSSFM10RAT::Reset()
{
  digitalWrite(Reset_Pin,LOW);
  delay(RESET_DELAY);
  digitalWrite(Reset_Pin,HIGH);
  delay(RESET_DELAY);

}

void WSSFM10RAT::Wakeup()
{
  digitalWrite(Wakeup_Pin,LOW);
  delay(WAKEUP_DELAY);
  digitalWrite(Wakeup_Pin,HIGH);
  delay(WAKEUP_DELAY);
}
/* Private functions -----------------------------------------------*/
/***********************************END OF FILE***************************************/
