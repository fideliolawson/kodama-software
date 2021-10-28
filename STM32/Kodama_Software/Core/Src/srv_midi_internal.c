/**
  ******************************************************************************
  * @file           : srv_midi_internal.c
  * @brief          : Service to send midi through UART
  ******************************************************************************
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "srv_midi_internal.h"



/* Private function prototypes -----------------------------------------------*/
void srv_midi_internal_sendNote(uint8_t Note,uint8_t channel, uint8_t velocity, UART_HandleTypeDef uart);
void srv_midi_internal_controlChange(uint8_t controlNumber, uint8_t controlValue);

/* Private variables ---------------------------------------------------------*/
uint8_t aTxBuffer[3];

/* Public functions -----------------------------------------------*/
void srv_midi_internal_sendNote(uint8_t note,uint8_t channel, uint8_t velocity,UART_HandleTypeDef uart){
	uint8_t channelBuffer, noteBuffer, velocityBuffer;
	channelBuffer = 0x0F & channel; //To be sure that channel is 4 bits value
	noteBuffer = 0x7F & note; //To be sure that note is 7 bits value
	velocityBuffer = 0x7F & velocity; //To be sure that velocity is 7 bits value
	aTxBuffer[0] = channelBuffer+NOTE_ON;
	aTxBuffer[1] = noteBuffer;
	aTxBuffer[2] = velocityBuffer;
	HAL_UART_Transmit(&uart, aTxBuffer, 3, 1000);
	aTxBuffer[0] = channelBuffer+NOTE_OFF;
	aTxBuffer[1] = noteBuffer;
	aTxBuffer[2] = velocityBuffer;
	HAL_Delay(300); //Wait 300 ms
	HAL_UART_Transmit(&uart, aTxBuffer, 3, 1000); //Send Note Off

}
