/*
 * srv_midi_internal.h
 *
 *  Created on: Oct 28, 2021
 *      Author: fidel
 */

#ifndef INC_SRV_MIDI_INTERNAL_H_
#define INC_SRV_MIDI_INTERNAL_H_




#endif /* INC_SRV_MIDI_INTERNAL_H_ */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f0xx_hal.h"

/* Private defines ------------------------------------------------------------------*/
#define NOTE_ON  0x90
#define NOTE_OFF 0x80
#define CONTROL_CHANGE 0xB0

/* Private function prototypes -----------------------------------------------*/
void srv_midi_internal_sendNote(uint8_t Note,uint8_t channel, uint8_t velocity, UART_HandleTypeDef uart);
void srv_midi_internal_controlChange(uint8_t controlNumber, uint8_t controlValue, UART_HandleTypeDef uart);
