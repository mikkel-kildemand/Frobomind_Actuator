/****************************************************************************
# Frobit FroboMind Controller interface
# Copyright (c) 2014-2016, Mikkel K. Larsen
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the copyright holder nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*****************************************************************************
# File: actuator.c
# Project: Frobomind Row Cleaner Actuator
# Platform: FroboMind Controller Feb. 2014 http://www.frobomind.org
# Microcontroller: AT90CAN128
# Author: Mikkel K. Larsen
# Created:  2014-08-28 Mikkel K. Larsen
# Modified: 
****************************************************************************/
/* includes */
#include <avr/interrupt.h>
#include "actuator.h"
#include "fmctrl_def.h"
#include "pid_ctrl_int.h"

/***************************************************************************/
/* defines */
#define STATE_NO_ERR		2 /* maximum system state value before shutting down motors */

/* Actuator defines  */

#define ACTUATOR_PORT			PORTE


/* Errormassage defines  */
#define ACTUATOR_ERROR_PORT			PINA


#define TICKS_BUF_LEN		10

/* PID control */


/***************************************************************************/
/* global and static variables */
extern char state;

/* actuator variables */
short dist;
long height;
uint16_t Cnt3;					//Used for counting how many times ADC1_value is far from dist set by UART
uint8_t State;			//Variable for switch case - Actuator detect.

/***************************************************************************/
/***************************************************************************/
/***************************************************************************/
/***************************************************************************/
void actuator_set_param(short dist, long height)
{
//	if (Cnt3 > 0x3FFF )
//		State = 4;						//Default, High outputs
//	else if ((PINA & 0x0F) != 0x00)
//		State = 5;						//Fault detected, try reset bu puthing actuator out
//	else 
if ((dist) > (height + 0x20))
		State = 1;						//Actuator out
	else if	((dist) < height)
		State = 2;						//Actuator in
	else
		State =3;

	switch (State)
		{
		case 1:
			PORTE |= (1<<PE5);  	//Pin 6 High State
			PORTE &= ~(1<<PE3);  	//Pin 3 low State ()
			PORTE |= (1<<PE4);  	//Pin 4 High State
//			Cnt3++;
			break;

		case 2:
			PORTE |= (1<<PE5);  	//Pin 6 High State
			PORTE |= (1<<PE3);  	//Pin 3 High State
			PORTE &= ~(1<<PE4);  	//Pin 4 low State ()
//			Cnt3++;
			break;

		case 3:
			PORTE |= (1<<PE5);  	//Pin 6 High State
			PORTE |= (1<<PE4);  	//Pin 4 High State
			PORTE |= (1<<PE3);  	//Pin 3 High State
			Cnt3 = 0;
			break;

		case 4:
			PORTE |= (1<<PE5);  	//Pin 6 High State
			PORTE |= (1<<PE4);  	//Pin 4 High State
			PORTE |= (1<<PE3);  	//Pin 3 High State
			for(; Cnt3 > 0; Cnt3--)
			break;

		case 5:
			PORTE &= ~(1<<PE5);  	//Pin 6 low State (Ext LED on)
			PORTE &= ~(1<<PE3);  	//Pin 3 low State ()
			PORTE |= (1<<PE4);  	//Pin 4 High State
			Cnt3++;
			break;

		default:
			PORTE |= (1<<PE5);  	//Pin 6 High State
			PORTE |= (1<<PE4);  	//Pin 4 High State
			PORTE |= (1<<PE3);  	//Pin 3 High State
			Cnt3 = 0;
			break;
		}
}
/***************************************************************************/
