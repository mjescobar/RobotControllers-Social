/**
 * Movi.cpp - Movi library implementation file.
 * Copyright (c) 2013 Fabian Rubilar. All rights reserved.
 *
 * This file is part of Movi library for MODI robots.
 *
 * Movi library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Movi library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Movi library.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "Movi.h"
#include "Arduino.h"

Movi::Movi(){
	STBY = 7;
	PWMA = 3;
	AIN1 = 2;
	AIN2 = 4;
	PWMB = 11;
	BIN1 = 12;
	BIN2 = 8;
	status = 0;
}
Movi::Movi(int flag){
	if(flag&NORMAL){
		STBY = 7;
		if(flag&MIRROR){
			PWMA = 3;
			AIN1 = 2;
			AIN2 = 4;
			PWMB = 11;
			BIN1 = 12;
			BIN2 = 8;
		}
		else{
			PWMA = 11;
			AIN1 = 12;
			AIN2 = 8;
			PWMB = 3;
			BIN1 = 2;
			BIN2 = 4;
		}
		status = 0;
	}
	if(flag&REVERSE){
		STBY = 7;
		if(flag&MIRROR){
			PWMA = 3;
			AIN1 = 4;
			AIN2 = 2;
			PWMB = 11;
			BIN1 = 8;
			BIN2 = 12;
		}
		else{
			PWMA = 11;
			AIN1 = 8;
			AIN2 = 12;
			PWMB = 3;
			BIN1 = 4;
			BIN2 = 2;		
		}
		status = 0;
	}
}
void Movi::initPin(){
	pinMode(STBY, OUTPUT);
	pinMode(PWMA, OUTPUT);
	pinMode(AIN1, OUTPUT);
	pinMode(AIN2, OUTPUT);
	pinMode(PWMB, OUTPUT);
	pinMode(BIN1, OUTPUT);
	pinMode(BIN2, OUTPUT);
}
int Movi::getStatus(){
	return status;
}
void Movi::raw(int m1spd, bool m1dir, int m2spd, bool m2dir){
	if(m1spd||m2spd){
		digitalWrite(STBY, HIGH); //disable standby
		bool inPin1A = m1dir? HIGH:LOW;
		bool inPin2A = m1dir? LOW:HIGH;
		bool inPin1B = m2dir? LOW:HIGH;
		bool inPin2B = m2dir? HIGH:LOW;
		digitalWrite(AIN1,inPin1A); digitalWrite(AIN2,inPin2A); analogWrite(PWMA,m1spd);
		digitalWrite(BIN1,inPin1B); digitalWrite(BIN2,inPin2B); analogWrite(PWMB,m2spd);
	}
	else digitalWrite(STBY,LOW); // standby
}
void Movi::left(int spd, bool close){
	if(close) raw(spd,1,spd,1);
	else raw(spd,1,0,1);
}
void Movi::right(int spd, bool close){
	if(close) raw(spd,0,spd,0);
	else raw(0,0,spd,0);
}
void Movi::fw(int spd){
	raw(spd,1,spd,0);
}
void Movi::bw(int spd){
	raw(spd,0,spd,1);
}
void Movi::stop(int opt){
	if(opt==1) analogWrite(PWMA,0);
	if(opt==2) analogWrite(PWMB,0);
	if(opt==3) digitalWrite(STBY,LOW);
}
void Movi::automov(uint8_t pck1,uint8_t pck2,uint8_t pck3,uint8_t pck4){
	if(pck1 == MOVSIMPLE){
		if(pck2==MOVFW) fw(pck4);
		if(pck2==MOVBW) bw(pck4);
		if(pck2==MOVLE){ 
			if(pck3==TURNSHORT) left(pck4,1);
			if(pck3==TURNLONG) left(pck4,0);
		}
		if(pck2==MOVRI){ 
			if(pck3==TURNSHORT) right(pck4,1);
			if(pck3==TURNLONG) right(pck4,0);
		}
		if(pck2==MOVST){
			if(pck4==MOTOR1) stop(1);
			if(pck4==MOTOR2) stop(2);
			if(pck4==MOTORALL) stop(3);
		}
	}
	else if(pck1 == MOVRAW){
		if(pck2==BWBW)	raw(pck3,0,pck4,1);
		if(pck2==FWBW)	raw(pck3,0,pck4,0);
		if(pck2==BWFW)	raw(pck3,1,pck4,1);
		if(pck2==FWFW)	raw(pck3,1,pck4,0);
	}





}
	