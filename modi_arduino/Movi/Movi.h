/**
 * Movi.h - Movi library header file.
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
#ifndef __MOVI_H__
#define __MOVI_H__
#include "Arduino.h"
// #include <inttypes.h>

#define MOVSIMPLE 1
#define MOVRAW 2
#define MOVFW 1
#define MOVBW 2
#define MOVLE 3
#define MOVRI 4
#define MOVST 5
#define TURNSHORT 1
#define TURNLONG 2
#define MOTOR1 1
#define MOTOR2 2
#define MOTORALL 3
#define BWBW 0
#define BWFW 1
#define FWBW 2
#define FWFW 3

#define NORMAL 1
#define REVERSE 2
#define MIRROR 4


class Movi{
public:
	Movi();
	Movi(int flag);
	void initPin();
	void left(int spd, bool close);
	void right(int spd, bool close);
	void fw(int spd);
	void bw(int spd);
	void stop(int opt);
	void raw(int m1spd, bool m1dir, int m2spd, bool m2dir);
	int getStatus();
	void automov(uint8_t pck1,uint8_t pck2,uint8_t pck3,uint8_t pck4);
private:
	int STBY;
	int PWMA;
	int AIN1;
	int AIN2;
	int PWMB;
	int BIN1;
	int BIN2;
	int status;

};
#endif