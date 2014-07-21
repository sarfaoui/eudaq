/*
 * Keithley.h
 *
 *  Created on: Aug 13, 2013
 *      Author: Mathieu Benoit mbenoitATcern.ch
 */

#include <iostream>
#include "gpib/ib.h"
#include <string.h>
#include <stdio.h>
#include <ctype.h>
#include <errno.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdint.h>
#include <getopt.h>

using namespace std;



#ifndef KEITHLEY_H_
#define KEITHLEY_H_

class Keithley {
public:
	Keithley(int gpib_num);
	void initialize();
	virtual ~Keithley();
	void fprint_status( FILE* filep, char *msg );
	void Read();
	void Write(char* command);
	virtual void SetMeasureCurrent();
	virtual void SetMeasureVoltage();
	virtual void SetMeasureResistance2W();
	virtual void SetMeasureResistance4W();
	virtual double ReadValue();

protected :

	char cmd[80];

	int ud;
	int pad;
	int minor;
	int sad;
	int send_eoi;
	int eos_mode;
	int timeout ;
	char* buffer;
	int buffer_size ;
};

#endif /* KEITHLEY_H_ */
