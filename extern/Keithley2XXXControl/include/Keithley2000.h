/*
 * Keithley2000.h
 *
 *  Created on: Aug 13, 2013
 *      Author: Mathieu Benoit mbenoitATcern.ch
 */

#ifndef KEITHLEY2000_H_
#define KEITHLEY2000_H_

#include "Keithley.h"

class Keithley2000: public Keithley {
public:
	Keithley2000(int gpib_num);
	virtual ~Keithley2000();
	void SetMeasureCurrent();
	void SetMeasureVoltage();
	void SetMeasureResistance2W();
	void SetMeasureResistance4W();
	double ReadValue();
	void Read();
	void Write(char* command);
};

#endif /* KEITHLEY2000_H_ */
