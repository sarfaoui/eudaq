/*
 * Keithley2410.h
 *
 *  Created on: Aug 14, 2013
 *      Author: Mathieu Benoit mbenoitATcern.ch
 */

#ifndef KEITHLEY2410_H_
#define KEITHLEY2410_H_

#include "Keithley.h"

class Keithley2410: public Keithley {
public:
	Keithley2410(int gpib_num);
	virtual ~Keithley2410();
	void SetMeasureCurrent();
	void SetMeasureVoltage();
	void SetMeasureResistance2W();
	void SetMeasureResistance4W();
	void SetSourceVoltage4W();
	void SetOutputVoltage(double Voltage);
	void SetCurrentLimit(double current_limit);
	double ReadValue();
	void OutputOn();
	void OutputOff();
	void Read();
	void Write(char* command);

};

#endif /* KEITHLEY2410_H_ */
