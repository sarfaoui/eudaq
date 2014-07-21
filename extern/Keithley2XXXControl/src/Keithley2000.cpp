/*
 * Keithley2000.cpp
 *
 *  Created on: Aug 13, 2013
 *      Author: Mathieu Benoit mbenoitATcern.ch
 */

#include "Keithley2000.h"

Keithley2000::Keithley2000(int gpib_num):Keithley(gpib_num) {
	// TODO Auto-generated constructor stub
	this->pad=GetPAD(gpib_num);
	this->sad=GetSAD(gpib_num);
	cout << pad << " " << sad << endl;
	minor = 0;
	send_eoi = 1;
	eos_mode = 0;
	timeout = T1s;
    buffer_size = 256;
    buffer =  (char*)malloc(256);


    cout << "[Keithley2000] Clearing Device State "  << endl;
    if(ibclr(pad) & ERR){
    		printf("Clear Failed !! \n");
    		}
#ifdef KEITHLEYDEBUG
    fprint_status(stdout,"status is : ");
#endif
}

Keithley2000::~Keithley2000() {
	// TODO Auto-generated destructor stub
}

void Keithley2000::SetMeasureCurrent(){

	cout << "[Keithley200] Setting to Current Measurement" << endl;
	this->Write(":conf:curr:dc");

}

void  Keithley2000::SetMeasureVoltage(){

	cout << "[Keithley2000] Setting to Voltage Measurement" << endl;
	this->Write(":conf:volt:dc");
}

void Keithley2000::SetMeasureResistance2W(){

	cout << "[Keithley2000] Setting to Resistance Measurement with 2 wires" << endl;
	this->Write(":conf:res");

}

void Keithley2000::SetMeasureResistance4W(){

	cout << "[Keithley2000] Setting to Resistance Measurement with 4 wires" << endl;
	this->Write(":conf:fres");


}

double Keithley2000::ReadValue(){

	cout << "[Keithley2000] Reading"<< endl;;
	this->Write("read? ");
	this->Read();
	double value;
	//cout << "[Keithley2000] buffer value " << buffer << "C " << endl;
	sscanf(buffer,"%le",&value);

	return value;


}



void Keithley2000::Write(char* command){

	sprintf(buffer,command);
	if(ibwrt(ud,buffer,strlen(buffer)) & ERR){
		printf("Write Failed !! \n");
		}
#ifdef KEITHLEYDEBUG
	   fprint_status(stdout,"status is : ");
#endif
}

void Keithley2000::Read(){

	if(ibrd(ud,buffer,buffer_size-1) & ERR){
		printf("Read Failed !! \n");
		}
#ifdef KEITHLEYDEBUG
	   fprint_status(stdout,"status is : ");
#endif
}
