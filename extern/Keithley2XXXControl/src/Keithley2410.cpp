/*
 * Keithley2410.cpp
 *
 *  Created on: Aug 14, 2013
 *      Author: Mathieu Benoit mbenoitATcern.ch
 */

#include "Keithley2410.h"

Keithley2410::Keithley2410(int gpib_num):Keithley(gpib_num) {

//	this->pad=GetPAD(gpib_num);
//	this->sad=GetSAD(gpib_num);
//	cout << pad << " " << sad << endl;
//	minor = 0;
//	send_eoi = 1;
//	eos_mode = 0;
//	timeout = T10s;
//    buffer_size = 256;
//    buffer =  (char*)malloc(256);

//    cout << "[Keithley2410] Clearing Device State "  << endl;
//    if(ibclr(pad) & ERR){
//    		printf("Clear Failed !! \n");
//    		}
//#ifdef KEITHLEYDEBUG
//    fprint_status(stdout,"status is : ");
//#endif


}

Keithley2410::~Keithley2410() {
	// TODO Auto-generated destructor stub
}

void Keithley2410::SetMeasureCurrent(){

	cout << "[Keithley2410] Setting to Current Measurement" << endl;
	this->Write(":FUNC:CONC OFF");
	this->Write(":conf:curr:dc");


}

void  Keithley2410::SetMeasureVoltage(){

	cout << "[Keithley2410] Setting to Voltage Measurement" << endl;
	this->Write(":conf:volt:dc");
}

void Keithley2410::SetMeasureResistance2W(){

	cout << "[Keithley2410] Setting to Resistance Measurement with 2 wires" << endl;
	this->Write(":conf:res");

}

void Keithley2410::SetMeasureResistance4W(){

	cout << "[Keithley2410] Setting to Resistance Measurement with 4 wires" << endl;
	this->Write(":conf:fres");


}

void Keithley2410::OutputOn(){

	cout << "[Keithley2410] Output On" << endl;
	this->Write(":OUTP ON");
}

void Keithley2410::OutputOff(){

	cout << "[Keithley2410] Output Off" << endl;
	this->Write(":OUTP OFF");
}


double Keithley2410::ReadValue(){

	cout << "[Keithley2410] Reading"<< endl;;
	this->Write("READ?");
	this->Read();
	double voltage_sense,value;
	//cout << "[Keithley2410] buffer value " << buffer << "C " << endl;
	sscanf(buffer,"%le, %le",&voltage_sense,&value);

	return value;


}

// modified by km
/*
void Keithley2410::Write(char* command){

	if(ibwrt(ud,command,strlen(command)) & ERR){
		printf("Write Failed !! \n");
		}
#ifdef KEITHLEYDEBUG
	   fprint_status(stdout,"status is : ");
#endif
}
*/

void Keithley2410::Write(char* command){

	sprintf(buffer,command);
	if(ibwrt(ud,buffer,strlen(buffer)) & ERR){
		printf("Write Failed !! \n");
		}
#ifdef KEITHLEYDEBUG
	   fprint_status(stdout,"status is : ");
#endif
}
//end mod by km

void Keithley2410::Read(){

	if(ibrd(ud,buffer,buffer_size-1) & ERR){
		printf("Read Failed !! \n");
		}
#ifdef KEITHLEYDEBUG
	   fprint_status(stdout,"status is : ");
#endif
}

void Keithley2410::SetSourceVoltage4W(){


	cout << "[Keithley2410] Switching to Voltage Source" << endl;
	this->Write(":SOUR:VOLT:MODE FIX");
	this->Write(":SYST:RSEN ON");

}

void Keithley2410::SetOutputVoltage(double voltage){

	cout << "[Keithley2410] Switching Source to " << voltage << "V" << endl;
	sprintf(buffer,":SOUR:VOLT:LEV %.2f",voltage);
	this->Write(buffer);

}

