/*
 * Keithley.cpp
 *
 *  Created on: Aug 13, 2013
 *      Author: Mathieu Benoit mbenoitATcern.ch
 */

#include "Keithley.h"

//#define KEITHLEYDEBUG

Keithley::Keithley(int gpib_num) {

	this->pad=GetPAD(gpib_num);
	this->sad=GetSAD(gpib_num);
	minor = 0;
	send_eoi = 1;
	eos_mode = 0;
	timeout = T10s; // mod by km
    buffer_size = 1024;
    buffer =  (char*)malloc(buffer_size);

    cout << "[Keithley] Initializing with gpib address " << pad << endl;
    ud=ibdev(minor,pad,sad,timeout,send_eoi,eos_mode);
    if(ud<=0){
    		printf("No GPIB in the house !! \n");
    		exit(1);
    		}

    cout << "[Keithley] Init ud " << ud << endl;
#ifdef KEITHLEYDEBUG
    	fprint_status(stdout,"status is : ");
#endif

}

Keithley::~Keithley() {


}

void Keithley::fprint_status( FILE* filep, char *msg )
{
	fprintf( filep, "%s\n", msg);

	fprintf( filep, "ibsta = 0x%x  < ", ThreadIbsta() );
	if(ThreadIbsta() & ERR)  fprintf(filep, "ERR ");
	if(ThreadIbsta() & TIMO) fprintf(filep, "TIMO ");
	if(ThreadIbsta() & END)  fprintf(filep, "END ");
	if(ThreadIbsta() & SRQI) fprintf(filep, "SRQI ");
	if(ThreadIbsta() & RQS) fprintf(filep, "RQS ");
	if(ThreadIbsta() & SPOLL) fprintf(filep, "SPOLL ");
	if(ThreadIbsta() & EVENT) fprintf(filep, "EVENT ");
	if(ThreadIbsta() & CMPL) fprintf(filep, "CMPL ");
	if(ThreadIbsta() & LOK) fprintf(filep, "LOK ");
	if(ThreadIbsta() & REM)  fprintf(filep, "REM ");
	if(ThreadIbsta() & CIC)  fprintf(filep, "CIC ");
	if(ThreadIbsta() & ATN)  fprintf(filep, "ATN ");
	if(ThreadIbsta() & TACS) fprintf(filep, "TACS ");
	if(ThreadIbsta() & LACS) fprintf(filep, "LACS ");
	if(ThreadIbsta() & DCAS) fprintf(filep, "DCAS ");
	if(ThreadIbsta() & DTAS) fprintf(filep, "DTAS ");
	fprintf( filep, ">\n" );

	fprintf( filep,"iberr= %d\n", iberr);
	if( ( ThreadIbsta() & ERR ) )
	{
		fprintf( filep, "%s\n", gpib_error_string( ThreadIberr() ) );
	}

	fprintf( filep, "\n" );

	fprintf( filep, "ibcnt = %d\n", ibcnt );
}

void Keithley::Write(char* command){

	sprintf(buffer,command);
	if(ibwrt(pad,buffer,strlen(buffer)) & ERR){
		printf("Write Failed !! \n");
		}
#ifdef KEITHLEYDEBUG
	   fprint_status(stdout,"status is : ");
#endif
}

void Keithley::Read(){

	if(ibrd(pad,buffer,buffer_size-1) & ERR){
		printf("Read Failed !! \n");
		}
#ifdef KEITHLEYDEBUG
	   fprint_status(stdout,"status is : ");
#endif
}
void Keithley::SetMeasureCurrent(){

	this->Write(":conf:curr:dc");

}

void  Keithley::SetMeasureVoltage(){

	this->Write(":conf:volt:dc");
}

void Keithley::SetMeasureResistance2W(){

	this->Write(":conf:res");

}

void Keithley::SetMeasureResistance4W(){

	this->Write(":conf:fres");

}

double Keithley::ReadValue(){

	this->Write("read?");
	sleep(1);
	this->Read();
	double value;
	sscanf(buffer,"%f",value);
	return value;


}
