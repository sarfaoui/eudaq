//KeithleyGPIB.cpp
//Takes readings from a Keithley 2000 multimeter
#include <fstream>
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
#include "Keithley2000.h"
#include "Keithley2410.h"
using namespace std;

float GetPt100Temperature(float r)
{
      float const Pt100[] = {80.31, 82.29, 84.27, 86.25, 88.22, 90.19, 92.16, 94.12, 96.09, 98.04,
                            100.0, 101.95, 103.9, 105.85, 107.79, 109.73, 111.67, 113.61, 115.54, 117.47,
                            119.4, 121.32, 123.24, 125.16, 127.07, 128.98, 130.89, 132.8, 134.7, 136.6,
                            138.5, 140.39, 142.29, 157.31, 175.84, 195.84};

      int t = -50, i = 0, dt = 0;

      if (r > Pt100[0])
         while (250 > t)
         {
               dt = (t < 110) ? 5 : (t > 110) ? 50 : 40;

               if (r < Pt100 [++i])
                  return t + ( r - Pt100[i-1])*dt/(Pt100[i]-Pt100[i-1]);
                  t += dt;
         };

      return t;

}

void try2000(){
	Keithley2000 *k2000 = new Keithley2000(16);
		sleep(1);
		//Keithley2410 *k2410 = new Keithley2410(12);

		sleep(1);
		//k2410->OutputOn();
		sleep(1);

		k2000->SetMeasureResistance4W();
		sleep(1);

		//k2410->SetMeasureCurrent();


		while(1){
			double R=k2000->ReadValue();
			cout << "Resistance is : " << GetPt100Temperature(R) << endl;
			sleep(1);
	//		cout << "Current is : " << k2410->ReadValue() << endl;
	//
		}


	//	k2410->OutputOff();

	//	if(ibdev(minor,pad,sad,timeout,send_eoi,eos_mode) & ERR){
	//		printf("No GPIB in the house !! \n");
	//		exit(1);
	//		}
	//	fprint_status(stdout,"status is : ");
	//	cout << "ud is " << k2000 << endl;
	//
	//	if(ibclr(k2000) & ERR){
	//		printf("Clear Failed !! \n");
	//		}
	//
	//	sprintf(buffer,"func \'res\' ");
	//	if(ibwrt(k2000,buffer,strlen(buffer)) & ERR){
	//		printf("Write Failed !! \n");
	//		}
	//
	//
	//	for(int i = 0;i<100;i++){
	//	sprintf(buffer,"fetc?");
	//	if(ibwrt(k2000,buffer,strlen(buffer)) & ERR){
	//		printf("Write Failed !! \n");
	//		}
	//
	//	if(ibrd(k2000,buffer,buffer_size-1) & ERR){
	//		printf("Read Failed !! \n");
	//		}


		//delete k2410;
		delete k2000;
}

void try2410(){
		Keithley2410 *k2410 = new Keithley2410(12);


		sleep(1);
		k2410->OutputOn();
		sleep(1);

		k2410->SetMeasureCurrent();
		sleep(1);
		k2410->SetSourceVoltage4W();
		sleep(1);
		k2410->SetOutputVoltage(15);

//		while(1){
//			double current=k2410->ReadValue();
//			cout << "Current is : " << (current) << "A" << endl;
//			sleep(1);
//
//		}


		//k2410->OutputOff();
		
		delete k2410;
}

void tryBoth(){

	Keithley2410 *k2410 = new Keithley2410(12);
	Keithley2000 *k2000 = new Keithley2000(16);

	sleep(1);
	k2000->SetMeasureResistance4W();


	sleep(1);
	k2410->OutputOn();
	sleep(1);

	k2410->SetMeasureCurrent();
	sleep(1);
	k2410->SetSourceVoltage4W();
	sleep(1);
	k2410->SetOutputVoltage(15);

	while(1){
		double current=k2410->ReadValue();
		cout << "Current is : " << (current) << endl;
		sleep(1);
		double R=k2000->ReadValue();
		cout << "Temperature is : " << GetPt100Temperature(R) << "C" << endl;
	}


	k2410->OutputOff();

	delete k2410;

}


void doIV(int nstep, double VMax){

	Keithley2410 *k2410 = new Keithley2410(12);

	k2410->OutputOn();
	sleep(1);

	k2410->SetMeasureCurrent();
	sleep(1);
	k2410->SetSourceVoltage4W();
	sleep(1);
	k2410->SetOutputVoltage(0);

	double vstep = VMax/nstep;

	double V=0;

	ofstream out("IV.txt");


	for (int i = 0; i<nstep;i++){

		V+=vstep;
		k2410->SetOutputVoltage(V);
		sleep(2);
		double current=k2410->ReadValue();
		cout << "Current is : " << (current) << "at V = " << V <<  endl;
		out << V << " " << current*1e9 << "\n";
	}
	k2410->SetOutputVoltage(0);
	out.close();

}


int main() {
	//try2000();
	//try2410();
	//tryBoth();
	doIV(30,-60);
	return 0;
}
