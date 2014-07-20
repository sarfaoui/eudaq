OBJS = tmp/main.o tmp/Keithley.o tmp/Keithley2000.o  tmp/Keithley2410.o
SRC = src/Keithley.cpp src/Keithley2000.cpp src/Keithley2410.cpp
CC = g++ -D_REENTRANT -g -O2
CFLAGS =-c 
LFLAGS =  -L/usr/local/lib  -lpthread 
IFLAGS=-I./include -I/usr/local/include

all : $(OBJS) 
	$(CC) $(LFLAGS) $(IFLAGS) $(OBJS)  -o GPIB_Reader /usr/local/lib/libgpib.so

lib :  lib/libKeithley.so

lib/libKeithley.so : $(OBJ) 
	$(CC) $(IFLAGS) -fPIC -shared -o lib/libKeithley.so  $(OBJS) 

tmp/main.o : src/main.cxx 
	$(CC) $(CFLAGS) $(IFLAGS) src/main.cxx -o tmp/main.o 

tmp/Keithley2000.o : src/Keithley2000.cpp tmp/Keithley.o
	$(CC) $(CFLAGS) $(IFLAGS) src/Keithley2000.cpp -o tmp/Keithley2000.o 	

tmp/Keithley2410.o : src/Keithley2410.cpp tmp/Keithley.o
	$(CC) $(CFLAGS) $(IFLAGS) src/Keithley2410.cpp -o tmp/Keithley2410.o 	
	
tmp/Keithley.o : src/Keithley.cpp 
	$(CC) $(CFLAGS) $(IFLAGS) src/Keithley.cpp -o tmp/Keithley.o 
	
clean:
	\rm tmp/*.o  GPIB_Reader

	

