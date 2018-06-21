# Potentiostat

This is the Cheapstat modified code designed to work with the specified conditions of CDAC kolkata

Build instructions (on linux Debian based system, preferrably Ubuntu)

Requires:

avrdude
avr-libc
gcc-avr
binutils-avr


Installation of required softwares on Linux: 
sudo apt-get install gcc-avr binutils-avr gdb-avr avr-libc avrdude


///////////////////////  Makefile instructions  //////////////////
There is a Makefile present in the directory which you can use to execute commands to obtain the below mentioned results:
make hex   - builds software and creates .hex and .eep files.;
make flash - flashs firmware to cheapstat using the avrispmkII programmer;
make clean - clean source directory ;
//////////////////////   End of Makefile instructions  //////////////

