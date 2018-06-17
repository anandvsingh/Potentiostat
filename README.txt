Build instructions (on linux)

Requires:

avrdude
avr-libc
gcc-avr
binutils-avr


Installation:

make hex   - builds software and creates .hex and .eep files.
make flash - flashs firmware to cheapstat using the avrispmkII programmer
make clean - clean source directory 
