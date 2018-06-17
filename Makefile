help:
	@echo 'Help details:'
	@echo 'hex: compile hex file'
	@echo 'flash: install hex file'
	@echo 'program: compile hex and install'


.PHONY: flash clean

program: hex flash
	
hex:
	avr-gcc -Os -DF_CPU=2000000 -mmcu=atxmega32a4 -std=gnu99 -c *.c 
	avr-gcc -DF_CPU=2000000 -mmcu=atxmega32a4 -o cheapstat.elf *.o 
	avr-objcopy -j .eeprom --set-section-flags=.eeprom="alloc,load" --change-section-lma .eeprom=0 --no-change-warnings -O ihex  cheapstat.elf cheapstat.eep
	avr-objcopy -R .eeprom -O ihex cheapstat.elf cheapstat.hex
	
flash:
	avrdude -c avrispmkII -p x32a4  -U flash:w:cheapstat.hex -U eeprom:w:cheapstat.eep

clean:
	rm -f *.o
	rm -f *.elf
	rm -f *.hex
	rm -f *.eep
