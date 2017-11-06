#SETTINGS
SRCDIR=src
BINDIR=bin

TARGET=858Dplus
MCU=atmega328p
SOURCES=main.c watchdog.c

PROGRAMMER=avrispmkii
#auskommentieren für automatische Wahl
PORT=-Pusb
#BAUD=-B115200

#Ab hier nichts verändern
OBJECTS=$(addprefix $(BINDIR)/, $(SOURCES:.c=.o))
CFLAGS=-c -Os
LDFLAGS=

all: flash eeprom

flash: $(TARGET)_flash.hex

eeprom: $(TARGET)_eeprom.hex


$(TARGET)_flash.hex: $(TARGET).elf
	avr-objcopy -O ihex -j .data -j .text $(BINDIR)/$(TARGET).elf $(BINDIR)/$(TARGET)_flash.hex

$(TARGET)_eeprom.hex: $(TARGET).elf
	avr-objcopy -O ihex -j .eeprom --change-section-lma .eeprom=0 $(BINDIR)/$(TARGET).elf $(BINDIR)/$(TARGET)_eeprom.hex

$(TARGET).elf: $(OBJECTS)
	avr-gcc $(LDFLAGS) -mmcu=$(MCU) $(OBJECTS) -o $(BINDIR)/$(TARGET).elf

$(BINDIR)/%.o: $(SRCDIR)/%.c | $(BINDIR)
	avr-gcc $(CFLAGS) -mmcu=$(MCU) $< -o $@

$(BINDIR):
	mkdir $(BINDIR)


size:
	avr-size --mcu=$(MCU) -C $(BINDIR)/$(TARGET).elf

program:
	avrdude -p$(MCU) $(PORT) $(BAUD) -c$(PROGRAMMER) -Uflash:w:$(BINDIR)/$(TARGET)_flash.hex:a

erase:
	avrdude -p$(MCU) $(PORT) $(BAUD) -c$(PROGRAMMER) -e


clean_tmp:
	rm -rf $(BINDIR)/*.o
	rm -rf $(BINDIR)/*.elf

clean:
	rm -rf $(BINDIR)/*.o
	rm -rf $(BINDIR)/*.elf
	rm -rf $(BINDIR)/*.hex 
