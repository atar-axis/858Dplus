#SETTINGS
OUTDIR=bin
TARGET=858Dplus
MCU=atmega328p
SOURCES=main.c watchdog.c

PROGRAMMER=avrispmkii
#auskommentieren für automatische Wahl
PORT=-Pusb
#BAUD=-B115200

#Ab hier nichts verändern
OBJECTS=$(addprefix $(OUTDIR)/, $(SOURCES:.c=.o))
CFLAGS=-c -Os
LDFLAGS=

all: flash eeprom

flash: $(TARGET)_flash.hex

eeprom: $(TARGET)_eeprom.hex


$(TARGET)_flash.hex: $(TARGET).elf
	avr-objcopy -O ihex -j .data -j .text $(OUTDIR)/$(TARGET).elf $(OUTDIR)/$(TARGET)_flash.hex

$(TARGET)_eeprom.hex: $(TARGET).elf
	avr-objcopy -O ihex -j .eeprom --change-section-lma .eeprom=0 $(OUTDIR)/$(TARGET).elf $(OUTDIR)/$(TARGET)_eeprom.hex

$(TARGET).elf: $(OBJECTS)
	avr-gcc $(LDFLAGS) -mmcu=$(MCU) $(OBJECTS) -o $(OUTDIR)/$(TARGET).elf

$(OUTDIR)/%.o: %.c | $(OUTDIR)
	avr-gcc $(CFLAGS) -mmcu=$(MCU) $< -o $@

$(OBJDIR):
	mkdir $(OBJDIR)


size:
	avr-size --mcu=$(MCU) -C $(OUTDIR)/$(TARGET).elf

program:
	avrdude -p$(MCU) $(PORT) $(BAUD) -c$(PROGRAMMER) -Uflash:w:$(OUTDIR)/$(TARGET)_flash.hex:a

erase:
	avrdude -p$(MCU) $(PORT) $(BAUD) -c$(PROGRAMMER) -e


clean_tmp:
	rm -rf *.o
	rm -rf *.elf

clean:
	rm -rf *.o
	rm -rf *.elf
	rm -rf *.hex 
