# 858Dplus

This is an alternative firmware for the 858D+,
it is based on madworms youyue-858D-plus repo, which is located here: https://github.com/madworm/Youyue-858D-plus

The main difference is, that this variant is pure C without all those Arduino specific things.

## Compilation

To compile the firmware on linux, you need at least  
`sudo apt-get install avr-libc gcc-avr make build-essential`

All you need to do is running `make` in the directory where the Makefile is.

## Upload

Upload using e.g. `avr-dude`, fuses are described here:  
https://www.eevblog.com/forum/reviews/youyue-858d-some-reverse-engineering-custom-firmware/msg1320798/#msg1320798
