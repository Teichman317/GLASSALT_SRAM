## GLASSALT Instrument Panel — Raspberry Pi Build
## Run from ~/glassalt/ on the Pi

CC      = gcc
CFLAGS  = -O2 $(shell sdl2-config --cflags)
LDFLAGS = $(shell sdl2-config --libs) -lm

all: simulator/panel_pi

simulator/panel_pi: simulator/panel_pi.c
	$(CC) $(CFLAGS) -o $@ $< $(LDFLAGS)

clean:
	rm -f simulator/panel_pi

.PHONY: all clean
