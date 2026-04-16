# Makefile for PC host simulators (Windows + MSYS2 UCRT64)
#
# Usage:
#   mingw32-make                  # build all host SDL2 simulators
#   mingw32-make altimeter_host   # build a single target
#   mingw32-make clean
#
# Requires MSYS2 with mingw-w64-ucrt-x86_64-gcc and SDL2 packages installed.
# The *_pi.c sources are NOT built here — they target the Raspberry Pi (Linux/DRM).

MSYS    = C:/msys64/ucrt64
CC      = $(MSYS)/bin/gcc.exe
CFLAGS  = -O2 -I$(MSYS)/include/SDL2
LDFLAGS = -L$(MSYS)/lib
SDL_LIBS = -lmingw32 -lSDL2main -lSDL2
WIN_LIBS = -lws2_32
MATH_LIB = -lm

SDL_HOSTS = altimeter_host.exe asi_host.exe vsi_host.exe hsi_host.exe adi_host.exe
TOOLS     = check_strip.exe gen_baro_strip.exe vsi_cal.exe

all: $(SDL_HOSTS)

# SDL2 + winsock UDP host instruments
%_host.exe: %_host.c
	$(CC) $(CFLAGS) -o $@ $< $(LDFLAGS) $(SDL_LIBS) $(WIN_LIBS) $(MATH_LIB)

# Plain C utilities (no SDL, no sockets)
check_strip.exe: check_strip.c
	$(CC) $(CFLAGS) -o $@ $<

gen_baro_strip.exe: gen_baro_strip.c
	$(CC) $(CFLAGS) -o $@ $< $(MATH_LIB)

vsi_cal.exe: vsi_cal.c
	$(CC) $(CFLAGS) -o $@ $< $(MATH_LIB)

tools: $(TOOLS)

clean:
	rm -f $(SDL_HOSTS) $(TOOLS)

.PHONY: all clean tools
