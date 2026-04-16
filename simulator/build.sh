#!/bin/bash
# Build PC host simulators (Windows + MSYS2 UCRT64)
#
# Usage:
#   ./build.sh                 # build all SDL2 host instruments
#   ./build.sh altimeter_host  # build a single target
#
# Requires MSYS2 with mingw-w64-ucrt-x86_64-gcc and SDL2 packages installed.
# *_pi.c sources are NOT built here — they target the Raspberry Pi.

set -e

MSYS=/c/msys64/ucrt64
export PATH="$MSYS/bin:$PATH"

CC="$MSYS/bin/gcc.exe"
CFLAGS="-O2 -I$MSYS/include/SDL2"
LDFLAGS="-L$MSYS/lib"
SDL_LIBS="-lmingw32 -lSDL2main -lSDL2"
WIN_LIBS="-lws2_32"
MATH_LIB="-lm"

SDL_HOSTS="altimeter_host asi_host vsi_host hsi_host adi_host"

cd "$(dirname "$0")"

build_host() {
    local name="$1"
    local exe="$name.exe"
    echo "Building $exe..."
    # On Windows a running .exe can't be overwritten, but it CAN be renamed.
    # Move the old one aside first so the link step always succeeds.
    if [ -f "$exe" ]; then
        rm -f "$exe.old" 2>/dev/null || true
        mv "$exe" "$exe.old" 2>/dev/null || true
    fi
    $CC $CFLAGS -o "$exe" "$name.c" $LDFLAGS $SDL_LIBS $WIN_LIBS $MATH_LIB
    rm -f "$exe.old" 2>/dev/null || true   # cleanup; OS keeps it open if running
}

if [ $# -eq 0 ]; then
    for h in $SDL_HOSTS; do build_host "$h"; done
else
    for h in "$@"; do build_host "$h"; done
fi

echo "Done."
