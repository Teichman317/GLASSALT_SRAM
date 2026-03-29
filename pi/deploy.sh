#!/bin/bash
#
# Deploy GLASSALT instrument simulators to Raspberry Pi
# Run from MSYS2 terminal on Windows:
#   cd /d/Dev/GLASSALT_SRAM
#   bash pi/deploy.sh
#
# Optional: pass Pi address as argument:
#   bash pi/deploy.sh machiii@192.168.1.100

PI="${1:-machiii@192.168.1.129}"
DEST="~/glassalt"

echo "=== Deploying to $PI:$DEST ==="

# Create directory structure on Pi
echo "Creating directories..."
ssh "$PI" "mkdir -p $DEST/simulator '$DEST/Vertical speed' $DEST/HSI"

# Copy Makefile
echo "Copying Makefile..."
scp pi/Makefile "$PI:$DEST/Makefile"

# Copy source files (no changes needed — pure C + SDL2)
echo "Copying source files..."
scp simulator/main.c simulator/vsi_sim.c simulator/hsi_sim.c \
    "$PI:$DEST/simulator/"

# Copy altimeter assets
echo "Copying altimeter assets..."
scp image.bin 100sWheel.bin pointerargb4444.bin \
    1000sWheel.bin 10000sWheel.bin \
    "$PI:$DEST/"

# Copy VSI assets
echo "Copying VSI assets..."
scp "Vertical speed/vsi_bg.bin" "Vertical speed/vsi_pointer.bin" \
    "$PI:$DEST/Vertical speed/"

# Copy HSI assets
echo "Copying HSI assets..."
scp HSI/background.bin HSI/compass_card.bin HSI/cdi_dots.bin \
    HSI/course_pointer.bin HSI/cdi_bar.bin HSI/heading_bug.bin \
    HSI/lubber_aircraft.bin \
    "$PI:$DEST/HSI/"

echo ""
echo "=== Deploy complete ==="
echo ""
echo "Now SSH into the Pi and build:"
echo "  ssh $PI"
echo "  sudo apt update && sudo apt install -y libsdl2-dev   # first time only"
echo "  cd ~/glassalt"
echo "  make"
echo ""
echo "Run an instrument:"
echo "  cd simulator"
echo "  ./altimeter_sim"
echo "  ./vsi_sim"
echo "  ./hsi_sim"
echo ""
echo "If running headless (no desktop), use KMS/DRM:"
echo "  SDL_VIDEODRIVER=kmsdrm ./hsi_sim"
