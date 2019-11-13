#!/bin/sh
#ffmpeg -i Tile.png -f rawvideo -pix_fmt rgb565 tile.raw
/home/Peruri/esp/esplay-base-firmware/tools/mkfw/mkfw.exe OGO-Shell media/tile.raw 0 16 1048576 ogo-shell build/ogo-shell.bin 0 17 655360 nesemu /home/Peruri/esp/esplay-retro-emulation/esplay-nofrendo/build/esplay-nofrendo.bin 0 18 655360 gnuboy /home/Peruri/esp/esplay-retro-emulation/esplay-gnuboy/build/esplay-gnuboy.bin 0 19 1376256 smsplusgx /home/Peruri/esp/esplay-retro-emulation/esplay-smsplusgx/build/esplay-smsplusgx.bin
rm ogo-shell.fw
mv firmware.fw ogo-shell.fw
