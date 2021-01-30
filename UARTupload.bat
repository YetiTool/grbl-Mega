@echo off

REM Run this script from cmd line starting at \YETI drive\Engineering\Firmware\Production\
REM update relevant PORT and FIRMWARE variables below. Make sure to use the hex file with bootloader (named GRBLXXXandBL.hex)

REM Arduino as ISP takes 96s to complete
REM USPasp takes 40s to complete 375kHz
REM Pololu ISP emulator takes 21s to complete at 200kHz
REM Pololu ISP emulator takes 10s to complete at 2000kHz

SET PORT=COM6
REM SET FIRMWARE=GRBL_1.0.4\GRBL104andBL.hex
SET FIRMWARE=Debug\grbl_mega.hex 
SET AVRDUDE=C:\Yeti Drive\Engineering\Firmware\Production\tools\6.3.0-arduino14

REM SET PROGRAMMER=-cavrisp -P%PORT%
REM SET PROGRAMMER=-cusbasp-clone
REM SET PROGRAMMER=-c avrisp2 -P%PORT%
SET PROGRAMMER=-cwiring -P%PORT% -b115200 -D


echo """"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
echo "   __     ________ _______ _____     ______ _           _                 "
echo "   \ \   / /  ____|__   __|_   _|   |  ____| |         | |                "
echo "    \ \_/ /| |__     | |    | |     | |__  | | __ _ ___| |__   ___ _ __   "
echo "     \   / |  __|    | |    | |     |  __| | |/ _` / __| '_ \ / _ \ '__|  "
echo "      | |  | |____   | |   _| |_    | |    | | (_| \__ \ | | |  __/ |     "
echo "      |_|  |______|  |_|  |_____|   |_|    |_|\__,_|___/_| |_|\___|_|     "
echo "                                                                          "
echo """"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""

REM m2560: nano, UNO
REM "%AVRDUDE%\bin\avrdude" -C"%AVRDUDE%\etc\avrdude.conf" -q -q %PROGRAMMER% -p m2560 -U lfuse:w:0xff:m -U hfuse:w:0xd8:m -U efuse:w:0xfd:m
"%AVRDUDE%\bin\avrdude" -C"%AVRDUDE%\etc\avrdude.conf" %PROGRAMMER% -p m2560 -U flash:w:%FIRMWARE%
REM "%AVRDUDE%\bin\avrdude" -C"%AVRDUDE%\etc\avrdude.conf" -q %PROGRAMMER% -p m2560 -U lock:w:0x2f:m
