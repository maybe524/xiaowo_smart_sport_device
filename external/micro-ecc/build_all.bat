@ECHO OFF

:: This script will use git (must be in %PATH%) and arm-none-eabi tools in combination with GNU Make
:: to both fetch and compile all variants of micro-ecc for the nRF5 families

WHERE >nul 2>nul git
IF %ERRORLEVEL% NEQ 0 (
    ECHO "git is not installed. Please install and append to PATH."
)

IF NOT EXIST micro-ecc/uECC.c (
    ECHO "micro-ecc not found! Let's pull it from HEAD."
    git clone https://github.com/kmackay/micro-ecc.git
)

C:\MinGW\bin\make.exe -C nrf51_armgcc/armgcc
C:\MinGW\bin\make.exe -C nrf51_iar/armgcc
C:\MinGW\bin\make.exe -C nrf51_keil/armgcc
C:\MinGW\bin\make.exe -C nrf52hf_armgcc/armgcc
C:\MinGW\bin\make.exe -C nrf52hf_iar/armgcc
C:\MinGW\bin\make.exe -C nrf52hf_keil/armgcc
C:\MinGW\bin\make.exe -C nrf52nf_armgcc/armgcc
C:\MinGW\bin\make.exe -C nrf52nf_iar/armgcc
C:\MinGW\bin\make.exe -C nrf52nf_keil/armgcc
