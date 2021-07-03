## Experimental SD card boot loader for BB3

Erase MCU  

Flash BB3_SD_bootloader.elf

Bootloader will search for o.s file in root of SD card - if exists it will start flashing    

Wait around 2,5 minutes until progress bar reach max 

For the first time put o.s file to SD card manually

Next time you can send new FW over eez Studio

Boot loader needs special fw with some changes - stock FW will not boot  

For test you can reload same FW
