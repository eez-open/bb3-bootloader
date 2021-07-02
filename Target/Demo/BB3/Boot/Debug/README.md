## Experimental SD card boot loader for BB3

Erase MCU  

Flash BB3_SD_bootloader.elf

After power up boot loader will flash screen very fast 10ms ON/OFF

It will search for o.s file in root of SD card - if exists it will flash    

For the first time put o.s file to SD card manually

After boot loader finish - it needs approximately 2 minutes it will boot into app

Next time you can send new FW over eez Studio

Boot loader needs special fw with some changes - stock FW will not boot  

For test you can reload same FW
