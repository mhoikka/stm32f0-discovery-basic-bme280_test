How to use this project:
-Fabricate a PCB using the Gerber and Drill files in the FabricationFiles folder
-Buy all components in the Components/AllComponents.csv file (breakout boards can be used for the NRF24L01+ and BME280 chips if soldering skills and access to a hot air gun are limiting factors. There is no need to solder on any of the components attached to the NRF and BME if the breakout boards are used.)
-Solder on all components according to the KiCAD schematic
-Run raspberry pi code in linked GitHub repo (https://github.com/mhoikka/SPItoNRF.git)
-Insert 2 CR2032 batteries to power on the device
-Collect data and modify code as needed