## AQMS-600

### AQMS-600 Rs232 Configuration
- DB9 male connector.
- Baud rate: 57600
- Data bits: 8 data bits with 1 stop bit.
- Parity: None

### Communication Flow
1. Initial Setup 
OneOpenAir.ino -> boardInit() line 829

2. Detect Device
OneOpenAir.ino -> line 359
3. Request NOx concentration parameters 
OneOpenAir.ino -> line 360
    1. Send read command for NOx (cmd = 0x3f) <br>
    aqms600.cpp -> line 40
    2. Receive data <br>
    aqms600.cpp -> line 41
    3. Parse the data <br>
    aqms600.cpp -> line 42

6. Print raw data to serial port
    AgValue.cpp -> printCurrentAverage() line 130
7. Upload to Dashboard  -> cilentPost/httpPostMeasure
    OneOpenAir.ino -> line 1240