ArduinoHondaOBD
===========

An arduino code that reads Honda OBD Protocol and translates it to ELM327 protocol.
I use Torque app to read and display the data on my android phone (via bluetooth),
and a LCD (2x6) to display it on my car's dashboard.

Please refer to the screenshots below.


Supports
--------
* Honda ECU's before 2002


Files
-----
* hobd_uni - unified code for ELM bluetooth and LCD display with other improvements.
* hobd_elm - implements Honda OBD to ELM OBD2 protocol (bluetooth) - not updated
* hobd_lcd - implements Honda OBD to LCD display - not updated
* UNI_wiring.png - Unified wiring diagram for arduino UNO (compatible)


Wiring for hobd_uni (Joined ELM and LCD codes)
--------------------
    Honda 3 Pin DLC           Arduino Uno
    Gnd --------------------- Gnd
    +12 --------------------- Vin
    K-line ------------------ Pin12

    HC-05 Bluetooth           Arduino Uno               
    Rx ---------------------- Pin11
    Tx ---------------------- Pin10

    LCD 16x2                  Arduino Uno               
    RS ---------------------- Pin9
    Enable ------------------ Pin8
    D4 ---------------------- Pin7
    D5 ---------------------- Pin6
    D6 ---------------------- Pin5
    D7 ---------------------- Pin4
    VO ---------------------- 10k Potentiometer (+5V to Gnd)

    Piezo Buzzer              Arduino Uno               
    (+) --------------------- Pin13
    (-) --------------------- Gnd

    Tact Switch               Arduino Uno               
    (+) --------------------- Pin17 (A3)
    (-) --------------------- Gnd

    Voltage Divider           Arduino Uno               
    +12V divider circuit ---- Pin14 (A0)
    (680k ohms and 220k ohms)
    

![Alt text](https://raw.github.com/kerpz/ArduinoHondaOBD/master/images/UNI_wiring.png "UNI Wiring Image")

Wiring for hobd_elm (Deprecated use hobd_uni)
--------------------
    Honda 3 Pin DLC           Arduino Uno
    Gnd --------------------- Gnd
    +12 --------------------- Vin
    K-line ------------------ Pin12

    HC-05 Bluetooth           Arduino Uno               
    Rx ---------------------- Pin11
    Tx ---------------------- Pin10


Wiring for hobd_lcd (Deprecated use hobd_uni)
---------------
    Honda 3 Pin DLC           Arduino Uno
    Gnd --------------------- Gnd
    +12 --------------------- Vin
    K-line ------------------ Pin12

    LCD 16x2                  Arduino Uno               
    RS ---------------------- Pin9
    Enable ------------------ Pin8
    D4 ---------------------- Pin7
    D5 ---------------------- Pin6
    D6 ---------------------- Pin5
    D7 ---------------------- Pin4
    VO ---------------------- 10k Potentiometer (+5V to Gnd)


Screenshots (LCD 16x2)
---------------

![Alt text](https://raw.github.com/kerpz/ArduinoHondaOBD/master/images/LCD_01.png "LCD Screenshot 01")
![Alt text](https://raw.github.com/kerpz/ArduinoHondaOBD/master/images/LCD_02.png "LCD Screenshot 02")
![Alt text](https://raw.github.com/kerpz/ArduinoHondaOBD/master/images/LCD_03.png "LCD Screenshot 03")

Screenshots (Andorid App TORQUE)
---------------

![Alt text](https://raw.github.com/kerpz/ArduinoHondaOBD/master/images/TORQUE_01.png "TORQUE Screenshot 01")
![Alt text](https://raw.github.com/kerpz/ArduinoHondaOBD/master/images/TORQUE_02.png "TORQUE Screenshot 02")
![Alt text](https://raw.github.com/kerpz/ArduinoHondaOBD/master/images/TORQUE_04.png "TORQUE Screenshot 04")
![Alt text](https://raw.github.com/kerpz/ArduinoHondaOBD/master/images/TORQUE_03.png "TORQUE Screenshot 03")

NOTES
-----
* Added button for changing page (5ms) and changing ecu mode (3s).
* Added Fault codes reader @ lcd page 3
* Added LCD @ I2C support
  - #define LCD_i2c TRUE // Using LCD 16x2 I2C mode
* Added direct HOBD access API (ELM mode) used for undefined PIDs on OBD2
  - // direct honda PID access
  - // 1 byte access (21AA) // 21 = 1 byte, AA = address
  - // 2 bytes access (22AA) // 22 = 2 bytes, AA = address
  - // 4 byte access (24AA) // 24 = 4 bytes, AA = address
* Added direct Arduino Pin access API (ELM mode) used for keyless entry and other stuff
  - // set arduino pin (ATSAPNNX) // NN = pin, X = value (value is 1 or 0)
  - // get arduino pin (ATDAPNN) // NN = pin
* Tested on P2T ODB2 stock and P30 ODB1 chipped

TODO
-----
* Add 128x64 LCD @ SPI support
* Add 20x4 LCD @ I2C support
