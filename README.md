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
* hobd_uni - implements Honda OBD with ELM OBD2 protocol (bluetooth) and LCD display
* hobd_elm - implements Honda OBD to ELM OBD2 protocol (bluetooth)
* hobd_lcd - implements Honda OBD to LCD display
* LCD_wiring.png - LCD wiring for arduino uno (10k potentiometer)


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

    LCD 2x16                  Arduino Uno               
    RS ---------------------- Pin9
    Enable ------------------ Pin8
    D4 ---------------------- Pin7
    D5 ---------------------- Pin6
    D6 ---------------------- Pin5
    D7 ---------------------- Pin4


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

TODO
-----
* ECU detection (Obd1 or Obd2) (How?)
* Fault codes reader (Work on progress)
* Navigation buttons using Analog input (up,down,left,right,select/enter) (DFRobot style)
* Add 128x64 LCD @ SPI support
* Add 20x4 LDC @ I2C support
* Integrate smart keyless entry via smart phone (Optional)
