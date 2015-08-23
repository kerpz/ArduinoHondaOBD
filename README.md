ArduinoHondaOBD
===========

An arduino code that reads Honda OBD Protocol and translates to ELM327 protocol.
I use Torque app (android) to read and display data via bluetooth, and
LCD to display basic data such as RPM, VSS, ECT, IAT, TPS, MAP, and VOLTS.


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
![Alt text](https://raw.github.com/kerpz/ArduinoHondaOBD/master/images/TORQUE_03.png "TORQUE Screenshot 03")

TODO
-----
* ECU detection (Obd1 or Obd2)
* Fault codes reader
