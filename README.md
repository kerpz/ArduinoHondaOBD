ArduinoHondaOBD1
===========

Implements Honda OBD1 protocol using Arduino


Supports
--------
* Honda ECU's before 2002


Files
-----
* hobd_elm - implements Honda OBD1 to ELM OBD2 protocol (bluetooth)
* hobd_lcd - implements Honda OBD1 to LCD display
* hobd_uni - implements Honda OBD1 with ELM OBD2 protocol (bluetooth) and LCD display
* LCD_wiring.png - LCD wiring for arduino uno (10k potentiometer)


Wiring for hobd_elm
--------------------
    Honda 3 Pin DLC           Arduino Uno
    Gnd --------------------- Gnd
    +12 --------------------- Vin
    K-line ------------------ Pin12

    HC-05 Bluetooth           Arduino Uno               
    Rx ---------------------- Pin11
    Tx ---------------------- Pin10


Wiring for hobd_lcd:
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


Wiring for hobd_uni
--------------------
    Honda 3 Pin DLC           Arduino Uno
    Gnd --------------------- Gnd
    +12 --------------------- Vin
    K-line ------------------ Pin12

    HC-05 Bluetooth           Arduino Uno               
    Rx ---------------------- Pin11
    Tx ---------------------- Pin10

    LCD 2x16                  Arduino Uno               
    RS ---------------------- Pin9
    Enable ------------------ Pin8
    D4 ---------------------- Pin7
    D5 ---------------------- Pin6
    D6 ---------------------- Pin5
    D7 ---------------------- Pin4


TODO
-----
* ECU detection (Obd1 or Obd2)
* Fault codes reader
* Images :)
