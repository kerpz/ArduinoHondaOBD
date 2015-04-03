ArduinoHondaOBD1
===========

Implements Honda OBD1 protocol using Arduino


Supports
--------
* Honda ECU's before 2002


Files
-----
* hobd_elm - implements Honda OBD1 to ELM OBD2 protocol
* hobd_lcd - implements Honda OBD1 to LCD display


Diagram for hobd_elm
--------------------
    Honda 3 Pin DLC           Arduino Uno
    Gnd --------------------- Gnd
    +12 --------------------- Vin
    K-line ------------------ Pin12

    Arduino Uno               HC-05 Bluetooth
    Pin8 -------------------- Tx
    Pin9 -------------------- Rx


Diagram for hobd_lcd:
---------------
    Honda 3 Pin DLC           Arduino Uno
    Gnd --------------------- Gnd
    +12 --------------------- Vin
    K-line ------------------ Pin12

    Arduino Uno               LCD


