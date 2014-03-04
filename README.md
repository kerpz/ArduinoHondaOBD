ArduinoHOBD
===========

Implements Honda OBD protocol using arduino


Supports:
Honda ECU's before 2002


Files:
hobd_elm - implements hobd to elm obd2 protocol
hobd_lcd - implements hobd to lcd display


Diagram for hobd_elm:

Honda 3 Pin DLC           Arduino Uno
Gnd --------------------- Gnd
+12 --------------------- Vin
K-line ------------------ Pin12

Arduino Uno               HC-05 Bluetooth
Pin8 -------------------- Tx
Pin9 -------------------- Rx


Diagram for hobd_lcd:

Honda 3 Pin DLC           Arduino Uno
Gnd --------------------- Gnd
+12 --------------------- Vin
K-line ------------------ Pin12

Arduino Uno               LCD


