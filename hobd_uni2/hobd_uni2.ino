/*
 Author:
 - Philip Bordado (kerpz@yahoo.com)

 Hardware:
 - Arduino Nano (Compatible board)
 - HC-05 Bluetooth module
 - Resistors 680k ohms and 220k ohms
 - Piezo Buzzer
 - LCD 16x2 I2C
 Optional:
 - 100 psi transducer for fuel pressure (0.5v - 4.5v)
 - AEM AFR UEGO
 - 10K NTC Thermistor

 Software:
 - Arduino 1.8.19 (Stable)
 - SoftwareSerialWithHalfDuplex (Library)
   https://github.com/nickstedman/SoftwareSerialWithHalfDuplex

 Formula:
 - IMAP = RPM * MAP / IAT / 2
 - MAF = (IMAP/60)*(VE/100)*(Eng Disp)*(MMA)/(R)
   Where: VE = 80% (Volumetric Efficiency), R = 8.314 J/°K/mole, MMA = 28.97 g/mole (Molecular mass of air)
   http://www.lightner.net/obd2guru/IMAP_AFcalc.html
   http://www.installuniversity.com/install_university/installu_articles/volumetric_efficiency/ve_computation_9.012000.htm

 Arduino Pin Mapping / Configuration:
 * Pulse Input - 50k ohms over 5.1v zener
 $ Switch Out - 1k ohms on NPN transistor (2n3904)
 
 - 00 = Serial RX
 - 01 = Serial TX
 - 02 = * Injector Input
 - 03 = * Injector Input
 - 04 = $ Door Lock | * RPM Input
 - 05 = $ Door Unlock | * VSS Input
 - 06 = $ AC Compressor
 - 07 = 
 - 08 = * Door Sensor
 - 09 = AEM AFR UEGO (Serial In)
 - 10 = Bluetooth RX
 - 11 = Bluetooth TX
 - 12 = K-Line (Serial In)
 - 13 = Piezo buzzer (+)

 - 14 = (A0) Voltage divider (Input Signal)
 - 15 = (A1) 100 PSI Fuel Pressure
 - 16 = (A2) AEM AFR UEGO (ADC)
 - 17 = (A3) NTC Thermistor
 - 18 = (A4) I2C / LCD I2C
 - 19 = (A5) I2C / LCD I2C
 - 20 = (A6) Navigation Button (reason: multiple button using resistors)
 - 21 = (A7)
*/

#define APPNAME "Honda UNI   v1.0" // 16 chars

#include <EEPROM.h>

// OUT
#define PIN_LOCK 4
#define PIN_UNLOCK 5
#define PIN_AC 6
#define PIN_BUZZER 13
// IN
#define PIN_DOOR 8
#define PIN_VOLT 14
#define PIN_FP 15
#define PIN_AFR 16
#define PIN_TH 17
#define PIN_BUTTON 20

#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 16, 2);

// comment the PCINT1_vect,PCINT2_vect,PCINT3_vect handle in softserial library
// since we are just using D10,D11,D12 and we want to handle interrupts @ A0 - A5
// PCINT0_vect  // D8 - D13
// PCINT1_vect  // A0 - A5
// PCINT2_vect  // D0 - D7
// PCINT3_vect 

#include <SoftwareSerialWithHalfDuplex.h>

SoftwareSerialWithHalfDuplex btSerial(10, 11); // RX, TX
SoftwareSerialWithHalfDuplex dlcSerial(12, 12, false, false);
//SoftwareSerialWithHalfDuplex aemSerial(9, 9, false, false);

bool elm_mode = false;
bool elm_memory = false;
bool elm_echo = false;
bool elm_space = true;
bool elm_linefeed = true;
bool elm_header = false;
byte elm_protocol = 0; // auto

// ecu data
int rpm=0, ect=0, iat=0,maps=0, baro=0, tps=0, volt=0, imap=0, sft=0, lft=0, inj=0, ign=0, lmt=0, iac=0, knoc=0;
// extra sensor data
float volt2,th,afr,fp;
bool cp;
int btn;

int rpmtop=0,volttop=0,mapstop=0,tpstop=0,ecttop=0,iattop=0;

unsigned long vsssum=0,running_time=0,idle_time=0,distance=0;
byte vss=0,vsstop=0,vssavg=0;


byte obd_select = 2; // 1 = obd1, 2 = obd2
byte pag_select = 0; // lcd page

byte ect_alarm = 98; // celcius
byte vss_alarm = 100; // kph
byte th_threshold = 4; // celcius / + 10

bool isButtonPressed = false;

// voltage divider
//float R1 = 30000.0;
//float R2 = 7500.0;
float R1 = 680000.0; // Resistance of R1 (680kohms)
float R2 = 220000.0; // Resistance of R2 (220kohms)
float R3 = 10000.0; // Thermistor divider

unsigned long err_timeout = 0, err_checksum = 0, ect_cnt = 0, vss_cnt = 0;

byte dlcdata[20] = {0};  // dlc data buffer

int dtcErrors[14], dtcCount = 0;


void bt_write(char *str) {
  char c = *str;
  while (*str != '\0') {
    if (!elm_linefeed && *str == 10) *str++; // skip linefeed for all reply
    if (c == '4' && !elm_space && *str == 32) *str++; // skip space for obd reply
    btSerial.write(*str++);
  }
}

void dlcInit() {
  dlcSerial.write(0x68);
  dlcSerial.write(0x6a);
  dlcSerial.write(0xf5);
  dlcSerial.write(0xaf);
  dlcSerial.write(0xbf);
  dlcSerial.write(0xb3);
  dlcSerial.write(0xb2);
  dlcSerial.write(0xc1);
  dlcSerial.write(0xdb);
  dlcSerial.write(0xb3);
  dlcSerial.write(0xe9);
  delay(300);
}

int dlcCommand(byte cmd, byte num, byte loc, byte len) {
  byte crc = (0xFF - (cmd + num + loc + len - 0x01)); // checksum FF - (cmd + num + loc + len - 0x01)

  unsigned long timeOut = millis() + 200; // timeout @ 200 ms

  memset(dlcdata, 0, sizeof(dlcdata));

  dlcSerial.listen();

  dlcSerial.write(cmd);  // header/cmd read memory ??
  dlcSerial.write(num);  // num of bytes to send
  dlcSerial.write(loc);  // address
  dlcSerial.write(len);  // num of bytes to read
  dlcSerial.write(crc);  // checksum
  
  int i = 0;
  while (i < (len+3) && millis() < timeOut) {
    if (dlcSerial.available()) {
      dlcdata[i] = dlcSerial.read();
      //delay(1); // this is required
      i++;
    }
  }

  if (i < (len+3)) { // timeout
    err_timeout++;
    return 0;  // data error
  }
  // checksum
  crc = 0;
  for (i=0; i<len+2; i++) {
    crc = crc + dlcdata[i];
  }
  crc = 0xFF - (crc - 1);
  if (crc != dlcdata[len+2]) { // checksum failed
    err_checksum++;
    return 0; // data error
  }
  return 1; // success
}

long readVcc()
{
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
  #if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
    ADMUX = _BV(MUX5) | _BV(MUX0);
  #elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
    ADMUX = _BV(MUX3) | _BV(MUX2);
  #else
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #endif  
 
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA,ADSC)); // measuring
 
  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH  
  uint8_t high = ADCH; // unlocks both
 
  long vcc = (high<<8) | low;
  vcc = 1125300L / vcc; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  return vcc;
}

// bluetooth routines
void procbtSerial() {
    char btdata1[20]={0};  // bt data in buffer
    char btdata2[20]={0};  // bt data out buffer
    int i = 0;
  
    while (btSerial.available()) {
      btdata1[i] = toupper(btSerial.read());
      
      delay(1); // this is required
      
      if (btdata1[i] == '\r') { // terminate at \r
        btdata1[i] = '\0';
  
        byte len = strlen(btdata1);
        if (!strcmp(btdata1, "ATD")) { // defaults
          elm_echo = false;
          elm_space = true;
          elm_linefeed = true;
          elm_header = false;
          sprintf_P(btdata2, PSTR("OK\r\n>"));
        }
        else if (!strcmp(btdata1, "ATI")) { // print id / general
          sprintf_P(btdata2, "%s\r\n>", APPNAME);
        }
        else if (!strcmp(btdata1, "ATZ")) { // reset all / general
          elm_echo = false;
          elm_space = true;
          elm_linefeed = true;
          elm_header = false;
          sprintf_P(btdata2, "%s\r\n>", APPNAME);
        }
        else if (len == 4 && strstr(btdata1, "ATE")) { // echo on/off / general
          elm_echo = (btdata1[3] == '1' ? true : false);
          sprintf_P(btdata2, PSTR("OK\r\n>"));
        }
        else if (len == 4 && strstr(btdata1, "ATL")) { // linfeed on/off / general
          elm_linefeed = (btdata1[3] == '1' ? true : false);
          sprintf_P(btdata2, PSTR("OK\r\n>"));
        }
        else if (len == 4 && strstr(btdata1, "ATM")) { // memory on/off / general
          //elm_memory = (btdata1[3] == '1' ? true : false);
          sprintf_P(btdata2, PSTR("OK\r\n>"));
        }
        else if (len == 4 && strstr(btdata1, "ATS")) { // space on/off / obd
          elm_space = (btdata1[3] == '1' ? true : false);
          sprintf_P(btdata2, PSTR("OK\r\n>"));
        }
        else if (len == 4 && strstr(btdata1, "ATH")) { // headers on/off / obd
          //elm_header = (btdata1[3] == '1' ? true : false);
          sprintf_P(btdata2, PSTR("OK\r\n>"));
        }
        else if (len == 5 && strstr(btdata1, "ATSP")) { // set protocol to ? and save it / obd
          //elm_protocol = atoi(btdata1[4]);
          sprintf_P(btdata2, PSTR("OK\r\n>"));
        }
        else if (!strcmp(btdata1, "ATDP")) { // display protocol / obd
          sprintf_P(btdata2, PSTR("AUTO\r\n>"));
        }
        else if (!strcmp(btdata1, "ATRV")) { // read voltage in float / volts
          //btSerial.print("12.0V\r\n>");
          byte v1 = 0, v2 = 0;
          //unsigned int volt2 = round(readVoltageDivider(14) * 10); // to cV
          long vcc = readVcc(); // in mV
          unsigned int volt2 = round((((analogRead(PIN_VOLT) * vcc) / 1024.0) / (R2/(R1+R2))) * 10);
          v1 = volt2 / 10;
          v2 = volt2 % 10;
          sprintf_P(btdata2, PSTR("%d.%dV\r\n>"), v1, v2);
        }
        // kerpz custom AT cmd
        else if (len == 6 && strstr(btdata1, "ATSHP")) { // set hobd protocol
          if (btdata1[5] == '1') { obd_select = 1; }
          if (btdata1[5] == '2') { obd_select = 2; }
          EEPROM.write(0, obd_select);
          sprintf_P(btdata2, PSTR("OK\r\n>"));
        }
        else if (!strcmp(btdata1, "ATDHP")) { // display hobd protocol
          sprintf_P(btdata2, PSTR("HOBD%d\r\n>"), obd_select);
        }
        else if (strstr(btdata1, "ATSAP")) { // set arduino pin value (1=hi,0=lo,T=toggle)
          byte pin = ((btdata1[5] > '9')? (btdata1[5] &~ 0x20) - 'A' + 10: (btdata1[5] - '0') * 16) +
                      ((btdata1[6] > '9')? (btdata1[6] &~ 0x20) - 'A' + 10: (btdata1[6] - '0'));
          if (btdata1[7] == 'T') { digitalWrite(pin, !digitalRead(pin)); }
          else { digitalWrite(pin, btdata1[7]); }
          
          sprintf_P(btdata2, PSTR("OK\r\n>"));
        }
        else if (strstr(btdata1, "ATDAP")) { // display arduino pin value (1=hi,0=lo)
          byte pin = ((btdata1[5] > '9')? (btdata1[5] &~ 0x20) - 'A' + 10: (btdata1[5] - '0') * 16) +
                      ((btdata1[6] > '9')? (btdata1[6] &~ 0x20) - 'A' + 10: (btdata1[6] - '0'));
          sprintf_P(btdata2, PSTR("%d\r\n>"), digitalRead(pin));
        }
        else if (strstr(btdata1, "ATPAP")) { // push arduino pin high for 1sec // used for locking/unlocking door
          byte pin = ((btdata1[5] > '9')? (btdata1[5] &~ 0x20) - 'A' + 10: (btdata1[5] - '0') * 16) +
                      ((btdata1[6] > '9')? (btdata1[6] &~ 0x20) - 'A' + 10: (btdata1[6] - '0'));
          pushPinHi(pin, 1000);

          sprintf_P(btdata2, PSTR("OK\r\n>"));
        }
        
        // https://en.wikipedia.org/wiki/OBD-II_PIDs
        // sprintf_P(cmd_str, PSTR("%02X%02X\r"), mode, pid);
        // sscanf(btdata1, "%02X%02X", mode, pid)
        else if (len == 2 && btdata1[0] == '0' && btdata1[1] == '4') { // mode 04
          // clear dtc / stored values
          // reset dtc/ecu honda
          // 21 04 01 DA / 01 03 FC
          dlcCommand(0x21, 0x04, 0x01, 0x00); // reset ecu
          sprintf_P(btdata2, PSTR("OK\r\n>"));
        }
        else if (len == 2 && btdata1[0] == '0' && btdata1[1] == '3') { // mode 03
          // do scan then report the errors
          // 43 01 33 00 00 00 00 = P0133
          //sprintf_P(btdata2, PSTR("43 01 33 00 00 00 00\r\n>"), a);
          //sprintf_P(btdata2, PSTR("OK\r\n>"));
        }
        else if (len <= 5 && btdata1[0] == '0' && btdata1[1] == '1') { // mode 01
          // multi pid 010C0B0D040E05
          if (strstr(&btdata1[2], "00")) {
            sprintf_P(btdata2, PSTR("41 00 BE 3E B0 11\r\n>"));
          }
          else if (strstr(&btdata1[2], "01")) {
            // dtc / AA BB CC DD / A7 = MIL on/off, A6-A0 = DTC_CNT
            if (dlcCommand(0x20, 0x05, 0x0B, 0x01)) {
              byte v = ((dlcdata[2] >> 5) & 1) << 7; // get bit 5 on dlcdata[2], set it to a7
              sprintf_P(btdata2, PSTR("41 01 %02X 00 00 00\r\n>"), v);
            }
          }
          /*
          else if (strstr(&btdata1[2], "02")) { // freeze dtc / 00 61 ???
            if (dlcCommand(0x20, 0x05, 0x98, 0x02)) {
              sprintf_P(btdata2, PSTR("41 02 %02X %02X\r\n>"), dlcdata[2], dlcdata[3]);
            }
          }
          else if (strstr(&btdata1[2], "03")) { // fuel system status / 01 00 ???
            //if (dlcCommand(0x20, 0x05, 0x0F, 0x01)) { // flags
            //  byte a = dlcdata[2] & 1; // get bit 0 on dlcdata[2]
            //  a = (dlcdata[2] == 1 ? 2 : 1); // convert to comply obd2
            //  sprintf_P(btdata2, PSTR("41 03 %02X 00\r\n>"), a);
            // }
            if (dlcCommand(0x20, 0x05, 0x9a, 0x02)) {
              sprintf_P(btdata2, PSTR("41 03 %02X %02X\r\n>"), dlcdata[2], dlcdata[3]);
            }
          }
          else if (strstr(&btdata1[2], "04")) { // engine load (%)
            if (dlcCommand(0x20, 0x05, 0x9c, 0x01)) {
              sprintf_P(btdata2, PSTR("41 04 %02X\r\n>"), dlcdata[2]);
            }
          }
          */
          else if (strstr(&btdata1[2], "05")) { // ect (°C)
            if (dlcCommand(0x20, 0x05, 0x10, 0x01)) {
              float f = dlcdata[2];
              f = 155.04149 - f * 3.0414878 + pow(f, 2) * 0.03952185 - pow(f, 3) * 0.00029383913 + pow(f, 4) * 0.0000010792568 - pow(f, 5) * 0.0000000015618437;
              dlcdata[2] = round(f) + 40; // A-40
              sprintf_P(btdata2, PSTR("41 05 %02X\r\n>"), dlcdata[2]);
            }
          }
          else if (strstr(&btdata1[2], "06")) { // short FT (%)
            if (dlcCommand(0x20, 0x05, 0x20, 0x01)) {
              sprintf_P(btdata2, PSTR("41 06 %02X\r\n>"), dlcdata[2]);
            }
          }
          else if (strstr(&btdata1[2], "07")) { // long FT (%)
            if (dlcCommand(0x20, 0x05, 0x22, 0x01)) {
              sprintf_P(btdata2, PSTR("41 07 %02X\r\n>"), dlcdata[2]);
            }
          }
          //else if (strstr(&btdata1[2], "0A")) { // fuel pressure
          //  btSerial.print("41 0A EF\r\n");
          //}
          else if (strstr(&btdata1[2], "0B")) { // map (kPa)
            if (dlcCommand(0x20, 0x05, 0x12, 0x01)) {
              int i = dlcdata[2] * 0.716 - 5; // 101 kPa @ off|wot // 10kPa - 30kPa @ idle
              sprintf_P(btdata2, PSTR("41 0B %02X\r\n>"), i);
            }
          }
          else if (strstr(&btdata1[2], "0C")) { // rpm
            if (dlcCommand(0x20, 0x05, 0x00, 0x02)) {
              int rpm = 0;
              if (obd_select == 1) { rpm = (1875000 / (dlcdata[2] * 256 + dlcdata[3] + 1)) * 4; } // OBD1
              if (obd_select == 2) { rpm = (dlcdata[2] * 256 + dlcdata[3]); } // OBD2
              // in odb1 rpm is -1
              if (rpm < 0) { rpm = 0; }
              sprintf_P(btdata2, PSTR("41 0C %02X %02X\r\n>"), highByte(rpm), lowByte(rpm)); //((A*256)+B)/4
            }
          }
          else if (strstr(&btdata1[2], "0D")) { // vss (km/h)
            if (dlcCommand(0x20, 0x05, 0x02, 0x01)) {
              sprintf_P(btdata2, PSTR("41 0D %02X\r\n>"), dlcdata[2]);
            }
          }
          else if (strstr(&btdata1[2], "0E")) { // timing advance (°)
            if (dlcCommand(0x20, 0x05, 0x26, 0x01)) {
              byte b = ((dlcdata[2] - 24) / 2) + 128;
              sprintf_P(btdata2, PSTR("41 0E %02X\r\n>"), b);
            }
          }
          else if (strstr(&btdata1[2], "0F")) { // iat (°C)
            if (dlcCommand(0x20, 0x05, 0x11, 0x01)) {
              float f = dlcdata[2];
              f = 155.04149 - f * 3.0414878 + pow(f, 2) * 0.03952185 - pow(f, 3) * 0.00029383913 + pow(f, 4) * 0.0000010792568 - pow(f, 5) * 0.0000000015618437;
              dlcdata[2] = round(f) + 40; // A-40
              sprintf_P(btdata2, PSTR("41 0F %02X\r\n>"), dlcdata[2]);
            }
          }
          else if (strstr(&btdata1[2], "11")) { // tps (%)
            if (dlcCommand(0x20, 0x05, 0x14, 0x01)) {
              byte b = (dlcdata[2] - 24) / 2;
              sprintf_P(btdata2, PSTR("41 11 %02X\r\n>"), b);
            }
          }
          else if (strstr(&btdata1[2], "13")) { // o2 sensor present
            sprintf_P(btdata2, PSTR("41 13 80\r\n>")); // 10000000 / assume bank 1 present
          }
          else if (strstr(&btdata1[2], "14")) { // o2 (V)
            if (dlcCommand(0x20, 0x05, 0x15, 0x01)) {
              sprintf_P(btdata2, PSTR("41 14 %02X FF\r\n>"), dlcdata[2]);
            }
          }
          else if (strstr(&btdata1[2], "1C")) {
            sprintf_P(btdata2, PSTR("41 1C 01\r\n>")); // obd2
          }
          else if (strstr(&btdata1[2], "20")) {
            sprintf_P(btdata2, PSTR("41 20 00 00 20 01\r\n>")); // pid 33 and 40
          }
          //else if (strstr(&btdata1[2], "2F")) { // fuel level (%)
          //  sprintf_P(btdata2, PSTR("41 2F FF\r\n>")); // max
          //}
          else if (strstr(&btdata1[2], "33")) { // baro (kPa)
            if (dlcCommand(0x20, 0x05, 0x13, 0x01)) {
              int i = dlcdata[2] * 0.716 - 5; // 101 kPa
              sprintf_P(btdata2, PSTR("41 0B %02X\r\n>"), i);
            }
          }
          else if (strstr(&btdata1[2], "40")) {
            sprintf_P(btdata2, PSTR("41 40 48 00 00 00\r\n>")); // pid 42 and 45
          }
          else if (strstr(&btdata1[2], "42")) { // ecu voltage (V)
            if (dlcCommand(0x20, 0x05, 0x17, 0x01)) {
              float f = dlcdata[2];
              f = f / 10.45;
              unsigned int u = f * 1000; // ((A*256)+B)/1000
              sprintf_P(btdata2, PSTR("41 42 %02X %02X\r\n>"), highByte(u), lowByte(u));
            }
          }
          else if (strstr(&btdata1[2], "45")) { // iacv / relative throttle position (%)
            if (dlcCommand(0x20, 0x05, 0x28, 0x01)) {
              sprintf_P(btdata2, PSTR("41 45 %02X\r\n>"), dlcdata[2]);
            }
          }
        }
        
        // direct honda PID access
        // 1 byte access (21AA) // 21 = 1 byte, AA = address
        else if (btdata1[0] == '2' && btdata1[1] == '1') {
          byte addr = ((btdata1[2] > '9')? (btdata1[2] &~ 0x20) - 'A' + 10: (btdata1[2] - '0') * 16) +
                      ((btdata1[3] > '9')? (btdata1[3] &~ 0x20) - 'A' + 10: (btdata1[3] - '0'));
          if (dlcCommand(0x20, 0x05, addr, 0x01)) {
            sprintf_P(btdata2, PSTR("60 %02X %02X\r\n>"), addr, dlcdata[2]);
          }
        }
        // 2 bytes access (22AA) // 22 = 2 bytes, AA = address
        else if (btdata1[0] == '2' && btdata1[1] == '2') {
          byte addr = ((btdata1[2] > '9')? (btdata1[2] &~ 0x20) - 'A' + 10: (btdata1[2] - '0') * 16) +
                      ((btdata1[3] > '9')? (btdata1[3] &~ 0x20) - 'A' + 10: (btdata1[3] - '0'));
          if (dlcCommand(0x20, 0x05, addr, 0x02)) {
            sprintf_P(btdata2, PSTR("60 %02X %02X %02X\r\n>"), addr, dlcdata[2], dlcdata[3]);
          }
        }
        // 4 byte access (24AA) // 24 = 4 bytes, AA = address
        else if (btdata1[0] == '2' && btdata1[1] == '4') {
          byte addr = ((btdata1[2] > '9')? (btdata1[2] &~ 0x20) - 'A' + 10: (btdata1[2] - '0') * 16) +
                      ((btdata1[3] > '9')? (btdata1[3] &~ 0x20) - 'A' + 10: (btdata1[3] - '0'));
          if (dlcCommand(0x20, 0x05, addr, 0x04)) {
            sprintf_P(btdata2, PSTR("60 %02X %02X %02X %02X %02X\r\n>"), addr, dlcdata[2], dlcdata[3], dlcdata[4], dlcdata[5]);
          }
        }

        if (strlen(btdata2) == 0) {
          sprintf_P(btdata2, PSTR("NO DATA\r\n>"));
        }
        bt_write(btdata2); // send reply

        break;
      }
      else if (btdata1[i] != 32 || btdata1[i] != 10) { // ignore space and newline
        ++i;
      }
    }
}  

void lcdZeroPaddedPrint(long i, byte len, bool decimal = false) {
  if (decimal) i *= 10;
  
  switch (len)
  {
    case 6:
      lcd.print(i/100000);
      i %= 100000;
    case 5:
      lcd.print(i/10000);
      i %= 10000;
    case 4:
      lcd.print(i/1000);
      i %= 1000;
    case 3:
      lcd.print(i/100);
      i %= 100;
    case 2:
      lcd.print(i/10);
      i %= 10;
      if (decimal) lcd.print(".");
    default:
      lcd.print(i);
  }
}

/* Useful Constants */
#define SECS_PER_MIN  (60UL)
#define SECS_PER_HOUR (3600UL)
#define SECS_PER_DAY  (SECS_PER_HOUR * 24L)

/* Useful Macros for getting elapsed time */
#define numberOfSeconds(_time_) (_time_ % SECS_PER_MIN)  
#define numberOfMinutes(_time_) ((_time_ / SECS_PER_MIN) % SECS_PER_MIN) 
#define numberOfHours(_time_) (( _time_% SECS_PER_DAY) / SECS_PER_HOUR)
#define elapsedDays(_time_) ( _time_ / SECS_PER_DAY)  

void lcdSecondsToTimePrint(unsigned long i) {
  lcdZeroPaddedPrint(numberOfHours(i), 2);
  lcd.print(":");
  lcdZeroPaddedPrint(numberOfMinutes(i), 2);
  lcd.print(":");
  lcdZeroPaddedPrint(numberOfSeconds(i), 2);
}

// display routines (2x16 LCD)
void procDisplay(void) {
    //lcd.clear();
    if (pag_select == 0) {
      // display 1
      // R0000 S000 V00.0
      // E00 I00 M000 T00
  
      lcd.setCursor(0,0);
      lcd.print("R");
      lcdZeroPaddedPrint(rpm, 4);
      lcd.print(" S");
      lcdZeroPaddedPrint(vss, 3);
      lcd.print(" V");
      lcdZeroPaddedPrint(volt, 3, true);
  
      lcd.setCursor(0,1);
      lcd.print("E");
      lcdZeroPaddedPrint(ect, 2);
      lcd.print(" I");
      lcdZeroPaddedPrint(iat, 2);
      lcd.print(" M");
      lcdZeroPaddedPrint(maps, 3);
      lcd.print(" T");
      if (tps < 0) {
        lcd.print("-");
        lcdZeroPaddedPrint(tps, 1);
      }
      else {
        lcdZeroPaddedPrint(tps, 2);
      }
    }
    else if (pag_select == 1) {
      // display 2
      // IGN+16.5 BK0 AC0
      // INJ00 IAC00 KNC0
  
      lcd.setCursor(0,0);
      lcd.print("IGN");
      if (ign < 0) { lcd.print("-"); }
      else { lcd.print("+"); }
      //lcd.print(ign);
      lcdZeroPaddedPrint(ign, 3, true);
      lcd.print(" BR0");
      lcd.print(" AC0");
  
      lcd.setCursor(0,1);
      lcd.print("INJ");
      lcdZeroPaddedPrint(inj, 2);
      lcd.print(" IAC");
      lcdZeroPaddedPrint(iac, 2);
      lcd.print(" KNC");
      lcdZeroPaddedPrint(knoc, 1);
    }
    else if (pag_select == 2) {
      // display 3 // trip computer
      // S000  A000  T000
      // T00:00:00 D000.0
  
      lcd.setCursor(0,0);
      lcd.print("S");
      lcdZeroPaddedPrint(vss, 3);
      lcd.print("  A");
      lcdZeroPaddedPrint(vssavg, 3);
      lcd.print("  T");
      lcdZeroPaddedPrint(vsstop, 3);
  
      lcd.setCursor(0,1);
      unsigned long total_time = (idle_time + running_time) / 4; // running time in second @ 250ms
      lcd.print("T");
      lcdSecondsToTimePrint(total_time);
      lcd.print(" D");
      unsigned int total_distance = distance / 100; // in 000.0km format
      lcdZeroPaddedPrint(total_distance, 4, true);
    }
    else if (pag_select == 3) {
      // display 3 // CEL/MIL codes
      byte i;
  
      //byte hbits = i >> 4;
      //byte lbits = i & 0xf;
  
      // display up to 10 error codes
      // 00 00 00 00 00
      // 00 00 00 00 00
      lcd.setCursor(0,0);
      if (dtcCount == 0) {
        lcd.print("    NO ERROR    ");
        lcd.setCursor(0,1);
        lcd.print("                ");
      }
      else {
        for (i=0; i<dtcCount; i++) {
          if (dtcErrors[i] < 10) { lcd.print("0"); }
          lcd.print(dtcErrors[i]);
          lcd.print(" ");
  
          if (dtcCount == 5) {
            lcd.print("+");
            lcd.setCursor(0,1);
          }
          if (dtcCount == 10) {
            lcd.print("+");
            break;
          }
        }
      }
    }
    else if (pag_select == 4) {
      // display 4
      // AFR14.7  FP035.0
      // TH00.0 CP0 B0000
  
      lcd.setCursor(0,0);
      lcd.print("AFR");
      lcdZeroPaddedPrint(afr, 3, true);
      lcd.print("  FP");
      lcdZeroPaddedPrint(fp, 4, true);

      lcd.setCursor(0,1);
      lcd.print("TH");
      lcdZeroPaddedPrint(th, 2, true);
      lcd.print(" CP");
      lcdZeroPaddedPrint(cp, 1);
      lcd.print(" B");
      lcdZeroPaddedPrint(btn, 4);
    }
    else if (pag_select == 5) {
      // display 5
      // ERRC0000 ERRT000

      lcd.setCursor(0,0);
      lcd.print("ERRC");
      lcdZeroPaddedPrint(err_checksum, 4);
      lcd.print(" ERRT");
      lcdZeroPaddedPrint(err_timeout, 3);
      lcd.print("                ");
    }
    /*
    else if (pag_select == 5) {
      // Top Recorded
      // R0000 S000 V00.0
      // E00 I00 M000 T00
      lcd.setCursor(0,0);
      lcd.print("R");
  
      lcdZeroPaddedPrint(rpmtop, 4);
      lcd.print(" S");
      lcdZeroPaddedPrint(vsstop, 3);
      lcd.print(" V");
      lcdZeroPaddedPrint(volttop, 3, true);
  
      lcd.setCursor(0,1);
      lcd.print("E");
      lcdZeroPaddedPrint(ecttop, 2);
      lcd.print(" I");
      lcdZeroPaddedPrint(iattop, 2);
      lcd.print(" M");
      lcdZeroPaddedPrint(mapstop, 3);
      lcd.print(" T");
      if (tps < 0) {
        lcd.print("-");
        lcdZeroPaddedPrint(tpstop, 1);
      }
      else {
        lcdZeroPaddedPrint(tpstop, 2);
      }
    }
    */
}  

// Read DTC Error
void scanDtcError() {
  byte i;

  if (dlcCommand(0x20,0x05,0x40,0x10)) { // row 1
    for (i=0; i<14; i++) {
      if (dlcdata[i+2] >> 4) {
        dtcErrors[i] = i*2;
        dtcCount++;
      }   
      if (dlcdata[i+2] & 0xf) {
        // haxx
        //if (errnum == 23) errnum = 22;
        //if (errnum == 24) errnum = 23;
        dtcErrors[i] = (i*2)+1;
        // haxx
        if (dtcErrors[i] == 23) dtcErrors[i] = 22;
        if (dtcErrors[i] == 24) dtcErrors[i] = 23;
        dtcCount++;
      }
    }
  }
}

// Read ECU Data
void readEcuData() {
  float f;

  //char h_initobd2[12] = {0x68,0x6a,0xf5,0xaf,0xbf,0xb3,0xb2,0xc1,0xdb,0xb3,0xe9}; // 200ms - 300ms delay
  //byte h_cmd1[6] = {0x20,0x05,0x00,0x10,0xcb}; // row 1
  //byte h_cmd2[6] = {0x20,0x05,0x10,0x10,0xbb}; // row 2
  //byte h_cmd3[6] = {0x20,0x05,0x20,0x10,0xab}; // row 3
  //byte h_cmd4[6] = {0x20,0x05,0x76,0x0a,0x5b}; // ecu id

  if (dlcCommand(0x20,0x05,0x00,0x10)) { // row 1
    if (obd_select == 1) rpm = 1875000 / (dlcdata[2] * 256 + dlcdata[3] + 1); // OBD1
    if (obd_select == 2) rpm = (dlcdata[2] * 256 + dlcdata[3]) / 4; // OBD2
    // in odb1 rpm is -1
    if (rpm < 0) { rpm = 0; }

    vss = dlcdata[4];
  }

  if (dlcCommand(0x20,0x05,0x10,0x10)) { // row2
    f = dlcdata[2];
    ect = 155.04149 - f * 3.0414878 + pow(f, 2) * 0.03952185 - pow(f, 3) * 0.00029383913 + pow(f, 4) * 0.0000010792568 - pow(f, 5) * 0.0000000015618437;
    f = dlcdata[3];
    iat = 155.04149 - f * 3.0414878 + pow(f, 2) * 0.03952185 - pow(f, 3) * 0.00029383913 + pow(f, 4) * 0.0000010792568 - pow(f, 5) * 0.0000000015618437;
    maps = dlcdata[4] * 0.716 - 5; // 101 kPa @ off|wot // 10kPa - 30kPa @ idle
    //baro = dlcdata[5] * 0.716 - 5;
    tps = (dlcdata[6] - 24) / 2;

    /*
    f = dlcdata[7];
    f = f / 51.3; // o2 volt in V

    // 0v to 1v / stock sensor
    // 0v to 5v / AEM UEGO / linear
    f = (f * 2) + 10; // afr for AEM UEGO
    afr = round(f * 10); // x10 for display w/ 1 decimal
    */

    f = dlcdata[9];
    volt = f / 10.45; // batt volt in V
    //alt_fr = dlcdata[10] / 2.55
    //eld = 77.06 - dlcdata[11] / 2.5371
  }

  if (dlcCommand(0x20,0x05,0x20,0x10)) { // row3
    sft = (dlcdata[2] / 128 - 1) * 100; // -30 to 30
    lft = (dlcdata[3] / 128 - 1) * 100; // -30 to 30

    inj = (dlcdata[6] * 256 + dlcdata[7]) / 250; // 0 to 16

    //ign = (dlcdata[8] - 128) / 2;
    f = dlcdata[8];
    ign = (f - 24) / 4;

    //lmt = (dlcdata[9] - 128) / 2;
    f = dlcdata[9];
    lmt = (f - 24) / 4;

    iac = dlcdata[10] / 2.55;
  }

  if (dlcCommand(0x20,0x05,0x30,0x10)) { // row4
    // dlcdata[7] to dlcdata[12] unknown
    knoc = dlcdata[14] / 51; // 0 to 5
  }

  // IMAP = RPM * MAP / IAT / 2
  // MAF = (IMAP/60)*(VE/100)*(Eng Disp)*(MMA)/(R)
  // Where: VE = 80% (Volumetric Efficiency), R = 8.314 J/°K/mole, MMA = 28.97 g/mole (Molecular mass of air)
  float maf = 0.0;
  imap = rpm * maps / (iat + 273) / 2;
  // ve = 75, ed = 1.595, afr = 14.7
  maf = (imap / 60) * (80 / 100) * 1.595 * 28.9644 / 8.314472;
  // (gallons of fuel) = (grams of air) / (air/fuel ratio) / 6.17 / 454
  //gof = maf / afr / 6.17 / 454;
  //gear = vss / (rpm+1) * 150 + 0.3;
}

// Read Extra Sensors
void readExtraSensors() {
    float f;

    // read voltage sensor (volt2)
    f = readVcc() / 1000; // V read from ref. or 5.0
    f = (analogRead(PIN_VOLT) * f) / 1024.0; // V
    f = f / (R2/(R1+R2)); // voltage divider

    // air fuel ratio, x=afr(10-20), y=volts(0-5)
    // y = mx + b // slope intercept
    // x = (y - b) / m // derived for x
    // m = y2 - y1 / x2 - x1 = 0.5
    // y = 0.5x + 0 @ x = 10, b = -5

    // where:
    // y = volts
    // m = slope
    // b = y intercept
    // x = afr

    // x = (y + 5) / 0.5

    // read afr sensor (afr)
    f = readVcc() / 1000; // V read from ref. or 5.0
    f = (analogRead(PIN_AFR) * f) / 1024.0; // V
    f = (f + 5) / 0.5; // afr
    //f = 2 * f + 10;

    // fuel pressure, x=psi(0-100), y=volts(0.5-4.5)
    // y = mx + b
    // x = (y - b) / m // derived for x
    // m = y2 - y1 / x2 - x1 = 0.04
    // y = 0.04x + 0.5 @ x = 0, b = -5

    // where:
    // y = volts
    // m = slope
    // b = y intercept
    // x = psi

    // x = (y - 0.5) / 0.04

    // fuel pressur sensor (fp)
    f = readVcc() / 1000; // V read from ref. or 5.0
    f = (analogRead(PIN_FP) * f) / 1024.0; // V
    f = (f - 0.5) / 0.04; // psi

    // read thermal sensor (th)
    int b = 3950;
    f = analogRead(PIN_TH);
    
    f = R3 * (1023.0 / f - 1.0);
    f = log(f);
    f = 1.0 / (0.001129148 + (0.000234125 * f) + (0.0000000876741 * f * f * f));
    th = f - 273.15;                // convert to C
    // ideal temperature correctors
    if (th < 0) th = 0;
    if (th > 99) th = 99;
    
    /*
    f = 1023.0 / f  - 1.0;
    f = R3 / f;                 // resistance
    // th steinhart;
    f = f / 10000.0;            // (R/Ro)
    f = log(f);                 // ln(R/Ro)
    f /= b;                     // 1/B * ln(R/Ro)
    f += 1.0 / (25 + 273.15);   // + (1/To)
    f = 1.0 / f;                // Invert
    f -= 273.15;                // convert to C
    */
}

// Process Data
void execEvery(int ms) {
  static unsigned long msTick = millis();
  if (millis() - msTick >= ms) { // run every 250 ms
    msTick = millis();

    //readEcuData();
    readExtraSensors();
    procDisplay();
    
    
    if (rpm > rpmtop) {
      rpmtop = rpm;
    }
    if (vss > vsstop) {
      vsstop = vss;
    }
    if (ect > ecttop) {
      ecttop = ect;
    }
    if (iat > iattop) {
      iattop = iat;
    }
    if (volt > volttop) {
      volttop = volt;
    }
    if (maps > mapstop) {
      mapstop = maps;
    }
  
    // trip computer essentials
    if (rpm > 0) {
      if (vss > 0) { // running time
        running_time ++;
        vsssum += vss;
        vssavg = (vsssum / running_time);
  
        float f;
        //f = vssavg;
        //f = ((f * 1000) / 14400) * running_time; // @ 250ms
        //distance = round(f);
  
        // formula: distance = speed * fps / 3600
        // where: distance = kilometer(s), speed = km/h, fps in second(s)
        f = vss;
        f = f * 0.25 / 3600; // @ 250ms / km
        f = f * 1000; // km to meters
        distance = distance + round(f);
  
        // time = distance / speed
      }
      else { // idle time
        idle_time ++;
      }
    }
  
    // critical ect value and speed limit, alarm on
    if (ect > ect_alarm) {
      digitalWrite(PIN_BUZZER, HIGH);
    }
    else if (vss > 100 && vss < 105) {
      digitalWrite(PIN_BUZZER, HIGH);
    }
    else {
      digitalWrite(PIN_BUZZER, LOW);
    }
  
    // aircon controller (replace the OEM one)
    if (th <= th_threshold) {
      digitalWrite(PIN_AC, LOW);
      cp = 0;
    }
    else if (th >= (th_threshold + 10)) {
      digitalWrite(PIN_AC, HIGH);
      cp = 1;
    }
  }
}

void pushPinHi(byte pin, unsigned int delayms)
{
  digitalWrite(pin, HIGH);
  delay(delayms);
  digitalWrite(pin, LOW);
}

void procButtons() {
  static unsigned long buttonsTick = 0;
  static bool button_press_old = HIGH;

  int button_press_new;

  btn = analogRead(PIN_BUTTON); // temporary setup: vcc triggered / change to multiple button
  button_press_new = (btn == 1023) ? LOW : HIGH;

  if (button_press_new != button_press_old) { // change state
    if (button_press_new == HIGH) { // on released
      if (millis() - buttonsTick >= 5000) { // long press 5 secs
        scanDtcError();
        //resetECU();
      }
      else if (millis() - buttonsTick >= 3000) { // long press 3 secs
        obd_select++;
        if (obd_select > 2) {
          obd_select = 1;
        }
        EEPROM.write(0, obd_select);
      }
      else if (millis() - buttonsTick >= 5) { // short press 5 ms
        pag_select++;
        if (pag_select > 5) {
          pag_select = 0;
        }
        //EEPROM.write(1, pag_select);
      }
      buttonsTick = 0; // reset timer
      isButtonPressed = false;
    }
    else { // on pressed
      buttonsTick = millis(); // start timer
      isButtonPressed = true;
    }
    button_press_old = button_press_new;
  }

  if (button_press_new == LOW) { // while pressed
    if (millis() - buttonsTick == 5) { // beep @ 5 ms
      pushPinHi(PIN_BUZZER, 50); // beep 50ms
    }
    else if (millis() - buttonsTick == 3000) { // beep @  3 secs
      pushPinHi(PIN_BUZZER, 50); // beep 50ms
    }
    else if (millis() - buttonsTick == 5000) { // beep @  5 secs
      pushPinHi(PIN_BUZZER, 50); // beep 50ms
    }
  }
}

void setup()
{
  pinMode(PIN_AC, OUTPUT); // Air Condition
  pinMode(PIN_BUZZER, OUTPUT); // Piezo Buzzer

  pinMode(PIN_DOOR, INPUT); // Door
  pinMode(PIN_VOLT, INPUT); // Volt meter
  pinMode(PIN_FP, INPUT); // 100psi Fuel Pressure
  pinMode(PIN_AFR, INPUT); // AEM UEGO AFR
  pinMode(PIN_TH, INPUT); // 10k Thermistor

  pinMode(PIN_BUTTON, INPUT); // Button

  //Serial.begin(115200); // For debugging
  btSerial.begin(9600);
  //btSerial.begin(38400);
  dlcSerial.begin(9600);

  // LCD I2C init
  lcd.init();
  lcd.backlight();

  // initial beep
  for (int i=0; i<3; i++) {
    pushPinHi(PIN_BUZZER, 50); // beep 50ms
    delay(80);
  }

  if (EEPROM.read(0) == 0xff) { EEPROM.write(0, obd_select); }
  //if (EEPROM.read(1) == 0xff) { EEPROM.write(1, pag_select); }
  //if (EEPROM.read(2) == 0xff) { EEPROM.write(2, ect_alarm); }
  //if (EEPROM.read(3) == 0xff) { EEPROM.write(3, vss_alarm); }
  //if (EEPROM.read(4) == 0xff) { EEPROM.write(4, th_threshold); }

  obd_select = EEPROM.read(0);
  //pag_select = EEPROM.read(1);
  //ect_alarm = EEPROM.read(2); // over heat ???
  //vss_alarm = EEPROM.read(3); // over speed ???
  //th_threshold = EEPROM.read(4); // compressor cutoff


  dlcInit();

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(APPNAME);
  lcd.setCursor(0,1);
  lcd.print(" ECU Type: OBD");
  lcd.print(obd_select);

  delay(1000);
}

void loop() {
  static unsigned long btTick = 0;

  procButtons();
  
  if (!isButtonPressed) {
    btSerial.listen();
    if (btSerial.available()) {
      if (!elm_mode) {
        elm_mode = true;
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print(" Bluetooth Mode");
        lcd.setCursor(0,1);
        lcd.print(" ECU Type: OBD");
        lcd.print(obd_select);
      }
      procbtSerial();
      btTick = millis();
    }
    if (millis() - btTick >= 2000) { // bt timeout 2 secs
      elm_mode = false;
    }
    if (!elm_mode) {
      execEvery(250);
    }
  }
}
