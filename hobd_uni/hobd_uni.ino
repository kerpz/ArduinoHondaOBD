/*
 Author:
 - Philip Bordado (kerpz@yahoo.com)

 Hardware:
 - Arduino UNO (Compatible board)
 - HC-05 Bluetooth module
 - Resistors 680k ohms and 220k ohms
 - Tact switch
 - Piezo Buzzer
 - LCD 16x2 and 10k Potentiometer
 
 Software:
 - Arduino 1.6.9
 - SoftwareSerialWithHalfDuplex (Library)
   https://github.com/nickstedman/SoftwareSerialWithHalfDuplex
 - LiquidCrystal (Library)
   
 Formula:
 - IMAP = RPM * MAP / IAT / 2
 - MAF = (IMAP/60)*(VE/100)*(Eng Disp)*(MMA)/(R)
   Where: VE = 80% (Volumetric Efficiency), R = 8.314 J/°K/mole, MMA = 28.97 g/mole (Molecular mass of air)
   http://www.lightner.net/obd2guru/IMAP_AFcalc.html
   http://www.installuniversity.com/install_university/installu_articles/volumetric_efficiency/ve_computation_9.012000.htm

 Arduino Pin Mapping:

 - 00 = Serial RX
 - 01 = Serial TX
 - 02 = Injector Input
 - 03 = Injector Input
 - 04 = LCD D7
 - 05 = LCD D6
 - 06 = LCD D5
 - 07 = LCD D4
 - 08 = LCD EN
 - 09 = LCD RS
 - 10 = Bluetooth RX
 - 11 = Bluetooth TX
 - 12 = K-Line
 - 13 = Piezo buzzer (+)

 - 14 = (A0) Voltage divider (Input Signal)
 - 15 = (A1) VSS (Input Signal)
 - 16 = (A2) Door Status (Input Signal)
 - 17 = (A3) Navigation Button
 - 18 = (A4) I2C
 - 19 = (A5) I2C

 Potentiometer
 - END = 5V
 - MID = LCD VO
 - END = GND
  
*/

#include <EEPROM.h>

//#define LCD_i2c TRUE // Using LCD 16x2 I2C mode

#if defined(LCD_i2c)
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x3f, 2, 1, 0, 4, 5, 6, 7);
#else
#include <LiquidCrystal.h>
LiquidCrystal lcd(9, 8, 7, 6, 5, 4);
#endif

// PCINT0_vect  // D8 - D13
// PCINT1_vect  // A0 - A5
// PCINT2_vect  // D0 - D7
// PCINT3_vect 

#include <SoftwareSerialWithHalfDuplex.h>

SoftwareSerialWithHalfDuplex btSerial(10, 11); // RX, TX
SoftwareSerialWithHalfDuplex dlcSerial(12, 12, false, false);


bool elm_mode = false;
bool elm_memory = false;
bool elm_echo = false;
bool elm_space = false;
bool elm_linefeed = false;
bool elm_header = false;
int  elm_protocol = 0; // auto

byte obd_select = 2; // 0 = obd0, 1 = obd1, 2 = obd2
byte pag_select = 0; // lcd page

byte ect_alarm = 98; // celcius
byte vss_alarm = 100; // kph

void bt_write(char *str) {
  while (*str != '\0')
    btSerial.write(*str++);
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

int dlcCommand(byte cmd, byte num, byte loc, byte len, byte data[]) {
  byte crc = (0xFF - (cmd + num + loc + len - 0x01)); // checksum FF - (cmd + num + loc + len - 0x01)

  unsigned long timeOut = millis() + 200; // timeout @ 200 ms

  dlcSerial.listen();

  dlcSerial.write(cmd);  // header/cmd read memory ??
  dlcSerial.write(num);  // num of bytes to send
  dlcSerial.write(loc);  // address
  dlcSerial.write(len);  // num of bytes to read
  dlcSerial.write(crc);  // checksum
  
  int i = 0;
  while (i < (len+3) && millis() < timeOut) {
    if (dlcSerial.available()) {
      data[i] = dlcSerial.read();
      i++;
    }
  }
  if (data[0] != 0x00 && data[1] != (len+3)) { // or use checksum?
    return 0; // error
  }
  if (i < (len+3)) { // timeout
    return 0; // error
  }
  return 1; // success
}

unsigned int readVoltageDivider(int pin) {
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
 
  //result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  vcc = 1125.3 / vcc; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  
  // kerpz haxx
  float R1 = 680000.0; // Resistance of R1 (680kohms)
  float R2 = 220000.0; // Resistance of R2 (220kohms)
  //unsigned int volt2 = (((analogRead(14) * vcc) / 1024.0) / (R2/(R1+R2))) * 10.0; // convertion & voltage divider
  //temp = ((analogRead(pinTemp) * vcc) / 1024.0) * 100.0; // LM35 celcius
  return (((analogRead(pin) * vcc) / 1024.0) / (R2/(R1+R2))) * 10.0; // convertion & voltage divider
}

void lcdZeroPaddedPrint(long i, byte len, bool decimal = false) {
  if (i < 0) { // negative values
    lcd.print(i);
  }
  else {
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

void procbtSerial() {
  //static unsigned long msTick = millis();
  
  //if (millis() - msTick >= 250) { // run every 250 ms
  //  msTick = millis();

    char btdata1[20]={0};  // bt data in buffer
    char btdata2[20]={0};  // bt data out buffer
    byte dlcdata[20]={0};  // dlc data buffer
    int i = 0;
  
    //btSerial.listen();
    while (btSerial.available()) {
      btdata1[i] = toupper(btSerial.read());
      if (btdata1[i] == '\r') { // terminate at \r
        btdata1[i] = '\0';
  
        if (!strcmp(btdata1, "ATD")) {
          sprintf_P(btdata2, PSTR("OK\r\n>"));
        }
        else if (!strcmp(btdata1, "ATI")) { // print id / general
          sprintf_P(btdata2, PSTR("Honda OBD v1.0\r\n>"));
        }
        else if (!strcmp(btdata1, "ATZ")) { // reset all / general
          sprintf_P(btdata2, PSTR("Honda OBD v1.0\r\n>"));
        }
        else if (strlen(btdata1) == 4 && strstr(btdata1, "ATE")) { // echo on/off / general
          elm_echo = (btdata1[3] == '1' ? true : false);
          sprintf_P(btdata2, PSTR("OK\r\n>"));
        }
        else if (strlen(btdata1) == 4 && strstr(btdata1, "ATL")) { // linfeed on/off / general
          elm_linefeed = (btdata1[3] == '1' ? true : false);
          sprintf_P(btdata2, PSTR("OK\r\n>"));
        }
        else if (strlen(btdata1) == 4 && strstr(btdata1, "ATM")) { // memory on/off / general
          elm_memory = (btdata1[3] == '1' ? true : false);
          sprintf_P(btdata2, PSTR("OK\r\n>"));
        }
        else if (strlen(btdata1) == 4 && strstr(btdata1, "ATS")) { // space on/off / obd
          elm_space = (btdata1[3] == '1' ? true : false);
          sprintf_P(btdata2, PSTR("OK\r\n>"));
        }
        else if (strlen(btdata1) == 4 && strstr(btdata1, "ATH")) { // headers on/off / obd
          elm_header = (btdata1[3] == '1' ? true : false);
          sprintf_P(btdata2, PSTR("OK\r\n>"));
        }
        else if (strlen(btdata1) == 5 && strstr(btdata1, "ATSP")) { // set protocol to ? and save it / obd
          //elm_protocol = atoi(btdata1[4]);
          sprintf_P(btdata2, PSTR("OK\r\n>"));
        }
        else if (!strcmp(btdata1, "ATDP")) { // display protocol / obd
          sprintf_P(btdata2, PSTR("AUTO\r\n>"));
        }
        else if (!strcmp(btdata1, "ATRV")) { // read voltage in float / volts
          //btSerial.print("12.0V\r\n>");
          byte v1 = 0, v2 = 0, v3 = 0;
          unsigned int volt2 = readVoltageDivider(14);
          v1 = volt2 / 10;
          volt2 %= 10;
          v2 = volt2;
          sprintf_P(btdata2, PSTR("%d.%dV\r\n>"), v1, v2);
        }
        // kerpz custom AT cmd
        else if (strlen(btdata1) == 6 && strstr(btdata1, "ATSHP")) { // set hobd protocol
          if (btdata1[5] == '0') { obd_select = 0; }
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
        
        // sprintf_P(cmd_str, PSTR("%02X%02X\r"), mode, pid);
        // sscanf(data, "%02X%02X", mode, pid)
        // reset dtc/ecu honda
        // 21 04 01 DA / 01 03 FC
        else if (!strcmp(btdata1, "04")) { // clear dtc / stored values
          dlcCommand(0x21, 0x04, 0x01, 0x00, dlcdata); // reset ecu
          sprintf_P(btdata2, PSTR("OK\r\n>"));
        }
        else if (!strcmp(btdata1, "03")) { // request dtc
          // do scan then report the errors
          // 43 01 33 00 00 00 00 = P0133
          //sprintf_P(btdata2, PSTR("43 01 33 00 00 00 00\r\n>"), a);
          //sprintf_P(btdata2, PSTR("OK\r\n>"));
        }
        else if (!strcmp(btdata1, "0100")) {
          sprintf_P(btdata2, PSTR("41 00 BE 3E B0 11\r\n>"));
        }
        else if (!strcmp(btdata1, "0101")) { // dtc / AA BB CC DD / A7 = MIL on/off, A6-A0 = DTC_CNT
          if (dlcCommand(0x20, 0x05, 0x0B, 0x01, dlcdata)) {
            byte a = ((dlcdata[2] >> 5) & 1) << 7; // get bit 5 on dlcdata[2], set it to a7
            sprintf_P(btdata2, PSTR("41 01 %02X 00 00 00\r\n>"), a);
          }
        }
        //else if (!strcmp(btdata1, "0102")) { // freeze dtc / 00 61 ???
        //  if (dlcCommand(0x20, 0x05, 0x98, 0x02, dlcdata)) {
        //    sprintf_P(btdata2, PSTR("41 02 %02X %02X\r\n>"), dlcdata[2], dlcdata[3]);
        //  }
        //}
        else if (!strcmp(btdata1, "0103")) { // fuel system status / 01 00 ???
          //if (dlcCommand(0x20, 0x05, 0x0F, 0x01, dlcdata)) { // flags
          //  byte a = dlcdata[2] & 1; // get bit 0 on dlcdata[2]
          //  a = (dlcdata[2] == 1 ? 2 : 1); // convert to comply obd2
          //  sprintf_P(btdata2, PSTR("41 03 %02X 00\r\n>"), a);
          // }
          if (dlcCommand(0x20, 0x05, 0x9a, 0x02, dlcdata)) {
            sprintf_P(btdata2, PSTR("41 03 %02X %02X\r\n>"), dlcdata[2], dlcdata[3]);
          }
        }
        else if (!strcmp(btdata1, "0104")) { // engine load (%)
          if (dlcCommand(0x20, 0x05, 0x9c, 0x01, dlcdata)) {
            sprintf_P(btdata2, PSTR("41 04 %02X\r\n>"), dlcdata[2]);
          }
        }
        else if (!strcmp(btdata1, "0105")) { // ect (°C)
          if (dlcCommand(0x20, 0x05, 0x10, 0x01, dlcdata)) {
            float f = dlcdata[2];
            f = 155.04149 - f * 3.0414878 + pow(f, 2) * 0.03952185 - pow(f, 3) * 0.00029383913 + pow(f, 4) * 0.0000010792568 - pow(f, 5) * 0.0000000015618437;
            dlcdata[2] = round(f) + 40; // A-40
            sprintf_P(btdata2, PSTR("41 05 %02X\r\n>"), dlcdata[2]);
          }
        }
        else if (!strcmp(btdata1, "0106")) { // short FT (%)
          if (dlcCommand(0x20, 0x05, 0x20, 0x01, dlcdata)) {
            sprintf_P(btdata2, PSTR("41 06 %02X\r\n>"), dlcdata[2]);
          }
        }
        else if (!strcmp(btdata1, "0107")) { // long FT (%)
          if (dlcCommand(0x20, 0x05, 0x22, 0x01, dlcdata)) {
            sprintf_P(btdata2, PSTR("41 07 %02X\r\n>"), dlcdata[2]);
          }
        }
        //else if (!strcmp(data, "010A")) { // fuel pressure
        //  btSerial.print("41 0A EF\r\n");
        //}
        else if (!strcmp(btdata1, "010B")) { // map (kPa)
          if (dlcCommand(0x20, 0x05, 0x12, 0x01, dlcdata)) {
            int i = dlcdata[2] * 0.716 - 5; // 101 kPa @ off|wot // 10kPa - 30kPa @ idle
            sprintf_P(btdata2, PSTR("41 0B %02X\r\n>"), i);
          }
        }
        else if (!strcmp(btdata1, "010C")) { // rpm
          if (dlcCommand(0x20, 0x05, 0x00, 0x02, dlcdata)) {
            int rpm = 0;
            if (obd_select == 1) { rpm = (1875000 / (dlcdata[2] * 256 + dlcdata[3] + 1)) * 4; } // OBD1
            if (obd_select == 2) { rpm = (dlcdata[2] * 256 + dlcdata[3]); } // OBD2
            // in odb1 rpm is -1
            if (rpm < 0) { rpm = 0; }
            sprintf_P(btdata2, PSTR("41 0C %02X %02X\r\n>"), highByte(rpm), lowByte(rpm)); //((A*256)+B)/4
          }
        }
        else if (!strcmp(btdata1, "010D")) { // vss (km/h)
          if (dlcCommand(0x20, 0x05, 0x02, 0x01, dlcdata)) {
            sprintf_P(btdata2, PSTR("41 0D %02X\r\n>"), dlcdata[2]);
          }
        }
        else if (!strcmp(btdata1, "010E")) { // timing advance (°)
          if (dlcCommand(0x20, 0x05, 0x26, 0x01, dlcdata)) {
            sprintf_P(btdata2, PSTR("41 0E %02X\r\n>"), dlcdata[2]);
          }
        }
        else if (!strcmp(btdata1, "010F")) { // iat (°C)
          if (dlcCommand(0x20, 0x05, 0x11, 0x01, dlcdata)) {
            float f = dlcdata[2];
            f = 155.04149 - f * 3.0414878 + pow(f, 2) * 0.03952185 - pow(f, 3) * 0.00029383913 + pow(f, 4) * 0.0000010792568 - pow(f, 5) * 0.0000000015618437;
            dlcdata[2] = round(f) + 40; // A-40
            sprintf_P(btdata2, PSTR("41 0F %02X\r\n>"), dlcdata[2]);
          }
        }
        else if (!strcmp(btdata1, "0111")) { // tps (%)
          if (dlcCommand(0x20, 0x05, 0x14, 0x01, dlcdata)) {
            int i = (dlcdata[2] - 24) / 2;
            if (i < 0) i = 0; // haxx
            sprintf_P(btdata2, PSTR("41 11 %02X\r\n>"), i);
          }
        }
        else if (!strcmp(btdata1, "0113")) { // o2 sensor present ???
          sprintf_P(btdata2, PSTR("41 13 80\r\n>")); // 10000000 / assume bank 1 present
        }
        else if (!strcmp(btdata1, "0114")) { // o2 (V)
          if (dlcCommand(0x20, 0x05, 0x15, 0x01, dlcdata)) {
            sprintf_P(btdata2, PSTR("41 14 %02X FF\r\n>"), dlcdata[2]);
          }
        }
        else if (!strcmp(btdata1, "011C")) { // obd2
          sprintf_P(btdata2, PSTR("41 1C 01\r\n>"));
        }
        else if (!strcmp(btdata1, "0120")) {
          sprintf_P(btdata2, PSTR("41 20 00 00 00 01\r\n>"));
        }
        //else if (!strcmp(btdata1, "012F")) { // fuel level (%)
        //  sprintf_P(btdata2, PSTR("41 2F FF\r\n>")); // max
        //}
        else if (!strcmp(btdata1, "0130")) {
          sprintf_P(btdata2, PSTR("41 30 20 00 00 01\r\n>"));
        }
        else if (!strcmp(btdata1, "0133")) { // baro (kPa)
          if (dlcCommand(0x20, 0x05, 0x13, 0x01, dlcdata)) {
            int i = dlcdata[2] * 0.716 - 5; // 101 kPa
            sprintf_P(btdata2, PSTR("41 0B %02X\r\n>"), i);
          }
        }
        else if (!strcmp(btdata1, "0140")) {
          sprintf_P(btdata2, PSTR("41 40 48 00 00 00\r\n>"));
        }
        else if (!strcmp(btdata1, "0142")) { // ecu voltage (V)
          if (dlcCommand(0x20, 0x05, 0x17, 0x01, dlcdata)) {
            float f = dlcdata[2];
            f = f / 10.45;
            unsigned int u = f * 1000; // ((A*256)+B)/1000
            sprintf_P(btdata2, PSTR("41 42 %02X %02X\r\n>"), highByte(u), lowByte(u));
          }
        }
        else if (!strcmp(btdata1, "0145")) { // iacv / relative throttle position
          if (dlcCommand(0x20, 0x05, 0x28, 0x01, dlcdata)) {
            sprintf_P(btdata2, PSTR("41 45 %02X\r\n>"), dlcdata[2]);
          }
        }
        
        // direct honda PID access
        // 1 byte access (21AA) // 21 = 1 byte, AA = address
        else if (btdata1[0] == '2' && btdata1[1] == '1') {
          byte addr = ((btdata1[2] > '9')? (btdata1[2] &~ 0x20) - 'A' + 10: (btdata1[2] - '0') * 16) +
                      ((btdata1[3] > '9')? (btdata1[3] &~ 0x20) - 'A' + 10: (btdata1[3] - '0'));
          if (dlcCommand(0x20, 0x05, addr, 0x01, dlcdata)) {
            sprintf_P(btdata2, PSTR("60 %02X %02X\r\n>"), addr, dlcdata[2]);
          }
        }
        // 2 bytes access (22AA) // 22 = 2 bytes, AA = address
        else if (btdata1[0] == '2' && btdata1[1] == '2') {
          byte addr = ((btdata1[2] > '9')? (btdata1[2] &~ 0x20) - 'A' + 10: (btdata1[2] - '0') * 16) +
                      ((btdata1[3] > '9')? (btdata1[3] &~ 0x20) - 'A' + 10: (btdata1[3] - '0'));
          if (dlcCommand(0x20, 0x05, addr, 0x02, dlcdata)) {
            sprintf_P(btdata2, PSTR("60 %02X %02X %02X\r\n>"), addr, dlcdata[2], dlcdata[3]);
          }
        }
        // 4 byte access (24AA) // 24 = 4 bytes, AA = address
        else if (btdata1[0] == '2' && btdata1[1] == '4') {
          byte addr = ((btdata1[2] > '9')? (btdata1[2] &~ 0x20) - 'A' + 10: (btdata1[2] - '0') * 16) +
                      ((btdata1[3] > '9')? (btdata1[3] &~ 0x20) - 'A' + 10: (btdata1[3] - '0'));
          if (dlcCommand(0x20, 0x05, addr, 0x04, dlcdata)) {
            sprintf_P(btdata2, PSTR("60 %02X %02X %02X %02X %02X\r\n>"), addr, dlcdata[2], dlcdata[3], dlcdata[4], dlcdata[5]);
          }
        }

        if (strlen(btdata2) == 0) {
          sprintf_P(btdata2, PSTR("NO DATA\r\n>"));
        }
        bt_write(btdata2); // send reply

        break;
      }
      else if (btdata1[i] != ' ') { // ignore space
        ++i;
      }
    }
    //btSerial.end();
  //}
}  

void procdlcSerial() {
  static unsigned long msTick = millis();

  if (millis() - msTick >= 250) { // run every 250 ms
    msTick = millis();

    //char h_initobd2[12] = {0x68,0x6a,0xf5,0xaf,0xbf,0xb3,0xb2,0xc1,0xdb,0xb3,0xe9}; // 200ms - 300ms delay
    //byte h_cmd1[6] = {0x20,0x05,0x00,0x10,0xcb}; // row 1
    //byte h_cmd2[6] = {0x20,0x05,0x10,0x10,0xbb}; // row 2
    //byte h_cmd3[6] = {0x20,0x05,0x20,0x10,0xab}; // row 3
    //byte h_cmd4[6] = {0x20,0x05,0x76,0x0a,0x5b}; // ecu id
    byte data[20];
    int rpm=0,ect=0,iat=0,maps=0,baro=0,tps=0,volt=0,volt2=0,imap=0;
  
    static unsigned long vsssum=0,running_time=0,idle_time=0,distance=0;
    static byte vss=0,vsstop=0,vssavg=0;
  
    if (dlcCommand(0x20,0x05,0x00,0x10,data)) { // row 1
      if (obd_select == 1) rpm = 1875000 / (data[2] * 256 + data[3] + 1); // OBD1
      if (obd_select == 2) rpm = (data[2] * 256 + data[3]) / 4; // OBD2
      // in odb1 rpm is -1
      if (rpm < 0) { rpm = 0; }

      vss = data[4];
    }
    
    if (dlcCommand(0x20,0x05,0x10,0x10,data)) { // row2
      float f;
      f = data[2];
      f = 155.04149 - f * 3.0414878 + pow(f, 2) * 0.03952185 - pow(f, 3) * 0.00029383913 + pow(f, 4) * 0.0000010792568 - pow(f, 5) * 0.0000000015618437;
      ect = round(f);
      f = data[3];
      f = 155.04149 - f * 3.0414878 + pow(f, 2) * 0.03952185 - pow(f, 3) * 0.00029383913 + pow(f, 4) * 0.0000010792568 - pow(f, 5) * 0.0000000015618437;
      iat = round(f);
      maps = data[4] * 0.716 - 5; // 101 kPa @ off|wot // 10kPa - 30kPa @ idle
      //baro = data[5] * 0.716 - 5;
      tps = (data[6] - 24) / 2;
      f = data[9];
      f = (f / 10.45) * 10.0; // cV
      volt = round(f);
      //alt_fr = data[10] / 2.55
      //eld = 77.06 - data[11] / 2.5371
    }
  
    if (vss > vsstop) { // top speed
      vsstop = vss;
    }
  
    if (rpm > 0) {
      if (vss > 0) { // running time
          running_time ++;
          vsssum += vss;
          vssavg = (vsssum / running_time);
    
          float d;
          d = vssavg;
          d = ((d * 1000) / 14400) * running_time; // @ 250ms
          distance = round(d);
          //d = vss; // instant distance
          //d = (d * 1000) / 14400; // @ 250ms
          //distance += round(d);
      
          // time = distance / speed
      }
      else { // idle time
          idle_time ++;
      }
    }
  
    // critical ect value or speed limit, alarm on
    if (ect > ect_alarm || vss > vss_alarm) { digitalWrite(13, HIGH); }
    else { digitalWrite(13, LOW); }
  
    // IMAP = RPM * MAP / IAT / 2
    // MAF = (IMAP/60)*(VE/100)*(Eng Disp)*(MMA)/(R)
    // Where: VE = 80% (Volumetric Efficiency), R = 8.314 J/°K/mole, MMA = 28.97 g/mole (Molecular mass of air)
    float maf = 0.0;
    imap = (rpm * maps) / (iat + 273);
    // ve = 75, ed = 1.5.95, afr = 14.7
    maf = (imap / 120) * (80 / 100) * 1.595 * 28.9644 / 8.314472;
  
    volt2 = readVoltageDivider(14);

    //lcd.clear();
    
    if (pag_select == 0) {
      // display 1
      // R0000 S000 V00.0
      // E00 I00 M000 T00
    
      lcd.setCursor(0,0);
    
      lcd.print("R");
      lcdZeroPaddedPrint(rpm, 4);
    
      lcd.print(" ");
      
      lcd.print("S");
      lcdZeroPaddedPrint(vss, 3);
    
      lcd.print(" ");
      
      lcd.print("V");
      lcdZeroPaddedPrint(volt2, 3, true);
    
      lcd.setCursor(0,1);
    
      lcd.print("E");
      lcdZeroPaddedPrint(ect, 2);
    
      lcd.print(" ");
    
      lcd.print("I");
      lcdZeroPaddedPrint(iat, 2);
    
      lcd.print(" ");
    
      lcd.print("M");
      lcdZeroPaddedPrint(maps, 3);

      lcd.print(" ");
    
      lcd.print("T");
      lcdZeroPaddedPrint(tps, 2);
    }
    else if (pag_select == 1) {
      // display 2 // trip computer
      // S000  A000  T000
      // 00:00:00 D000000
      lcd.setCursor(0,0);
    
      lcd.print("S");
      lcdZeroPaddedPrint(vss, 3);
      lcd.print("  A");
      lcdZeroPaddedPrint(vssavg, 3);
      lcd.print("  T");
      lcdZeroPaddedPrint(vsstop, 3);

      lcd.setCursor(0,1);
    
      unsigned long total_time = (idle_time + running_time) / 4; // running time in second @ 250ms
      lcdSecondsToTimePrint(total_time);
      lcd.print(" ");
    
      lcd.print("D");
      lcdZeroPaddedPrint(distance, 6);
    }
    else if (pag_select == 2) {
      byte errnum, errcnt = 0, i;

      //byte hbits = i >> 4;
      //byte lbits = i & 0xf;

      //lcd.clear();
    
      // display up to 10 error codes
      // 00 00 00 00 00 
      // 00 00 00 00 00
      lcd.setCursor(0,0);
      if (dlcCommand(0x20,0x05,0x40,0x10,data)) { // row 1
        for (i=0; i<14; i++) {
          if (data[i+2] >> 4) {
            errnum = i*2;
            if (errnum < 10) { lcd.print("0"); }
            lcd.print(errnum);
            lcd.print(" ");
            errcnt++;
          }   
          else if (data[i+2] & 0xf) {
            errnum = (i*2)+1;
            // haxx
            if (i>10) errnum = i*2;
            if (errnum < 10) { lcd.print("0"); }
            lcd.print(errnum);
            lcd.print(" ");
            errcnt++;
          }
          if (errcnt > 5) {
            lcd.print(" ");
            lcd.setCursor(0,1);
          }
          if (errcnt > 10) {
            lcd.print(" ");
            break;
          }
        }
      }
      //lcd.setCursor(0,1);
      if (dlcCommand(0x20,0x05,0x50,0x10,data)) { // row 2
        for (i=0; i<16; i++) {
          if (data[i+2] >> 4) {
            errnum = (i*2)+32;
            if (errnum < 10) { lcd.print("0"); }
            lcd.print(errnum);
            lcd.print(" ");
            errcnt++;
          }   
          else if (data[i+2] & 0xf) {
            errnum = (i*2)+33;
            if (errnum < 10) { lcd.print("0"); }
            lcd.print(errnum);
            lcd.print(" ");
            errcnt++;
          }
          if (errcnt > 5) {
            lcd.print(" ");
            lcd.setCursor(0,1);
          }
          if (errcnt > 10) {
            lcd.print(" ");
            break;
          }
        }
      }
      if (errcnt == 0) {
        lcd.print("    NO ERROR    ");
        lcd.setCursor(0,1);
        lcd.print("                ");
      }
      else {
        for (i=errcnt; i<10; i++) {
          lcd.print("   ");
          if (i > 5) {
            lcd.print(" ");
            lcd.setCursor(0,1);
          }
          if (i > 10) {
            lcd.print(" ");
            break;
          }
        }
      }
      // shift 4 bits left and right to get a value
      // 40 0=ecu 1=o2a
      // 41 2=o2b 3=map
      // 42 4=ckp 5=map
      // 43 6=ect 7=tps
      // 44 8=tdc 9=cyp
      // 45 10=iat
      // 46 12=egr 13=baro
      // 47 14=iac 15=ign
      // 48 16=vss
    }
  }
}  

void procButtons() {
  static unsigned long buttonsTick = 0;
  static int button_press_old = HIGH;

  int button_press_new = digitalRead(17);

  if (button_press_new != button_press_old) { // change state
    if (button_press_new == HIGH) { // on released
      if (millis() - buttonsTick >= 5000) { // long press 5 secs
        //scanDTC();
        //resetECU();
      }
      else if (millis() - buttonsTick >= 3000) { // long press 3 secs
        obd_select++;
        if (obd_select > 2) {
          obd_select = 0;
        }
        EEPROM.write(0, obd_select);
      }
      else if (millis() - buttonsTick >= 5) { // short press 5 ms
        pag_select++;
        if (pag_select > 2) {
          pag_select = 0;
        }
        EEPROM.write(1, pag_select);
      }
      buttonsTick = 0; // reset timer
    }
    else { // on pressed
      buttonsTick = millis(); // start timer
    }
    button_press_old = button_press_new;
  }

  if (button_press_new == LOW) { // while pressed
      if (millis() - buttonsTick == 5) { // beep @ 5 ms
        pushPinHi(13, 50); // beep 50ms
      }
      else if (millis() - buttonsTick == 3000) { // beep @  3 secs
        pushPinHi(13, 50); // beep 50ms
      }
      else if (millis() - buttonsTick == 5000) { // beep @  5 secs
        pushPinHi(13, 50); // beep 50ms
      }
  }
}

void pushPinHi(byte pin, unsigned char delayms)
{
  digitalWrite(pin, HIGH);
  delay(delayms);
  digitalWrite(pin, LOW);
}

void setup()
{
  pinMode(13, OUTPUT); // Piezo Buzzer
  
  pinMode(17, INPUT); // Button
  digitalWrite(17, HIGH); // Configure internal pull-up resistor

  // 18 and 19 used by i2c
  //pinMode(18, OUTPUT); // Door lock
  //pinMode(19, OUTPUT); // Door unlock

  
  //Serial.begin(115200); // For debugging
  btSerial.begin(9600);
  dlcSerial.begin(9600);

#if defined(LCD_i2c)
  lcd.setBacklightPin(3, POSITIVE);
  lcd.setBacklight(HIGH); // NOTE: You can turn the backlight off by setting it to LOW instead of HIGH
#endif

  lcd.begin(16, 2); // sets the LCD's rows and colums:

  // initial beep
  for (int i=0; i<3; i++) {
    pushPinHi(13, 50); // beep 50ms
    delay(80);
  }

  if (EEPROM.read(0) == 0xff) { EEPROM.write(0, obd_select); }
  if (EEPROM.read(1) == 0xff) { EEPROM.write(0, pag_select); }
  //if (EEPROM.read(2) == 0xff) { EEPROM.write(0, ect_alarm); }
  //if (EEPROM.read(3) == 0xff) { EEPROM.write(0, vss_alarm); }
  
  obd_select = EEPROM.read(0);
  pag_select = EEPROM.read(1);
  //ect_alarm = EEPROM.read(2); // over heat ???
  //vss_alarm = EEPROM.read(3); // over speed ???

  
  dlcInit();

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Honda OBD v1.0");
  lcd.setCursor(0,1);
  lcd.print("OBD Select: ");
  lcd.print(obd_select);

  delay(1000);
}

void loop() {
  static unsigned long btTick = 0;

  procButtons();

  btSerial.listen();
  if (btSerial.available()) {
    if (!elm_mode) {
      elm_mode = true;
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Bluetooth Mode");
      lcd.setCursor(0,1);
      lcd.print("OBD Select: ");
      lcd.print(obd_select);
    }
    procbtSerial();
    btTick = millis();
  }

  if (millis() - btTick >= 2000) { // bt timeout 2 secs
    elm_mode = false;
  }

  if (!elm_mode) {
    procdlcSerial();
  }
}

