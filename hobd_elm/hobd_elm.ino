/*
 Author:
 - Philip Bordado (kerpz@yahoo.com)
 Hardware:
 - HC-05 Bluetooth module at pin 10 (Rx) pin 11 (Tx) 
 - DLC(K-line) at pin 12
 Software:
 - Arduino 1.0.5
 - SoftwareSerialWithHalfDuplex
   https://github.com/nickstedman/SoftwareSerialWithHalfDuplex
   
 Formula:
 - IMAP = RPM * MAP / IAT / 2
 - MAF = (IMAP/60)*(VE/100)*(Eng Disp)*(MMA)/(R)
   Where: VE = 80% (Volumetric Efficiency), R = 8.314 J/°K/mole, MMA = 28.97 g/mole (Molecular mass of air)
   http://www.lightner.net/obd2guru/IMAP_AFcalc.html
   http://www.installuniversity.com/install_university/installu_articles/volumetric_efficiency/ve_computation_9.012000.htm
*/
#include <SoftwareSerialWithHalfDuplex.h>

SoftwareSerialWithHalfDuplex btSerial(10, 11); // RX, TX
SoftwareSerialWithHalfDuplex dlcSerial(12, 12, false, false);

bool elm_memory = false;
bool elm_echo = false;
bool elm_space = false;
bool elm_linefeed = false;
bool elm_header = false;
int  elm_protocol = 0; // auto

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

  unsigned long timeOut = millis() + 250; // timeout @ 250 ms
  memset(data, 0, sizeof(data));

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

void procbtSerial(void) {
  char btdata1[20];  // bt data in buffer
  char btdata2[20];  // bt data out buffer
  byte dlcdata[20];  // dlc data buffer
  int i = 0;

  memset(btdata1, 0, sizeof(btdata1));
  while (i < 20)
  {
    if (btSerial.available()) {
      btdata1[i] = toupper(btSerial.read());
      if (btdata1[i] == '\r') { // terminate at \r
        btdata1[i] = '\0';
        break;
      }
      else if (btdata1[i] != ' ') { // ignore space
        ++i;
      }
    }
  }

  memset(btdata2, 0, sizeof(btdata2));

  if (!strcmp(btdata1, "ATD")) {
    sprintf_P(btdata2, PSTR("OK\r\n>"));
  }
  else if (!strcmp(btdata1, "ATI")) { // print id / general
    sprintf_P(btdata2, PSTR("HOBD v1.0\r\n>"));
  }
  else if (!strcmp(btdata1, "ATZ")) { // reset all / general
    sprintf_P(btdata2, PSTR("HOBD v1.0\r\n>"));
  }
  else if (strstr(btdata1, "ATE")) { // echo on/off / general
    elm_echo = (btdata1[3] == '1' ? true : false);
    sprintf_P(btdata2, PSTR("OK\r\n>"));
  }
  else if (strstr(btdata1, "ATL")) { // linfeed on/off / general
    elm_linefeed = (btdata1[3] == '1' ? true : false);
    sprintf_P(btdata2, PSTR("OK\r\n>"));
  }
  else if (strstr(btdata1, "ATM")) { // memory on/off / general
    elm_memory = (btdata1[3] == '1' ? true : false);
    sprintf_P(btdata2, PSTR("OK\r\n>"));
  }
  else if (strstr(btdata1, "ATS")) { // space on/off / obd
    elm_space = (btdata1[3] == '1' ? true : false);
    sprintf_P(btdata2, PSTR("OK\r\n>"));
  }
  else if (strstr(btdata1, "ATH")) { // headers on/off / obd
    elm_header = (btdata1[3] == '1' ? true : false);
    sprintf_P(btdata2, PSTR("OK\r\n>"));
  }
  else if (!strcmp(btdata1, "ATSP")) { // set protocol to ? and save it / obd
    //elm_protocol = atoi(data[4]);
    sprintf_P(btdata2, PSTR("OK\r\n>"));
  }
  //else if (!strcmp(data, "ATRV")) { // read voltage in float / volts
  //  btSerial.print("12.0V\r\n>");
  //}
  // sprintf_P(cmd_str, PSTR("%02X%02X\r"), mode, pid);
  // sscanf(data, "%02X%02X", mode, pid)
  // reset dtc/ecu honda
  // 21 04 01 DA / 01 03 FC
  else if (!strcmp(btdata1, "04")) { // clear dtc / stored values
    dlcCommand(0x21, 0x04, 0x01, 0x00, dlcdata); // reset ecu
    sprintf_P(btdata2, PSTR("OK\r\n>"));
  }
  else if (!strcmp(btdata1, "0100")) {
    sprintf_P(btdata2, PSTR("41 00 BE 3E B0 11\r\n>"));
  }
  else if (!strcmp(btdata1, "0101")) { // dtc / AA BB CC DD / A7 = MIL on/off, A6-A0 = DTC_CNT
    if (dlcCommand(0x20, 0x05, 0x0B, 0x01, dlcdata)) {
      byte a = ((dlcdata[2] >> 5) & 1) << 7; // get bit 5 on dlcdata[2], set it to a7
      sprintf_P(btdata2, PSTR("41 01 %02X 00 00 00\r\n>"), a);
    }
    else {
      sprintf_P(btdata2, PSTR("DATA ERROR\r\n>"));
    }
  }
  //else if (!strcmp(btdata1, "0102")) { // freeze dtc / 00 61 ???
  //  if (dlcCommand(0x20, 0x05, 0x98, 0x02, dlcdata)) {
  //    sprintf_P(btdata2, PSTR("41 02 %02X %02X\r\n>"), dlcdata[2], dlcdata[3]);
  //  }
  //  else {
  //    sprintf_P(btdata2, PSTR("DATA ERROR\r\n>"));
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
    else {
      sprintf_P(btdata2, PSTR("DATA ERROR\r\n>"));
    }
  }
  else if (!strcmp(btdata1, "0104")) { // engine load (%)
    if (dlcCommand(0x20, 0x05, 0x9c, 0x01, dlcdata)) {
      sprintf_P(btdata2, PSTR("41 04 %02X\r\n>"), dlcdata[2]);
    }
    else {
      sprintf_P(btdata2, PSTR("DATA ERROR\r\n>"));
    }
  }
  else if (!strcmp(btdata1, "0105")) { // ect (°C)
    if (dlcCommand(0x20, 0x05, 0x10, 0x01, dlcdata)) {
      float f = dlcdata[2];
      f = 155.04149 - f * 3.0414878 + pow(f, 2) * 0.03952185 - pow(f, 3) * 0.00029383913 + pow(f, 4) * 0.0000010792568 - pow(f, 5) * 0.0000000015618437;
      dlcdata[2] = round(f) + 40; // A-40
      sprintf_P(btdata2, PSTR("41 05 %02X\r\n>"), dlcdata[2]);
    }
    else {
      sprintf_P(btdata2, PSTR("DATA ERROR\r\n>"));
    }
  }
  else if (!strcmp(btdata1, "0106")) { // short FT (%)
    if (dlcCommand(0x20, 0x05, 0x20, 0x01, dlcdata)) {
      sprintf_P(btdata2, PSTR("41 06 %02X\r\n>"), dlcdata[2]);
    }
    else {
      sprintf_P(btdata2, PSTR("DATA ERROR\r\n>"));
    }
  }
  else if (!strcmp(btdata1, "0107")) { // long FT (%)
    if (dlcCommand(0x20, 0x05, 0x22, 0x01, dlcdata)) {
      sprintf_P(btdata2, PSTR("41 07 %02X\r\n>"), dlcdata[2]);
    }
    else {
      sprintf_P(btdata2, PSTR("DATA ERROR\r\n>"));
    }
  }
  //else if (!strcmp(data, "010A")) { // fuel pressure
  //  btSerial.print("41 0A EF\r\n");
  //}
  else if (!strcmp(btdata1, "010B")) { // map (kPa)
    if (dlcCommand(0x20, 0x05, 0x12, 0x01, dlcdata)) {
      sprintf_P(btdata2, PSTR("41 0B %02X\r\n>"), dlcdata[2]);
    }
    else {
      sprintf_P(btdata2, PSTR("DATA ERROR\r\n>"));
    }
  }
  else if (!strcmp(btdata1, "010C")) { // rpm
    if (dlcCommand(0x20, 0x05, 0x00, 0x02, dlcdata)) {
      sprintf_P(btdata2, PSTR("41 0C %02X %02X\r\n>"), dlcdata[2], dlcdata[3]);
    }
    else {
      sprintf_P(btdata2, PSTR("DATA ERROR\r\n>"));
    }
  }
  else if (!strcmp(btdata1, "010D")) { // vss (km/h)
    if (dlcCommand(0x20, 0x05, 0x02, 0x01, dlcdata)) {
      sprintf_P(btdata2, PSTR("41 0D %02X\r\n>"), dlcdata[2]);
    }
    else {
      sprintf_P(btdata2, PSTR("DATA ERROR\r\n>"));
    }
  }
  else if (!strcmp(btdata1, "010E")) { // timing advance (°)
    if (dlcCommand(0x20, 0x05, 0x26, 0x01, dlcdata)) {
      sprintf_P(btdata2, PSTR("41 0E %02X\r\n>"), dlcdata[2]);
    }
    else {
      sprintf_P(btdata2, PSTR("DATA ERROR\r\n>"));
    }
  }
  else if (!strcmp(btdata1, "010F")) { // iat (°C)
    if (dlcCommand(0x20, 0x05, 0x11, 0x01, dlcdata)) {
      float f = dlcdata[2];
      f = 155.04149 - f * 3.0414878 + pow(f, 2) * 0.03952185 - pow(f, 3) * 0.00029383913 + pow(f, 4) * 0.0000010792568 - pow(f, 5) * 0.0000000015618437;
      dlcdata[2] = round(f) + 40; // A-40
      sprintf_P(btdata2, PSTR("41 0F %02X\r\n>"), dlcdata[2]);
    }
    else {
      sprintf_P(btdata2, PSTR("DATA ERROR\r\n>"));
    }
  }
  else if (!strcmp(btdata1, "0111")) { // tps (%)
    if (dlcCommand(0x20, 0x05, 0x14, 0x01, dlcdata)) {
      sprintf_P(btdata2, PSTR("41 11 %02X\r\n>"), dlcdata[2]);
    }
    else {
      sprintf_P(btdata2, PSTR("DATA ERROR\r\n>"));
    }
  }
  else if (!strcmp(btdata1, "0113")) { // o2 sensor present ???
    sprintf_P(btdata2, PSTR("41 13 80\r\n>")); // 10000000 / assume bank 1 present
  }
  else if (!strcmp(btdata1, "0114")) { // o2 (V)
    if (dlcCommand(0x20, 0x05, 0x15, 0x01, dlcdata)) {
      sprintf_P(btdata2, PSTR("41 14 %02X FF\r\n>"), dlcdata[2]);
    }
    else {
      sprintf_P(btdata2, PSTR("DATA ERROR\r\n>"));
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
      sprintf_P(btdata2, PSTR("41 33 %02X\r\n>"), dlcdata[2]);
    }
    else {
      sprintf_P(btdata2, PSTR("DATA ERROR\r\n>"));
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
    else {
      sprintf_P(btdata2, PSTR("DATA ERROR\r\n>"));
    }
  }
  else if (!strcmp(btdata1, "0145")) { // iacv / relative throttle position
    if (dlcCommand(0x20, 0x05, 0x28, 0x01, dlcdata)) {
      sprintf_P(btdata2, PSTR("41 45 %02X\r\n>"), dlcdata[2]);
    }
    else {
      sprintf_P(btdata2, PSTR("DATA ERROR\r\n>"));
    }
  }
  else if (!strcmp(btdata1, "2008")) { // custom hobd mapping / flags
    if (dlcCommand(0x20, 0x05, 0x08, 0x01, dlcdata)) {
      sprintf_P(btdata2, PSTR("60 08 %02X\r\n>"), dlcdata[2]);
    }
    else {
      sprintf_P(btdata2, PSTR("NO DATA\r\n>"));
    }
  }
  else if (!strcmp(btdata1, "200B")) { // custom hobd mapping / flags
    //sprintf_P(btdata2, PSTR("60 0C AA\r\n>")); // 10101010 / test data
    if (dlcCommand(0x20, 0x05, 0x0B, 0x01, dlcdata)) {
      sprintf_P(btdata2, PSTR("60 0B %02X\r\n>"), dlcdata[2]);
    }
    else {
      sprintf_P(btdata2, PSTR("NO DATA\r\n>"));
    }
  }
  else if (!strcmp(btdata1, "200C")) { // custom hobd mapping / flags
    if (dlcCommand(0x20, 0x05, 0x0C, 0x01, dlcdata)) {
      sprintf_P(btdata2, PSTR("60 0C %02X\r\n>"), dlcdata[2]);
    }
    else {
      sprintf_P(btdata2, PSTR("NO DATA\r\n>"));
    }
  }
  else if (!strcmp(btdata1, "200F")) { // custom hobd mapping / flags
    if (dlcCommand(0x20, 0x05, 0x0F, 0x01, dlcdata)) {
      sprintf_P(btdata2, PSTR("60 0F %02X\r\n>"), dlcdata[2]);
    }
    else {
      sprintf_P(btdata2, PSTR("NO DATA\r\n>"));
    }
  }
  else {
    sprintf_P(btdata2, PSTR("NO DATA\r\n>"));
  }

  if (btdata2) bt_write(btdata2); // send reply
}  

void setup()
{
  //Serial.begin(9600);
  btSerial.begin(9600);
  dlcSerial.begin(9600);

  delay(100);
  dlcInit();
}

void loop() {
  btSerial.listen();
  if (btSerial.available()) {
    procbtSerial();
  }
}

