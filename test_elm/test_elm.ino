/*
 Application:
 - ELM327 Bluetooth Simulation
 
 Author:
 - Philip Bordado (kerpz@yahoo.com)

 Hardware:
 - Arduino UNO (Compatible board)
 - HC-05/HC-06 Bluetooth module

 Software:
 - Arduino 1.6.9
 - SoftwareSerialWithHalfDuplex (Library)
   https://github.com/nickstedman/SoftwareSerialWithHalfDuplex
   
 Arduino Pin Mapping:

 - 00 = Serial RX
 - 01 = Serial TX
 - 10 = Bluetooth RX
 - 11 = Bluetooth TX

*/

#include <SoftwareSerial.h>

SoftwareSerial btSerial(10, 11); // RX, TX

bool elm_mode = false;
bool elm_memory = false;
bool elm_echo = false;
bool elm_space = true;
bool elm_linefeed = true;
bool elm_header = false;
int  elm_protocol = 0; // auto

void bt_write(char *str) {
  char c = *str;
  while (*str != '\0') {
    if (!elm_linefeed && *str == 10) *str++; // skip linefeed for all reply
    if (c == '4' && !elm_space && *str == 32) *str++; // skip space for obd reply
    btSerial.write(*str++);
  }
}

void procbtSerial(void) {
    char btdata1[20]={0};  // bt data in buffer
    char btdata2[20]={0};  // bt data out buffer
    int i = 0;
    while (btSerial.available()) {
      btdata1[i] = toupper(btSerial.read());
      
      //Serial.print(btdata1[i]); // debug
      
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
          sprintf_P(btdata2, PSTR("Honda OBD v1.0\r\n>"));
        }
        else if (!strcmp(btdata1, "ATZ")) { // reset all / general
          elm_echo = false;
          elm_space = true;
          elm_linefeed = true;
          elm_header = false;
          sprintf_P(btdata2, PSTR("Honda OBD v1.0\r\n>"));
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
          float volts = 12.85;
          byte v1 = 0, v2 = 0;
          unsigned int volt2 = round(volts * 10); // to cV
          v1 = volt2 / 10;
          v2 = volt2 % 10;
          sprintf_P(btdata2, PSTR("%d.%dV\r\n>"), v1, v2);
        }
        
        // https://en.wikipedia.org/wiki/OBD-II_PIDs
        // sprintf_P(cmd_str, PSTR("%02X%02X\r"), mode, pid);
        // sscanf(btdata1, "%02X%02X", mode, pid)
        else if (len == 2 && btdata1[0] == '0' && btdata1[1] == '4') { // mode 04
          // clear dtc / stored values
          // reset dtc/ecu honda
          // 21 04 01 DA / 01 03 FC
          //dlcCommand(0x21, 0x04, 0x01, 0x00, dlcdata); // reset ecu
          sprintf_P(btdata2, PSTR("OK\r\n>"));
        }
        else if (len == 2 && btdata1[0] == '0' && btdata1[1] == '3') { // mode 03
          // request dtc
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
          }
          //else if (strstr(&btdata1[2], "02")) { // freeze dtc / 00 61 ???
          //  if (dlcCommand(0x20, 0x05, 0x98, 0x02, dlcdata)) {
          //    sprintf_P(btdata2, PSTR("41 02 %02X %02X\r\n>"), dlcdata[2], dlcdata[3]);
          //  }
          //}
          //else if (strstr(&btdata1[2], "03")) { // fuel system status / 01 00 ???
          //}
          else if (strstr(&btdata1[2], "04")) { // engine load (%)
            byte v = 0;
            v = (v * 255) / 100; // conversion to byte range
            sprintf_P(btdata2, PSTR("41 04 %02X\r\n>"), v);
          }
          else if (strstr(&btdata1[2], "05")) { // ect (°C)
            byte v = 40;
            v = v + 40; // conversion
            sprintf_P(btdata2, PSTR("41 05 %02X\r\n>"), v);
          }
          else if (strstr(&btdata1[2], "06")) { // short FT (%)
            byte v = 20; // -100 too rich, 99.2 too lean
            v = ((v + 100) * 128) / 100; // conversion
            sprintf_P(btdata2, PSTR("41 06 %02X\r\n>"), v);
          }
          else if (strstr(&btdata1[2], "07")) { // long FT (%)
            byte v = 20; // -100 too rich, 99.2 too lean
            v = ((v + 100) * 128) / 100; // conversion
            sprintf_P(btdata2, PSTR("41 07 %02X\r\n>"), v);
          }
          else if (strstr(&btdata1[2], "0A")) { // fuel pressure
            byte v = 255; // 255 kPa / 37 psi
            v = v / 3; // conversion
            sprintf_P(btdata2, PSTR("41 0A %02X\r\n>"), v);
          }
          else if (strstr(&btdata1[2], "0B")) { // map (kPa)
            byte v = 30; // 101 kPa @ off|wot // 10kPa - 30kPa @ idle
            sprintf_P(btdata2, PSTR("41 0B %02X\r\n>"), v);
          }
          else if (strstr(&btdata1[2], "0C")) { // rpm
            int v = 750;
            v = v * 4; // conversion
            sprintf_P(btdata2, PSTR("41 0C %02X %02X\r\n>"), highByte(v), lowByte(v)); //((A*256)+B)/4
          }
          else if (strstr(&btdata1[2], "0D")) { // vss (km/h)
            byte v = 0;
            sprintf_P(btdata2, PSTR("41 0D %02X\r\n>"), v);
          }
          else if (strstr(&btdata1[2], "0E")) { // timing advance (°)
            float f = 16.50;
            byte v = (f + 64) * 2; // conversion
            sprintf_P(btdata2, PSTR("41 0E %02X\r\n>"), v);
          }
          else if (strstr(&btdata1[2], "0F")) { // iat (°C)
            byte v = 30;
            v = v + 40; // conversion
            sprintf_P(btdata2, PSTR("41 0F %02X\r\n>"), v);
          }
          else if (strstr(&btdata1[2], "11")) { // tps (%)
            byte v = 0;
            v = (v * 255) / 100; // conversion to byte range
            sprintf_P(btdata2, PSTR("41 11 %02X\r\n>"), v);
          }
          else if (strstr(&btdata1[2], "13")) { // o2 sensor present
            sprintf_P(btdata2, PSTR("41 13 80\r\n>")); // 10000000 / assume bank 1 present
          }
          else if (strstr(&btdata1[2], "14")) { // o2 (V)
            //sprintf_P(btdata2, PSTR("41 14 %02X FF\r\n>"), dlcdata[2]);
          }
          else if (strstr(&btdata1[2], "1C")) {
            sprintf_P(btdata2, PSTR("41 1C 01\r\n>")); // obd2
          }
          else if (strstr(&btdata1[2], "20")) {
            sprintf_P(btdata2, PSTR("41 20 00 00 20 01\r\n>")); // pid 33 and 40
          }
          else if (strstr(&btdata1[2], "2F")) { // fuel level (%)
            byte v = 75;
            v = (v * 255) / 100; // conversion to byte range
            sprintf_P(btdata2, PSTR("41 2F %02X\r\n>"), v);
          }
          else if (strstr(&btdata1[2], "33")) { // baro (kPa)
            byte v = 101; // 101 kPa
            sprintf_P(btdata2, PSTR("41 33 %02X\r\n>"), v);
          }
          else if (strstr(&btdata1[2], "40")) {
            sprintf_P(btdata2, PSTR("41 40 48 00 00 00\r\n>")); // pid 42 and 45
          }
          else if (strstr(&btdata1[2], "42")) { // ecu voltage (V)
            float f = 12.95;
            unsigned int u = f * 1000; // ((A*256)+B)/1000
            sprintf_P(btdata2, PSTR("41 42 %02X %02X\r\n>"), highByte(u), lowByte(u));
          }
          else if (strstr(&btdata1[2], "45")) { // iacv / relative throttle position (%)
            byte v = 15;
            v = (v * 255) / 100; // conversion to byte range
            sprintf_P(btdata2, PSTR("41 45 %02X\r\n>"), v);
          }
        }

        if (strlen(btdata2) == 0)
          sprintf_P(btdata2, PSTR("NO DATA\r\n>"));

        bt_write(btdata2); // send reply

        break;
      }
      else if (btdata1[i] != 32 || btdata1[i] != 10) { // ignore space and newline
        ++i;
      }
    }

}

void setup()
{
  Serial.begin(115200); // for debugging
  
  btSerial.begin(38400); // if cmd is set high before power on
  //btSerial.begin(9600); // if cmd is set high after power on
}

void loop() {
  if (btSerial.available()) {
    procbtSerial();
  }
}

