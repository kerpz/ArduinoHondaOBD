#include <SoftwareSerial.h>

SoftwareSerial btSerial(10, 11); // RX, TX

bool elm_mode = false;
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

void read_reply() {
    delay(200);
    while (btSerial.available()) {
      Serial.write(btSerial.read());
    }
}

void procbtSerial(void) {
    char btdata1[20]={0};  // bt data in buffer
    char btdata2[20]={0};  // bt data out buffer
    int i = 0;
    while (btSerial.available()) {
      btdata1[i] = toupper(btSerial.read());
      Serial.print(btdata1[i]);
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
          float volts = 12.85;
          byte v1 = 0, v2 = 0;
          unsigned int volt2 = round(volts * 10); // to cV
          v1 = volt2 / 10;
          v2 = volt2 % 10;
          sprintf_P(btdata2, PSTR("%d.%dV\r\n>"), v1, v2);
        }
        
        // sprintf_P(cmd_str, PSTR("%02X%02X\r"), mode, pid);
        // sscanf(data, "%02X%02X", mode, pid)
        // reset dtc/ecu honda
        // 21 04 01 DA / 01 03 FC
        else if (!strcmp(btdata1, "04")) { // clear dtc / stored values
          //dlcCommand(0x21, 0x04, 0x01, 0x00, dlcdata); // reset ecu
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
        }
        //else if (!strcmp(btdata1, "0102")) { // freeze dtc / 00 61 ???
        //  if (dlcCommand(0x20, 0x05, 0x98, 0x02, dlcdata)) {
        //    sprintf_P(btdata2, PSTR("41 02 %02X %02X\r\n>"), dlcdata[2], dlcdata[3]);
        //  }
        //}
        //else if (!strcmp(btdata1, "0103")) { // fuel system status / 01 00 ???
        //}
        //else if (!strcmp(btdata1, "0104")) { // engine load (%)
        //}
        else if (!strcmp(btdata1, "0105")) { // ect (°C)
          sprintf_P(btdata2, PSTR("41 05 50\r\n>")); // 40
        }
        /*
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
        */
        // multi pid 010C0B0D040E05
        //else if (!strcmp(data, "010A")) { // fuel pressure
        //  btSerial.print("41 0A EF\r\n");
        //}
        else if (!strcmp(btdata1, "010B")) { // map (kPa)
          byte maps = 30; // 101 kPa @ off|wot // 10kPa - 30kPa @ idle
          sprintf_P(btdata2, PSTR("41 0B %02X\r\n>"), maps);
        }
        else if (!strcmp(btdata1, "010C")) { // rpm
          int rpm = 750;
          rpm = rpm * 4;
          sprintf_P(btdata2, PSTR("41 0C %02X %02X\r\n>"), highByte(rpm), lowByte(rpm)); //((A*256)+B)/4
        }
        else if (!strcmp(btdata1, "010D")) { // vss (km/h)
          byte vss = 0;
          sprintf_P(btdata2, PSTR("41 0D %02X\r\n>"), vss); // 0
        }
        else if (!strcmp(btdata1, "010E")) { // timing advance (°)
          //byte b = ((dlcdata[2] - 24) / 2) + 128;
          //sprintf_P(btdata2, PSTR("41 0E %02X\r\n>"), b);
        }
        else if (!strcmp(btdata1, "010F")) { // iat (°C)
          sprintf_P(btdata2, PSTR("41 0F 46\r\n>")); // 30
        }
        else if (!strcmp(btdata1, "0111")) { // tps (%)
          byte tps = 0;
          sprintf_P(btdata2, PSTR("41 11 %02X\r\n>"), tps);
        }
        else if (!strcmp(btdata1, "0113")) { // o2 sensor present ???
          sprintf_P(btdata2, PSTR("41 13 80\r\n>")); // 10000000 / assume bank 1 present
        }
        else if (!strcmp(btdata1, "0114")) { // o2 (V)
          //sprintf_P(btdata2, PSTR("41 14 %02X FF\r\n>"), dlcdata[2]);
        }
        else if (!strcmp(btdata1, "011C")) {
          sprintf_P(btdata2, PSTR("41 1C 01\r\n>")); // obd2
        }
        else if (!strcmp(btdata1, "0120")) {
          sprintf_P(btdata2, PSTR("41 20 00 00 20 01\r\n>")); // pid 33 and 40
        }
        //else if (!strcmp(btdata1, "012F")) { // fuel level (%)
        //  sprintf_P(btdata2, PSTR("41 2F FF\r\n>")); // max
        //}
        else if (!strcmp(btdata1, "0133")) { // baro (kPa)
          byte baro = 101; // 101 kPa
          sprintf_P(btdata2, PSTR("41 0B %02X\r\n>"), baro);
        }
        else if (!strcmp(btdata1, "0140")) {
          sprintf_P(btdata2, PSTR("41 40 48 00 00 00\r\n>")); // pid 42 and 45
        }
        else if (!strcmp(btdata1, "0142")) { // ecu voltage (V)
          float f = 12.95;
          unsigned int u = f * 1000; // ((A*256)+B)/1000
          sprintf_P(btdata2, PSTR("41 42 %02X %02X\r\n>"), highByte(u), lowByte(u));
        }
        else if (!strcmp(btdata1, "0145")) { // iacv / relative throttle position
          //byte iacv = 15 * (255 / 100);
          byte iacv = 38; // 15 %
          sprintf_P(btdata2, PSTR("41 45 %02X\r\n>"), iacv);
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

}

void setup()
{
  Serial.begin(115200); // for debugging
  
  btSerial.begin(38400); // if cmd is set high before power on
  //btSerial.begin(9600); // if cmd is set high after power on
}

void loop() {
  /*
  if (Serial.available()) {
    btSerial.write(Serial.read());
  }
  
  if (btSerial.available()) {
    Serial.write(btSerial.read());
  }
  */
  
  if (btSerial.available()) {
    procbtSerial();
  }

  if (Serial.available()) {
    //Serial.print(Serial.read());
    
    Serial.read();
    Serial.println("Ready. press any key to program BT.");
    
    btSerial.print("AT");
    read_reply();

    //btSerial.println("at+version?");
    //read_reply();

    /*
    btSerial.println("at+name=HOBD");
    read_reply();

    btSerial.println("at+pswd=aki3k3rpz3");
    read_reply();

    btSerial.println("at+uart=9600,0,0");
    read_reply();
    */

    //btSerial.println("at+role?");
    //read_reply();

    //btSerial.println("at+name?");
    //read_reply();

    //btSerial.println("at+state?");
    //read_reply();

    //btSerial.println("at+polar?");
    //read_reply();

    Serial.println("Done.");
  }
}

