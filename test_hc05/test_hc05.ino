#include <SoftwareSerial.h>

SoftwareSerial btSerial(8, 9); // RX, TX

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
      if (btdata1[i] == '\r') { // terminate at \r
        btdata1[i] = '\0';
  
        if (!strcmp(btdata1, "ATD")) {
          sprintf_P(btdata2, PSTR("OK\r\n>"));
        }
        else if (!strcmp(btdata1, "ATI")) { // print id / general
          sprintf_P(btdata2, PSTR("Honda OBD v1.0\r\n>"));
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
  
  //btSerial.begin(38400); // if cmd is set high before power on
  btSerial.begin(9600); // if cmd is set high after power on
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
    
    btSerial.println("at");
    read_reply();

    btSerial.println("at+version?");
    read_reply();

    /*
    btSerial.println("at+name=HOBD");
    read_reply();

    btSerial.println("at+pswd=aki3k3rpz3");
    read_reply();

    btSerial.println("at+uart=9600,0,0");
    read_reply();
    */

    btSerial.println("at+role?");
    read_reply();

    btSerial.println("at+name?");
    read_reply();

    btSerial.println("at+state?");
    read_reply();

    btSerial.println("at+polar?");
    read_reply();

    Serial.println("Done.");
  }
}

