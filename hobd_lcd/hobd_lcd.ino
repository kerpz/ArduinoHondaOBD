#include <LiquidCrystal.h>
#include <SoftwareSerialWithHalfDuplex.h>
//#include <IRremote.h>

// input signals: ign, door
// output signals: alarm horn, lock door, unlock door, immobilizer

/*
* LCD RS pin 9
* LCD Enable pin 8
* LCD D4 pin 7
* LCD D5 pin 6
* LCD D6 pin 5
* LCD D7 pin 4
* LCD R/W pin to ground
* 10K potentiometer:
* ends to +5V and ground
* wiper to LCD VO pin (pin 3)
*/
LiquidCrystal lcd(9, 8, 7, 6, 5, 4);
/*
* DLC pin 12
*/
SoftwareSerialWithHalfDuplex dlcSerial(12, 12, false, false);

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

void procdlcSerial(void) {
  //char h_initobd2[12] = {0x68,0x6a,0xf5,0xaf,0xbf,0xb3,0xb2,0xc1,0xdb,0xb3,0xe9}; // 200ms - 300ms delay
  //byte h_cmd1[6] = {0x20,0x05,0x00,0x10,0xcb}; // row 1
  //byte h_cmd2[6] = {0x20,0x05,0x10,0x10,0xbb}; // row 2
  //byte h_cmd3[6] = {0x20,0x05,0x20,0x10,0xab}; // row 3
  //byte h_cmd4[6] = {0x20,0x05,0x76,0x0a,0x5b}; // ecu id
  byte data[20];
  unsigned int rpm=0,vss=0,ect=0,iat=0,maps=0,tps=0,volt=0, imap=0;

  if (dlcCommand(0x20,0x05,0x00,0x10,data)) { // row 1
    rpm = (data[2] * 256 + data[3]) / 4;
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
    maps = data[4];
    tps = (data[6] - 24) / 2;
    f = data[9];
    f = (f / 10.45) * 10.0; // cV
    volt = round(f);
  }
  
  // IMAP = RPM * MAP / IAT / 2
  // MAF = (IMAP/60)*(VE/100)*(Eng Disp)*(MMA)/(R)
  // Where: VE = 80% (Volumetric Efficiency), R = 8.314 J/°K/mole, MMA = 28.97 g/mole (Molecular mass of air)
  float maf = 0.0;
  imap = (rpm * maps) / (iat + 273);
  // ve = 75, ed = 1.5.95, afr = 14.7
  maf = (imap / 120) * (80 / 100) * 1.595 * 28.9644 / 8.314472;

  
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
  float R1 = 680000.0; // Resistance of R1
  float R2 = 220000.0; // Resistance of R2
  unsigned int volt2 = (((analogRead(14) * vcc) / 1024.0) / (R2/(R1+R2))) * 10.0; // convertion & voltage divider
  //temp = ((analogRead(pinTemp) * vcc) / 1024.0) * 100.0; // LM35 celcius
  

  // display
  // R0000 S000 V00.0
  // E00 I00    V00.0
  unsigned short i = 0;
  
  lcd.clear();
  lcd.setCursor(0,0);

  lcd.print("R");
  i = rpm;
  lcd.print(i/1000);
  i %= 1000;
  lcd.print(i/100);
  i %= 100;
  lcd.print(i/10);
  i %= 10;
  lcd.print(i);

  lcd.print(" ");
  
  lcd.print("S");
  i = vss;
  lcd.print(i/100);
  i %= 100;
  lcd.print(i/10);
  i %= 10;
  lcd.print(i);

  lcd.print(" ");
  
  lcd.print("V");
  i = volt2;
  lcd.print(i/100);
  i %= 100;
  lcd.print(i/10);
  i %= 10;
  lcd.print(".");
  lcd.print(i);

  lcd.setCursor(0,1);

  lcd.print("E");
  i = ect;
  lcd.print(i/10);
  i %= 10;
  lcd.print(i);

  lcd.print(" ");

  lcd.print("I");
  i = iat;
  lcd.print(i/10);
  i %= 10;
  lcd.print(i);

  lcd.print(" ");

  lcd.print("M");
  i = maps;
  lcd.print(i/100);
  i %= 100;
  lcd.print(i/10);
  i %= 10;
  lcd.print(i);
  
  lcd.print(" ");

  lcd.print("T");
  i = tps;
  lcd.print(i/10);
  i %= 10;
  lcd.print(i);
  
}  

void setup() {
  lcd.begin(0, 2); // sets the LCD's rows and colums:
  dlcSerial.begin(9600);
  dlcInit();
}

void loop() {
  procdlcSerial();
  delay(300);
}

