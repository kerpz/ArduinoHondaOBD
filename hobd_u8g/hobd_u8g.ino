#include "U8glib.h"

U8GLIB_ST7920_128X64_4X u8g(6, 5, 4); // SPI Com: SCK = en = 6, MOSI = rw = 5, CS = di = 4

void draw_page_1() {
  u8g.setFont(u8g_font_DS_Digital_V1);
  u8g.setFontPosTop();

  //u8g.setScale2x2();
  u8g.drawStr(0, 3, "022");
  //u8g.undoScale();
  
  u8g.setFont(u8g_font_fixed_v0);
  u8g.setFontPosTop();
  u8g.drawStr(52, 19, "KPH");

  u8g.setScale2x2();
  u8g.drawStr(35, 0, "4400");
  u8g.undoScale();
  u8g.drawStr(98, 19, "RPM");

  u8g.setScale2x2();
  u8g.drawStr(43, 15, "28");
  u8g.undoScale();
  u8g.drawStr(110, 31, "0");

  u8g.drawStr(0, 31, "A-KPH = 135.0");
  u8g.drawStr(0, 39, "TRIP = 0063.6");
  u8g.drawStr(0, 47, "BAT V = 12.00");
  u8g.drawStr(0, 55, "22:35:33 THU 09JUN16");

  u8g.drawBox(120,30,8,33);
}

void draw_page_2() {
  u8g.setFont(u8g_font_fixed_v0);
  //u8g.setFontRefHeightExtendedText();
  //u8g.setDefaultForegroundColor();
  u8g.setFontPosTop();

  u8g.drawBox(0,0,90,9);
  u8g.setColorIndex(0);
  u8g.drawStr(1, 0, "X:Sec Y:RPM,KPH");
  u8g.setColorIndex(1);
  
  u8g.drawBox(0,10,25,9);
  u8g.setColorIndex(0);
  u8g.drawStr(1, 10, "AKPH");
  u8g.setColorIndex(1);
  u8g.drawStr(28, 10, "135.");
  u8g.drawStr(50, 10, "0");
  
  u8g.drawBox(0,20,25,9);
  u8g.setColorIndex(0);
  u8g.drawStr(1, 20, "MKPH");
  u8g.setColorIndex(1);
  u8g.drawStr(28, 20, "032");

  u8g.drawFrame(57,10,33,19);
  u8g.setScale2x2();
  u8g.drawStr(30, 5, "28");
  u8g.undoScale();
  u8g.drawStr(83, 11, "0");

  u8g.drawFrame(1,30,89,34);

  u8g.drawLine(92, 0, 92, 63);

  u8g.drawBox(94,0,33,9);
  u8g.setColorIndex(0);
  u8g.drawStr(96, 0, "SPEED");
  u8g.setColorIndex(1);

  u8g.drawStr(102, 11, "019");

  u8g.drawBox(94,21,33,9);
  u8g.setColorIndex(0);
  u8g.drawStr(102, 21, "RPM");
  u8g.setColorIndex(1);
  
  u8g.drawStr(99, 32, "3600");

  u8g.drawBox(94,42,33,9);
  u8g.setColorIndex(0);
  u8g.drawStr(99, 42, "TRIP");
  u8g.setColorIndex(1);
  
  u8g.drawStr(94, 53, "036.");
  u8g.drawStr(116, 53, "98");
}

void setup(void) {

}

void loop(void) {
  
  u8g.firstPage();  
  do {
    draw_page_1();
  } while( u8g.nextPage() );
  
  // rebuild the picture after some delay
  delay(200);
}


