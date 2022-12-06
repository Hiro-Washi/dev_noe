#include <ros.h>
#include <std_msgs/Empty.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <SPI.h>
#include <BluetoothSerial.h>
#define TFT_RST    5
#define TFT_DC     6
#define TFT_CS     7
#define TFT_SCLK 13
#define TFT_MOSI 11
#define display_width 160
#define display_height 80
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS,  TFT_DC, TFT_RST);
int display_pixel_x, display_pixel_y, Offset_pixel = 20;

ros::NodeHandle nh;

BluetoothSerial BTS;
int val;
int val_input;

void setup() {
  Serial.begin(115200);
  BTS.begin("Start ESP32 test");
}

void loop() {
  if (BTS.available()) {
    String recieveData = BTS.readStringUntil(";");
    val_input = receiveData.toInt(); // to int

    if (val_input < 0 | 180 < val_input {
      BTS.print("invalid input\n"); // send message to phone
    }
    else {
      val = val_input;
      BTS.print("Set Number" + recieveData);
    }
    
  }
  Serial.print("val");
  
}
