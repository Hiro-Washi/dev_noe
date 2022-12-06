#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <SPI.h>
#define TFT_CS   5
#define TFT_RST  4
#define TFT_DC   22
#define TFT_SCLK 18
#define TFT_MOSI 23
#define display_width 160
#define display_height 80
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);
int display_pixel_x,display_pixel_y,Offset_pixel=20;
ros::NodeHandle nh;

void espCB(const std_msgs::String& eye) {
    ROS_INFO_STREAM(eye.data);
    
    if (eye.data == "true") {
      Emoji("(^▽^)", ST7735_YELLOW);
    }
    else if (eye.data == "false") {
      Emoji("x o x", ST7735_BLUE);
    }
    else {
      void(0); //pass
    }
    delay(100);
}

ros::Subscriber<std_msgs::String> sub("eye", &espCB );

void setup(void) {
    nh.initNode();
    nh.subscribe(sub);
    Serial.begin(9600);
    tft.initR(INITR_MINI160x80);
    tft.setRotation(3);
    tft.fillScreen(ST7735_BLACK);
    tft.setTextSize(3);
    tft.setCursor(15, 15);
    tft.print("SUTA-TO!");
    delay(2000);
    tft.fillScreen(ST7735_BLACK);
    }
}

void loop(void) {
    Emoji("o _ o", ST7735_RED);
    Serial.print("Waiting for message...");
    nh.spinOnce();
    delay(100);
}

void Emoji(char *text, int16_t *color) {
    tft.setCursor(5, 15);
    tft.setTextSize(5);
    tft.setTextColor(color);
    tft.setTextWrap(true);
    tft.print(text);
}
