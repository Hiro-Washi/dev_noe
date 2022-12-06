#include <ros.h>
#include <std_msgs/Empty.h>

#include <std_msgs/String.h>

#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <SPI.h>

#define TFT_CS     6
#define TFT_RST    5
#define TFT_DC     7
#define TFT_SCLK 13
#define TFT_MOSI 11

#define display_width 160
#define display_height 80

Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS,  TFT_DC, TFT_RST);

int display_pixel_x, display_pixel_y,Offset_pixel=40;
int16_t max_result;

float pi = 3.145926;
int speed = 9600;

ros::NodeHandle nh;

void arduinoCB(const std_msgs::Empty& toggle_msg) {
  
  delay(100);
}

ros::Subscriber<std_msgs::Empty> sub("toggle_led", &arduinoCB );

void setup(void) {
    Serial.begin(speed);
    tft.initR(INITR_MINI160x80);
    //tft.setRotation(3);
    tft.fillScreen(ST7735_BLACK);

    Serial.print("Let' do it!");
    tft.print("Start");
    tft.setTextSize(4);
    tft.print("ST7735");

    delay(500);
    }

void loop(void) {
    TextWrap("^ ▽ ^", YELLOW)
}

void textWrap(char *text, unit16_t color) {
    tft.setCursor(0, 0);
    tft.setTextSize(8);
    tft.setTextColor(color);
    tft.print(text);
}

