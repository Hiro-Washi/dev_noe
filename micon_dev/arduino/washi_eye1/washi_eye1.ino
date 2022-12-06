#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <SPI.h>
#define TFT_RST    5
#define TFT_DC     6
#define TFT_CS     7
#define TFT_SCLK 13
#define TFT_MOSI 11
#define display_width 160
#define display_height 80
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS,  TFT_DC, TFT_RST);
int display_pixel_x, display_pixel_y, Offset_pixel = 2;

ros::NodeHandle nh;
#define ROS_INFO_STREAM

// after receive chatter
void arduinoCB( const std_msgs::String& eye_topic) {
  ROS_INFO_STREAM(eye_topic.data);

  if (eye_topic.data == "true") {
    Emoji("(^▽^)", ST7735_YELLOW);
  }
  else if (eye_topic.data == "false") {
    Emoji("x o x", ST7735_BLUE);
  }
  else {
    void(0); //pass
  }
  delay(3000);
  tft.fillScreen(ST7735_BLACK);
}

ros::Subscriber<std_msgs::String> sub("eye_topic", &arduinoCB );

void setup(void) {
  nh.initNode();
  nh.subscribe(sub);

  Serial.begin(9600);
  tft.initR(INITR_MINI160x80);
  Serial.println(F("Initialized"));
  tft.setRotation(3);
  tft.fillScreen(ST7735_BLACK);

  tft.setTextSize(3);
  tft.setCursor(15, 15);
  tft.setTextColor(ST7735_BLUE);
  tft.print("SUTA-TO!");
  delay(2000);

  tft.fillScreen(ST7735_BLACK);
}

void loop(void) {
  Serial.print("Waiting for message...");
  tft.fillScreen(ST7735_WHITE);
  Emoji("(^O^)", ST7735_GREEN);
  delay(1000);
  tft.fillScreen(ST7735_MAGENTA);
  Emoji("(x_x)", ST7735_BLUE);
  delay(1000);
  tft.fillScreen(ST7735_BLACK);  
  nh.spinOnce();
}

void Emoji(char *text, int16_t *color) {
  tft.setCursor(5, 15);
  tft.setTextSize(5);
  tft.setTextColor(color);
  tft.setTextWrap(true); //行を折り返す
  tft.print(text);
}
