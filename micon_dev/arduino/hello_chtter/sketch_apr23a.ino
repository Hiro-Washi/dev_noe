ros::NodeHandle　nh;

std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);

char hello[13] = "hello world!";
|
void setup()
{
  nh.initNode();
}

void loop()
{
  str_msg.data
　
}
