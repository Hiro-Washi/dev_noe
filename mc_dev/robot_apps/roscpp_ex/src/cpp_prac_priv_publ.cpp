#include <ros/ros.h>
#include <std_msgs/String.h>

class Car
{
    private:              // 外部からアクセスできません
    Color color;
    SIntN speed;

    void SetColor'(UInt08 r, UInt08 g, UInt08 b)'
    {
        color.r = r;
        color.g = g;
        color.b = b;
    }
public:
    void SetSpeed'(SIntN s)'
    {
        // signed integer(=8bit?) 2bite -> -32,768~32,767
        // long integer -> 4bite, -2,147,483,648~2,147,483,647
        // unit  -> unsigned integer type 
        speed = s;
        SetColor(s, s/2, 255-s);
        //color.r = s;
        //color.g = s / 2;
        //color.b = 255 -s;
    }

    SIntN GetSpeed() {
        return speed;
    }

    Color GetColor() {
        return color;
    }
};

Car car;
car.SetSpeed(100);
