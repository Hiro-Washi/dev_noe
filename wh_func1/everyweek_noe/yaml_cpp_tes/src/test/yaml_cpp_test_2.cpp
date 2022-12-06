#include "yaml-cpp/yaml.h"

struct Enemy{
  string name;
  int hp;
  int atk;
} 

struct Info{ 
  vector<Enemy> enemy;
  string world;
  string player; 
}


