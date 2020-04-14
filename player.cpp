#include "player.h"

#include <iostream>

#include "API.h"
#include "Constant.h"
#include "OS_related.h"
using namespace THUAI3;
Protobuf::Talent initTalent =
    Protobuf::Talent::None;  //指定人物天赋。选手代码必须定义此变量，否则报错
void play() {
  char c;
  cin.clear();
  cin.ignore();
  cin >> c;
  switch (c) {
    case 'd':
      move(Protobuf::Direction::Right, 1000);
      break;
    case 'e':
      move(Protobuf::Direction::RightUp, 1000);
      break;
    case 'w':
      move(Protobuf::Direction::Up, 1000);
      break;
    case 'q':
      move(Protobuf::Direction::LeftUp, 1000);
      break;
    case 'a':
      move(Protobuf::Direction::Left, 1000);
      break;
    case 'z':
      move(Protobuf::Direction::LeftDown, 1000);
      break;
    case 'x':
      move(Protobuf::Direction::Down, 1000);
      break;
    case 'c':
      move(Protobuf::Direction::RightDown, 1000);
      break;

    default:
      break;
  }
  cout << "Input two interger to print a map cell :" << endl;
  int x, y;
  cin.clear();
  cin.ignore();
  cin >> x >> y;
  list<Obj> l = MapInfo::get_mapcell(x, y);
  cout << "objs in map[" << x << "][" << y << "] :" << endl;
  for (list<Obj>::iterator i = l.begin(); i != l.end(); i++) {
    cout << "\tblocktype : " << i->blockType << endl;
  }

  /*  玩家在这里写代码  */
}
