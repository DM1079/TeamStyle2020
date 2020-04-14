#include "player.h"

#include <math.h>

#include <iostream>
#include <list>
#include <vector>

#include "API.h"
#include "Constant.h"
#include "OS_related.h"

using namespace THUAI3;
Protobuf::Talent initTalent =
    Protobuf::Talent::None;  //指定人物天赋。选手代码必须定义此变量，否则报错

const int kCost1 = 10;    //直移一格消耗
const int kCost2 = 9999;  //斜移一格消耗
const char dir[3][3] = {{'z', 'x', 'c'}, {'a', 's', 'd'}, {'q', 'w', 'e'}};
chrono::milliseconds time_200ms(200);
chrono::milliseconds time_50ms(200);

struct Point {
  int x, y;  //点坐标，这里为了方便按照C++的数组来计算，x代表横排，y代表竖列
  int F, G, H;    // F=G+H
  Point* parent;  // parent的坐标，这里没有用指针，从而简化代码
  Point(int _x, int _y)
      : x(_x),
        y(_y),
        F(0),
        G(0),
        H(0),
        parent(NULL)  //变量初始化
  {}
};

class Astar {
 public:
  void InitAstar(const std::vector<std::vector<short>>& _maze);
  std::list<char> GetPath(double x, double y, bool isIgnoreCorner);
  std::list<char> gotolist;  // qweadzxc

 private:
  Point* findPath(Point& startPoint, Point& endPoint, bool isIgnoreCorner);
  std::vector<Point*> getSurroundPoints(const Point* point,
                                        bool isIgnoreCorner) const;
  bool isCanreach(const Point* point,
                  const Point* target,
                  bool isIgnoreCorner) const;  //判断某点是否可以用于下一步判断
  Point* isInList(const std::list<Point*>& list,
                  const Point* point) const;  //判断开启/关闭列表中是否包含某点
  Point* getLeastFpoint();  //从开启列表中返回F值最小的节点
                            //计算FGH值
  int calcG(Point* temp_start, Point* point);
  int calcH(Point* point, Point* end);
  int calcF(Point* point);

 private:
  std::vector<std::vector<short>> maze;
  std::list<Point*> openList;   //开启列表
  std::list<Point*> closeList;  //关闭列表
};

void Astar::InitAstar(const std::vector<std::vector<short>>& _maze) {
  maze = _maze;
}

int Astar::calcG(Point* temp_start, Point* point) {
  int extraG =
      (abs(point->x - temp_start->x) + abs(point->y - temp_start->y)) == 1
          ? kCost1
          : kCost2;
  int parentG = point->parent == NULL
                    ? 0
                    : point->parent->G;  //如果是初始节点，则其父节点是空
  return parentG + extraG;
}

int Astar::calcH(Point* point, Point* end) {
  //用简单的欧几里得距离计算H，这个H的计算是关键，还有很多算法，没深入研究^_^
  return sqrt((double)(end->x - point->x) * (double)(end->x - point->x) +
              (double)(end->y - point->y) * (double)(end->y - point->y)) *
         kCost1;
}

int Astar::calcF(Point* point) {
  return point->G + point->H;
}

Point* Astar::getLeastFpoint() {
  if (!openList.empty()) {
    auto resPoint = openList.front();
    for (auto& point : openList)
      if (point->F < resPoint->F)
        resPoint = point;
    return resPoint;
  }
  return NULL;
}

Point* Astar::findPath(Point& startPoint,
                       Point& endPoint,
                       bool isIgnoreCorner) {
  openList.push_back(new Point(
      startPoint.x, startPoint.y));  //置入起点,拷贝开辟一个节点，内外隔离
  while (!openList.empty()) {
    auto curPoint = getLeastFpoint();  //找到F值最小的点
    openList.remove(curPoint);         //从开启列表中删除
    closeList.push_back(curPoint);     //放到关闭列表
                                    ////1,找到当前周围八个格中可以通过的格子
    auto surroundPoints = getSurroundPoints(curPoint, isIgnoreCorner);
    for (auto& target : surroundPoints) {
      // 2,对某一个格子，如果它不在开启列表中，加入到开启列表，设置当前格为其父节点，计算F
      // G H
      if (!isInList(openList, target)) {
        target->parent = curPoint;
        target->G = calcG(curPoint, target);
        target->H = calcH(target, &endPoint);
        target->F = calcF(target);
        openList.push_back(target);
      }  // 3，对某一个格子，它在开启列表中，计算G值, 如果比原来的大,
         // 就什么都不做, 否则设置它的父节点为当前点,并更新G和F
      else {
        int tempG = calcG(curPoint, target);
        if (tempG < target->G) {
          target->parent = curPoint;
          target->G = tempG;
          target->F = calcF(target);
        }
      }
      Point* resPoint = isInList(openList, &endPoint);
      if (resPoint)
        return resPoint;  //返回列表里的节点指针，不要用原来传入的endpoint指针，因为发生了深拷贝
    }
  }
  return NULL;
}
int sgn(double x) {
  if (x > 0)
    return 1;
  if (x == 0)
    return 0;
  return -1;
}
std::list<char> Astar::GetPath(double x, double y, bool isIgnoreCorner) {
  Point startPoint((int)(PlayerInfo.position.x),
                   (int)(PlayerInfo.position.y));  //网上往右是+
  cout << "start" << startPoint.x << startPoint.y << endl;
  Point endPoint(x, y);
  // A*算法找寻路径
  Point* result = findPath(startPoint, endPoint, isIgnoreCorner);
  std::list<Point*> path;  //返回路径，如果没找到路径，返回空链表
  while (result) {
    path.push_front(result);
    result = result->parent;
  }  // 清空临时开闭列表，防止重复执行GetPath导致结果异常
  openList.clear();
  closeList.clear();
  // return path
  list<Point*>::iterator p;
  list<char>::iterator lp;
  gotolist.clear();
  int i = 1;
  for (p = path.begin();;) {
    Point* pre = *p;
    p++;
    int movex = (*p)->x - pre->x;  // dir[3][3] = { {'z','x','c'
                                   // },{'a','s','d'},{'q','w','e'} };
    int movey = (*p)->y - pre->y;
    char c = dir[movey + 1][movex + 1];
    gotolist.push_back(c);  //例如dir[2][1]=w,dir[0][2]=c
    if (i == path.size() - 1)
      break;
    i++;
  }
  return gotolist;
}

Point* Astar::isInList(const std::list<Point*>& list, const Point* point)
    const {  //判断某个节点是否在列表中，这里不能比较指针，因为每次加入列表是新开辟的节点，只能比较坐标
  for (auto p : list)
    if (p->x == point->x && p->y == point->y)
      return p;
  return NULL;
}

bool Astar::isCanreach(const Point* point,
                       const Point* target,
                       bool isIgnoreCorner) const {
  if (target->x < 0 || target->x > maze.size() - 1 || target->y < 0 ||
      target->y > maze[0].size() - 1 || maze[target->x][target->y] == 1 ||
      target->x == point->x && target->y == point->y ||
      isInList(
          closeList,
          target))  //如果点与当前节点重合、超出地图、是障碍物、或者在关闭列表中，返回false
    return false;
  else {
    if (abs(point->x - target->x) + abs(point->y - target->y) ==
        1)  //非斜角可以
      return true;
    else {  //斜对角要判断是否绊住
      if (maze[point->x][target->y] == 0 && maze[target->x][point->y] == 0)
        return true;
      else
        return isIgnoreCorner;
    }
  }
}
//只保留上下左右四个方向，不然太容易卡住了呜呜呜
std::vector<Point*> Astar::getSurroundPoints(const Point* _point,
                                             bool isIgnoreCorner) const {
  std::vector<Point*> surroundPoints;
  int x = _point->x - 1;
  int y = _point->y;
  if (isCanreach(_point, new Point(x, y), isIgnoreCorner))
    surroundPoints.push_back(new Point(x, y));
  x = _point->x + 1;
  y = _point->y;
  if (isCanreach(_point, new Point(x, y), isIgnoreCorner))
    surroundPoints.push_back(new Point(x, y));
  x = _point->x;
  y = _point->y + 1;
  if (isCanreach(_point, new Point(x, y), isIgnoreCorner))
    surroundPoints.push_back(new Point(x, y));
  x = _point->x;
  y = _point->y - 1;
  if (isCanreach(_point, new Point(x, y), isIgnoreCorner))
    surroundPoints.push_back(new Point(x, y));
  return surroundPoints;
}
static const std::vector<std::vector<short>>
    init_mapinfo_block =  // 0表示可以通过，1表示障碍物，这个和原版的地图是一样的，就是把0以外的数字都改成了1.
    {{1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
      1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
      1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
     {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
     {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
     {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
     {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
     {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
     {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
     {1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0,
      0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1},
     {1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0,
      0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
     {1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0,
      0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
     {1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
     {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
     {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
     {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
     {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
     {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 1, 0, 0, 1, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
     {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
      0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
     {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
      0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
     {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 1},
     {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 1},
     {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 1},
     {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 1},
     {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 1},
     {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 1},
     {1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 1, 1, 1,
      1, 1, 0, 0, 0, 1, 0, 0, 0, 1, 1, 1, 1, 0, 0, 1},
     {1, 0, 0, 0, 0, 1, 0, 0, 0, 1, 1, 0, 0, 1, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 1, 0, 0,
      0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 1},
     {1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
      0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
     {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
     {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
     {1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
     {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
     {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
     {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      1, 1, 1, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
     {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1,
      1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
     {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
     {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
     {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
      0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
     {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
     {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
     {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
     {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
      1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
     {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
     {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1},
     {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
     {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
     {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
     {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
     {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
     {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
     {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
      1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
      1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1}};

void move_dir(char c) {
  //按照c中表示的方向移动
  switch (c) {
    case 'd':
      move(Protobuf::Direction::Right, 200);
      break;
    case 'e':
      move(Protobuf::Direction::RightUp, 265);
      this_thread::sleep_for(time_50ms);
      break;
    case 'w':
      move(Protobuf::Direction::Up, 200);
      break;
    case 'q':
      move(Protobuf::Direction::LeftUp, 265);
      this_thread::sleep_for(time_50ms);
      break;
    case 'a':
      move(Protobuf::Direction::Left, 200);
      break;
    case 'z':
      move(Protobuf::Direction::LeftDown, 265);
      this_thread::sleep_for(time_50ms);
      break;
    case 'x':
      move(Protobuf::Direction::Down, 200);
      break;
    case 'c':
      move(Protobuf::Direction::RightDown, 265);
      this_thread::sleep_for(time_50ms);
      break;

    default:
      break;
  }
  this_thread::sleep_for(time_200ms);
}
void play() {
  char c;
  Astar astar;
  astar.InitAstar(init_mapinfo_block);
  cout << "goto 25,5" << endl;
  cout << PlayerInfo.position.x << "," << PlayerInfo.position.y << endl;
  list<char>::iterator lp;
  // 用Getpath获得一个由字符组成的链表，字符代表移动方向，每一个是用200ms的时间走一格。
  // Getpath自动将当前坐标向下取整设置为起点，参数作为终点，就是下面的（24，5）
  for (auto& lp : astar.GetPath(24, 5, false)) {
    //食物生成点1
    move_dir(lp);
    cout << lp << endl;
    cout << PlayerInfo.position.x << "," << PlayerInfo.position.y << endl;
  }
  this_thread::sleep_for(time_200ms);
  this_thread::sleep_for(time_200ms);
  this_thread::sleep_for(time_200ms);
  this_thread::sleep_for(time_200ms);
  this_thread::sleep_for(time_200ms);

  cout << "goto 42,40" << endl;
  cout << PlayerInfo.position.x << "," << PlayerInfo.position.y << endl;
  for (auto& lp : astar.GetPath(41, 40, false)) {
    //食物生成点2
    move_dir(lp);
    cout << lp << endl;
    cout << PlayerInfo.position.x << "," << PlayerInfo.position.y << endl;
  }

  this_thread::sleep_for(time_200ms);
  this_thread::sleep_for(time_200ms);
  this_thread::sleep_for(time_200ms);
  this_thread::sleep_for(time_200ms);
  this_thread::sleep_for(time_200ms);
}
