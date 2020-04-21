#include "API.h"
#include "Constant.h"
#include "player.h"
#include <iostream>
#include "OS_related.h"
#include <vector>
#include <list>
#include <math.h>
#include <string>
#include <sstream>

#define PI 3.1415926535
using namespace THUAI3;
Protobuf::Talent initTalent = Protobuf::Talent::Cook;//指定人物天赋。选手代码必须定义此变量，否则报错

const int kCost1 = 10; //直移一格消耗
const int kCost2 = 9999; //斜移一格消耗
const char dir[3][3] = { {'z','x','c' },{'a','s','d'},{'q','w','e'} };

int cooking[6] = { 0,0,0,0,0,0 };//{0dish,1灶台label,2用时，3起，4止，5保护}
int state = 0;
int label;//记录“据点”是第几个灶台

chrono::milliseconds time_200ms(200);
chrono::milliseconds time_50ms(50);
chrono::milliseconds time_25ms(25);


struct Point {
    int x, y; //点坐标，这里为了方便按照C++的数组来计算，x代表横排，y代表竖列
    int F, G, H; //F=G+H	
    Point* parent; //parent的坐标，这里没有用指针，从而简化代码
    Point(int _x, int _y) :x(_x), y(_y), F(0), G(0), H(0), parent(NULL)  //变量初始化	
    {
    }
};

class Astar
{
public:
    Astar(const std::vector<std::vector<short>>& _maze)
    {
        maze = _maze;
        initmaze = _maze;
    }
    void setmap(int x, int y, int label)
    {
        maze[x][y] = label;//临时改变map
    }
    void resetmap()
    {
        maze = initmaze;//恢复初始map
    }
    std::list<char> GetPath(double x, double y, bool isIgnoreCorner);
    std::list<char> gotolist; //qweadzxc
    std::vector<std::vector<short>> maze;
private:
    Point* findPath(Point& startPoint, Point& endPoint, bool isIgnoreCorner);
    std::vector<Point*> getSurroundPoints(const Point* point, bool isIgnoreCorner) const;
    bool isCanreach(const Point* point, const Point* target, bool isIgnoreCorner) const; //判断某点是否可以用于下一步判断	
    Point* isInList(const std::list<Point*>& list, const Point* point) const; //判断开启/关闭列表中是否包含某点	
    Point* getLeastFpoint(); //从开启列表中返回F值最小的节点	
                             //计算FGH值	
    int calcG(Point* temp_start, Point* point);
    int calcH(Point* point, Point* end);
    int calcF(Point* point);
private:
    std::list<Point*> openList;  //开启列表	
    std::list<Point*> closeList; //关闭列表
    std::vector<std::vector<short>> initmaze;
};

int Astar::calcG(Point* temp_start, Point* point) {
    int extraG = (abs(point->x - temp_start->x) + abs(point->y - temp_start->y)) == 1 ? kCost1 : kCost2;
    int parentG = point->parent == NULL ? 0 : point->parent->G; //如果是初始节点，则其父节点是空	
    return parentG + extraG;
}

int Astar::calcH(Point* point, Point* end) {
    //用简单的欧几里得距离计算H，这个H的计算是关键，还有很多算法，没深入研究^_^	
    return sqrt((double)(end->x - point->x) * (double)(end->x - point->x) + (double)(end->y - point->y) * (double)(end->y - point->y)) * kCost1;
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


Point* Astar::findPath(Point& startPoint, Point& endPoint, bool isIgnoreCorner)
{
    openList.push_back(new Point(startPoint.x, startPoint.y)); //置入起点,拷贝开辟一个节点，内外隔离	
    while (!openList.empty()) {
        auto curPoint = getLeastFpoint(); //找到F值最小的点		
        openList.remove(curPoint); //从开启列表中删除		
        closeList.push_back(curPoint); //放到关闭列表		//1,找到当前周围八个格中可以通过的格子		
        auto surroundPoints = getSurroundPoints(curPoint, isIgnoreCorner);
        for (auto& target : surroundPoints)
        {
            //2,对某一个格子，如果它不在开启列表中，加入到开启列表，设置当前格为其父节点，计算F G H			
            if (!isInList(openList, target))
            {
                target->parent = curPoint;
                target->G = calcG(curPoint, target);
                target->H = calcH(target, &endPoint);
                target->F = calcF(target);
                openList.push_back(target);
            }			//3，对某一个格子，它在开启列表中，计算G值, 如果比原来的大, 就什么都不做, 否则设置它的父节点为当前点,并更新G和F			
            else
            {
                int tempG = calcG(curPoint, target);
                if (tempG < target->G)
                {
                    target->parent = curPoint;
                    target->G = tempG;
                    target->F = calcF(target);
                }
            }
            Point* resPoint = isInList(openList, &endPoint);
            if (resPoint)
                return resPoint; //返回列表里的节点指针，不要用原来传入的endpoint指针，因为发生了深拷贝		
        }
    }
    return NULL;
}
int sgn(double x)
{
    if (x > 0) return 1;
    if (x == 0) return 0;
    if (x < 0) return -1;

}
std::list<char> Astar::GetPath(double x, double y, bool isIgnoreCorner)
{
    Point startPoint((int)(PlayerInfo.position.x), (int)(PlayerInfo.position.y));	//网上往右是+
    cout << "start:" << PlayerInfo.position.x << "," << PlayerInfo.position.y << endl;
    Point endPoint(x, y);
    //A*算法找寻路径	
    Point* result = findPath(startPoint, endPoint, isIgnoreCorner);
    std::list<Point*> path;	//返回路径，如果没找到路径，返回空链表	
    while (result) {
        path.push_front(result);
        result = result->parent;
    }     // 清空临时开闭列表，防止重复执行GetPath导致结果异常   
    openList.clear();
    closeList.clear();
    //return path
    list<Point*>::iterator p;
    list<char>::iterator lp;
    gotolist.clear();
    int i = 1;
    for (p = path.begin();;)
    {
        Point* pre = *p;
        p++;
        int movex = (*p)->x - pre->x;//dir[3][3] = { {'z','x','c' },{'a','s','d'},{'q','w','e'} };
        int movey = (*p)->y - pre->y;
        char c = dir[movey + 1][movex + 1];
        gotolist.push_back(c);//例如dir[2][1]=w,dir[0][2]=c
        if (i == path.size() - 1)break;
        i++;
    }
    return gotolist;
}

Point* Astar::isInList(const std::list<Point*>& list, const Point* point) const
{	//判断某个节点是否在列表中，这里不能比较指针，因为每次加入列表是新开辟的节点，只能比较坐标	
    for (auto p : list)
        if (p->x == point->x && p->y == point->y)
            return p;
    return NULL;
}

bool Astar::isCanreach(const Point* point, const Point* target, bool isIgnoreCorner) const {
    if (target->x<0 || target->x>maze.size() - 1 || target->y<0 || target->y>maze[0].size() - 1 || maze[target->x][target->y] == 1 || target->x == point->x && target->y == point->y || isInList(closeList, target)) //如果点与当前节点重合、超出地图、是障碍物、或者在关闭列表中，返回false		
        return false;
    else
    {
        if (abs(point->x - target->x) + abs(point->y - target->y) == 1) //非斜角可以			
            return true;
        else
        {			//斜对角要判断是否绊住			
            if (maze[point->x][target->y] == 0 && maze[target->x][point->y] == 0)
                return true;
            else
                return isIgnoreCorner;
        }
    }
}
//只保留上下左右四个方向，不然太容易卡住了呜呜呜
std::vector<Point*> Astar::getSurroundPoints(const Point* point, bool isIgnoreCorner) const {
    std::vector<Point*> surroundPoints;
    int x = point->x - 1;
    int y = point->y;
    if (isCanreach(point, new Point(x, y), isIgnoreCorner))
        surroundPoints.push_back(new Point(x, y));
    x = point->x + 1;
    y = point->y;
    if (isCanreach(point, new Point(x, y), isIgnoreCorner))
        surroundPoints.push_back(new Point(x, y));
    x = point->x;
    y = point->y + 1;
    if (isCanreach(point, new Point(x, y), isIgnoreCorner))
        surroundPoints.push_back(new Point(x, y));
    x = point->x;
    y = point->y - 1;
    if (isCanreach(point, new Point(x, y), isIgnoreCorner))
        surroundPoints.push_back(new Point(x, y));
    return surroundPoints;
}
const std::vector<std::vector<short>> init_mapinfo_block =//0表示可以通过，1表示障碍物，这个和原版的地图是一样的，就是把0以外的数字都改成了1.
{
    {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1},
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 1},
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 1},
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 1},
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 1},
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 1},
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 1},
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 1, 0, 0, 0, 1, 1, 1, 1, 0, 0, 1},
    {1, 0, 0, 0, 0, 1, 0, 0, 0, 1, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 1},
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1},
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
    {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1} };

Astar astar(init_mapinfo_block);

void move_dir(char c, int i) //重载一下，i=1时只移动50ms表示时间，主要用于修改朝向。
{
    if (i == 1)
    {
        switch (c)
        {
        case 'd':move(Protobuf::Direction::Right, 50);
            this_thread::sleep_for(time_50ms);
            break;
        case 'e':move(Protobuf::Direction::RightUp, 50);
            this_thread::sleep_for(time_50ms);
            break;
        case 'w':move(Protobuf::Direction::Up, 50);
            this_thread::sleep_for(time_50ms);
            break;
        case 'q':move(Protobuf::Direction::LeftUp, 50);
            this_thread::sleep_for(time_50ms);
            break;
        case 'a':move(Protobuf::Direction::Left, 50);
            this_thread::sleep_for(time_50ms);
            break;
        case 'z':move(Protobuf::Direction::LeftDown, 50);
            this_thread::sleep_for(time_50ms);
            break;
        case 'x':move(Protobuf::Direction::Down, 50);
            this_thread::sleep_for(time_50ms);
            break;
        case 'c':move(Protobuf::Direction::RightDown, 50);
            this_thread::sleep_for(time_50ms);
            break;

        default:
            break;
        }
    }
}

void move_dir(char c) //按照c中表示的方向移动
{
    switch (c)
    {
    case 'd':move(Protobuf::Direction::Right, 200);
        this_thread::sleep_for(time_200ms);
        break;
    case 'e':move(Protobuf::Direction::RightUp, 265);
        this_thread::sleep_for(time_200ms);
        this_thread::sleep_for(time_50ms);
        break;
    case 'w':move(Protobuf::Direction::Up, 200);
        this_thread::sleep_for(time_200ms);
        break;
    case 'q':move(Protobuf::Direction::LeftUp, 265);
        this_thread::sleep_for(time_200ms);
        this_thread::sleep_for(time_50ms);
        break;
    case 'a':move(Protobuf::Direction::Left, 200);
        this_thread::sleep_for(time_200ms);
        break;
    case 'z':move(Protobuf::Direction::LeftDown, 265);
        this_thread::sleep_for(time_200ms);
        this_thread::sleep_for(time_50ms);
        break;
    case 'x':move(Protobuf::Direction::Down, 200);
        this_thread::sleep_for(time_200ms);
        break;
    case 'c':move(Protobuf::Direction::RightDown, 265);
        this_thread::sleep_for(time_200ms);
        this_thread::sleep_for(time_50ms);
        break;

    default:
        break;
    }

}

double calcdis(Point& point, Point& end) {
    //用简单的欧几里得距离计算H，这个H的计算是关键，还有很多算法，没深入研究^_^	
    return sqrt((double)(end.x - point.x) * (double)(end.x - point.x) + (double)(end.y - point.y) * (double)(end.y - point.y));
}

////////////////////////////////////////////////////////////////////////////////

Point findnearfood()//找最近的食物生成点
{
    Point Point1(25, 4), Point2(42, 39), Point3(7, 40);//三个食物生成点下方一格，这样统一再往上走一步就好
    Point Pos(PlayerInfo.position.x, PlayerInfo.position.y);
    double dis1 = calcdis(Point1, Pos);
    double dis2 = calcdis(Point2, Pos);
    double dis3 = calcdis(Point3, Pos);
    if (dis1 <= dis2 && dis1 <= dis3) return Point1;
    if (dis2 <= dis1 && dis2 <= dis3) return Point2;
    if (dis3 <= dis1 && dis3 <= dis2) return Point3;
}

Point findsecondfood()//找第二近的食物生成点
{
    Point Point1(25, 4), Point2(42, 39), Point3(7, 40);//三个食物生成点下方一格，这样统一再往上走一步就好
    Point Pos(PlayerInfo.position.x, PlayerInfo.position.y);
    double dis1 = calcdis(Point1, Pos);
    double dis2 = calcdis(Point2, Pos);
    double dis3 = calcdis(Point3, Pos);
    if ((dis1 <= dis2) != (dis1 < dis3)) return Point1;
    if ((dis2 <= dis1) != (dis2 < dis3)) return Point2;
    if ((dis3 <= dis1) != (dis3 < dis2)) return Point3;
}

char dir_4[5] = { 0,'a','w','d','x' };//用这个数组存一下最后一步的方向
double angle_4[5] = { 0, PI,PI / 2,0,-PI / 2 };//用这个数组存一下最后扔的方向
int cooklabel[5][5] = { {}, {8,25,1,4,0}, {25,37,2,2,0}, {40,28,3,3,0}, {33,17,4,2,0} };
//第一行都是空着备用。
//【灶台边空地坐标xy,灶台编号，最后一步的方向,label(检查的时候方便吧大概)，在做菜吗】

Point findfarfood()//找第二近的食物生成点
{
    Point Point1(25, 4), Point2(42, 39), Point3(7, 40);//三个食物生成点下方一格，这样统一再往上走一步就好
    Point Pos(PlayerInfo.position.x, PlayerInfo.position.y);
    double dis1 = calcdis(Point1, Pos);
    double dis2 = calcdis(Point2, Pos);
    double dis3 = calcdis(Point3, Pos);
    if (dis1 >= dis2 && dis1 >= dis3) return Point1;
    if (dis2 >= dis1 && dis2 >= dis3) return Point2;
    if (dis3 >= dis1 && dis3 >= dis2) return Point3;
}

int findnearcook()//找最近的灶台
{
    Point Point1(8, 25), Point2(25, 37), Point3(40, 28), Point4(33, 17);//四个灶台
    Point Pos(PlayerInfo.position.x, PlayerInfo.position.y);
    double dis1 = calcdis(Point1, Pos);
    double dis2 = calcdis(Point2, Pos);
    double dis3 = calcdis(Point3, Pos);
    double dis4 = calcdis(Point4, Pos);
    if (dis1 <= dis2 && dis1 <= dis3 && dis1 <= dis4) return 1;
    if (dis2 <= dis1 && dis2 <= dis3 && dis2 <= dis4) return 2;
    if (dis3 <= dis1 && dis3 <= dis2 && dis3 <= dis4) return 3;
    if (dis4 <= dis1 && dis4 <= dis2 && dis4 <= dis3) return 4;
}

//从字符获得下一步的x和y怎么变化
int nextx(char c)
{
    if (c == 'd')return 1;
    if (c == 'a')return -1;
    else return 0;
}

int nexty(char c)
{
    if (c == 'w')return 1;
    if (c == 'x')return -1;
    else return 0;
}

int gotodest(Point& dest, int istimelimited = 0)//默认不限制，如果限制，把20000改成cooklabel[4]-5000,返回值为1表示成功
{
    list<char>::iterator lp;
    int x = dest.x;
    int y = dest.y;
    //用Getpath获得一个由字符组成的链表，字符代表移动方向，每一个是用200ms的时间走一格。
    cout << "speed" << PlayerInfo.moveSpeed << endl;
    if (astar.maze[dest.x][dest.y] == 1)//首先检测目标是否是障碍物，如果是，搜索周围四格中不是障碍物的点作为目标。
    {
        cout << "change dest" << endl;
        if (astar.maze[dest.x + 1][dest.y] == 0) {
            x = dest.x + 1;
            y = dest.y;
        }
        if (astar.maze[dest.x - 1][dest.y] == 0) {
            x = dest.x - 1;
            y = dest.y;
        }
        if (astar.maze[dest.x][dest.y + 1] == 0) {
            x = dest.x;
            y = dest.y + 1;
        }
        if (astar.maze[dest.x + 1][dest.y - 1] == 0) {
            x = dest.x;
            y = dest.y - 1;
        }
    }
    if ((int)PlayerInfo.position.x == x && (int)PlayerInfo.position.y == y)//如果已经在目的地，没必要移动。
    {
        cout << "no need move" << endl;
        return 1;
    }
    int timeinit = getGameTime();
    int time;
    while (1) {
        if (istimelimited == 0) {
            time = getGameTime() - timeinit;
            cout << "time" << time << endl;
            if (time >= 20000) {
                cout << "kia zhu le !!!" << endl;
                return 0;//这么久肯定是被卡住了，放弃吧。
            }
        }
        else
        {
            if (cooking[4] < getGameTime() + 5000) {
                cout << "xian hui qu ba" << endl;
                return 0;//正在做菜呢，先回去不然糊了
            }
        }
        for (auto& lp : astar.GetPath(x, y, false))
        {
            double pos_prex = PlayerInfo.position.x;
            double pos_prey = PlayerInfo.position.y;
            double pos_nextx = pos_prex + nextx(lp);
            double pos_nexty = pos_prey + nexty(lp);
            //list<Obj> l = MapInfo::get_mapcell((int)pos_nextx, (int)pos_nexty);//先检查下一个格子有没有能捡起来的或者trigger
            //trigger不检查了 就检查脚下有没有东西吧
            list<Obj> l = MapInfo::get_mapcell((int)pos_prex, (int)pos_prey);
            for (list<Obj>::iterator i = l.begin(); i != l.end(); i++)
            {
                if (i->objType == Tool) {
                    int t = i->tool;
                    cout << "tool" << t << endl;
                    switch (t)
                    {
                    case StrengthBuff://3
                    case TeleScope://4
                    case BreastPlate://7 护心镜
                        pick(TRUE, Tool, t);
                        this_thread::sleep_for(time_50ms);
                        if (PlayerInfo.tool == t) {
                            use(1, 0, 0);
                            this_thread::sleep_for(time_50ms);
                        }
                        cout << "use tool " << t << endl;
                        break;
                    case SpaceGate://8 传送门
                        pick(TRUE, Tool, t);
                        this_thread::sleep_for(time_50ms);
                        if (PlayerInfo.tool == t) {
                            use(1, x - (int)PlayerInfo.position.x, y - (int)PlayerInfo.position.y == y);//相对位移
                            this_thread::sleep_for(time_50ms);
                            cout << "use tool " << t << endl;
                            if (PlayerInfo.position.x == x && PlayerInfo.position.y == y) return 1;
                        }
                        break;
                    case Condiment://调料
                        pick(TRUE, Tool, t);
                        this_thread::sleep_for(time_50ms);
                        cout << "pick condiment " << t << endl;
                        break;
                    default:break;
                    }
                    //if (i->objType == 4) cout << "trigger" << i->trigger << endl;
                }
            }
            move_dir(lp);//处理完刚刚的问题再移动
            //cout << "form" << PlayerInfo.position.x << "  " << PlayerInfo.position.y << "  move:" << lp << endl;
            if (PlayerInfo.position.x == pos_prex && PlayerInfo.position.y == pos_prey) {
                cout << "error!" << PlayerInfo.position.x << "," << PlayerInfo.position.y << endl;
                //char dir_4[5] = { 0,'a','w','d','x' };//用这个数组存一下最后一步的方向
                int rand1 = rand() % 4 + 1;//1,2,3,4
                int rand2 = rand() % 2 - 1;//-1,0,1  0~5
                int add12 = rand1 + rand2;
                if (add12 == 0)add12 = 4;
                if (add12 == 5) add12 = 1;
                cout << "random move" << dir_4[rand1] << dir_4[add12] << endl;;
                move_dir(dir_4[rand1]);
                move_dir(dir_4[add12]);
                break;//break for 循环
            }//如果移动失败了，随机向某个方向移动一格，无论是否成功，都break循环，重新生成Getpath
            //cout << THUAI3::getGameTime() << endl;
        }
        if ((int)PlayerInfo.position.x == (int)x && (int)PlayerInfo.position.y == (int)y)
        {
            cout << "finish!" << endl;
            return 1;
        }
        //直到到达目的地再break循环。
    }
    return 1;
}

void smallmove(double x, double y)//仅用于微小的移动，通过50ms=0.25个单位来实现,即差距乘4，四舍五入之后移动50ms
{
    double posx = PlayerInfo.position.x;
    double posy = PlayerInfo.position.y;
    double detx = x - posx;
    double dety = y - posy;
    if (abs(detx) > 2 || abs(dety) > 2) return;
    if (detx >= 0.25)
    {
        int count = round(detx * 4);
        while (count != 0)
        {
            move_dir('d', 1);//右移50
            count--;
        }
    }
    if (detx <= -0.25)
    {
        int count = round(abs(detx * 4));
        while (count != 0)
        {
            move_dir('a', 1);//左移50
            count--;
        }
    }
    if (dety >= 0.25)
    {
        int count = round(dety * 4);
        while (count != 0)
        {
            move_dir('w', 1);//右移50
            count--;
        }
    }
    if (dety <= -0.25)
    {
        int count = round(abs(detx * 4));
        while (count != 0)
        {
            move_dir('x', 1);//左移50
            count--;
        }
    }
}

//走到灶台后第一件事检查黑暗料理
int throw_darkdish(int _label)//检查灶台上有无黑暗料理，如果有，尝试拾取，拾取成功就扔掉，拾取失败就标记这个灶台不行，找下一个灶台
{
    int rand1 = rand() % 4 + 1;//1,2,3,4
    int rand2 = rand() % 2 - 1;//-1,0,1  0~5
    int add12 = rand1 + rand2;
    if (add12 == 0)add12 = 4;
    if (add12 == 5) add12 = 1;
    if (cooklabel[_label][4] == 1)return 0;//这个灶台是自己正在做的，不要扔黑暗料理
    //有时候自己做了菜，但是没有拿到手，这时候会把label变成0，但是这还是我的菜啊
    int x = cooklabel[_label][0] + nextx(dir_4[cooklabel[_label][3]]);
    int y = cooklabel[_label][1] + nexty(dir_4[cooklabel[_label][3]]);
    char c = dir_4[cooklabel[_label][3]];
    list<Obj> l = MapInfo::get_mapcell(x, y);
    for (list<Obj>::iterator i = l.begin(); i != l.end(); i++)
    {
        if (i->blockType == 3 && i->dish != 0)//如果发现灶台里面有菜
        {
            cout << "dish in cook!" << endl;
            put(1, PI, TRUE);//先把手里的东西放脚下
            move_dir(c);//确保朝向
            pick(FALSE, Block, 0);//尝试拾取
            this_thread::sleep_for(time_50ms);
            cout << "DISH IN BLOCK  " << i->dish << "  _label  " << _label << " label " << label << "  is my cook  " << cooklabel[label][4] << endl;
            if (PlayerInfo.dish == 0)//说明没有拿到dish,也就是这个灶台正在烹饪
            {
                //随便走两步，再拿一下试试
                cout << "random move" << dir_4[rand1] << dir_4[add12] << endl;;
                move_dir(dir_4[rand1]);
                move_dir(dir_4[add12]);
                gotodest(Point(cooklabel[_label][0], cooklabel[_label][1]));
                smallmove(cooklabel[_label][0] + 0.5, cooklabel[_label][1] + 0.5);
                move_dir(c);//确保朝向
                pick(FALSE, Block, 0);//尝试拾取
                if (PlayerInfo.dish == 0) {
                    if (label == _label)label = 0;//如果这就是我标记的据点，那么取消标记。
                    return 1;//继续跑第二近的灶台。
                }
            }
            if (PlayerInfo.dish >= 49)//如果拿到了黑暗料理，扔出去
            {
                cout << "throw dark dish  " << PlayerInfo.dish << endl;
                put(2, 0, TRUE);//先往右扔两格,我下次看看最多能扔多远……
                return 2;//用这个灶台做菜，不要慌
            }
            else //不是黑暗料理！捡到宝了！
            {
                return 3;//准备提交食物，耶
            }
        }

    }
    return 2;//灶台里啥都没有，直接做菜。
}

int findallcook(int labelofcook)//从label开始转一圈圈,int=1表示找到了可以用的灶台，int=2表示拿到菜了。
{
    int flabel = labelofcook;
    int isfind = 0;
    while (isfind == 0)
    {
        flabel = labelofcook + 1;//2 3 4 5
        if (flabel == 5)flabel = 1;
        cout << "goto next label=" << flabel << endl;
        gotodest(Point(cooklabel[flabel][0], cooklabel[flabel][1])); //{x,y,编号，朝向，label}
        int nextstate = throw_darkdish(flabel);//到达灶台后，先检查有无黑暗料理
        cout << "label  " << label << "  is my cook  " << cooklabel[label][4] << endl;
        switch (nextstate)
        {
        case 0://这是我的据点？我正在做菜？bug吧？
            cout << "what wrong?" << endl;
            label = flabel;
            isfind = 1;
            break;
        case 1://继续跑第二近的灶台。
            cout << "next next label" << endl;
            break;
        case 2:
            label = flabel;//用这个灶台好啦
            cout << "this is ok!" << endl;
            isfind = 1;
            return 1;
            break;
        case 3:
            cout << "get dish" << endl;
            state = 2;
            isfind = 1;
            return 2;
            break;
        }
    }
}


int rawfood[52][3];//全局变量保平安，谁乐意传参啊hh
//每一行是 {有无食材 有1 无0，食材坐标x,食材坐标y}，如果有不止一个食材，直接覆盖，无所谓。
//第一行raw[0][0]=1,方便后面运算
//如果食材在队友手上，坐标记0，0，如果在自己手里，记50，50
//不如都先把食材扔下去吧？这样省事。
//第51行专门用来记录调料[有无][x][y]

int surround[25][2];//懒得考虑返回值了，直接全局变量
//1️⃣自身为中心共25格，顺序从远到近,从2，2开始顺时针绕回中心。
void get_surround()
{
    int x = PlayerInfo.position.x;
    int y = PlayerInfo.position.y;
    int k = 0;
    for (int i = 2; i >= -2; i--) {
        surround[k][0] = x + 2;
        surround[k][1] = y + i;
        k++;
    }
    for (int i = 1; i >= -2; i--)
    {
        surround[k][0] = x + i;
        surround[k][1] = y - 2;
        k++;
    }
    for (int i = -1; i <= 2; i++)
    {
        surround[k][0] = x - 2;
        surround[k][1] = y + i;
        k++;
    }
    for (int i = -1; i <= 1; i++)
    {
        surround[k][0] = x + i;
        surround[k][1] = y + 2;
        k++;
    }
    for (int i = 1; i >= -1; i--) {
        surround[k][0] = x + 1;
        surround[k][1] = y + i;
        k++;
    }
    surround[k][0] = x;
    surround[k][1] = y - 1;
    k++;
    surround[k][0] = x - 1;
    surround[k][1] = y - 1;
    k++;
    surround[k][0] = x - 1;
    surround[k][1] = y;
    k++;
    surround[k][0] = x - 1;
    surround[k][1] = y + 1;
    k++;
    surround[k][0] = x;
    surround[k][1] = y + 1;
    k++;
    surround[k][0] = x;
    surround[k][1] = y;
    k++;
}

void get_all_dish(int _label)//为了方便，改成灶台周围八格+自身周围八格，自身周围四格最后（这样可以覆盖前面）。
//返回一个0-51的数组，每个数字指示mapcell中是否有该食材，第0位为1时,说明什么都没有。
{
    for (int i = 0; i <= 50; i++)
    {
        for (int j = 0; j <= 2; j++)
        {
            rawfood[i][j] = 0;//初始化
        }
    }
    rawfood[0][0] = 1;//第一行raw[0][0]=1,方便后面运算
    int x = cooklabel[_label][0] + nextx(dir_4[cooklabel[_label][3]]);
    int y = cooklabel[_label][1] + nexty(dir_4[cooklabel[_label][3]]);
    get_surround();
    for (int i = 0; i <= 24; i++)
    {
        list<Obj> l = MapInfo::get_mapcell(surround[i][0], surround[i][1]);
        for (list<Obj>::iterator i = l.begin(); i != l.end(); i++)
        {
            if (i->dish != 0)//
            {
                rawfood[i->dish][0] = 1;
                rawfood[i->dish][1] = (int)i->position.x;//这是东西的位置啦
                rawfood[i->dish][2] = (int)i->position.y;
            }
            if (i->tool == Condiment)//调料
            {
                rawfood[51][0] = 1;
                rawfood[51][1] = (int)i->position.x;//这是东西的位置啦
                rawfood[51][2] = (int)i->position.y;
            }
        }
    }

}
int get_one_dish(int x, int y)//看看食物生成点有没有食物
{
    list<Obj> l = MapInfo::get_mapcell(x, y);
    for (list<Obj>::iterator i = l.begin(); i != l.end(); i++)
    {
        if (i->objType == Block)
        {
            cout << "dish:" << i->dish << endl;
            return i->dish;
        }
    }
}

int pick_dish_in_block(Point& food) {//Point 是食物生成点下方一格
    if (((int)PlayerInfo.position.x) != food.x || ((int)PlayerInfo.position.y) != food.y) {
        gotodest(Point(food.x, food.y));
        move_dir('w');
    }
    int dish = get_one_dish((int)PlayerInfo.position.x, (int)PlayerInfo.position.y + 1);
    while (dish == 0)//如果dish=0说明没有食物,就一直等着
    {
        this_thread::sleep_for(time_50ms);
        dish = get_one_dish((int)PlayerInfo.position.x, (int)PlayerInfo.position.y + 1);
        cout << "wait for dish" << endl;
    }

    pick(FALSE, Block, 0);//是block的时候第三个随便输入,表示捡起block里的食材。
    cout << "pick finish" << endl;
    move_dir(dir_4[rand() % 4 + 1]);//随机走一下，防止卡住
    return PlayerInfo.dish;
}
const int  howtodolist[21][4] =//菜肴编号-20=行号
{
    {1,0,0,0},//0-20:面粉
    {20,0,0,0},//1-21：面条
    {4,20,0,0},
    {2,0,0,0},
    {3,0,0,0},
    {5,0,0,0},
    {3,4,0,0},
    {21,26,0,0},
    {11,21,0,0},
    {12,18,23,0},
    {13,14,23,0},
    {12,15,0,0},
    {14,24,0,0},
    {8,9,0,0},
    {22,11,15,0},
    {10,25,0,0},
    {13,24,0,0},
    {16,4,0,0},
    {4,20,25,10},
    {9,0,0,0},
    {6,7,3,10}
};
int candolist[51];
int xiangguo;
int* get_candolist()//size=0~26,rawfood[0]=1，rawfood为int[51][3],有无，x,y
{
    //20-25为中间产物，26番茄炒蛋，27-40为成品菜，41-47为香锅，50位黑暗料理。
    for (int i = 0; i <= 50; i++)//初始化
    {
        candolist[i] = 0;
    }
    ///菜
    for (int i = 20; i <= 40; i++)
    {
        int i1 = i - 20;//菜肴编号-20用于检索howtodolist的行,rawfood[0][0]=1
        candolist[i] = rawfood[howtodolist[i1][0]][0] && rawfood[howtodolist[i1][1]][0] &&
            rawfood[howtodolist[i1][2]][0] && rawfood[howtodolist[i1][3]][0];
        if (candolist[i] != 0)cout << "," << i;
    }
    cout << "cando list generate finished!" << endl;
    //单独从tasklist处理香锅
    return candolist;
}

int whichfood() {
    list<DishType> tk = task_list;
    int sum = 0;
    for (int i = 26; i <= 40; i++)//先看能不能制作成品菜
    {
        sum += candolist[i];
    }
    cout << "sum=" << sum << endl;
    if (sum < 0) return 0;//鬼知道为啥
    if (sum != 0) {
        //优先制作任务列表里有的菜
        for (list<DishType>::iterator i = tk.end(); i != tk.begin(); i--)
        {
            cout << " tasklist :" << *i;
            if (*i <= 50 && *i >= 1 && candolist[*i] == 1) {
                cout << "make food in task list!" << endl;
                return *i;
            }
        }

        //如果都不在任务列表里，当你懒得考虑做什么菜，就随机一下,先看到哪个做哪个。
        vector<int> random;
        for (int i = 26; i <= 40; i++)
        {
            random.push_back(i);
        }
        random_shuffle(random.begin(), random.end());
        for (vector<int>::iterator i = random.begin(); i != random.end(); i++)
        {
            if (candolist[*i] == 1) return *i;
        }

    }
    else //一个成品菜都做不了，看看能不能合成中间产物,如果中间产物已经有了就算了不多做了。
    {
        if (candolist[20] == 1 && rawfood[20][0] != 1) return 20;//面粉》米饭》面条》番茄酱》奶油》面包
        if (candolist[23] == 1 && rawfood[23][0] != 1) return 23;
        if (candolist[24] == 1 && rawfood[24][0] != 1) return 24;
        if (candolist[21] == 1 && rawfood[21][0] != 1) return 21;
        if (candolist[25] == 1 && rawfood[25][0] != 1) return 25;
        if (candolist[22] == 1 && rawfood[21][0] != 1) return 22;
        return 0;//什么都做不了，return0
    }
    return 0;
}
//const char dir[3][3] = { {'z','x','c' },{'a','s','d'},{'q','w','e'} };

double angle_abs(Point& dest)//啊 是弧度
{
    double x = dest.x + 0.5 - PlayerInfo.position.x;
    double y = dest.y + 0.5 - PlayerInfo.position.y;
    cout << "destx=" << dest.x << ", posx=" << PlayerInfo.position.x << endl;
    cout << "desty=" << dest.y << ", posy=" << PlayerInfo.position.y << endl;
    if (x == 0) { x += 0.00000001; }
    if (y == 0) { y += 0.00000001; }//不知道=0会不会崩，偏一点点吧。
    return atan2(y, x);//atan2返回弧度
}
void put_dest(Point& dest, bool isdish)//计算从当前位置到目标位置，需要的角度和距离,目标x+0.5,y+0.5才是中心点
{
    double angle = angle_abs(dest);
    double dis = calcdis(Point(PlayerInfo.position.x, PlayerInfo.position.y), dest);
    cout << "put: dis=" << dis << "   angle:" << angle << endl;
    put(dis, angle, isdish);
    this_thread::sleep_for(time_50ms);
    return;
}
void move_allfood_to_left() {
    //先把灶台里所有的东西都丢到左边,并且修改刚刚的坐标到左边一格
    cout << "move all food to right" << endl;
    int cookx = cooklabel[label][0] + nextx(dir_4[cooklabel[label][3]]);//这是灶台的坐标
    int cooky = cooklabel[label][1] + nexty(dir_4[cooklabel[label][3]]);
    char c = dir_4[cooklabel[label][3]];//朝向
    gotodest(Point(cooklabel[label][0], cooklabel[label][1]));
    move_dir(c, 1);//调整朝向
    list<Obj> l = MapInfo::get_mapcell(cookx, cooky);//看看灶台里有啥
    for (list<Obj>::iterator i = l.begin(); i != l.end(); i++)
    {
        cout << "Obj TYPE :" << i->objType << " DISH:" << i->dish << endl;
        if (i->objType == Dish && i->dish != 0)
        {
            pick(FALSE, Dish, i->dish);
            this_thread::sleep_for(time_50ms);
            put(1, PI, TRUE);//扔到左边，修改坐标为本人左一格。
            rawfood[i->dish][1] = cooklabel[label][0] - 1;
            rawfood[i->dish][2] = cooklabel[label][1];
            this_thread::sleep_for(time_50ms);
        }
    }

}

//捡调料，参数为1要扔到灶台，参数为0不扔
int getcondiment(int isthrow)
{
    Point foodpos(rawfood[51][1], rawfood[51][2]);//查看食材所在位置
    gotodest(foodpos);
    int cookx = cooklabel[label][0] + nextx(dir_4[cooklabel[label][3]]);
    int cooky = cooklabel[label][1] + nexty(dir_4[cooklabel[label][3]]);
    int movex = sgn((int)foodpos.x - (int)PlayerInfo.position.x);
    int movey = sgn((int)foodpos.y - (int)PlayerInfo.position.y);
    char fooddir = dir[movey + 1][movex + 1];//计算最后的朝向
    if (fooddir == 's')//如果就在自己脚下
    {
        pick(TRUE, Tool, Condiment);
        this_thread::sleep_for(time_50ms);
        if (PlayerInfo.tool == 0)
        {
            cout << "pick condiment filed!" << endl;
            return 0;
        }//这样就是没捡起来，返回0失败，直接重新找食材。
        if (isthrow == 1)//如果要扔到灶台，那扔呗
        {
            put_dest(Point(cookx, cooky), FALSE);//这里+0.5没用，因为point是整数。
        }
    }
    else
    {
        move_dir(fooddir, 1);//调整朝向
        pick(FALSE, Tool, Condiment);
        this_thread::sleep_for(time_50ms);
        if (PlayerInfo.tool == 0)
        {
            cout << "pick condiment filed!" << endl;
            return 0;//这样就是没捡起来，返回0失败，直接重新找食材。

        }
        if (isthrow == 1)//如果要扔到灶台，那扔呗
        {
            put_dest(Point(cookx, cooky), FALSE);//这里+0.5没用，因为point是整数。
        }
    }
    return 1;
}


int makefood(int food)//传入目标的编号
{

    if (food == 0)return 0;//如果收到的是0，那就啥都做不了，接着找食材吧
    int cookx = cooklabel[label][0] + nextx(dir_4[cooklabel[label][3]]);
    int cooky = cooklabel[label][1] + nexty(dir_4[cooklabel[label][3]]);
    char c = dir_4[cooklabel[label][3]];//朝向
    int row = food - 20;//行号
    move_allfood_to_left();
    //首先查找howtodolist[21][4]={{需要的食材编号,最多四种，多的置0}}第0行对应20面粉。
    cout << "start to pick food to cook" << endl;
    for (int i = 0; i <= 3; i++)
    {
        int destraw = howtodolist[row][i];//制作food需要食材的编号
        if (destraw == 0)break;//如果遇到0，说明后面没有需要的食材了。
        Point foodpos(rawfood[destraw][1], rawfood[destraw][2]);//查看食材所在位置
        gotodest(foodpos);
        int movex = sgn((int)foodpos.x - (int)PlayerInfo.position.x);
        int movey = sgn((int)foodpos.y - (int)PlayerInfo.position.y);
        char fooddir = dir[movey + 1][movex + 1];//计算最后的朝向
        if (fooddir == 's')//如果就在自己脚下
        {
            pick(TRUE, Dish, destraw);
            this_thread::sleep_for(time_50ms);
            if (PlayerInfo.dish == 0)return 0;//这样就是没捡起来，返回0失败，直接重新找食材。
            put_dest(Point(cookx, cooky), TRUE);//这里+0.5没用，因为point是整数。

        }
        else
        {
            move_dir(fooddir, 1);//调整朝向
            cout << "move : " << fooddir << endl;
            pick(FALSE, Dish, destraw);
            this_thread::sleep_for(time_50ms);
            if (PlayerInfo.dish == 0)
            {
                cout << "pick filed!" << endl;
                return 0;//这样就是没捡起来，返回0失败，直接重新找食材。
            }
            put_dest(Point(cookx, cooky), TRUE);
        }
        cout << "picked :" << destraw << endl;
    }
    //如果要做香锅，调用getcondiment(int isthrow=1)。
    gotodest(Point(cooklabel[label][0], cooklabel[label][1]));//再回到灶台跟前
    move_dir(c, 1);//调整朝向

    list<Obj> l = MapInfo::get_mapcell(cookx, cooky);//看看灶台里有啥
    for (list<Obj>::iterator i = l.begin(); i != l.end(); i++)
    {
        if (i->blockType == Dish) {
            int isneed = 0;//isneed=0表示不需要这个dish
            if (i->dish != 0) {
                for (int j = 0; j <= 4; j++)
                {
                    int destraw = howtodolist[row][j];//制作food需要食材的编号
                    if (destraw == 0)break;//如果遇到0，说明后面没有需要的食材了。
                    if (i->dish == destraw) {//如果能够和某个需要的食材匹配，就不扔出去了。
                        isneed = 1;
                    }
                }
                if (isneed == 0) {//不需要就统一扔到左边一格
                    pick(FALSE, Dish, i->dish);
                    this_thread::sleep_for(time_50ms);
                    put(1, PI, TRUE);
                    this_thread::sleep_for(time_50ms);
                }
                isneed = 0;
            }
        }
    }
    smallmove(cooklabel[label][0], cooklabel[label][1]);
    move_dir(c, 1);//调整朝向
    use(0, 0, 0);//开始做菜
    this_thread::sleep_for(time_50ms);
    smallmove(cooklabel[label][0], cooklabel[label][1]);
    move_dir(c, 1);//调整朝向
    use(0, 0, 0);//多试一次呗
    cout << "*****start to cook:" << food << endl;
    this_thread::sleep_for(time_50ms);
    l = MapInfo::get_mapcell(cookx, cooky);//看看灶台里有啥
    for (list<Obj>::iterator i = l.begin(); i != l.end(); i++)
    {
        if (i->blockType == 3 && i->dish == 50) return 1;//在做了在做了
        cout << "error :: block type:" << i->blockType << "  dish:" << i->dish << endl;
    }
    return 0;//如果刚刚没有return1，那就是没做上。
}

const static int DishInfo[22][4] =
{
{int(Protobuf::Flour), 0,10000,0 },//20=0
{int(Protobuf::Noodle), 0,10000,0 },
{int(Protobuf::Bread), 0,10000,0 },
{int(Protobuf::CookedRice), 0,10000,0 },
{int(Protobuf::Ketchup), 0,10000,0 },
{int(Protobuf::Cream), 0,10000,0 },//25
{int(Protobuf::TomatoFriedEgg), 50,10000,60000 },
{int(Protobuf::TomatoFriedEggNoodle), 100,15000,90000},
{int(Protobuf::BeefNoodle), 80,20000,90000 },
{int(Protobuf::OverRice), 90,20000,90000},
{int(Protobuf::YellowPheasant), 100,20000,90000 },//30
{int(Protobuf::Barbecue), 55,20000,90000 },
{int(Protobuf::FrenchFries), 60,15000,90000},
{int(Protobuf::PlumJuice), 50,10000,90000 },
{int(Protobuf::Hamburger), 110,20000,100000 },
{int(Protobuf::StrawberryIcecream), 60,10000,90000 },//35
{int(Protobuf::PopcornChicken), 60,15000,90000 },
{int(Protobuf::AgaricFriedEgg), 50,15000,90000 },
{int(Protobuf::Cake), 160,30000,120000 },
{int(Protobuf::SugarCoatedHaws), 20,10000,60000 },
{int(Protobuf::FruitSalad), 100,20000,120000 },//40=20
{int(Protobuf::SpicedPot), 0,60000,300000 },
};

Point findsave()//找最近的食物储藏
{
    Point save1(5, 15), save2(26, 47), save3(42, 9);//准备三个储藏点，放到最近的一个藏起来
    Point Pos(PlayerInfo.position.x, PlayerInfo.position.y);
    double dis1 = calcdis(save1, Pos);
    double dis2 = calcdis(save2, Pos);
    double dis3 = calcdis(save3, Pos);
    if (dis1 <= dis2 && dis1 <= dis3) return save1;
    if (dis2 <= dis1 && dis2 <= dis3) return save2;
    if (dis3 <= dis1 && dis3 <= dis2) return save3;
}

int commitTask() {
    //手上拿有成品菜式时调用此方法
    Point StoragePos();//储存成品的地点
    static vector<int[3]> finishedDish;  //这里放已经完成的菜品清单
    int flag = -1;                      // 1:去交菜品 2:放置菜品至储存点
    int randpoint = rand() % 2;
    Point destsubmit(26, 24 + randpoint);//随机去两个点之一交任务
    Point save = findsave();//准备最近的藏匿点，并且发消息告诉队友
    if (find(task_list.begin(), task_list.end(), PlayerInfo.dish) != task_list.end()) {
        //手上的菜品在任务清单里面
        gotodest(destsubmit);
        move_dir('a', 1);
        while (PlayerInfo.dish != 0)
        {
            if (PlayerInfo.tool == Condiment)
            {
                use(1, 0, 0);//用调料提交
                this_thread::sleep_for(time_50ms);
            }
            else
            {
                use(0, 0, 0);
                this_thread::sleep_for(time_50ms);
            }
        }
        return 1;//返回1说明提交成功
    }
    else //不在任务清单里 先存起来
    {
        stringstream sdishinfo;
        sdishinfo << 'd' << " " << PlayerInfo.dish << " " << (int)save.x << " " << (int)save.y;
        cout << "send" << sdishinfo.str() << endl;
        speakToFriend(string(sdishinfo.str()));
        gotodest(save);
        this_thread::sleep_for(time_50ms);
        put(0, 0, TRUE);
        this_thread::sleep_for(time_50ms);
        if (PlayerInfo.tool == Condiment)
        {
            put(0, 0, FALSE);
        }
        return 0;//返回0说明存起来了
    }
}
vector<DishType> finishedDish;  //这里放已经完成的菜品清单
int commitTask2() {
    for (auto i : task_list) {
        auto findResult = find(finishedDish.begin(), finishedDish.end(), i);
        if (findResult != finishedDish.end())  //任务所需物品是之前储存过的
        {
            //TODO
            //这里写到指定地点存放手中的菜品
            finishedDish.push_back(PlayerInfo.dish);
            //TODO
            //这里写去储存位置，捡起来新的菜品
            gotodest(Point(26, 24));
            move_dir('a');
            while (PlayerInfo.dish != 0) {
                list<DishType> tk = task_list;
                for (auto i = tk.end(); i != tk.begin(); i--) {
                    if (PlayerInfo.dish == *i) {
                        use(0, 0, 0);
                        for (auto it = finishedDish.begin(); it != finishedDish.end(); it++)
                            if (*it == PlayerInfo.dish)
                                finishedDish.erase(it);
                        this_thread::sleep_for(time_50ms);
                    }
                }
            }
            return 1;
        }

    }
}
int getspeak()
{
    char c;
    int l, dish,x, y;
    string re=PlayerInfo.recieveText;
    cout << re << endl;
    stringstream restream;
    restream << re.c_str();
    restream >> c;
    switch (c)
    {
    case 'l':
        restream >> c>>l;
        cout << "label change to" << l << endl;
        break;
    case 'd':
        restream >> c >>dish>> x >> y;
        cout << "dish to submit" << dish << "in :" << x << "," << y << endl;
        break;
    }

}
int first = 1;


void play()
{
    //state=3 没任务 游荡，一旦发现有可以提交的，就去提交，变成state=4，随时接受信息，搜索tk
    //state=4 

    //通信规范 字母 数值
    //'l' 数字            label的改变
    //'d' dish 坐标 坐标   存放dish
   /* if (first == 1) {
        first = 0;
        this_thread::sleep_for(time_25ms);
        gotodest(findfarfood());
    }*/
    getspeak();
    this_thread::sleep_for(time_200ms);
}
